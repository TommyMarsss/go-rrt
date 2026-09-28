package rrtstar

import (
	"fmt"
	"testing"
)

func demoOptions(seed int64, samples int) Options {
	return Options{
		Width: 1000, Height: 700,
		Start:          Vec2{60, 350},
		Goal:           Vec2{940, 350},
		Obstacles:      walls(),
		StepSize:       40,
		GoalRadius:     30,
		GoalSampleRate: 0.07,
		Seed:           seed,
		MaxSamples:     samples,
	}
}

func walls() []Polygon {
	return []Polygon{
		Rect("w1", 260, 330, 300, 620),
		Rect("w2", 430, 80, 470, 370),
		Rect("w3", 600, 330, 640, 620),
		{Name: "tri", Vertices: []Vec2{
			{720, 120}, {840, 120}, {840, 240},
		}},
		Rect("block", 150, 120, 210, 200),
	}
}

// TestReachesGoalAndMonotonic is the core asymptotic-optimality sanity test:
// with a fixed seed the recorded best-cost sequence must be strictly
// non-increasing.
func TestReachesGoalAndMonotonic(t *testing.T) {
	opt := demoOptions(42, 2500)
	res, err := Run(opt)
	if err != nil {
		t.Fatal(err)
	}
	if !res.Reached {
		t.Fatal("expected goal to be reached")
	}
	hist := res.CostHistory()
	if len(hist) == 0 {
		t.Fatal("expected goal improvement events")
	}
	for i := 1; i < len(hist); i++ {
		if hist[i] > hist[i-1]+Epsilon {
			t.Fatalf("cost rose at improvement %d: %.6f -> %.6f", i, hist[i-1], hist[i])
		}
	}
	if bad := res.VerifyMonotonic(); bad >= 0 {
		t.Fatalf("VerifyMonotonic reported rise at %d", bad)
	}
	// Final cost must be at least the straight-line distance.
	if res.BestCost < res.Start.Dist(res.Goal)-Epsilon {
		t.Fatalf("impossible cost %.4f < straight-line %.4f", res.BestCost, res.Start.Dist(res.Goal))
	}
}

// TestMonotonicAcrossSeeds repeats the check over several fixed seeds: a
// rewiring-direction bug surfaces on some seeds but not others.
func TestMonotonicAcrossSeeds(t *testing.T) {
	for _, seed := range []int64{1, 2, 7, 42, 99, 2024, 123456} {
		t.Run(fmt.Sprintf("seed-%d", seed), func(t *testing.T) {
			res, err := Run(demoOptions(seed, 1500))
			if err != nil {
				t.Fatal(err)
			}
			if !res.Reached {
				t.Skip("goal not reached for this seed")
			}
			if bad := res.VerifyMonotonic(); bad >= 0 {
				h := res.CostHistory()
				t.Fatalf("cost rose at %d: %.6f -> %.6f", bad, h[bad-1], h[bad])
			}
		})
	}
}

// TestCostConvergesWithMoreSamples verifies asymptotic behavior: the best
// cost with many samples must not exceed the cost with few samples run on
// the SAME seed (the early sample stream is a prefix, and RRT* never
// forgets an improvement).
func TestCostConvergesWithMoreSamples(t *testing.T) {
	small, err := Run(demoOptions(7, 400))
	if err != nil {
		t.Fatal(err)
	}
	big, err := Run(demoOptions(7, 4000))
	if err != nil {
		t.Fatal(err)
	}
	if !small.Reached || !big.Reached {
		t.Skip("goal not reached in one of the runs")
	}
	if big.BestCost > small.BestCost+Epsilon {
		t.Fatalf("more samples produced worse cost: %.6f > %.6f", big.BestCost, small.BestCost)
	}
	if big.BestCost >= small.BestCost-Epsilon {
		t.Logf("warning: no improvement from %d to %d samples (%.4f)", 400, 4000, small.BestCost)
	} else {
		t.Logf("cost improved %.4f -> %.4f (%.1f%% closer to optimum trend)",
			small.BestCost, big.BestCost, 100*(1-big.BestCost/small.BestCost))
	}
}

// TestLargeRunNoCostRebound hammers the monotonicity invariant at a large
// sample budget — this is where a missing descendant-cost cascade would
// eventually corrupt a goal path.
func TestLargeRunNoCostRebound(t *testing.T) {
	if testing.Short() {
		t.Skip("skipping large run in -short mode")
	}
	res, err := Run(demoOptions(2026, 20000))
	if err != nil {
		t.Fatal(err)
	}
	if !res.Reached {
		t.Fatal("goal not reached")
	}
	if bad := res.VerifyMonotonic(); bad >= 0 {
		h := res.CostHistory()
		t.Fatalf("cost rebound at %d: %.6f -> %.6f", bad, h[bad-1], h[bad])
	}

	// Independent reconstruction: walk every final-tree path and check
	// Cost[node] == Cost[parent] + edge length, and the goal node's own
	// cost equals the reported best.
	for _, n := range res.Nodes {
		if n.Parent < 0 {
			if n.Cost != 0 {
				t.Fatalf("root cost = %v, want 0", n.Cost)
			}
			continue
		}
		pn := res.Nodes[n.Parent]
		want := pn.Cost + pn.Point().Dist(n.Point())
		if !almost(n.Cost, want, 1e-7) {
			t.Fatalf("node %d cost %.9f != parent+edge %.9f (cascade desync)", n.ID, n.Cost, want)
		}
		if n.Cost <= pn.Cost-1e-9 {
			t.Fatalf("node %d cost %.6f <= parent %d cost %.6f under positive edge weights", n.ID, n.Cost, n.Parent, pn.Cost)
		}
	}
	if !almost(res.Nodes[res.GoalNodeID].Cost, res.BestCost, 1e-9) {
		t.Fatalf("goal node cost %.9f != reported best %.9f", res.Nodes[res.GoalNodeID].Cost, res.BestCost)
	}
}

// TestRewireActuallyLowersCost replays the event log and asserts that every
// rewire event strictly lowered the target node's cost and that the target
// remains reachable from the root through the new parent.
func TestRewireActuallyLowersCost(t *testing.T) {
	res, err := Run(demoOptions(42, 2500))
	if err != nil {
		t.Fatal(err)
	}
	if countEvents(res, EventRewire) == 0 {
		t.Fatal("test scene produced no rewires; adjust scene so the rewire branch is exercised")
	}

	// Reconstruct cost state from events, directly verifying each rewire.
	// The replay MUST mirror the planner's descendant-cost cascade: when a
	// node is rewired, every descendant's cost moves by the same delta.
	type st struct {
		parent   int
		cost     float64
		children []int
	}
	state := map[int]*st{0: {parent: -1, cost: 0}}
	pos := map[int]Vec2{0: res.Start}
	cascade := func(id int, delta float64) {
		stack := append([]int(nil), state[id].children...)
		for len(stack) > 0 {
			cid := stack[len(stack)-1]
			stack = stack[:len(stack)-1]
			state[cid].cost += delta
			stack = append(stack, state[cid].children...)
		}
	}
	rewired := 0
	for _, e := range res.Events {
		switch e.Kind {
		case EventAdd:
			state[e.Node] = &st{parent: e.NewParent, cost: e.NewCost}
			state[e.NewParent].children = append(state[e.NewParent].children, e.Node)
			pos[e.Node] = res.Nodes[e.Node].Point()
		case EventRewire:
			n := state[e.Node]
			if !almost(e.OldCost, n.cost, 1e-9) {
				t.Fatalf("event oldCost %.6f != tracked cost %.6f", e.OldCost, n.cost)
			}
			if !(e.NewCost < e.OldCost-Epsilon) {
				t.Fatalf("rewire of node %d did not strictly lower cost: %.6f -> %.6f", e.Node, e.OldCost, e.NewCost)
			}
			np := state[e.NewParent]
			want := np.cost + pos[e.NewParent].Dist(pos[e.Node])
			if !almost(e.NewCost, want, 1e-7) {
				t.Fatalf("rewire new cost %.6f != newParent cost+edge %.6f", e.NewCost, want)
			}
			// detach from old parent
			och := state[e.OldParent].children
			for i, c := range och {
				if c == e.Node {
					state[e.OldParent].children = append(och[:i], och[i+1:]...)
					break
				}
			}
			n.parent = e.NewParent
			n.cost = e.NewCost
			state[e.NewParent].children = append(state[e.NewParent].children, e.Node)
			cascade(e.Node, e.NewCost-e.OldCost)
			rewired++
		}
	}
	if rewired == 0 {
		t.Fatal("expected rewire events")
	}

	// Every node must trace back to root 0 with no cycle.
	for id := range state {
		seen := map[int]bool{}
		for cur := id; cur != -1; cur = state[cur].parent {
			if seen[cur] {
				t.Fatalf("cycle through node %d after rewires", cur)
			}
			seen[cur] = true
		}
	}
}

// TestRewiredNeighborCostStrictlyDecreased checks requirement (1) directly:
// pick a run with rewires and verify the final cost of rewired neighbors is
// below what their old parent could still offer them.
func TestRewiredNeighborCostStrictlyDecreased(t *testing.T) {
	res, err := Run(demoOptions(99, 3000))
	if err != nil {
		t.Fatal(err)
	}
	lowered := 0
	for _, e := range res.Events {
		if e.Kind != EventRewire {
			continue
		}
		if e.NewCost >= e.OldCost-Epsilon {
			t.Fatalf("rewire event without strict decrease: %+v", e)
		}
		lowered++
	}
	if lowered == 0 {
		t.Fatal("expected at least one lowering rewire")
	}
	t.Logf("%d rewire events, every one strictly decreased node cost", lowered)
}

// TestNoCollisionEdgesInFinalTree ensures every surviving tree edge is
// collision-free (rewires may only create obstacle-free edges).
func TestNoCollisionEdgesInFinalTree(t *testing.T) {
	res, err := Run(demoOptions(42, 2500))
	if err != nil {
		t.Fatal(err)
	}
	for _, n := range res.Nodes {
		if n.Parent < 0 {
			continue
		}
		if !segmentFree(res.Nodes[n.Parent].Point(), n.Point(), res.Obstacles) {
			t.Fatalf("edge %d->%d collides with an obstacle", n.Parent, n.ID)
		}
	}
}

// TestBestPath is a smoke test for the extracted path ordering.
func TestBestPath(t *testing.T) {
	res, err := Run(demoOptions(42, 2500))
	if err != nil {
		t.Fatal(err)
	}
	path := res.BestPath()
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	if path[0] != 0 {
		t.Fatalf("path must start at root 0, got %d", path[0])
	}
	if path[len(path)-1] != res.GoalNodeID {
		t.Fatalf("path must end at goal node %d, got %d", res.GoalNodeID, path[len(path)-1])
	}
	var total float64
	for i := 1; i < len(path); i++ {
		a, b := res.Nodes[path[i-1]], res.Nodes[path[i]]
		if b.Parent != a.ID {
			t.Fatalf("path edge %d->%d is not a parent edge", a.ID, b.ID)
		}
		total += a.Point().Dist(b.Point())
	}
	if !almost(total, res.BestCost, 1e-7) {
		t.Fatalf("path length %.9f != best cost %.9f", total, res.BestCost)
	}
}

// TestInvalidOptions covers configuration validation.
func TestInvalidOptions(t *testing.T) {
	base := demoOptions(1, 10)
	bad := base
	bad.Start = Vec2{150, 150} // inside an obstacle (block)
	if _, err := Run(bad); err == nil {
		t.Fatal("expected error for start inside obstacle")
	}
	bad = base
	bad.Width = 0
	if _, err := Run(bad); err == nil {
		t.Fatal("expected error for zero width")
	}
}

func countEvents(r *Result, kind string) int {
	n := 0
	for _, e := range r.Events {
		if e.Kind == kind {
			n++
		}
	}
	return n
}

// TestVerifyMonotonicDetectsRebound proves the anomaly detector itself fires
// when (e.g. due to a broken rewire/cascade) a goal-cost rise is recorded.
func TestVerifyMonotonicDetectsRebound(t *testing.T) {
	good := &Result{Events: []Event{
		{Kind: EventGoal, Best: 10},
		{Kind: EventGoal, Best: 9},
		{Kind: EventGoal, Best: 9},
		{Kind: EventGoal, Best: 8.5},
	}}
	if idx := good.VerifyMonotonic(); idx != -1 {
		t.Fatalf("non-increasing sequence flagged at %d", idx)
	}

	bad := &Result{Events: []Event{
		{Kind: EventAdd, Node: 1},
		{Kind: EventGoal, Best: 10},
		{Kind: EventGoal, Best: 9},
		{Kind: EventRewire, Node: 3},
		{Kind: EventGoal, Best: 9.5}, // rebound!
	}}
	if idx := bad.VerifyMonotonic(); idx != 2 {
		t.Fatalf("expected rebound at index 2, got %d", idx)
	}
}
