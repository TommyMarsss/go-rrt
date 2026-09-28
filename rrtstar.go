package rrtstar

import (
	"errors"
	"math"
	"math/rand"
	"sort"
)

// Options configures a planning run. Zero-valued numeric fields are replaced
// by documented defaults in Normalize.
type Options struct {
	Width  float64 // workspace x range [0,Width]
	Height float64 // workspace y range [0,Height]

	Start Vec2
	Goal  Vec2

	Obstacles []Polygon

	// StepSize (η-steer) is the maximum length of a single tree extension.
	StepSize float64
	// GoalRadius: a tree node within this (collision-free) distance of the
	// goal connects to it.
	GoalRadius float64
	// GoalSampleRate is the probability of sampling the goal exactly.
	GoalSampleRate float64

	// Connection radius for RRT* neighbor queries:
	//   r(n) = min(Eta, Gamma * sqrt(ln(n)/n))   (Karaman–Frazzoli, d=2)
	Gamma float64
	Eta   float64

	Seed       int64
	MaxSamples int
}

// Normalize fills defaults and validates the options.
func (o *Options) Normalize() error {
	if o.Width <= 0 || o.Height <= 0 {
		return errors.New("rrtstar: Width and Height must be positive")
	}
	if o.StepSize <= 0 {
		o.StepSize = math.Min(o.Width, o.Height) / 20
	}
	if o.GoalRadius <= 0 {
		o.GoalRadius = o.StepSize
	}
	if o.GoalSampleRate <= 0 {
		o.GoalSampleRate = 0.05
	}
	if o.Gamma <= 0 {
		// Ball-volume heuristic: 2*sqrt(A/pi) is the asymptotic scaling
		// constant for the unit-volume free-space bound.
		o.Gamma = 2 * math.Sqrt(o.Width*o.Height/math.Pi)
	}
	if o.Eta <= 0 {
		o.Eta = 4 * o.StepSize
	}
	if o.MaxSamples <= 0 {
		o.MaxSamples = 2000
	}
	if !pointFree(o.Start, o.Obstacles) {
		return errors.New("rrtstar: start lies inside or on an obstacle")
	}
	if !pointFree(o.Goal, o.Obstacles) {
		return errors.New("rrtstar: goal lies inside or on an obstacle")
	}
	if !segmentFree(o.Start, o.Goal, o.Obstacles) {
		// Not fatal: the planner simply has to navigate around obstacles.
		_ = o
	}
	return nil
}

// Node is a tree vertex. Parent is -1 for the root.
type Node struct {
	ID       int     `json:"id"`
	X        float64 `json:"x"`
	Y        float64 `json:"y"`
	Parent   int     `json:"parent"`
	Cost     float64 `json:"cost"`
	Children []int   `json:"children,omitempty"`
}

// Point returns the node position.
func (n *Node) Point() Vec2 { return Vec2{X: n.X, Y: n.Y} }

// Event kinds recorded for step-by-step replay.
const (
	// EventAdd: a node was attached to the tree. NewParent/NewCost are the
	// parent chosen after the min-cost neighbor scan.
	EventAdd = "add"
	// EventRewire: an EXISTING neighbor node got a strictly cheaper route
	// through the newly inserted node. Old/New parent and cost are recorded.
	EventRewire = "rewire"
	// EventGoal: the best goal-connected path improved (never increased).
	EventGoal = "goal"
)

// Event is one replayable planner step.
type Event struct {
	Iter      int     `json:"iter"`
	Kind      string  `json:"kind"`
	Node      int     `json:"node"`
	SampleX   float64 `json:"sx,omitempty"`
	SampleY   float64 `json:"sy,omitempty"`
	OldParent int     `json:"oldParent,omitempty"`
	NewParent int     `json:"newParent,omitempty"`
	OldCost   float64 `json:"oldCost,omitempty"`
	NewCost   float64 `json:"newCost,omitempty"`
	Best      float64 `json:"best,omitempty"` // best goal cost after this event
}

// Result is the output of a planning run. Events allow an exact, ordered
// reconstruction of every parent/cost mutation.
type Result struct {
	Options    Options   `json:"options"`
	Width      float64   `json:"width"`
	Height     float64   `json:"height"`
	Start      Vec2      `json:"start"`
	Goal       Vec2      `json:"goal"`
	Obstacles  []Polygon `json:"obstacles"`
	Nodes      []*Node   `json:"nodes"` // final tree
	Events     []Event   `json:"events"`
	BestCost   float64   `json:"bestCost"` // +Inf if the goal was never reached
	GoalNodeID int       `json:"goalNodeId"`
	Iterations int       `json:"iterations"`
	Reached    bool      `json:"reached"`
}

// BestPath returns node IDs from start to goal of the current best path, or
// nil if the goal has not been reached.
func (r *Result) BestPath() []int {
	if !r.Reached {
		return nil
	}
	path := []int{}
	for id := r.GoalNodeID; id != -1; id = r.Nodes[id].Parent {
		path = append(path, id)
		if len(path) > len(r.Nodes) {
			return nil // defensive: should never happen
		}
	}
	// reverse
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

// Planner holds mutable run state.
type Planner struct {
	opt   Options
	rng   *rand.Rand
	nodes []*Node
	// goal node is a normal tree node once first connected; -1 before that.
	goalID int
	events []Event
}

// NewPlanner constructs a validated, seeded planner.
func NewPlanner(opt Options) (*Planner, error) {
	if err := opt.Normalize(); err != nil {
		return nil, err
	}
	return &Planner{
		opt:    opt,
		rng:    rand.New(rand.NewSource(opt.Seed)),
		goalID: -1,
	}, nil
}

func (p *Planner) addNode(q Vec2, parent int, cost float64) *Node {
	id := len(p.nodes)
	n := &Node{ID: id, X: q.X, Y: q.Y, Parent: parent, Cost: cost}
	p.nodes = append(p.nodes, n)
	if parent >= 0 {
		p.nodes[parent].Children = append(p.nodes[parent].Children, id)
	}
	return n
}

// connectionRadius implements the shrinking-neighborhood radius r(n).
func (p *Planner) connectionRadius() float64 {
	n := float64(len(p.nodes))
	r := p.opt.Gamma * math.Sqrt(math.Log(n)/n)
	if r > p.opt.Eta {
		r = p.opt.Eta
	}
	return r
}

func (p *Planner) nearest(q Vec2) int {
	best, bestD := 0, math.Inf(1)
	for i, n := range p.nodes {
		if d := n.Point().Dist(q); d < bestD {
			best, bestD = i, d
		}
	}
	return best
}

// neighbors returns IDs of nodes within radius r of q.
func (p *Planner) neighbors(q Vec2, r float64) []int {
	r2 := r * r
	ids := make([]int, 0, 16)
	for i, n := range p.nodes {
		dx, dy := n.X-q.X, n.Y-q.Y
		if dx*dx+dy*dy <= r2+Epsilon {
			ids = append(ids, i)
		}
	}
	return ids
}

func (p *Planner) steer(a, b Vec2) Vec2 {
	d := a.Dist(b)
	if d <= p.opt.StepSize+Epsilon {
		return b
	}
	return a.Add(b.Sub(a).Scale(p.opt.StepSize / d))
}

// addCost propagates a cost delta through the subtree rooted at id (used
// after a successful rewire). Delta is negative by construction.
func (p *Planner) addCost(id int, delta float64) {
	stack := append([]int(nil), p.nodes[id].Children...)
	for len(stack) > 0 {
		cid := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		c := p.nodes[cid]
		c.Cost += delta
		stack = append(stack, c.Children...)
	}
}

// removeChild deletes child from n's child list.
func removeChild(n *Node, child int) {
	ch := n.Children
	for i, c := range ch {
		if c == child {
			n.Children = append(ch[:i], ch[i+1:]...)
			return
		}
	}
}

// rewireToParent detaches node id from its current parent and attaches it to
// newParent; its cost is updated and the delta cascades to descendants.
func (p *Planner) rewireToParent(id, newParent int, newCost float64) {
	n := p.nodes[id]
	oldParent := n.Parent
	delta := newCost - n.Cost // strictly negative
	if oldParent >= 0 {
		removeChild(p.nodes[oldParent], id)
	}
	n.Parent = newParent
	n.Cost = newCost
	p.nodes[newParent].Children = append(p.nodes[newParent].Children, id)
	if delta != 0 {
		p.addCost(id, delta)
	}
}

func isInf(v float64) bool { return math.IsInf(v, 1) }

// recordGoalImprovement emits a goal event when the goal node's cost is lower
// than the last recorded best.
func (p *Planner) recordGoalImprovement(iter int, lastBest *float64) {
	if p.goalID < 0 {
		return
	}
	c := p.nodes[p.goalID].Cost
	if c < *lastBest-Epsilon {
		p.events = append(p.events, Event{
			Iter: iter, Kind: EventGoal, Node: p.goalID,
			OldCost: *lastBest, NewCost: c, Best: c,
		})
		*lastBest = c
	}
}

// Run executes up to MaxSamples sampling iterations and returns the run
// record. The returned error is non-nil only for configuration problems.
func Run(opt Options) (*Result, error) {
	p, err := NewPlanner(opt)
	if err != nil {
		return nil, err
	}

	// Root.
	p.addNode(p.opt.Start, -1, 0)

	bestGoal := math.Inf(1)

	for iter := 1; iter <= p.opt.MaxSamples; iter++ {
		// --- Sample -------------------------------------------------------
		var sample Vec2
		if p.rng.Float64() < p.opt.GoalSampleRate {
			sample = p.opt.Goal
		} else {
			sample = Vec2{
				X: p.rng.Float64() * p.opt.Width,
				Y: p.rng.Float64() * p.opt.Height,
			}
		}

		// --- Extend -------------------------------------------------------
		nearID := p.nearest(sample)
		near := p.nodes[nearID]
		qNew := p.steer(near.Point(), sample)

		// Reject degenerate extensions onto existing vertices and any
		// extension that enters the closed obstacle set.
		if qNew.Dist(near.Point()) < Epsilon || !segmentFree(near.Point(), qNew, p.opt.Obstacles) {
			continue
		}
		// Steered endpoint identical to an existing node (e.g. exact goal
		// already in the tree): nothing new to insert.
		duplicate := -1
		for _, n := range p.nodes {
			if n.Point().Dist(qNew) < Epsilon {
				duplicate = n.ID
				break
			}
		}
		if duplicate >= 0 {
			continue
		}

		r := p.connectionRadius()

		// --- Choose parent: minimum-cost collision-free neighbor ---------
		bestParent := nearID
		bestCost := near.Cost + near.Point().Dist(qNew)
		for _, nid := range p.neighbors(qNew, r) {
			if nid == nearID {
				continue
			}
			cand := p.nodes[nid]
			if !segmentFree(cand.Point(), qNew, p.opt.Obstacles) {
				continue
			}
			c := cand.Cost + cand.Point().Dist(qNew)
			// Strictly better; ties keep the earlier choice.
			if c < bestCost-Epsilon {
				bestCost = c
				bestParent = nid
			}
		}

		newNode := p.addNode(qNew, bestParent, bestCost)
		p.events = append(p.events, Event{
			Iter: iter, Kind: EventAdd, Node: newNode.ID,
			SampleX: sample.X, SampleY: sample.Y,
			NewParent: bestParent, NewCost: bestCost,
		})

		// --- Reverse rewire: neighbors that get a cheaper route via new ---
		for _, nid := range p.neighbors(qNew, r) {
			if nid == bestParent || nid == newNode.ID {
				continue
			}
			cand := p.nodes[nid]
			d := qNew.Dist(cand.Point())
			newCandCost := newNode.Cost + d
			// THE REWIRING DIRECTION CHECK: the EXISTING node cand must
			// become cheaper by routing start -> ... -> newNode -> cand.
			// Comparing the other way (cand -> newNode) is the classic
			// silent bug: it "rewires" but never lowers any existing cost.
			if newCandCost >= cand.Cost-Epsilon {
				continue
			}
			if !segmentFree(qNew, cand.Point(), p.opt.Obstacles) {
				continue
			}
			oldParent, oldCost := cand.Parent, cand.Cost
			p.rewireToParent(nid, newNode.ID, newCandCost)
			p.events = append(p.events, Event{
				Iter: iter, Kind: EventRewire, Node: nid,
				OldParent: oldParent, NewParent: newNode.ID,
				OldCost: oldCost, NewCost: newCandCost,
			})
		}

		// --- Goal connection ---------------------------------------------
		// The goal is inserted as an ordinary tree node the first time a
		// node reaches it; afterwards it participates in normal rewire
		// iterations, so its cost can only decrease.
		if p.goalID < 0 && qNew.Dist(p.opt.Goal) <= p.opt.GoalRadius+Epsilon &&
			segmentFree(qNew, p.opt.Goal, p.opt.Obstacles) {
			gcost := newNode.Cost + qNew.Dist(p.opt.Goal)
			gn := p.addNode(p.opt.Goal, newNode.ID, gcost)
			p.goalID = gn.ID
			p.events = append(p.events, Event{
				Iter: iter, Kind: EventAdd, Node: gn.ID,
				NewParent: newNode.ID, NewCost: gcost,
			})
		}
		p.recordGoalImprovement(iter, &bestGoal)
	}

	res := &Result{
		Options:    p.opt,
		Width:      p.opt.Width,
		Height:     p.opt.Height,
		Start:      p.opt.Start,
		Goal:       p.opt.Goal,
		Obstacles:  p.opt.Obstacles,
		Nodes:      p.nodes,
		Events:     p.events,
		GoalNodeID: p.goalID,
		Iterations: p.opt.MaxSamples,
		BestCost:   bestGoal,
		Reached:    p.goalID >= 0,
	}
	return res, nil
}

// CostHistory returns the best goal cost AFTER each recorded goal event.
// By the RRT* invariant (rewires only lower costs) this sequence is strictly
// decreasing; any rise indicates a broken rewiring/cascade implementation.
func (r *Result) CostHistory() []float64 {
	h := make([]float64, 0)
	for _, e := range r.Events {
		if e.Kind == EventGoal {
			h = append(h, e.Best)
		}
	}
	return h
}

// VerifyMonotonic returns the first index where the goal cost rises relative
// to the previous goal event, or -1 when costs are monotonically non-increasing.
func (r *Result) VerifyMonotonic() int {
	hist := r.CostHistory()
	for i := 1; i < len(hist); i++ {
		if hist[i] > hist[i-1]+Epsilon {
			return i
		}
	}
	return -1
}

// SortedEventIDs is a small test/diagnostic helper.
func SortedEventIDs(ev []Event) []int {
	ids := make([]int, len(ev))
	for i, e := range ev {
		ids[i] = e.Node
	}
	sort.Ints(ids)
	return ids
}
