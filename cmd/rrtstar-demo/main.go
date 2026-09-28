// Command rrtstar-demo runs an RRT* planning problem, checks the
// monotonicity of the best-path cost, and writes a self-contained static
// HTML replay (report.html by default).
//
// Usage:
//
//	rrtstar-demo [-seed N] [-samples N] [-out report.html]
package main

import (
	"flag"
	"fmt"
	"os"

	rrt "github.com/TommyMarsss/go-rrt"
)

func buildScene() (float64, float64, rrt.Vec2, rrt.Vec2, []rrt.Polygon) {
	const W, H = 1000.0, 700.0
	start := rrt.Vec2{X: 60, Y: 350}
	goal := rrt.Vec2{X: 940, Y: 350}

	obs := []rrt.Polygon{
		// Two staggered vertical walls force a detour.
		rrt.Rect("wall-top", 260, 330, 300, 620),
		rrt.Rect("wall-bottom", 430, 80, 470, 370),
		rrt.Rect("wall-top2", 600, 330, 640, 620),
		// A diagonal triangle near the corridor exit.
		{Name: "tri", Vertices: []rrt.Vec2{
			{X: 720, Y: 120}, {X: 840, Y: 120}, {X: 840, Y: 240},
		}},
		// A central block that tangency cases can be tested against.
		rrt.Rect("block", 150, 120, 210, 200),
	}
	return W, H, start, goal, obs
}

func main() {
	seed := flag.Int64("seed", 42, "fixed RNG seed")
	samples := flag.Int("samples", 2500, "max sampling iterations")
	out := flag.String("out", "report.html", "output HTML report path")
	flag.Parse()

	W, H, start, goal, obs := buildScene()

	opt := rrt.Options{
		Width: W, Height: H,
		Start:          start,
		Goal:           goal,
		Obstacles:      obs,
		StepSize:       40,
		GoalRadius:     30,
		GoalSampleRate: 0.07,
		Seed:           *seed,
		MaxSamples:     *samples,
	}

	res, err := rrt.Run(opt)
	if err != nil {
		fmt.Fprintf(os.Stderr, "planner error: %v\n", err)
		os.Exit(1)
	}

	hist := res.CostHistory()
	fmt.Printf("RRT* finished: seed=%d samples=%d nodes=%d rewires=%d\n",
		*seed, *samples, len(res.Nodes), countRewires(res))
	if !res.Reached {
		fmt.Println("goal NOT reached")
		if err := rrt.WriteHTMLReport(*out, res); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		fmt.Printf("report written to %s\n", *out)
		return
	}

	fmt.Printf("best path cost: %.4f\n", res.BestCost)
	fmt.Printf("goal-cost improvements: %d\n", len(hist))
	if bad := res.VerifyMonotonic(); bad >= 0 {
		fmt.Printf("MONOTONICITY VIOLATION at improvement #%d: %.4f -> %.4f (cost rose!)\n",
			bad, hist[bad-1], hist[bad])
		os.Exit(2)
	}
	fmt.Println("monotonicity check: PASS (best cost never increases)")
	fmt.Print("cost history:")
	for i, c := range hist {
		if i%8 == 0 {
			fmt.Println()
		}
		fmt.Printf("  %.2f", c)
	}
	fmt.Println()

	if err := rrt.WriteHTMLReport(*out, res); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	fmt.Printf("static replay report written to %s (open it in a browser)\n", *out)
}

func countRewires(r *rrt.Result) int {
	n := 0
	for _, e := range r.Events {
		if e.Kind == rrt.EventRewire {
			n++
		}
	}
	return n
}
