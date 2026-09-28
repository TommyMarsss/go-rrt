package rrtstar

import (
	"math"
	"math/rand"
	"testing"
)

func squareObstacle() Polygon { return Rect("sq", 100, 100, 200, 200) }

func TestPointBoundaryIsCollision(t *testing.T) {
	obs := []Polygon{squareObstacle()}
	cases := []struct {
		name string
		p    Vec2
		want bool // expected collision
	}{
		{"corner", Vec2{100, 100}, true},
		{"midpoint of left edge", Vec2{100, 150}, true},
		{"midpoint of right edge", Vec2{200, 150}, true},
		{"midpoint of top edge", Vec2{150, 200}, true},
		{"midpoint of bottom edge", Vec2{150, 100}, true},
		{"strictly inside", Vec2{150, 150}, true},
		{"just outside", Vec2{99, 150}, false},
		{"outside corner diagonal", Vec2{50, 50}, false},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := !pointFree(c.p, obs)
			if got != c.want {
				t.Fatalf("pointFree(%v) = %v, want collision=%v", c.p, !got, c.want)
			}
			// Contains and PointFree must agree on boundary points.
			if obs[0].Contains(c.p) != c.want {
				t.Fatalf("Contains(%v) = %v, want %v (boundary rule inconsistent)", c.p, !c.want, c.want)
			}
		})
	}
}

func TestSegmentBoundaryCases(t *testing.T) {
	o := squareObstacle()
	cases := []struct {
		name string
		a, b Vec2
		want bool
	}{
		// Endpoint lands exactly on an edge.
		{"endpoint on left edge", Vec2{50, 150}, Vec2{100, 150}, true},
		{"endpoint on corner", Vec2{100, 50}, Vec2{100, 100}, true},
		// Segment grazes a single vertex and stays outside on both sides.
		{"tangent at corner", Vec2{50, 150}, Vec2{150, 50}, true},
		// Collinear partial overlap with the bottom edge y=100.
		{"collinear overlap", Vec2{0, 100}, Vec2{150, 100}, true},
		// Collinear, disjoint from the edge, same supporting line: no contact.
		{"collinear disjoint", Vec2{0, 100}, Vec2{99, 100}, false},
		// Proper crossing.
		{"proper crossing", Vec2{50, 150}, Vec2{250, 150}, true},
		// Fully outside.
		{"clear above", Vec2{0, 250}, Vec2{300, 250}, false},
		{"clear diagonal", Vec2{0, 0}, Vec2{90, 90}, false},
		// Endpoint strictly inside.
		{"endpoint inside", Vec2{150, 150}, Vec2{300, 300}, true},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := o.SegmentHits(c.a, c.b)
			if got != c.want {
				t.Fatalf("SegmentHits(%v,%v) = %v, want %v", c.a, c.b, got, c.want)
			}
		})
	}
}

// TestSegmentCollisionSymmetricAndStable pins down the consistency rule:
// swapping endpoints or repeating the query in different orders must never
// change the answer.
func TestSegmentCollisionSymmetricAndStable(t *testing.T) {
	obs := []Polygon{squareObstacle()}
	rng := rand.New(rand.NewSource(1))
	for i := 0; i < 5000; i++ {
		a := Vec2{X: rng.Float64() * 300, Y: rng.Float64() * 300}
		b := Vec2{X: rng.Float64() * 300, Y: rng.Float64() * 300}
		fwd := segmentFree(a, b, obs)
		rev := segmentFree(b, a, obs)
		if fwd != rev {
			t.Fatalf("asymmetric collision result for %v,%v: fwd=%v rev=%v", a, b, fwd, rev)
		}
		// Repeated evaluation must be identical (no order-dependent flips).
		for k := 0; k < 4; k++ {
			if segmentFree(a, b, obs) != fwd {
				t.Fatalf("inconsistent repeated collision result for %v,%v", a, b)
			}
		}
	}
}

// TestBoundaryExactValues checks exact-coordinate boundary configurations
// (the ones vulnerable to sign-classification flips).
func TestBoundaryExactValues(t *testing.T) {
	o := squareObstacle()
	// Horizontal segment ending exactly at x=100 along y=150 (edge interior).
	if !o.SegmentHits(Vec2{100, 150}, Vec2{100, 150}) {
		t.Fatal("zero-length segment exactly on boundary must collide")
	}
	// A point closer than Epsilon is BY CONTRACT treated as on the
	// boundary; use a gap clearly larger than Epsilon to test "free".
	if o.SegmentHits(Vec2{0, 150}, Vec2{99.999, 150}) {
		t.Fatal("segment approaching but not reaching edge (gap > Epsilon) must be free")
	}
	// And the tolerance contract itself: a point within Epsilon is boundary.
	if !o.SegmentHits(Vec2{0, 150}, Vec2{100 - Epsilon/2, 150}) {
		t.Fatal("segment endpoint within Epsilon of the edge must count as boundary contact")
	}
	// Segment whose supporting line coincides with an edge but whose closed
	// span only touches the edge endpoint exactly.
	if !o.SegmentHits(Vec2{0, 100}, Vec2{100, 100}) {
		t.Fatal("segment ending exactly at edge start must collide")
	}
}

func TestConcavePolygon(t *testing.T) {
	// L-shaped concave polygon; point in the "notch" must be free.
	p := Polygon{Vertices: []Vec2{
		{0, 0}, {100, 0}, {100, 40}, {40, 40}, {40, 100}, {0, 100},
	}}
	if !p.Contains(Vec2{20, 20}) {
		t.Fatal("inside L-arm should contain")
	}
	if p.Contains(Vec2{80, 80}) {
		t.Fatal("notch of L must be outside")
	}
	if !p.onBoundary(Vec2{40, 60}) {
		t.Fatal("notch edge is boundary")
	}
	if !p.SegmentHits(Vec2{20, 60}, Vec2{80, 60}) {
		t.Fatal("segment crossing notch wall must hit")
	}
	if p.SegmentHits(Vec2{60, 60}, Vec2{90, 90}) {
		t.Fatal("segment entirely in notch must be free")
	}
}

func almost(a, b, tol float64) bool { return math.Abs(a-b) <= tol }
