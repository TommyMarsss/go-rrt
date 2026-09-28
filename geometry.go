// Package rrtstar implements the RRT* (Rapidly-exploring Random Tree Star)
// sampling-based path planner in a 2-D workspace with polygonal obstacles.
//
// It depends only on the Go standard library and can emit a single,
// self-contained HTML file that replays tree growth and rewiring events.
package rrtstar

import "math"

// Epsilon is the single tolerance used for every geometric degeneracy test
// (collinearity, point-on-segment, "exactly on the boundary"). All boundary
// predicates share it so that the same configuration always yields the same
// answer, regardless of call order.
const Epsilon = 1e-9

// Vec2 is a 2-D point / vector.
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Sub returns a-b.
func (a Vec2) Sub(b Vec2) Vec2 { return Vec2{a.X - b.X, a.Y - b.Y} }

// Add returns a+b.
func (a Vec2) Add(b Vec2) Vec2 { return Vec2{a.X + b.X, a.Y + b.Y} }

// Scale returns a*s.
func (a Vec2) Scale(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }

// Dot returns the dot product a·b.
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }

// Cross returns the 2-D scalar cross product a×b.
func (a Vec2) Cross(b Vec2) float64 { return a.X*b.Y - a.Y*b.X }

// Norm returns the Euclidean length.
func (a Vec2) Norm() float64 { return math.Hypot(a.X, a.Y) }

// Dist returns the Euclidean distance between a and b.
func (a Vec2) Dist(b Vec2) float64 { return math.Hypot(a.X-b.X, a.Y-b.Y) }

// Polygon is an obstacle described by a simple polygon (CCW or CW, convex or
// concave). The interior together WITH its boundary is forbidden space:
// a configuration that lies exactly on an edge/vertex is in collision.
// This closed-set convention is the one consistent rule applied everywhere
// (endpoints on edges, tangential contact, collinear overlap).
type Polygon struct {
	Name     string `json:"name,omitempty"`
	Vertices []Vec2 `json:"vertices"`
}

// Rect returns an axis-aligned rectangular polygon with opposite corners
// (x1,y1) and (x2,y2).
func Rect(name string, x1, y1, x2, y2 float64) Polygon {
	return Polygon{
		Name: name,
		Vertices: []Vec2{
			{X: x1, Y: y1},
			{X: x2, Y: y1},
			{X: x2, Y: y2},
			{X: x1, Y: y2},
		},
	}
}

func signOrZero(v float64) float64 {
	switch {
	case v > Epsilon:
		return 1
	case v < -Epsilon:
		return -1
	default:
		return 0
	}
}

// crossOri returns (b-a)×(c-a).
func crossOri(a, b, c Vec2) float64 {
	return b.Sub(a).Cross(c.Sub(a))
}

// pointOnSegment reports whether p lies on the closed segment a-b, i.e.
// collinear within Epsilon AND within the axis-aligned bounding box of a-b.
func pointOnSegment(p, a, b Vec2) bool {
	if math.Abs(crossOri(a, b, p)) > Epsilon {
		return false
	}
	return p.X >= math.Min(a.X, b.X)-Epsilon &&
		p.X <= math.Max(a.X, b.X)+Epsilon &&
		p.Y >= math.Min(a.Y, b.Y)-Epsilon &&
		p.Y <= math.Max(a.Y, b.Y)+Epsilon
}

// segmentsIntersect reports whether the CLOSED segments ab and cd share at
// least one point. Endpoint contact, tangency at a vertex and collinear
// overlap all count as an intersection (boundary is never "free space").
func segmentsIntersect(a, b, c, d Vec2) bool {
	o1 := signOrZero(crossOri(a, b, c))
	o2 := signOrZero(crossOri(a, b, d))
	o3 := signOrZero(crossOri(c, d, a))
	o4 := signOrZero(crossOri(c, d, b))

	// Proper crossing (strictly opposite sides).
	if o1 != 0 && o2 != 0 && o1 == -o2 && o3 != 0 && o4 != 0 && o3 == -o4 {
		return true
	}
	// Every degenerate contact case reduces to "some endpoint lies on the
	// other segment": endpoint-on-edge, vertex-to-vertex tangency, or
	// collinear (partial or full) overlap.
	return pointOnSegment(a, c, d) || pointOnSegment(b, c, d) ||
		pointOnSegment(c, a, b) || pointOnSegment(d, a, b)
}

// onPolygonBoundary reports whether p lies on any polygon edge.
func (p Polygon) onBoundary(q Vec2) bool {
	n := len(p.Vertices)
	for i := 0; i < n; i++ {
		a := p.Vertices[i]
		b := p.Vertices[(i+1)%n]
		if pointOnSegment(q, a, b) {
			return true
		}
	}
	return false
}

// Contains reports whether q is inside the closed polygon. The boundary is
// treated as inside (= collision) by contract.
func (p Polygon) Contains(q Vec2) bool {
	if p.onBoundary(q) {
		return true
	}
	// Ray casting (horizontal ray to +x); boundary points were handled above.
	inside := false
	n := len(p.Vertices)
	for i, j := 0, n-1; i < n; j, i = i, i+1 {
		a, b := p.Vertices[j], p.Vertices[i]
		if (a.Y > q.Y) != (b.Y > q.Y) {
			xIntersect := a.X + (b.X-a.X)*(q.Y-a.Y)/(b.Y-a.Y)
			if xIntersect > q.X+Epsilon {
				inside = !inside
			}
		}
	}
	return inside
}

// SegmentHits reports whether the closed segment [a,b] intersects the closed
// obstacle region: endpoint on/in the polygon, edge crossing, vertex tangency
// or collinear overlap all return true. The predicate is deterministic and
// symmetric (SegmentHits(a,b) == SegmentHits(b,a)).
func (p Polygon) SegmentHits(a, b Vec2) bool {
	// Endpoint inside or on the boundary.
	if p.Contains(a) || p.Contains(b) {
		return true
	}
	n := len(p.Vertices)
	for i := 0; i < n; i++ {
		c := p.Vertices[i]
		d := p.Vertices[(i+1)%n]
		if segmentsIntersect(a, b, c, d) {
			return true
		}
	}
	return false
}

// SegmentFree reports whether segment [a,b] avoids every obstacle.
func segmentFree(a, b Vec2, obstacles []Polygon) bool {
	for _, o := range obstacles {
		if o.SegmentHits(a, b) {
			return false
		}
	}
	return true
}

// PointFree reports whether q lies outside every obstacle.
func pointFree(q Vec2, obstacles []Polygon) bool {
	for _, o := range obstacles {
		if o.Contains(q) {
			return false
		}
	}
	return true
}
