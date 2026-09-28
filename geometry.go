// Package rrtstar 实现二维空间中的 RRT*（Rapidly-exploring Random Tree Star）
// 渐近最优路径规划算法，仅依赖 Go 标准库。
//
// 核心设计目标：
//   - 重布线（rewire）方向严格正确：新节点先选最小代价父节点，再反向检查邻域内
//     已有节点能否经由新节点降低代价，且代价变化沿整棵子树传播；
//   - 碰撞判定边界一致：障碍物按闭集处理，端点落边、相切、过角点均判碰撞，
//     正反向、重复调用结果一致；
//   - 可验证渐近最优性：固定随机种子，逐迭代记录当前最优路径代价，并提供
//     代价回升（regression）检测器。
package rrtstar

import "math"

// collideEps 是碰撞判定的几何容差。所有“闭集”边界判断（在边上、在角点上、
// 相切）都在该容差内视为成立，保证同一几何关系不会因浮点误差得到不同结论。
const collideEps = 1e-9

// Vec2 是二维点/向量。
type Vec2 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// Add 返回 a+b。
func (a Vec2) Add(b Vec2) Vec2 { return Vec2{a.X + b.X, a.Y + b.Y} }

// Sub 返回 a-b。
func (a Vec2) Sub(b Vec2) Vec2 { return Vec2{a.X - b.X, a.Y - b.Y} }

// Scale 返回 a*s。
func (a Vec2) Scale(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }

// Dot 返回 a·b。
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }

// Distance2 返回欧氏距离的平方。
func (a Vec2) Distance2(b Vec2) float64 {
	dx, dy := a.X-b.X, a.Y-b.Y
	return dx*dx + dy*dy
}

// Distance 返回欧氏距离。
func (a Vec2) Distance(b Vec2) float64 { return math.Sqrt(a.Distance2(b)) }

// Arr 转为 [2]float64，便于序列化。
func (a Vec2) Arr() [2]float64 { return [2]float64{a.X, a.Y} }

// Rect 是轴对齐闭矩形 [Min.X, Max.X] × [Min.Y, Max.Y]。
// “闭”意味着边界本身属于障碍物（见 Contains / IntersectsSegment 的判定规则）。
type Rect struct {
	Min Vec2 `json:"min"`
	Max Vec2 `json:"max"`
}

// Width / Height 矩形尺寸。
func (r Rect) Width() float64  { return r.Max.X - r.Min.X }
func (r Rect) Height() float64 { return r.Max.Y - r.Min.Y }

// Valid 报告矩形是否合法（两点坐标有序、面积非负）。
func (r Rect) Valid() bool {
	return r.Min.X <= r.Max.X && r.Min.Y <= r.Max.Y
}

// Contains 按闭集规则判断点 p 是否在矩形内部或边界上。
// 边界（含四条边与四个角点）一律视为在障碍物内。
func (r Rect) Contains(p Vec2) bool {
	return p.X >= r.Min.X-collideEps && p.X <= r.Max.X+collideEps &&
		p.Y >= r.Min.Y-collideEps && p.Y <= r.Max.Y+collideEps
}

// IntersectsSegment 按闭集规则判断线段 ab 是否与矩形相交。
//
// 采用 Liang–Barsky 线性裁剪：把线段写成 a+t(b-a), t∈[0,1]，分别求它落在
// x、y 两个闭区间 [Min,Max] 内的 t 区间，两个区间（并上 [0,1]）交集非空即相交。
// 因此以下边界情形有完全一致的明确结论，且与参数方向（a→b 或 b→a）无关：
//   - 端点恰好落在边或角点上：t=0 或 t=1 处相切，判“相交”；
//   - 线段与某条边重合（沿边界行走）：平行且位于边界，判“相交”；
//   - 线段恰好穿过某个角点（单点相切）：交集退化为一点，判“相交”；
//   - 与边界距离超过 collideEps 的平行线段：判“不相交”。
func (r Rect) IntersectsSegment(a, b Vec2) bool {
	t0, t1 := 0.0, 1.0

	// x 方向板片 [Min.X, Max.X]
	if dx := b.X - a.X; math.Abs(dx) <= collideEps {
		// 线段平行于 y 轴：起点 x 必须落在闭区间内（容差内贴边也算）。
		if a.X < r.Min.X-collideEps || a.X > r.Max.X+collideEps {
			return false
		}
	} else {
		tEnter := (r.Min.X - a.X) / dx
		tExit := (r.Max.X - a.X) / dx
		if tEnter > tExit {
			tEnter, tExit = tExit, tEnter
		}
		t0 = math.Max(t0, tEnter)
		t1 = math.Min(t1, tExit)
		if t0 > t1+collideEps {
			return false
		}
	}

	// y 方向板片 [Min.Y, Max.Y]
	if dy := b.Y - a.Y; math.Abs(dy) <= collideEps {
		if a.Y < r.Min.Y-collideEps || a.Y > r.Max.Y+collideEps {
			return false
		}
	} else {
		tEnter := (r.Min.Y - a.Y) / dy
		tExit := (r.Max.Y - a.Y) / dy
		if tEnter > tExit {
			tEnter, tExit = tExit, tEnter
		}
		t0 = math.Max(t0, tEnter)
		t1 = math.Min(t1, tExit)
		if t0 > t1+collideEps {
			return false
		}
	}

	// 交集退化为单点（角点相切）也算相交——障碍物是闭集。
	return true
}

// PointBlocked 报告点 p 是否落在任一障碍物内部或边界上。
func PointBlocked(obstacles []Rect, p Vec2) bool {
	for i := range obstacles {
		if obstacles[i].Contains(p) {
			return true
		}
	}
	return false
}

// SegmentBlocked 报告线段 ab 是否与任一障碍物相交（闭集规则，见
// Rect.IntersectsSegment）。SegmentBlocked(o, a, b) 与 SegmentBlocked(o, b, a)
// 恒等，可重复调用且结论一致。
func SegmentBlocked(obstacles []Rect, a, b Vec2) bool {
	for i := range obstacles {
		if obstacles[i].IntersectsSegment(a, b) {
			return true
		}
	}
	return false
}
