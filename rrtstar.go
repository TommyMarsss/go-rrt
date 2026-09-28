package rrtstar

import (
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"math/rand"
)

// ChangeKind 标记一次父子关系变更的类型。
type ChangeKind string

const (
	// ChangeAdd 新节点作为子节点加入树。
	ChangeAdd ChangeKind = "add"
	// ChangeRewire 已有节点被改接到更小代价的父节点（重布线事件）。
	ChangeRewire ChangeKind = "rewire"
)

// ParentChange 描述一次父子关系（及代价）变化，是回放与正确性校验的最小事件单元。
type ParentChange struct {
	Kind      ChangeKind `json:"k"`
	Node      int        `json:"n"`
	OldParent int        `json:"op"` // 新增节点时为 -1
	NewParent int        `json:"np"`
	OldCost   float64    `json:"oc"` // 新增节点时为 0
	NewCost   float64    `json:"nc"`
}

// Frame 记录一次采样迭代结束后的全部结构变化。JS 回放器按顺序累计
// ParentChange 即可重建任意迭代时刻的树。
type Frame struct {
	Iter     int            `json:"it"`
	Sample   [2]float64     `json:"s"`  // 本次采样点（被拒绝的采样也会记录）
	Radius   float64        `json:"r"`  // 本次使用的邻域半径
	NewNode  int            `json:"nn"` // 新加入节点 id；采样被拒绝时为 -1
	Rejected bool           `json:"rj"` // 边碰撞 / 重复点导致本次未插入
	Changes  []ParentChange `json:"ch"` // 1 次新增 + 0..n 次重布线
	BestCost float64        `json:"bc"` // 迭代结束时的最优路径代价（不可达为 +Inf）
	GoalNode int            `json:"gn"` // 迭代结束时的目标节点 id（未到达为 -1）
}

// Node 是树节点。Parent 为 -1 表示根（起点）。
type Node struct {
	ID       int
	X, Y     float64
	Parent   int
	Cost     float64
	Children []int
	BornIter int // 该节点加入树时的迭代号，回放生长过程用
}

// Pos 返回节点坐标。
func (n *Node) Pos() Vec2 { return Vec2{n.X, n.Y} }

// Regression 描述一次最优代价回升（理论上正确的 RRT* 不应发生）。
type Regression struct {
	Iteration int     `json:"iter"`
	Prev      float64 `json:"prev"`
	Cur       float64 `json:"cur"`
}

// Config 是 RRT* 的规划参数。
type Config struct {
	Bounds     Rect    // 采样空间（闭区间），通常为 [0,W]×[0,H]
	Obstacles  []Rect  // 轴对齐障碍物（闭集，边界阻挡）
	Start      Vec2    // 起点（树根）
	Goal       Vec2    // 目标点
	GoalRadius float64 // 距目标小于该半径的节点视为到达目标；0 表示仅精确到达
	StepSize   float64 // 单步最大延伸长度
	GoalBias   float64 // 采样直接取目标点的概率，通常 0.05~0.15
	// RadiusGamma 控制邻域收缩：r = clamp(Gamma*sqrt(ln(n)/n), RadiusMin, RadiusMax)。
	RadiusGamma float64
	RadiusMin   float64
	RadiusMax   float64
	MaxIter     int     // 采样次数上限
	Seed        int64   // 随机种子（固定 => 结果可复现）
	Eps         float64 // 代价严格改进判据容差；<=0 时取默认 1e-9
}

// NodeExport 是节点的序列化形态（回放只需要坐标与出生迭代）。
type NodeExport struct {
	ID int     `json:"id"`
	X  float64 `json:"x"`
	Y  float64 `json:"y"`
	T  int     `json:"t"`
}

// Result 是一次完整规划的输出。
type Result struct {
	NodeJSON    []NodeExport `json:"nodes"`
	Frames      []Frame      `json:"frames"`
	Obstacles   []Rect       `json:"obstacles"`
	Bounds      Rect         `json:"bounds"`
	Start       Vec2         `json:"start"`
	Goal        Vec2         `json:"goal"`
	GoalRadius  float64      `json:"goalRadius"`
	StepSize    float64      `json:"stepSize"`
	Reached     bool         `json:"reached"`
	GoalNode    int          `json:"goalNode"`
	BestCost    float64      `json:"bestCost"`
	PathJSON    [][2]float64 `json:"path"`
	Regressions []Regression `json:"regressions"` // 规划过程中检测到的代价回升
	Seed        int64        `json:"seed"`
	MaxIter     int          `json:"maxIter"`
}

const defaultEps = 1e-9

// Validate 检查配置合法性。
func (c *Config) Validate() error {
	if !c.Bounds.Valid() {
		return errors.New("rrtstar: bounds 非法（Min 必须不大于 Max）")
	}
	for _, o := range c.Obstacles {
		if !o.Valid() {
			return errors.New("rrtstar: 存在非法障碍物矩形")
		}
		if !c.Bounds.Contains(o.Min) || !c.Bounds.Contains(o.Max) {
			return errors.New("rrtstar: 障碍物越出 Bounds")
		}
	}
	if !c.Bounds.Contains(c.Start) {
		return errors.New("rrtstar: 起点不在 Bounds 内")
	}
	if !c.Bounds.Contains(c.Goal) {
		return errors.New("rrtstar: 目标不在 Bounds 内")
	}
	if PointBlocked(c.Obstacles, c.Start) {
		return errors.New("rrtstar: 起点落在障碍物上（障碍物为闭集，边界同样阻挡）")
	}
	if PointBlocked(c.Obstacles, c.Goal) {
		return errors.New("rrtstar: 目标落在障碍物上（障碍物为闭集，边界同样阻挡）")
	}
	if c.StepSize <= 0 {
		return errors.New("rrtstar: StepSize 必须为正")
	}
	if c.MaxIter <= 0 {
		return errors.New("rrtstar: MaxIter 必须为正")
	}
	if c.GoalBias < 0 || c.GoalBias > 1 {
		return errors.New("rrtstar: GoalBias 必须在 [0,1]")
	}
	if c.RadiusGamma <= 0 || c.RadiusMin <= 0 || c.RadiusMax < c.RadiusMin {
		return errors.New("rrtstar: 邻域半径参数非法")
	}
	if c.GoalRadius < 0 {
		return errors.New("rrtstar: GoalRadius 不能为负")
	}
	return nil
}

// Planner 是增量式 RRT* 规划器（可逐迭代运行，便于测试与观察）。
type Planner struct {
	cfg         Config
	rng         *rand.Rand
	nodes       []*Node
	goalNode    int
	eps         float64
	Frames      []Frame
	Regressions []Regression
	prevBest    float64
}

// NewPlanner 构造规划器并放入根节点（起点）。
func NewPlanner(cfg Config) (*Planner, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	if cfg.Eps <= 0 {
		cfg.Eps = defaultEps
	}
	p := &Planner{
		cfg:      cfg,
		rng:      rand.New(rand.NewSource(cfg.Seed)),
		goalNode: -1,
		eps:      cfg.Eps,
		prevBest: math.Inf(1),
	}
	p.nodes = []*Node{{ID: 0, X: cfg.Start.X, Y: cfg.Start.Y, Parent: -1, Cost: 0, BornIter: 0}}
	return p, nil
}

// Nodes 返回当前所有节点（按 id 有序）。
func (p *Planner) Nodes() []*Node { return p.nodes }

// GoalNode 返回当前作为目标的节点 id，未到达为 -1。
func (p *Planner) GoalNode() int { return p.goalNode }

// BestCost 返回当前最优路径代价，未到达为 +Inf。
func (p *Planner) BestCost() float64 {
	if p.goalNode < 0 {
		return math.Inf(1)
	}
	return p.nodes[p.goalNode].Cost
}

func (p *Planner) sample() Vec2 {
	if p.rng.Float64() < p.cfg.GoalBias {
		return p.cfg.Goal
	}
	return Vec2{
		X: p.cfg.Bounds.Min.X + p.rng.Float64()*(p.cfg.Bounds.Max.X-p.cfg.Bounds.Min.X),
		Y: p.cfg.Bounds.Min.Y + p.rng.Float64()*(p.cfg.Bounds.Max.Y-p.cfg.Bounds.Min.Y),
	}
}

// nearest 返回距 q 最近的节点 id（节点非空，至少有根）。
// 距离相等时保留 id 较小者，保证固定种子下结果可复现。
func (p *Planner) nearest(q Vec2) int {
	best, bestD2 := 0, p.nodes[0].Pos().Distance2(q)
	for i := 1; i < len(p.nodes); i++ {
		if d2 := p.nodes[i].Pos().Distance2(q); d2 < bestD2 {
			bestD2, best = d2, i
		}
	}
	return best
}

func (p *Planner) steer(from, to Vec2) Vec2 {
	d := from.Distance(to)
	if d <= p.cfg.StepSize {
		return to
	}
	return from.Add(to.Sub(from).Scale(p.cfg.StepSize / d))
}

// radius 返回当前节点数 n 下的邻域半径（Karaman–Frazzoli 式对数收缩，带夹取）。
func (p *Planner) radius() float64 {
	n := float64(len(p.nodes))
	r := p.cfg.RadiusGamma * math.Sqrt(math.Log(n)/n)
	return clamp(r, p.cfg.RadiusMin, p.cfg.RadiusMax)
}

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// neighbors 返回所有距 q 不超过 r 的已有节点 id（按 id 升序）。
func (p *Planner) neighbors(q Vec2, r float64) []int {
	r2 := r * r
	var out []int
	for i, n := range p.nodes {
		if n.Pos().Distance2(q) <= r2 {
			out = append(out, i)
		}
	}
	return out
}

// rewireNode 把 node 改接到 newParent，并把代价差 delta 沿以 node 为根的整棵
// 子树传播——这是“反向重布线”最容易出错的一步：只改被重接节点自身的代价
// 而不更新后代，会让后代 cost 与其真实路径代价脱节，进而污染后续选父判据。
//
// newParent 是本轮刚插入的新节点，不可能是任何已有节点的后代，因此不会成环。
func (p *Planner) rewireNode(node, newParent int) ParentChange {
	n := p.nodes[node]
	oldParent := n.Parent
	oldCost := n.Cost
	newCost := p.nodes[newParent].Cost + n.Pos().Distance(p.nodes[newParent].Pos())
	delta := newCost - oldCost // 调用方保证 delta < -eps

	// 1) 从旧父节点的孩子列表摘除。
	kids := p.nodes[oldParent].Children
	for i, c := range kids {
		if c == node {
			p.nodes[oldParent].Children = append(kids[:i], kids[i+1:]...)
			break
		}
	}
	// 2) 挂到新父节点下。
	n.Parent = newParent
	p.nodes[newParent].Children = append(p.nodes[newParent].Children, node)
	// 3) 代价变化沿整棵子树传播（节点自身先更新）。
	stack := []int{node}
	for len(stack) > 0 {
		top := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		p.nodes[top].Cost += delta
		stack = append(stack, p.nodes[top].Children...)
	}
	return ParentChange{
		Kind:      ChangeRewire,
		Node:      node,
		OldParent: oldParent,
		NewParent: newParent,
		OldCost:   oldCost,
		NewCost:   newCost,
	}
}

// Step 执行一次采样迭代，返回该迭代的事件帧。
func (p *Planner) Step() Frame {
	iter := len(p.Frames) + 1
	qRand := p.sample()
	r := p.radius()
	frame := Frame{Iter: iter, Sample: qRand.Arr(), Radius: r, NewNode: -1, BestCost: p.BestCost()}

	nearID := p.nearest(qRand)
	qNew := p.steer(p.nodes[nearID].Pos(), qRand)

	// 重复采样点（零长度边）或落点进入障碍物（含边界）：拒绝本次采样。
	if qNew.Distance(p.nodes[nearID].Pos()) <= p.eps {
		frame.Rejected = true
	} else if PointBlocked(p.cfg.Obstacles, qNew) {
		frame.Rejected = true
	} else if SegmentBlocked(p.cfg.Obstacles, p.nodes[nearID].Pos(), qNew) {
		frame.Rejected = true
	}

	if !frame.Rejected {
		// ---- 第一阶段：在邻域内为新节点选代价最小且无碰撞的父节点 ----
		nb := p.neighbors(qNew, r)
		bestParent := nearID
		bestCost := p.nodes[nearID].Cost + qNew.Distance(p.nodes[nearID].Pos())
		for _, x := range nb {
			if x == nearID {
				continue
			}
			if !SegmentBlocked(p.cfg.Obstacles, p.nodes[x].Pos(), qNew) {
				c := p.nodes[x].Cost + qNew.Distance(p.nodes[x].Pos())
				if c < bestCost-p.eps { // 严格改进才换父，平局保留先出现者（id 较小）
					bestCost, bestParent = c, x
				}
			}
		}

		newID := len(p.nodes)
		n := &Node{
			ID:       newID,
			X:        qNew.X,
			Y:        qNew.Y,
			Parent:   bestParent,
			Cost:     bestCost,
			BornIter: iter,
		}
		p.nodes = append(p.nodes, n)
		p.nodes[bestParent].Children = append(p.nodes[bestParent].Children, newID)
		frame.Changes = append(frame.Changes, ParentChange{
			Kind: ChangeAdd, Node: newID, OldParent: -1, NewParent: bestParent,
			OldCost: 0, NewCost: bestCost,
		})
		frame.NewNode = newID

		// ---- 第二阶段：反向重布线 ----
		// 检查邻域内每个已有节点 x：若经由新节点到达 x 的代价严格更小，
		// 且新边无碰撞，则把 x 改接到新节点。注意判据方向是
		//   cost(new) + |new-x|  <  cost(x)
		// 即“新节点能否让 x 获益”，而不是反过来。
		for _, x := range nb {
			cand := bestCost + qNew.Distance(p.nodes[x].Pos())
			if cand < p.nodes[x].Cost-p.eps &&
				!SegmentBlocked(p.cfg.Obstacles, qNew, p.nodes[x].Pos()) {
				ch := p.rewireNode(x, newID)
				frame.Changes = append(frame.Changes, ch)
			}
		}

		// ---- 目标更新：新节点位置进入目标邻域即候选；取代价更小者 ----
		if d := qNew.Distance(p.cfg.Goal); d <= p.cfg.GoalRadius+collideEps {
			if p.goalNode < 0 || bestCost < p.nodes[p.goalNode].Cost-p.eps {
				p.goalNode = newID
			}
		}
	}

	// ---- 渐近最优性监控：最优代价只允许下降，不允许回升 ----
	best := p.BestCost()
	if reg, ok := checkRegression(p.prevBest, best, iter, p.eps); ok {
		p.Regressions = append(p.Regressions, reg)
	}
	if !math.IsInf(best, 0) {
		p.prevBest = best
	}
	frame.BestCost = best
	frame.GoalNode = p.goalNode
	p.Frames = append(p.Frames, frame)
	return frame
}

// checkRegression 是代价回升检测的纯函数，便于独立测试：
// prev/cur 任一为 +Inf（尚未到达）时无历史可比，不报告；
// cur > prev+eps 才报告一次回升；下降、持平、容差内抖动均不报告。
func checkRegression(prev, cur float64, iter int, eps float64) (Regression, bool) {
	if math.IsInf(prev, 0) || math.IsInf(cur, 0) || !(cur > prev+eps) {
		return Regression{}, false
	}
	return Regression{Iteration: iter, Prev: prev, Cur: cur}, true
}

// Run 执行最多 MaxIter 次迭代并汇总结果。
func Run(cfg Config) (*Result, error) {
	p, err := NewPlanner(cfg)
	if err != nil {
		return nil, err
	}
	for i := 0; i < cfg.MaxIter; i++ {
		p.Step()
	}
	return p.Result(), nil
}

// Path 回溯 goalNode 到根的路径（含根），未到达返回 nil。
func (p *Planner) Path() []Vec2 {
	if p.goalNode < 0 {
		return nil
	}
	var path []Vec2
	for id := p.goalNode; id >= 0; id = p.nodes[id].Parent {
		path = append(path, p.nodes[id].Pos())
	}
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	// 目标半径语义下，终点节点可能与目标点略有偏差：可视化路径补到精确目标点。
	if last := path[len(path)-1]; last.Distance(p.cfg.Goal) > p.eps {
		path = append(path, p.cfg.Goal)
	}
	return path
}

// Result 汇总当前规划状态。
func (p *Planner) Result() *Result {
	r := &Result{
		Frames:      p.Frames,
		Obstacles:   p.cfg.Obstacles,
		Bounds:      p.cfg.Bounds,
		Start:       p.cfg.Start,
		Goal:        p.cfg.Goal,
		GoalRadius:  p.cfg.GoalRadius,
		StepSize:    p.cfg.StepSize,
		Reached:     p.goalNode >= 0,
		GoalNode:    p.goalNode,
		BestCost:    p.BestCost(),
		Regressions: p.Regressions,
		Seed:        p.cfg.Seed,
		MaxIter:     p.cfg.MaxIter,
	}
	for _, n := range p.nodes {
		r.NodeJSON = append(r.NodeJSON, NodeExport{ID: n.ID, X: n.X, Y: n.Y, T: n.BornIter})
	}
	for _, v := range p.Path() {
		r.PathJSON = append(r.PathJSON, v.Arr())
	}
	return r
}

// marshalBestCost 把未到达时的 +Inf 代价编码为 null，保证输出是合法 JSON。
func marshalBestCost(c float64) []byte {
	if math.IsInf(c, 1) {
		return []byte("null")
	}
	b, _ := json.Marshal(c)
	return b
}

// MarshalJSON：未到达目标的帧 BestCost 为 +Inf，标准库无法直接编码，输出 null。
func (f Frame) MarshalJSON() ([]byte, error) {
	return []byte(fmt.Sprintf(
		`{"it":%d,"s":[%g,%g],"r":%g,"nn":%d,"rj":%t,"ch":%s,"bc":%s,"gn":%d}`,
		f.Iter, f.Sample[0], f.Sample[1], f.Radius, f.NewNode, f.Rejected,
		jsonSlice(f.Changes), marshalBestCost(f.BestCost), f.GoalNode,
	)), nil
}

// MarshalJSON：Result.BestCost 未到达时输出 null。
func (r Result) MarshalJSON() ([]byte, error) {
	return []byte(fmt.Sprintf(
		`{"nodes":%s,"frames":%s,"obstacles":%s,"bounds":%s,"start":%s,"goal":%s,`+
			`"goalRadius":%g,"stepSize":%g,"reached":%t,"goalNode":%d,"bestCost":%s,`+
			`"path":%s,"regressions":%s,"seed":%d,"maxIter":%d}`,
		jsonSlice(r.NodeJSON), jsonSlice(r.Frames), jsonSlice(r.Obstacles),
		jsonString(r.Bounds), jsonString(r.Start), jsonString(r.Goal),
		r.GoalRadius, r.StepSize, r.Reached, r.GoalNode, marshalBestCost(r.BestCost),
		jsonSlice(r.PathJSON), jsonSlice(r.Regressions), r.Seed, r.MaxIter,
	)), nil
}

// jsonString 把任意可 JSON 编码的值序列化为字符串（供手写 MarshalJSON 复用）。
func jsonString(v any) string {
	b, err := json.Marshal(v)
	if err != nil {
		return "null"
	}
	return string(b)
}

// jsonSlice 与 jsonString 相同，但 nil 切片输出 [] 而非 null。
func jsonSlice[T any](v []T) string {
	if v == nil {
		return "[]"
	}
	return jsonString(v)
}
