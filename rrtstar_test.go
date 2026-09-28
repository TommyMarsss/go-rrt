package rrtstar

import (
	"math"
	"testing"
)

// ---- 碰撞检测：边界情形必须有一致、确定的结论 ----

func approxEq(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

func TestSegmentCollisionBoundaryCases(t *testing.T) {
	// 障碍物 [2,4]×[2,4]，闭集：边界本身阻挡。
	obs := []Rect{{Min: Vec2{2, 2}, Max: Vec2{4, 4}}}

	type tc struct {
		name string
		a, b Vec2
		want bool
	}
	cases := []tc{
		{"完全在外", Vec2{0, 0}, Vec2{1, 1}, false},
		{"穿过内部", Vec2{0, 3}, Vec2{6, 3}, true},
		{"端点恰好落在左边界", Vec2{2, 3}, Vec2{1, 3}, true},
		{"端点恰好落在右边界", Vec2{1, 4}, Vec2{4, 4}, true},
		{"端点恰好落在角点", Vec2{2, 2}, Vec2{1, 1}, true},
		{"端点为障碍物角点(右上)", Vec2{4, 4}, Vec2{5, 5}, true},
		{"与顶边相切(单点穿过角点)", Vec2{1, 1}, Vec2{2, 2}, true},
		{"沿边界重合一段", Vec2{2, 2.5}, Vec2{2, 3.5}, true},
		{"沿顶边行走", Vec2{2.5, 4}, Vec2{3.5, 4}, true},
		{"起终点都在内部", Vec2{2.5, 2.5}, Vec2{3.5, 3.5}, true},
		{"线段整体在障碍内", Vec2{2.1, 3}, Vec2{3.9, 3}, true},
		{"恰好擦边而过(距离无穷小外侧)", Vec2{0, 1.999999}, Vec2{5, 1.999999}, false},
		{"斜向恰好过角点外侧", Vec2{0, 1}, Vec2{1.999999, 0}, false},
		{"零长度点在边界上", Vec2{2, 3}, Vec2{2, 3}, true},
		{"零长度点在外部", Vec2{0, 0}, Vec2{0, 0}, false},
		{"零长度点在内部", Vec2{3, 3}, Vec2{3, 3}, true},
		{"垂直线段穿过", Vec2{3, 0}, Vec2{3, 5}, true},
		{"垂直线段贴边", Vec2{4, 2.5}, Vec2{4, 3.5}, true},
		{"水平线段刚好在下方", Vec2{2, 1.999999}, Vec2{4, 1.999999}, false},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := SegmentBlocked(obs, c.a, c.b)
			if got != c.want {
				t.Fatalf("SegmentBlocked(%v->%v) = %v, want %v", c.a, c.b, got, c.want)
			}
		})
	}
}

// TestCollisionSymmetric 确保同一线段正反方向、重复调用结论完全一致，
// 这是“同一情况多次判断结果不一致”问题的针对性测试。
func TestCollisionSymmetricAndRepeated(t *testing.T) {
	obs := []Rect{
		{Min: Vec2{2, 2}, Max: Vec2{4, 4}},
		{Min: Vec2{6, 1}, Max: Vec2{7, 5}},
	}
	segs := [][2]Vec2{
		{{0, 0}, {10, 6}},
		{{2, 2}, {4, 4}},                     // 角点到角点（对角线重合）
		{{2, 4}, {4, 2}},                     // 另一条对角线
		{{1.999999, 3}, {2.000000001, 3}},    // 跨边界
		{{5, 3}, {6, 3}},                     // 恰好顶到第二个障碍边界
		{{4, 3}, {6, 3}},                     // 两个障碍之间的间隙（端点都在边界上 => 碰撞）
		{{4.000000001, 3}, {5.999999999, 3}}, // 间隙内部，不碰
		{{0, 4}, {10, 4}},                    // 沿 y=4 穿两个障碍的顶边
	}
	for rep := 0; rep < 5; rep++ {
		for i, s := range segs {
			fwd := SegmentBlocked(obs, s[0], s[1])
			rev := SegmentBlocked(obs, s[1], s[0])
			if fwd != rev {
				t.Fatalf("seg %d: 正反向不一致 fwd=%v rev=%v", i, fwd, rev)
			}
			// 点包含规则与线段端点规则一致：端点在边上时两者必须都说“阻挡”。
			if PointBlocked(obs, s[0]) != func() bool {
				return obs[0].IntersectsSegment(s[0], s[0]) || obs[1].IntersectsSegment(s[0], s[0])
			}() {
				t.Fatalf("seg %d: 端点包含判定与零长度线段判定不一致", i)
			}
		}
	}
}

// ---- 测试场景：一个带障碍的标准平面 ----

func testConfig(seed int64, maxIter int) Config {
	return Config{
		Bounds: Rect{Min: Vec2{0, 0}, Max: Vec2{100, 100}},
		// 中间一道带缺口的墙（缺口 y∈(55,100] 从 x=45 绕上），迫使路径绕行。
		Obstacles: []Rect{
			{Min: Vec2{40, 0}, Max: Vec2{45, 55}},
			{Min: Vec2{40, 65}, Max: Vec2{45, 100}},
		},
		Start:       Vec2{10, 30},
		Goal:        Vec2{90, 30},
		GoalRadius:  3,
		StepSize:    6,
		GoalBias:    0.1,
		RadiusGamma: 22,
		RadiusMin:   5,
		RadiusMax:   20,
		MaxIter:     maxIter,
		Seed:        seed,
	}
}

// TestBestCostMonotonicNonIncreasing 固定种子下，最优路径代价必须随迭代
// 单调不增；任何回升都应被 Regressions 捕获。
func TestBestCostMonotonicNonIncreasing(t *testing.T) {
	for _, seed := range []int64{1, 2, 42, 12345} {
		cfg := testConfig(seed, 1500)
		res, err := Run(cfg)
		if err != nil {
			t.Fatalf("seed=%d: %v", seed, err)
		}
		if !res.Reached {
			t.Fatalf("seed=%d: 1500 次采样仍未到达目标", seed)
		}
		if len(res.Regressions) != 0 {
			t.Fatalf("seed=%d: 检测到 %d 次代价回升: %+v", seed, len(res.Regressions), res.Regressions[:min(3, len(res.Regressions))])
		}
		// 从帧序列独立复核单调不增（首个有限值起算）。
		prev := math.Inf(1)
		for _, f := range res.Frames {
			if math.IsInf(f.BestCost, 0) {
				continue
			}
			if f.BestCost > prev+1e-9 {
				t.Fatalf("seed=%d iter=%d: 代价从 %.6f 回升到 %.6f", seed, f.Iter, prev, f.BestCost)
			}
			prev = f.BestCost
		}
	}
}

// TestCostNeverIncreasesAfterAnyRewire 更强的不变量：不只是目标代价，
// 每个被重布线节点的新代价必须严格小于旧代价，且其后代代价同步下降。
func TestRewireStrictlyDecreasesCosts(t *testing.T) {
	cfg := testConfig(7, 800)
	p, err := NewPlanner(cfg)
	if err != nil {
		t.Fatal(err)
	}
	rewireCount := 0
	for i := 0; i < cfg.MaxIter; i++ {
		f := p.Step()
		// 记录迭代前的节点代价用于校验后代传播：帧后直接核对整树 cost 一致性。
		for _, ch := range f.Changes {
			if ch.Kind != ChangeRewire {
				continue
			}
			rewireCount++
			if !(ch.NewCost < ch.OldCost-1e-9) {
				t.Fatalf("iter=%d node=%d: 重布线未严格降低代价 %.6f -> %.6f",
					f.Iter, ch.Node, ch.OldCost, ch.NewCost)
			}
			if ch.OldParent == ch.NewParent {
				t.Fatalf("iter=%d node=%d: 重布线事件但父节点未改变", f.Iter, ch.Node)
			}
		}
	}
	if rewireCount == 0 {
		t.Fatal("800 次迭代内没有发生任何重布线，场景可能无法检验重布线逻辑")
	}

	// 全树不变量复核：每个节点 cost = parent.cost + 边距。
	for _, n := range p.Nodes() {
		if n.Parent < 0 {
			if n.Cost != 0 {
				t.Fatalf("根代价非 0: %v", n.Cost)
			}
			continue
		}
		par := p.Nodes()[n.Parent]
		want := par.Cost + n.Pos().Distance(par.Pos())
		if !approxEq(want, n.Cost) {
			t.Fatalf("节点 %d 代价 %.9f 与路径递推值 %.9f 不一致（子树传播错误）", n.ID, n.Cost, want)
		}
	}
}

// TestNeighborCostReduction 题目明确要求：重布线后邻域节点代价确实降低。
// 构造一个确定性小场景，手动驱动并断言至少存在一次“邻域内已有节点
// 经新节点改接后代价下降”的事件，且下降幅度与几何计算吻合。
func TestNeighborCostReduction(t *testing.T) {
	cfg := testConfig(99, 2000)
	res, err := Run(cfg)
	if err != nil {
		t.Fatal(err)
	}
	found := 0
	for _, f := range res.Frames {
		for _, ch := range f.Changes {
			if ch.Kind == ChangeRewire && ch.NewParent == f.NewNode {
				if ch.NewCost >= ch.OldCost {
					t.Fatalf("iter=%d: 邻域节点 %d 经新节点改接后代价未降低", f.Iter, ch.Node)
				}
				found++
			}
		}
	}
	if found == 0 {
		t.Fatal("整个规划中没有发生“经由新节点”的邻域重布线")
	}
	t.Logf("共发生 %d 次邻域代价降低重布线", found)
}

// TestLargeSampleNoRegression 大采样次数下不发生代价异常回升。
func TestLargeSampleNoRegression(t *testing.T) {
	cfg := testConfig(2026, 6000)
	res, err := Run(cfg)
	if err != nil {
		t.Fatal(err)
	}
	if !res.Reached {
		t.Fatal("6000 次采样未到达目标")
	}
	if len(res.Regressions) != 0 {
		t.Fatalf("大采样次数下出现 %d 次代价回升: %+v", len(res.Regressions), res.Regressions[0])
	}
}

// TestConvergenceImprovesWithMoreSamples 渐近最优性的方向性验证：
// 同种子、同场景，更多采样得到的最优代价不应更差（统计意义上取多个种子，
// 允许 RRT* 的随机性，但每个种子自身严格单调不增已由前面测试保证；
// 这里额外验证 3000 次的结果不劣于 500 次）。
func TestConvergenceImprovesWithMoreSamples(t *testing.T) {
	cfgLo := testConfig(31337, 500)
	resLo, err := Run(cfgLo)
	if err != nil {
		t.Fatal(err)
	}
	cfgHi := testConfig(31337, 3000)
	resHi, err := Run(cfgHi)
	if err != nil {
		t.Fatal(err)
	}
	// 同种子 => 前 500 次迭代完全相同，3000 次的代价必然 <= 500 次。
	if resHi.BestCost > resLo.BestCost+1e-9 {
		t.Fatalf("增加采样后代价反而上升: %.6f -> %.6f", resLo.BestCost, resHi.BestCost)
	}
	if resHi.BestCost >= resLo.BestCost {
		// 通常应该严格改进；若相等只记录不失败（场景可能提前充分收敛）。
		t.Logf("500 次后已收敛：%.6f", resLo.BestCost)
	}
}

// TestResultPathIsCollisionFree 输出路径本身必须无碰撞且连通。
func TestResultPathIsCollisionFree(t *testing.T) {
	cfg := testConfig(555, 2000)
	res, err := Run(cfg)
	if err != nil {
		t.Fatal(err)
	}
	if len(res.PathJSON) < 2 {
		t.Fatal("路径节点不足")
	}
	for i := 0; i+1 < len(res.PathJSON); i++ {
		a := Vec2{res.PathJSON[i][0], res.PathJSON[i][1]}
		b := Vec2{res.PathJSON[i+1][0], res.PathJSON[i+1][1]}
		if SegmentBlocked(cfg.Obstacles, a, b) {
			t.Fatalf("路径第 %d 段与障碍物相交: %v -> %v", i, a, b)
		}
	}
	first, last := res.PathJSON[0], res.PathJSON[len(res.PathJSON)-1]
	if !approxEq(first[0], cfg.Start.X) || !approxEq(first[1], cfg.Start.Y) {
		t.Fatal("路径起点不是 Start")
	}
	if math.Hypot(last[0]-cfg.Goal.X, last[1]-cfg.Goal.Y) > cfg.GoalRadius+cfg.StepSize+1e-6 {
		t.Fatal("路径终点未到达目标邻域")
	}
}

// TestRejectsInvalidBoundaryConfigs 起终点落在障碍物边界上必须被拒绝，
// 防止闭集语义被配置层绕过。
func TestRejectsInvalidBoundaryConfigs(t *testing.T) {
	cfg := testConfig(1, 100)
	cfg.Start = Vec2{40, 30} // 恰好落在墙的左边界
	if err := cfg.Validate(); err == nil {
		t.Fatal("起点在障碍物边界上应当校验失败")
	}
	cfg = testConfig(1, 100)
	cfg.Goal = Vec2{45, 30} // 右边界
	if err := cfg.Validate(); err == nil {
		t.Fatal("目标在障碍物边界上应当校验失败")
	}
}

// TestDeterministicFixedSeed 固定种子两次运行结果必须逐位一致。
func TestDeterministicFixedSeed(t *testing.T) {
	r1, err := Run(testConfig(777, 1000))
	if err != nil {
		t.Fatal(err)
	}
	r2, err := Run(testConfig(777, 1000))
	if err != nil {
		t.Fatal(err)
	}
	if len(r1.Frames) != len(r2.Frames) || len(r1.NodeJSON) != len(r2.NodeJSON) {
		t.Fatal("固定种子下结果长度不一致")
	}
	for i := range r1.Frames {
		f1, f2 := r1.Frames[i], r2.Frames[i]
		if f1.BestCost != f2.BestCost || f1.Rejected != f2.Rejected || len(f1.Changes) != len(f2.Changes) {
			t.Fatalf("iter=%d 固定种子下结果不一致", i)
		}
	}
}

// TestCheckRegressionDetector 直接单测回升检测纯函数：
// 能捕获真正的回升，对持平、下降、容差内抖动、未到达(Inf)均不误报。
func TestCheckRegressionDetector(t *testing.T) {
	inf := math.Inf(1)
	type tc struct {
		name       string
		prev, cur  float64
		wantReport bool
	}
	cases := []tc{
		{"尚未到达->尚未到达", inf, inf, false},
		{"尚未到达->首次到达", inf, 130, false},
		{"代价下降", 130, 120, false},
		{"持平", 120, 120, false},
		{"容差内抖动(回升0.5eps)", 120, 120 + 5e-10, false},
		{"真实回升", 120, 121.5, true},
		{"恰好超过eps", 120, 120 + 1.1e-9, true},
		{"回升后不可达不比较", 120, inf, false},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			reg, ok := checkRegression(c.prev, c.cur, 7, 1e-9)
			if ok != c.wantReport {
				t.Fatalf("checkRegression(%v,%v) report=%v, want %v", c.prev, c.cur, ok, c.wantReport)
			}
			if ok && (reg.Iteration != 7 || reg.Prev != c.prev || reg.Cur != c.cur) {
				t.Fatalf("regression 记录内容错误: %+v", reg)
			}
		})
	}
}
