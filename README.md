# go-rrt：RRT* 路径规划与重布线正确性验证

纯 Go 标准库实现的 **RRT\*（Rapidly-exploring Random Tree Star）** 二维路径规划器，
聚焦三个容易出错且不易察觉的问题：

1. 新节点插入后邻域内的**双向重布线**（选父节点 + 反向改善邻域节点）；
2. 线段与多边形障碍物的**精确边界碰撞判定**（端点落边、顶点相切、共线重叠）；
3. **渐近最优性**的可执行验证：固定种子下最优路径代价必须单调不增，
   任何代价回升都会被检测出来并使测试 / demo 失败。

规划完成后可导出**单一静态 HTML 文件**（数据与逻辑全部内嵌，零第三方依赖、
不启动服务），在浏览器中逐帧回放树的生长、重布线事件与最优路径变化。

## 目录结构

| 文件 | 内容 |
| --- | --- |
| `geometry.go` | 向量、多边形、点/线段碰撞检测（边界规则的唯一实现处） |
| `rrtstar.go` | RRT* 主循环、邻域半径、选父节点、反向重布线、代价级联、事件记录 |
| `report.go` | 自包含静态 HTML 回放生成器（原生 HTML/CSS/JS，无框架无图形库） |
| `cmd/rrtstar-demo/main.go` | 命令行演示：规划 → 单调性校验 → 导出 `report.html` |
| `*_test.go` | 碰撞边界、重布线降代价、单调不增、大样本无回升等自动化测试 |

## 快速开始

```bash
# 运行全部测试
go test ./...

# 大样本测试（默认也会跑；-short 可跳过）
go test ./... -run TestLargeRunNoCostRebound -v

# 生成回放报告
go run ./cmd/rrtstar-demo -seed 42 -samples 2500 -out report.html
open report.html        # 直接用浏览器打开，无需服务器
```

demo 输出示例（seed=42, 2500 次采样）：

```
nodes=2088 rewires=1472
best path cost: 909.8837
goal-cost improvements: 23
monotonicity check: PASS (best cost never increases)
```

若检测到代价回升，demo 以退出码 2 失败并打印第一次回升的位置。

## 算法步骤（每次迭代）

1. **采样**：以概率 `GoalSampleRate` 直接采目标点，否则在 `[0,W]×[0,H]` 均匀采样。
2. **扩展（steer）**：找最近节点，朝样本点最多前进 `StepSize`，得到 `qNew`；
   落进障碍物闭集的扩展被拒绝。
3. **邻域**：半径采用 Karaman–Frazzoli 收缩邻域
   `r(n) = min(Eta, Gamma·sqrt(ln(n)/n))`，`Gamma` 默认取球体积常数
   `2·sqrt(A/π)`。
4. **选择父节点**：在半径内所有**无碰撞**邻域节点中，选
   `cost(neighbor) + |neighbor−qNew|` 最小者作为新节点父节点（初始候选为最近点）。
5. **反向重布线**（关键，见下节）。
6. **目标连接**：节点进入 `GoalRadius` 且与目标连线无碰撞时，把目标作为普通
   树节点插入；之后目标节点也参与正常重布线，因此其代价只会下降。

## 一、重布线判据（为什么方向不能反）

第 5 步对邻域内每个**已存在**的节点 `x` 计算：

```
newCost(x) = cost(qNew) + |qNew − x|
```

仅当 **`newCost(x) < cost(x) − ε`（严格更小）且线段 `qNew–x` 无碰撞** 时，
才把 `x` 的父节点从旧父节点改为 `qNew`。

方向判断写成反方向（检查 `cost(x) + |x−qNew| < cost(qNew)`，即"让新节点挂靠
旧节点"）是经典隐蔽 bug：父节点选择阶段已经做过这件事，反向检查若也朝这个
方向写，重布线分支永远不会降低任何已有节点的代价——树依然能长出来、路径依然
存在，只是结构系统性次优，常规可视化很难发现。本实现：

- 判据方向固定为"**已有节点能否通过新节点得到更短路径**"；
- 相等不重写（ties 保留原父节点），保证事件流确定；
- 换父后把代价差 `Δ = newCost − oldCost < 0` **级联到整棵子树**
  （`addCost`），否则后代节点的 `cost` 字段会与实际路径脱节——
  这是另一个会让"表面上重布线成功、后代代价却是旧值"的隐患；
- 每次重布线记录一条 `rewire` 事件：`node / oldParent / newParent /
  oldCost / newCost`，供回放与测试逐条审计。

## 二、碰撞检测规则（单一、一致的闭集约定）

所有几何退化判断共用同一个容差 `Epsilon = 1e-9`，规则只有一条：

> **障碍物多边形是闭集——内部和边界都属于碰撞空间。**

由此得到一致且对称（`SegmentHits(a,b) == SegmentHits(b,a)`）的判定：

| 几何情形 | 判定 |
| --- | --- |
| 端点严格在多边形内部 | 碰撞 |
| 端点恰好落在边上或顶点上 | **碰撞** |
| 线段与边严格相交 | 碰撞 |
| 线段在顶点处相切（擦过角点） | **碰撞** |
| 与边共线且区间有重叠（含仅端点相接） | **碰撞** |
| 共线但区间不相交（中间有缝隙） | 自由 |
| 零长度线段且点在边界上 | **碰撞** |

实现方式（`geometry.go`）：定向测试 `cross((b−a),(c−a))` 先归类为
`−1/0/+1`（以 `Epsilon` 夹逼为 0），严格穿套用异号判定；其余所有边界情形
统一归约为"某个端点落在另一条闭线段上"（共线 + 包围盒夹逼），
不再使用第二套阈值，因此同一构型无论以何种顺序、重复多少次调用结果都相同。
凹多边形用射线法判内部，边界点在进入射线法之前先行拦截。

## 三、渐近最优性验证方法

RRT\* 的不变量是：**已记录的最优路径代价永不上升**。每次目标代价下降都会
发出一条 `goal` 事件，`Result.CostHistory()` 给出代价序列，
`Result.VerifyMonotonic()` 返回第一次回升的下标（-1 表示通过）。

测试覆盖（`*_test.go`）：

- `TestReachesGoalAndMonotonic`：固定 seed=42、2500 次采样，代价序列单调不增；
- `TestMonotonicAcrossSeeds`：7 个不同固定种子全部无回升
  （方向写反的 bug 往往只在部分种子上暴露）；
- `TestRewireActuallyLowersCost`：独立重放全部事件，逐条断言
  ① 每个 rewire 事件都严格降低目标节点代价；② `newCost ==
  cost(newParent)+边长`；③ 重放后所有节点沿父指针可回到根且无环
  （重放时同样做子树代价级联，与实现一一对应）；
- `TestRewiredNeighborCostStrictlyDecreased`：统计场景下确有重布线发生且全部降代价；
- `TestCostConvergesWithMoreSamples`：同种子 400 → 4000 次采样，
  代价不得变差（长流是短流的前缀，RRT* 不会遗忘已有的改善），并打印改善幅度；
- `TestLargeRunNoCostRebound`：20000 次采样无回升；同时遍历**最终树**逐条边校验
  `cost(node) == cost(parent)+边长`（级联不同步会在这里暴露）、正边权下
  子代价大于父代价、所有现存边无碰撞；
- `TestPointBoundaryIsCollision` / `TestSegmentBoundaryCases` /
  `TestBoundaryExactValues`：边界点、端点落边、角点相切、共线重叠/分离、
  零长度线段等精确构型；
- `TestSegmentCollisionSymmetricAndStable`：5000 对随机线段，正反方向与
  重复调用结果必须一致；
- `TestWriteHTMLReport`：报告无任何外部 URL / 外链资源，JSON 未被模板二次转义。

## 四、HTML 回放

`WriteHTMLReport` 把扁平事件流折叠为"每迭代一帧"，内嵌为
`<script type="application/json">`，页面用原生 Canvas 2D 绘制：

- ⏮/◀/▶/▶| 逐帧前进后退、拖动进度条、可调速自动播放（键盘 ←/→/空格/Home/End）；
- 绿色 = 本帧新节点，**琥珀色脉冲 = 本帧发生重布线的节点**，
  红色虚线 = 被替换的旧父边，粗琥珀线 = 重布线新父边，紫色 = 当前最优路径；
- 侧栏实时显示节点数、累计/本帧重布线次数、当前最优代价；
- 事件日志逐行列出 `ADD / REW / GOAL / BEST`（含旧→新父节点与代价差 Δ）；
- 页头在检测到代价回升时显示红色警告。

浏览器端重建树时镜像了 Go 端的子树代价级联，因此任意一帧暂停时看到的
节点代价都与规划器当时的内部状态一致。报告文件即数据即逻辑，
可离线分发、归档，CI 中也可以直接作为产物保存。

## 关键参数

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `StepSize` | 单次扩展最大长度 η-steer | `min(W,H)/20` |
| `GoalRadius` | 判定连接目标的半径 | `StepSize` |
| `GoalSampleRate` | 直接采目标点的概率 | `0.05` |
| `Gamma` | 邻域收缩常数 | `2·sqrt(W·H/π)` |
| `Eta` | 邻域半径上限 | `4·StepSize` |
| `Seed` | 随机种子（固定即可复现） | — |
| `MaxSamples` | 采样迭代上限 | `2000` |
