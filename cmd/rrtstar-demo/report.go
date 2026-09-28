package main

import "strings"

// reportHTML 把规划数据内嵌进单一静态 HTML：数据放在
// <script type="application/json"> 中，JS 用 JSON.parse 读取，
// 不启动任何服务、不引用任何第三方资源。
func reportHTML(data []byte) string {
	return strings.ReplaceAll(reportTemplate, "__RRT_DATA__", string(data))
}

// 注意：本文件 JS 代码刻意不使用反引号模板字符串（Go raw string 限制）。
const reportTemplate = `<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RRT* 树生长与重布线回放</title>
<style>
  :root {
    color-scheme: light;
    --page: #f9f9f7;
    --surface: #fcfcfb;
    --ink: #0b0b0b;
    --ink-2: #52514e;
    --muted: #898781;
    --grid: #e1e0d9;
    --axis: #c3c3b7;
    --border: rgba(11,11,11,0.10);
    --series-1: #2a78d6;  /* 树边 / 最优路径 / 代价曲线 */
    --series-2: #eb6834;  /* 新节点 */
    --series-7: #4a3aa7;  /* 重布线 */
    --good: #0ca30c;
    --critical: #d03b3b;
    --obs-fill: #d8d7d0;
    --obs-edge: #8a887f;
    --tree-edge: rgba(42,120,214,0.35);
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      color-scheme: dark;
      --page: #0d0d0d;
      --surface: #1a1a19;
      --ink: #ffffff;
      --ink-2: #c3c2b7;
      --muted: #898781;
      --grid: #2c2c2a;
      --axis: #383835;
      --border: rgba(255,255,255,0.10);
      --series-1: #3987e5;
      --series-2: #d95926;
      --series-7: #9085e9;
      --good: #0ca30c;
      --critical: #e66767;
      --obs-fill: #2e2e2b;
      --obs-edge: #6b6a63;
      --tree-edge: rgba(120,170,235,0.40);
    }
  }
  :root[data-theme="dark"] {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #fff; --ink-2: #c3c2b7;
    --muted: #898781; --grid: #2c2c2a; --axis: #383835;
    --border: rgba(255,255,255,0.10);
    --series-1: #3987e5; --series-2: #d95926; --series-7: #9085e9;
    --good: #0ca30c; --critical: #e66767;
    --obs-fill: #2e2e2b; --obs-edge: #6b6a63;
    --tree-edge: rgba(120,170,235,0.40);
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--page); color: var(--ink);
    font-family: system-ui, -apple-system, "Segoe UI", sans-serif;
    font-size: 14px; line-height: 1.5;
  }
  .wrap { max-width: 1080px; margin: 0 auto; padding: 24px 20px 64px; }
  h1 { font-size: 22px; margin: 0 0 4px; }
  .sub { color: var(--ink-2); margin: 0 0 18px; }
  .card {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 10px; padding: 16px; margin-bottom: 16px;
  }
  .stats { display: flex; flex-wrap: wrap; gap: 10px; }
  .stat {
    border: 1px solid var(--border); border-radius: 8px;
    padding: 8px 12px; min-width: 110px; background: var(--surface);
  }
  .stat .k { font-size: 12px; color: var(--muted); }
  .stat .v { font-size: 18px; font-weight: 600; font-variant-numeric: tabular-nums; }
  .badge {
    display: inline-flex; align-items: center; gap: 6px;
    padding: 4px 10px; border-radius: 999px; font-weight: 600; font-size: 13px;
    border: 1px solid var(--border);
  }
  .badge .dot { width: 9px; height: 9px; border-radius: 50%; }
  .badge.ok { color: var(--good); }
  .badge.ok .dot { background: var(--good); }
  .badge.bad { color: var(--critical); }
  .badge.bad .dot { background: var(--critical); }
  .head-row { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 10px; }
  button {
    font: inherit; color: var(--ink); background: var(--surface);
    border: 1px solid var(--border); border-radius: 7px;
    padding: 6px 12px; cursor: pointer;
  }
  button:hover { border-color: var(--muted); }
  button:disabled { opacity: 0.4; cursor: default; }
  .controls { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; margin: 12px 0; }
  .controls input[type=range] { flex: 1 1 260px; accent-color: var(--series-1); }
  .controls label { color: var(--ink-2); display: inline-flex; align-items: center; gap: 5px; white-space: nowrap; }
  .toggles { display: flex; flex-wrap: wrap; gap: 14px; margin-top: 8px; color: var(--ink-2); }
  .toggles label { display: inline-flex; align-items: center; gap: 5px; cursor: pointer; }
  canvas { display: block; width: 100%; height: auto; border-radius: 6px; }
  .legend { display: flex; flex-wrap: wrap; gap: 14px; margin: 10px 2px 0; color: var(--ink-2); font-size: 13px; }
  .legend .item { display: inline-flex; align-items: center; gap: 6px; }
  .swatch { width: 16px; height: 3px; border-radius: 2px; display: inline-block; }
  .swatch.line-tree { background: var(--series-1); opacity: 0.45; }
  .swatch.line-path { background: var(--series-1); height: 5px; }
  .swatch.dot { width: 11px; height: 11px; border-radius: 50%; }
  .frameinfo { font-variant-numeric: tabular-nums; color: var(--ink-2); margin-top: 10px; }
  .frameinfo b { color: var(--ink); }
  .chart-head { display: flex; justify-content: space-between; align-items: baseline; flex-wrap: wrap; gap: 8px; }
  .chart-title { font-weight: 600; }
  .chart-note { color: var(--muted); font-size: 12px; }
  .chart-box { position: relative; }
  #tooltip {
    position: absolute; pointer-events: none; display: none; z-index: 5;
    background: var(--surface); border: 1px solid var(--border); border-radius: 7px;
    padding: 6px 9px; font-size: 12px; box-shadow: 0 2px 10px rgba(0,0,0,0.18);
    font-variant-numeric: tabular-nums; white-space: nowrap;
  }
  h2 { font-size: 17px; margin: 4px 0 8px; }
  .rules p, .rules li { color: var(--ink-2); }
  .rules code { background: var(--page); border: 1px solid var(--border); border-radius: 4px; padding: 0 4px; }
  .rules pre {
    background: var(--page); border: 1px solid var(--border); border-radius: 8px;
    padding: 10px 12px; overflow-x: auto; font-size: 12.5px; color: var(--ink-2);
  }
  details { margin-top: 10px; }
  summary { cursor: pointer; color: var(--ink-2); }
  .tablewrap { max-height: 260px; overflow: auto; margin-top: 8px; border: 1px solid var(--border); border-radius: 8px; }
  table { border-collapse: collapse; width: 100%; font-variant-numeric: tabular-nums; font-size: 12.5px; }
  th, td { padding: 4px 12px; text-align: right; border-bottom: 1px solid var(--grid); }
  th { position: sticky; top: 0; background: var(--surface); color: var(--ink-2); }
  td.l, th.l { text-align: left; }
</style>
</head>
<body>
<div class="wrap" id="app">
  <div class="head-row">
    <div>
      <h1>RRT* 树生长与重布线逐步回放</h1>
      <p class="sub">固定随机种子，逐步观察采样插入、邻域选父与反向重布线（rewire）事件。</p>
    </div>
    <div>
      <span class="badge ok" id="regBadge"><span class="dot"></span><span id="regText"></span></span>
      <button id="themeBtn" type="button">切换深浅色</button>
    </div>
  </div>

  <div class="card">
    <div class="stats" id="stats"></div>
  </div>

  <div class="card">
    <div class="controls">
      <button id="btnReset" type="button">⏮ 重置</button>
      <button id="btnPrev" type="button">← 上一步</button>
      <button id="btnPlay" type="button">▶ 播放</button>
      <button id="btnNext" type="button">下一步 →</button>
      <button id="btnPrevRewire" type="button">↑ 上一重布线</button>
      <button id="btnNextRewire" type="button">下一重布线 ↓</button>
      <input type="range" id="scrub" min="0" max="0" value="0" step="1" aria-label="迭代进度">
      <label>速度 <input type="range" id="speed" min="1" max="60" value="12" style="accent-color:var(--series-1)"> <span id="speedVal">12</span>/秒</label>
    </div>
    <div class="toggles">
      <label><input type="checkbox" id="tgTree" checked> 树边</label>
      <label><input type="checkbox" id="tgPath" checked> 当前最优路径</label>
      <label><input type="checkbox" id="tgRadius"> 邻域半径</label>
      <label><input type="checkbox" id="tgSamples"> 被拒绝采样点</label>
    </div>
    <div class="frameinfo" id="frameinfo"></div>
    <div style="margin-top:12px">
      <canvas id="tree" width="1000" height="1000" aria-label="RRT 树生长画布"></canvas>
    </div>
    <div class="legend">
      <span class="item"><span class="swatch line-tree"></span>树边</span>
      <span class="item"><span class="swatch line-path"></span>当前最优路径</span>
      <span class="item"><span class="swatch dot" style="background:var(--series-2)"></span>本次新节点</span>
      <span class="item"><span class="swatch dot" style="background:var(--series-7)"></span>重布线节点（新父边同色）</span>
      <span class="item"><span class="swatch dot" style="background:var(--good)"></span>起点</span>
      <span class="item"><span class="swatch dot" style="background:var(--critical)"></span>目标 / 目标邻域</span>
      <span class="item"><span class="swatch" style="background:var(--obs-fill);border:1px solid var(--obs-edge);height:12px"></span>障碍物（闭集，边界阻挡）</span>
    </div>
  </div>

  <div class="card">
    <div class="chart-head">
      <div class="chart-title">当前最优路径代价 vs. 采样迭代次数</div>
      <div class="chart-note">渐近最优性：曲线必须单调不增；将鼠标移到图上查看逐迭代数值。</div>
    </div>
    <div class="chart-box">
      <canvas id="chart" width="1000" height="280" aria-label="代价收敛曲线"></canvas>
      <div id="tooltip"></div>
    </div>
    <details>
      <summary>查看代价数据表（仅列出到达目标后的迭代）</summary>
      <div class="tablewrap"><table id="costTable"></table></div>
    </details>
  </div>

  <div class="card rules">
    <h2>判定规则与验证方法</h2>
    <p><b>重布线（rewire）判据：</b>新节点 <code>x_new</code> 插入后，对邻域内每个已有节点 <code>x</code>，
    当且仅当 <code>cost(x_new) + |x_new − x| &lt; cost(x) − ε</code> 且线段 <code>x_new—x</code> 无碰撞时，
    才把 <code>x</code> 改接到 <code>x_new</code>。判据方向是“<b>已有节点能否借新节点降低自己的代价</b>”，
    反向写错会得到一棵次优但表面合法的树。改接后代价差 <code>Δ</code> 必须沿 <code>x</code> 的整棵子树传播。</p>
    <p><b>碰撞检测规则：</b>障碍物为<b>闭集</b>矩形，线段相交采用 Liang–Barsky 裁剪，
    参数区间在 <code>[0,1]</code> 内交集非空即碰撞。因此：端点落在边/角点上判碰撞；
    与边重合判碰撞；斜穿角点（单点相切）判碰撞；判定与线段方向无关、重复调用一致。</p>
    <p><b>渐近最优性验证：</b>固定种子下逐迭代记录最优代价 <code>J_i</code>，要求
    <code>J_{i+1} ≤ J_i</code>。规划器内置回升检测：任何 <code>J_i &gt; J_{i-1} + ε</code>
    都会记录到 <code>regressions</code>，页面顶部徽章与自动化测试 <code>TestBestCostMonotonicNonIncreasing</code>
    / <code>TestLargeSampleNoRegression</code> 都会失败告警。</p>
    <pre>go test ./...    # 边界碰撞一致性 / 重布线代价下降 / 单调不增 / 6000 次大采样无回升
go run ./cmd/rrtstar-demo -seed 20260928 -iter 2500 -out report.html</pre>
  </div>
</div>

<script id="rrt-data" type="application/json">__RRT_DATA__</script>
<script>
(function () {
  'use strict';
  var DATA = JSON.parse(document.getElementById('rrt-data').textContent);
  var FRAMES = DATA.frames, NODES = DATA.nodes;
  var N = NODES.length;

  // ---- 播放状态（树拓扑/代价通过累计事件帧重放）----
  var parentArr = new Int32Array(N).fill(-1);
  var costArr = new Float64Array(N);
  var children = [];
  for (var i = 0; i < N; i++) children.push([]);
  var aliveCount = 1; // 根节点始终存在
  var appliedTo = -1; // 已应用到的帧下标

  function resetTree() {
    parentArr.fill(-1);
    for (var i = 0; i < N; i++) children[i].length = 0;
    costArr[0] = 0;
    aliveCount = 1;
    appliedTo = -1;
  }

  function propagateDelta(node, delta) {
    var stack = [node];
    while (stack.length) {
      var t = stack.pop();
      costArr[t] += delta;
      var ch = children[t];
      for (var i = 0; i < ch.length; i++) stack.push(ch[i]);
    }
  }

  function applyChange(ch) {
    if (ch.k === 'add') {
      parentArr[ch.n] = ch.np;
      costArr[ch.n] = ch.nc;
      children[ch.np].push(ch.n);
      if (ch.n + 1 > aliveCount) aliveCount = ch.n + 1;
    } else { // rewire
      var oldKids = children[ch.op];
      for (var i = 0; i < oldKids.length; i++) {
        if (oldKids[i] === ch.n) { oldKids.splice(i, 1); break; }
      }
      parentArr[ch.n] = ch.np;
      children[ch.np].push(ch.n);
      propagateDelta(ch.n, ch.nc - ch.oc);
    }
  }

  function applyTo(target) {
    if (target < appliedTo) { resetTree(); }
    while (appliedTo < target) {
      appliedTo++;
      var chs = FRAMES[appliedTo].ch;
      for (var i = 0; i < chs.length; i++) applyChange(chs[i]);
    }
  }

  // ---- 重布线事件帧索引（快速跳转）----
  var rewireFrameIdx = [];
  for (var f = 0; f < FRAMES.length; f++) {
    var cs = FRAMES[f].ch, has = false;
    for (var c = 0; c < cs.length; c++) if (cs[c].k === 'rewire') { has = true; break; }
    if (has) rewireFrameIdx.push(f);
  }

  // ---- 统计 ----
  var nRewire = 0, nReject = 0, firstReach = -1;
  for (var fi = 0; fi < FRAMES.length; fi++) {
    if (FRAMES[fi].rj) nReject++;
    if (firstReach < 0 && FRAMES[fi].gn >= 0) firstReach = fi;
    var ch2 = FRAMES[fi].ch;
    for (var ci = 0; ci < ch2.length; ci++) if (ch2[ci].k === 'rewire') nRewire++;
  }
  var finalCost = DATA.bestCost;
  var regressions = DATA.regressions || [];

  var statsEl = document.getElementById('stats');
  function stat(k, v) {
    return '<div class="stat"><div class="k">' + k + '</div><div class="v">' + v + '</div></div>';
  }
  statsEl.innerHTML =
    stat('随机种子', DATA.seed) +
    stat('采样上限', DATA.maxIter) +
    stat('树节点数', N) +
    stat('重布线事件', nRewire) +
    stat('被拒采样', nReject) +
    stat('首次到达迭代', firstReach >= 0 ? firstReach + 1 : '—') +
    stat('最终最优代价', finalCost == null ? '不可达' : fmt(finalCost)) +
    stat('代价回升次数', regressions.length);

  var badge = document.getElementById('regBadge');
  if (regressions.length > 0) {
    badge.className = 'badge bad';
    document.getElementById('regText').textContent = '检测到 ' + regressions.length + ' 次代价回升（异常！）';
  } else {
    badge.className = 'badge ok';
    document.getElementById('regText').textContent = '代价单调不增，无回升';
  }

  function fmt(x) { return (Math.round(x * 10000) / 10000).toFixed(4); }

  // ---- 控件 ----
  var scrub = document.getElementById('scrub');
  scrub.max = FRAMES.length - 1;
  var speed = document.getElementById('speed');
  var speedVal = document.getElementById('speedVal');
  var cur = -1;
  var playing = false, timer = null;
  var showTree = document.getElementById('tgTree');
  var showPath = document.getElementById('tgPath');
  var showRadius = document.getElementById('tgRadius');
  var showSamples = document.getElementById('tgSamples');

  function setCur(i) {
    cur = Math.max(-1, Math.min(FRAMES.length - 1, i));
    applyTo(cur);
    scrub.value = cur < 0 ? 0 : cur;
    render();
  }
  function stopPlay() {
    playing = false;
    if (timer) { clearInterval(timer); timer = null; }
    document.getElementById('btnPlay').textContent = '▶ 播放';
  }
  function togglePlay() {
    if (playing) { stopPlay(); return; }
    if (cur >= FRAMES.length - 1) setCur(-1);
    playing = true;
    document.getElementById('btnPlay').textContent = '⏸ 暂停';
    timer = setInterval(function () {
      if (cur >= FRAMES.length - 1) { stopPlay(); return; }
      setCur(cur + 1);
    }, 1000 / parseInt(speed.value, 10));
  }

  document.getElementById('btnReset').onclick = function () { stopPlay(); setCur(-1); };
  document.getElementById('btnPrev').onclick = function () { stopPlay(); setCur(cur - 1); };
  document.getElementById('btnNext').onclick = function () { stopPlay(); setCur(cur + 1); };
  document.getElementById('btnPlay').onclick = togglePlay;
  scrub.oninput = function () { stopPlay(); setCur(parseInt(scrub.value, 10)); };
  speed.oninput = function () {
    speedVal.textContent = speed.value;
    if (playing) { stopPlay(); togglePlay(); }
  };
  function jumpRewire(dir) {
    stopPlay();
    // 找严格大于/小于 cur 的重布线帧
    var lo = 0, hi = rewireFrameIdx.length - 1, res = -1;
    if (dir > 0) {
      for (var i = 0; i < rewireFrameIdx.length; i++) if (rewireFrameIdx[i] > cur) { res = rewireFrameIdx[i]; break; }
    } else {
      for (var j = rewireFrameIdx.length - 1; j >= 0; j--) if (rewireFrameIdx[j] < cur) { res = rewireFrameIdx[j]; break; }
    }
    if (res >= 0) setCur(res);
  }
  document.getElementById('btnPrevRewire').onclick = function () { jumpRewire(-1); };
  document.getElementById('btnNextRewire').onclick = function () { jumpRewire(1); };
  [showTree, showPath, showRadius, showSamples].forEach(function (cb) {
    cb.onchange = render;
  });

  // ---- 主题 ----
  var root = document.documentElement;
  document.getElementById('themeBtn').onclick = function () {
    root.setAttribute('data-theme', cssVar('--page').trim() === '#f9f9f7' ? 'dark' : 'light');
    render();
  };
  function cssVar(name) {
    return getComputedStyle(document.getElementById('app')).getPropertyValue(name);
  }
  if (window.matchMedia) {
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', render);
  }

  // ---- 画布与坐标变换 ----
  var treeCanvas = document.getElementById('tree');
  var tctx = treeCanvas.getContext('2d');
  var W = DATA.bounds.max.x, H = DATA.bounds.max.y;
  var PAD = 34;

  function prepCanvas(cv, ctx, cssH) {
    var dpr = window.devicePixelRatio || 1;
    var cssW = cv.clientWidth || cv.width;
    var ch = cssH || cssW;
    // 仅在实际尺寸变化时重设位图（重设 width 会清空画布，高频 hover 时避免抖动）。
    var bw = Math.round(cssW * dpr), bh = Math.round(ch * dpr);
    if (cv.width !== bw || cv.height !== bh) { cv.width = bw; cv.height = bh; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { w: cssW, h: ch };
  }
  function tx(x) { return PAD + x / W * (treeSize.w - 2 * PAD); }
  function ty(y) { return treeSize.h - PAD - y / H * (treeSize.h - 2 * PAD); }
  var treeSize = { w: 1000, h: 1000 };

  function nodeXY(id) { return [tx(NODES[id].x), ty(NODES[id].y)]; }

  function drawTree() {
    var s = prepCanvas(treeCanvas, tctx, treeCanvas.clientWidth || 1000);
    treeSize = s;
    var ctx = tctx;
    ctx.clearRect(0, 0, s.w, s.h);

    // 背景网格
    ctx.strokeStyle = cssVar('--grid'); ctx.lineWidth = 1;
    ctx.beginPath();
    for (var gx = 0; gx <= W; gx += 10) { ctx.moveTo(tx(gx), ty(0)); ctx.lineTo(tx(gx), ty(H)); }
    for (var gy = 0; gy <= H; gy += 10) { ctx.moveTo(tx(0), ty(gy)); ctx.lineTo(tx(W), ty(gy)); }
    ctx.stroke();
    // 边框
    ctx.strokeStyle = cssVar('--axis');
    ctx.strokeRect(tx(0), ty(H), tx(W) - tx(0), ty(0) - ty(H));

    // 障碍物（闭集）
    ctx.fillStyle = cssVar('--obs-fill');
    ctx.strokeStyle = cssVar('--obs-edge');
    DATA.obstacles.forEach(function (o) {
      var x = tx(o.min.x), y = ty(o.max.y);
      var w = tx(o.max.x) - tx(o.min.x), h = ty(o.min.y) - ty(o.max.y);
      ctx.fillRect(x, y, w, h);
      ctx.strokeRect(x, y, w, h);
    });

    // 被拒绝采样点（累计到当前帧）
    if (showSamples.checked) {
      ctx.fillStyle = cssVar('--muted');
      for (var ri = 0; ri <= cur; ri++) {
        var fr = FRAMES[ri];
        if (!fr.rj) continue;
        ctx.fillRect(tx(fr.s[0]) - 1.5, ty(fr.s[1]) - 1.5, 3, 3);
      }
    }

    var frame = cur >= 0 ? FRAMES[cur] : null;

    // 树边
    if (showTree.checked) {
      ctx.strokeStyle = cssVar('--tree-edge');
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (var id = 1; id < aliveCount; id++) {
        var p = parentArr[id];
        if (p < 0) continue;
        var a = nodeXY(id), b = nodeXY(p);
        ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
      }
      ctx.stroke();
    }

    // 当前最优路径
    var gn = frame ? frame.gn : (cur >= 0 ? FRAMES[cur].gn : -1);
    if (showPath.checked && gn >= 0) {
      ctx.strokeStyle = cssVar('--series-1');
      ctx.lineWidth = 3.5; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      var cur0 = gn, first = true;
      while (cur0 >= 0) {
        var q = nodeXY(cur0);
        if (first) { ctx.moveTo(q[0], q[1]); first = false; } else ctx.lineTo(q[0], q[1]);
        cur0 = parentArr[cur0];
      }
      // 终点补到精确目标点
      ctx.lineTo(tx(DATA.goal.x), ty(DATA.goal.y));
      ctx.stroke();
    }

    // 目标邻域
    ctx.strokeStyle = cssVar('--critical');
    ctx.setLineDash([4, 4]); ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.arc(tx(DATA.goal.x), ty(DATA.goal.y),
      DATA.goalRadius / W * (treeSize.w - 2 * PAD), 0, Math.PI * 2); ctx.stroke();
    ctx.setLineDash([]);

    // 起终点
    dot(ctx, tx(DATA.start.x), ty(DATA.start.y), 6, cssVar('--good'), '#ffffff');
    dot(ctx, tx(DATA.goal.x), ty(DATA.goal.y), 6, cssVar('--critical'), '#ffffff');
    label(ctx, '起点', tx(DATA.start.x) + 9, ty(DATA.start.y) - 8);
    label(ctx, '目标', tx(DATA.goal.x) + 9, ty(DATA.goal.y) - 8);

    if (frame) {
      // 邻域半径
      if (showRadius.checked && frame.nn >= 0) {
        var nn = nodeXY(frame.nn);
        ctx.strokeStyle = cssVar('--series-7');
        ctx.globalAlpha = 0.55; ctx.setLineDash([5, 4]); ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.arc(nn[0], nn[1],
          frame.r / W * (treeSize.w - 2 * PAD), 0, Math.PI * 2); ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha = 1;
      }

      // 本次采样点十字
      var sx = tx(frame.s[0]), sy = ty(frame.s[1]);
      ctx.strokeStyle = frame.rj ? cssVar('--critical') : cssVar('--muted');
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(sx - 5, sy); ctx.lineTo(sx + 5, sy);
      ctx.moveTo(sx, sy - 5); ctx.lineTo(sx, sy + 5);
      ctx.stroke();

      // 重布线高亮：旧边虚线 + 新父边粗线 + 节点光环
      frame.ch.forEach(function (ch) {
        if (ch.k !== 'rewire') return;
        var child = nodeXY(ch.n), np = nodeXY(ch.np), op = nodeXY(ch.op);
        ctx.strokeStyle = cssVar('--series-7'); ctx.globalAlpha = 0.45;
        ctx.setLineDash([4, 4]); ctx.lineWidth = 1.4;
        ctx.beginPath(); ctx.moveTo(child[0], child[1]); ctx.lineTo(op[0], op[1]); ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha = 1;
        ctx.strokeStyle = cssVar('--series-7'); ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(child[0], child[1]); ctx.lineTo(np[0], np[1]); ctx.stroke();
        halo(ctx, child[0], child[1], 9, cssVar('--series-7'));
        label(ctx, '#' + ch.n + ' 重布线 ' + fmt(ch.oc) + '→' + fmt(ch.nc),
          child[0] + 11, child[1] + 4);
      });

      // 新节点
      if (frame.nn >= 0) {
        var p2 = nodeXY(frame.nn);
        halo(ctx, p2[0], p2[1], 8, cssVar('--series-2'));
        dot(ctx, p2[0], p2[1], 4.5, cssVar('--series-2'), '#fff');
      } else if (frame.rj) {
        label(ctx, '采样被拒绝（碰撞/重复）', sx + 9, sy + 16);
      }
    }
  }

  function dot(ctx, x, y, r, fill, stroke) {
    ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2);
    ctx.fillStyle = fill; ctx.fill();
    if (stroke) { ctx.lineWidth = 1.5; ctx.strokeStyle = stroke; ctx.stroke(); }
  }
  function halo(ctx, x, y, r, color) {
    ctx.save();
    ctx.globalAlpha = 0.25; ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
  }
  function label(ctx, text, x, y) {
    ctx.font = '12px system-ui, sans-serif';
    var tw = ctx.measureText(text).width;
    if (x + tw + 6 > treeSize.w) x = treeSize.w - tw - 6;
    ctx.fillStyle = cssVar('--surface');
    ctx.globalAlpha = 0.82;
    ctx.fillRect(x - 3, y - 11, tw + 6, 15);
    ctx.globalAlpha = 1;
    ctx.fillStyle = cssVar('--ink-2');
    ctx.fillText(text, x, y);
  }

  // ---- 代价收敛曲线 ----
  var chartCanvas = document.getElementById('chart');
  var cctx = chartCanvas.getContext('2d');
  var CPAD = { l: 76, r: 18, t: 16, b: 34 };
  var chartSize = { w: 1000, h: 280 };

  // 代价序列：未到达迭代记 null
  var series = FRAMES.map(function (fr) { return fr.bc; });
  var costVals = series.filter(function (v) { return v != null; });
  var yMin = Math.min.apply(null, costVals);
  var yMax = Math.max.apply(null, costVals);
  if (!(yMax > yMin)) { yMax = yMin + 1; }
  yMin = Math.max(0, yMin - (yMax - yMin) * 0.08);
  yMax = yMax + (yMax - yMin) * 0.08;
  var xFirst = firstReach >= 0 ? firstReach : 0;

  function cx(i) { return CPAD.l + (i + 1) / FRAMES.length * (chartSize.w - CPAD.l - CPAD.r); }
  function cy(v) {
    return chartSize.h - CPAD.b - (v - yMin) / (yMax - yMin) * (chartSize.h - CPAD.t - CPAD.b);
  }

  function drawChart(hoverIdx) {
    var s = prepCanvas(chartCanvas, cctx, 280);
    chartSize = s;
    var ctx = cctx;
    ctx.clearRect(0, 0, s.w, s.h);

    // 网格 + y 轴刻度
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    ctx.strokeStyle = cssVar('--grid'); ctx.lineWidth = 1;
    ctx.fillStyle = cssVar('--muted');
    var ticks = 5;
    for (var t = 0; t <= ticks; t++) {
      var v = yMin + (yMax - yMin) * t / ticks;
      var yy = cy(v);
      ctx.beginPath(); ctx.moveTo(CPAD.l, yy); ctx.lineTo(s.w - CPAD.r, yy); ctx.stroke();
      ctx.fillText(fmt(v), CPAD.l - 8, yy);
    }
    // x 轴刻度
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    var xt = 5;
    for (var k = 0; k <= xt; k++) {
      var iv = Math.round(k * (FRAMES.length - 1) / xt);
      var xx = cx(iv);
      ctx.fillText(String(iv + 1), xx, s.h - CPAD.b + 8);
      ctx.strokeStyle = cssVar('--grid');
      ctx.beginPath(); ctx.moveTo(xx, CPAD.t); ctx.lineTo(xx, s.h - CPAD.b); ctx.stroke();
    }
    // 轴线
    ctx.strokeStyle = cssVar('--axis');
    ctx.beginPath();
    ctx.moveTo(CPAD.l, CPAD.t); ctx.lineTo(CPAD.l, s.h - CPAD.b);
    ctx.lineTo(s.w - CPAD.r, s.h - CPAD.b);
    ctx.stroke();
    ctx.fillStyle = cssVar('--muted'); ctx.textAlign = 'center';
    ctx.fillText('采样迭代次数', (CPAD.l + s.w - CPAD.r) / 2, s.h - 12);
    ctx.save();
    ctx.translate(14, (CPAD.t + s.h - CPAD.b) / 2); ctx.rotate(-Math.PI / 2);
    ctx.fillText('最优路径代价 J', 0, 0);
    ctx.restore();

    // 阶跃式单调曲线（仅在到达后）
    ctx.strokeStyle = cssVar('--series-1'); ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    var pen = false;
    for (var i = 0; i < series.length; i++) {
      if (series[i] == null) { pen = false; continue; }
      var X = cx(i), Y = cy(series[i]);
      if (!pen) { ctx.moveTo(X, Y); pen = true; }
      else { ctx.lineTo(X, Y); }
    }
    ctx.stroke();
    // 终点直接标注（单序列不画图例框，按规范用直接标注）
    if (costVals.length) {
      var fx = cx(series.length - 1), fy = cy(series[series.length - 1]);
      ctx.fillStyle = cssVar('--series-1');
      ctx.beginPath(); ctx.arc(fx, fy, 3.5, 0, Math.PI * 2); ctx.fill();
      ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
      ctx.fillText('最终 ' + fmt(finalCost), s.w - CPAD.r, fy - 5);
    }

    // 当前回放位置竖线
    if (cur >= 0) {
      ctx.strokeStyle = cssVar('--series-2'); ctx.setLineDash([4, 3]); ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.moveTo(cx(cur), CPAD.t); ctx.lineTo(cx(cur), s.h - CPAD.b); ctx.stroke();
      ctx.setLineDash([]);
    }

    // 十字线 + hover 点
    var tip = document.getElementById('tooltip');
    if (hoverIdx != null && series[hoverIdx] != null) {
      var hx = cx(hoverIdx), hy = cy(series[hoverIdx]);
      ctx.strokeStyle = cssVar('--axis'); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(hx, CPAD.t); ctx.lineTo(hx, s.h - CPAD.b);
      ctx.moveTo(CPAD.l, hy); ctx.lineTo(s.w - CPAD.r, hy); ctx.stroke();
      ctx.fillStyle = cssVar('--series-1');
      ctx.beginPath(); ctx.arc(hx, hy, 4, 0, Math.PI * 2); ctx.fill();
      tip.style.display = 'block';
      tip.innerHTML = '迭代 <b>' + (hoverIdx + 1) + '</b><br>最优代价 <b>' + fmt(series[hoverIdx]) + '</b>';
      var rect = chartCanvas.getBoundingClientRect();
      var px = hx / s.w * rect.width, py = hy / s.h * rect.height;
      tip.style.left = Math.min(px + 12, rect.width - 120) + 'px';
      tip.style.top = Math.max(py - 40, 2) + 'px';
    } else {
      tip.style.display = 'none';
    }
  }

  chartCanvas.addEventListener('mousemove', function (e) {
    var rect = chartCanvas.getBoundingClientRect();
    var mx = (e.clientX - rect.left) / rect.width * chartSize.w;
    var i = Math.round((mx - CPAD.l) / (chartSize.w - CPAD.l - CPAD.r) * FRAMES.length - 1);
    i = Math.max(0, Math.min(FRAMES.length - 1, i));
    render(i);
  });
  chartCanvas.addEventListener('mouseleave', function () { render(null); });

  // ---- 帧信息 ----
  var info = document.getElementById('frameinfo');
  function frameInfo() {
    if (cur < 0) {
      info.innerHTML = '尚未开始：树中只有起点。点击「播放」或「下一步」开始回放（共 <b>' +
        FRAMES.length + '</b> 次迭代，<b>' + nRewire + '</b> 次重布线）。';
      return;
    }
    var fr = FRAMES[cur];
    var rw = 0;
    fr.ch.forEach(function (ch) { if (ch.k === 'rewire') rw++; });
    var prev = cur > 0 ? FRAMES[cur - 1].bc : null;
    var delta = '';
    if (fr.bc != null && prev != null && fr.bc < prev - 1e-12) {
      delta = '，代价下降 <b>' + fmt(prev - fr.bc) + '</b>';
    }
    info.innerHTML = '迭代 <b>' + fr.it + '</b> / ' + FRAMES.length +
      (fr.rj ? '　状态：<b>采样被拒绝</b>' : '　新节点：<b>#' + fr.nn + '</b>') +
      '　本次重布线：<b>' + rw + '</b> 次' +
      '　邻域半径 r = <b>' + fmt(fr.r) + '</b>' +
      '　当前最优代价：<b>' + (fr.bc == null ? '尚未到达' : fmt(fr.bc)) + '</b>' + delta;
  }

  // ---- 数据表 ----
  (function buildTable() {
    var html = '<thead><tr><th class="l">迭代</th><th>最优代价</th><th>相对前次变化</th></tr></thead><tbody>';
    var prevV = null;
    FRAMES.forEach(function (fr, idx) {
      if (fr.bc == null) return;
      var d = prevV == null ? '—' : (fr.bc < prevV - 1e-12 ? '−' + fmt(prevV - fr.bc) : '0');
      html += '<tr><td class="l">' + fr.it + '</td><td>' + fmt(fr.bc) + '</td><td>' + d + '</td></tr>';
      prevV = fr.bc;
    });
    document.getElementById('costTable').innerHTML = html + '</tbody>';
  })();

  // ---- 总渲染 ----
  var hover = null;
  function render(hoverIdx) {
    if (hoverIdx !== undefined) hover = hoverIdx;
    frameInfo();
    drawTree();
    drawChart(hover);
  }

  window.addEventListener('resize', function () { render(); });
  resetTree();

  // 支持通过 URL hash 直接定位：#frame=1200、#final、#play（自动播放）；
  // 主题可用 &dark / &light 指定（可与定位组合，如 #final&light）。
  var h = (window.location.hash || '').slice(1);
  var parts = h.split('&');
  parts.forEach(function (p) {
    if (p === 'dark' || p === 'light') root.setAttribute('data-theme', p);
  });
  var loc = parts[0];
  if (loc === 'final') setCur(FRAMES.length - 1);
  else if (loc === 'play') togglePlay();
  else if (loc.indexOf('frame=') === 0) setCur(parseInt(loc.slice(6), 10) - 1);
  else setCur(-1);
})();
</script>
</body>
</html>`
