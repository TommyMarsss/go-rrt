package rrtstar

import (
	"encoding/json"
	"fmt"
	"html/template"
	"os"
	"strings"
)

// ---- JSON payload types ---------------------------------------------------

type addRec struct {
	ID      int     `json:"id"`
	X       float64 `json:"x"`
	Y       float64 `json:"y"`
	Parent  int     `json:"parent"`
	Cost    float64 `json:"cost"`
	SampleX float64 `json:"sx"`
	SampleY float64 `json:"sy"`
}

type rewireRec struct {
	Node      int     `json:"node"`
	OldParent int     `json:"oldParent"`
	NewParent int     `json:"newParent"`
	OldCost   float64 `json:"oldCost"`
	NewCost   float64 `json:"newCost"`
}

type frameRec struct {
	Iter      int         `json:"iter"`
	Adds      []addRec    `json:"adds"`
	Rewires   []rewireRec `json:"rewires"`
	GoalAdded bool        `json:"goalAdded"`
	Best      *float64    `json:"best"` // null until the goal is first reached
}

type reportData struct {
	Width      float64    `json:"width"`
	Height     float64    `json:"height"`
	Start      Vec2       `json:"start"`
	Goal       Vec2       `json:"goal"`
	GoalNodeID int        `json:"goalNodeId"`
	Obstacles  []Polygon  `json:"obstacles"`
	StepSize   float64    `json:"stepSize"`
	Gamma      float64    `json:"gamma"`
	Eta        float64    `json:"eta"`
	Seed       int64      `json:"seed"`
	MaxSamples int        `json:"maxSamples"`
	BestCost   *float64   `json:"bestCost"`
	Monotonic  bool       `json:"monotonic"`
	RewireCnt  int        `json:"rewireCount"`
	Frames     []frameRec `json:"frames"`
}

func fptr(f float64) *float64 { return &f }

// buildReportData collapses the flat event log into per-iteration frames.
func buildReportData(r *Result) reportData {
	d := reportData{
		Width:      r.Width,
		Height:     r.Height,
		Start:      r.Start,
		Goal:       r.Goal,
		GoalNodeID: r.GoalNodeID,
		Obstacles:  r.Obstacles,
		StepSize:   r.Options.StepSize,
		Gamma:      r.Options.Gamma,
		Eta:        r.Options.Eta,
		Seed:       r.Options.Seed,
		MaxSamples: r.Options.MaxSamples,
		Monotonic:  r.VerifyMonotonic() < 0,
		Frames:     []frameRec{},
	}
	if r.Reached {
		d.BestCost = fptr(r.BestCost)
	}

	var best *float64
	var cur *frameRec

	flush := func() {
		if cur != nil {
			d.Frames = append(d.Frames, *cur)
		}
	}

	for _, e := range r.Events {
		if cur == nil || cur.Iter != e.Iter {
			flush()
			cur = &frameRec{Iter: e.Iter, Adds: []addRec{}, Rewires: []rewireRec{}}
		}
		switch e.Kind {
		case EventAdd:
			n := r.Nodes[e.Node]
			cur.Adds = append(cur.Adds, addRec{
				ID: e.Node, X: n.X, Y: n.Y,
				Parent: e.NewParent, Cost: e.NewCost,
				SampleX: e.SampleX, SampleY: e.SampleY,
			})
			if e.Node == r.GoalNodeID {
				cur.GoalAdded = true
			}
		case EventRewire:
			d.RewireCnt++
			cur.Rewires = append(cur.Rewires, rewireRec{
				Node:      e.Node,
				OldParent: e.OldParent,
				NewParent: e.NewParent,
				OldCost:   e.OldCost,
				NewCost:   e.NewCost,
			})
		case EventGoal:
			b := e.Best
			best = &b
		}
		cur.Best = best
	}
	flush()
	return d
}

// reportTemplate is a fully self-contained page: no external scripts, styles,
// fonts or libraries. It reconstructs tree state in the browser by replaying
// the per-iteration frames, supports forward/backward scrubbing and
// highlights every rewiring event.
const reportTemplate = `<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RRT* 回放 — seed {{.Data.Seed}}</title>
<style>
  :root {
    --bg: #0f1420; --panel: #171e2e; --ink: #e6ebf5; --muted: #8b96ad;
    --tree: #3b82f6; --newnode: #22c55e; --rewire: #f59e0b;
    --oldedge: #ef4444; --goal: #a855f7; --obst: #334155; --obst-edge: #64748b;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0; background: var(--bg); color: var(--ink);
    font: 14px/1.5 -apple-system, "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
  }
  header { padding: 14px 20px 6px; }
  header h1 { font-size: 18px; margin: 0 0 4px; }
  header .meta { color: var(--muted); font-size: 12px; }
  .layout { display: flex; flex-wrap: wrap; gap: 16px; padding: 12px 20px 24px; }
  .stage {
    flex: 1 1 720px; min-width: 320px; background: var(--panel);
    border: 1px solid #232c42; border-radius: 10px; padding: 8px;
  }
  canvas { width: 100%; display: block; border-radius: 6px; background: #0b1020; }
  .side { flex: 0 0 300px; display: flex; flex-direction: column; gap: 12px; }
  .card {
    background: var(--panel); border: 1px solid #232c42;
    border-radius: 10px; padding: 12px 14px;
  }
  .card h2 { font-size: 13px; margin: 0 0 8px; color: var(--muted);
    text-transform: uppercase; letter-spacing: .06em; }
  .row { display: flex; justify-content: space-between; gap: 8px;
    padding: 2px 0; font-variant-numeric: tabular-nums; }
  .row span:last-child { text-align: right; }
  .ok { color: var(--newnode); }
  .bad { color: var(--oldedge); font-weight: 700; }
  .controls { display: flex; flex-wrap: wrap; gap: 6px; align-items: center; }
  button {
    background: #23304d; color: var(--ink); border: 1px solid #33415f;
    border-radius: 6px; padding: 6px 10px; cursor: pointer; font-size: 13px;
  }
  button:hover { background: #2c3c60; }
  button:disabled { opacity: .4; cursor: default; }
  input[type=range] { flex: 1 1 160px; accent-color: var(--tree); }
  input[type=range]:nth-of-type(2) { accent-color: var(--rewire); }
  .legend { display: grid; grid-template-columns: 14px 1fr; gap: 4px 8px;
    align-items: center; font-size: 12px; color: var(--muted); }
  .swatch { width: 14px; height: 3px; border-radius: 2px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; }
  #eventlog {
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
    font-size: 12px; max-height: 170px; overflow: auto;
    background: #0b1020; border-radius: 6px; padding: 8px; white-space: pre-wrap;
  }
  .pill { display: inline-block; padding: 1px 7px; border-radius: 999px;
    font-size: 11px; font-weight: 600; }
  .pill.add { background: rgba(34,197,94,.18); color: var(--newnode); }
  .pill.rew { background: rgba(245,158,11,.18); color: var(--rewire); }
  .pill.goal { background: rgba(168,85,247,.2); color: var(--goal); }
</style>
</head>
<body>
<header>
  <h1>RRT* 树生长与重布线回放</h1>
  <div class="meta">
    seed={{.Data.Seed}} · 采样上限={{.Data.MaxSamples}} ·
    场域={{.Data.Width}}×{{.Data.Height}} · η={{.Data.Eta}} ·
    γ={{.Data.Gamma}} · 重布线事件总数={{.Data.RewireCnt}} ·
    代价单调性：
    {{if .Data.Monotonic}}<b class="ok">通过（非递增）</b>{{else}}<b class="bad">异常：检测到代价回升！</b>{{end}}
  </div>
</header>

<div class="layout">
  <div class="stage">
    <canvas id="cv" width="1200" height="760"></canvas>
  </div>

  <div class="side">
    <div class="card">
      <h2>回放控制</h2>
      <div class="controls">
        <button id="btnFirst" title="回到起点">⏮</button>
        <button id="btnPrev" title="上一帧">◀</button>
        <button id="btnPlay" title="播放/暂停">▶</button>
        <button id="btnNext" title="下一帧">▶|</button>
      </div>
      <div style="margin-top:8px">
        <div class="row"><span>帧（采样迭代）</span><span id="frameTxt"></span></div>
        <input type="range" id="scrub" min="0" max="1" step="1" value="0">
      </div>
      <div style="margin-top:6px">
        <div class="row"><span>播放速度</span><span id="speedTxt"></span></div>
        <input type="range" id="speed" min="0" max="8" step="1" value="4">
      </div>
    </div>

    <div class="card">
      <h2>当前状态</h2>
      <div class="row"><span>已插入节点</span><span id="statNodes">0</span></div>
      <div class="row"><span>累计重布线</span><span id="statRew">0</span></div>
      <div class="row"><span>本帧新增</span><span id="statAdd">0</span></div>
      <div class="row"><span>本帧重布线</span><span id="statFrameRew">0</span></div>
      <div class="row"><span>当前最优代价</span><span id="statBest">∞</span></div>
      <div class="row"><span>最终最优代价</span><span id="statFinal"></span></div>
    </div>

    <div class="card">
      <h2>图例</h2>
      <div class="legend">
        <div class="swatch" style="background:var(--tree)"></div><div>树边（parent → child）</div>
        <div class="dot" style="background:var(--newnode)"></div><div>本帧新插入节点</div>
        <div class="dot" style="background:var(--rewire)"></div><div>重布线高亮节点（代价下降）</div>
        <div class="swatch" style="background:var(--oldedge);height:2px;border-top:1px dashed var(--oldedge);background:none"></div><div>被替换的旧父边</div>
        <div class="swatch" style="background:var(--rewire);height:4px"></div><div>重布线后的新父边</div>
        <div class="dot" style="background:var(--goal)"></div><div>当前最优路径</div>
      </div>
    </div>

    <div class="card">
      <h2>事件日志</h2>
      <div id="eventlog"></div>
    </div>
  </div>
</div>

<script id="rrt-data" type="application/json">{{.JSON}}</script>
<script>
(function () {
  "use strict";
  var DATA = JSON.parse(document.getElementById("rrt-data").textContent);
  var cv = document.getElementById("cv"), ctx = cv.getContext("2d");

  // ---- world -> screen mapping (uniform scale, letterboxed) ----
  var PAD = 30;
  var scale = Math.min((cv.width - 2*PAD) / DATA.width, (cv.height - 2*PAD) / DATA.height);
  function X(x) { return PAD + x * scale; }
  function Y(y) { return cv.height - PAD - y * scale; }

  // ---- mutable tree state, rebuilt per gotoFrame ----
  var nodes = {};        // id -> {x,y,cost,parent,children:[]}
  var goalID = DATA.goalNodeId;
  var curFrame = -1;     // -1 = empty (root only)
  var rewireCount = 0;

  function resetState() {
    nodes = {};
    nodes[0] = {x: DATA.start.x, y: DATA.start.y, cost: 0, parent: -1, children: []};
    rewireCount = 0;
  }
  function applyAdd(a) {
    nodes[a.id] = {x: a.x, y: a.y, cost: a.cost, parent: a.parent, children: []};
    nodes[a.parent].children.push(a.id);
  }
  function applyRewire(r) {
    var n = nodes[r.node];
    var delta = r.newCost - r.oldCost; // < 0
    // detach from old parent
    var op = nodes[r.oldParent].children;
    op.splice(op.indexOf(r.node), 1);
    n.parent = r.newParent;
    n.cost = r.newCost;
    nodes[r.newParent].children.push(r.node);
    // cascade delta through the subtree, mirroring Planner.addCost in Go
    var st = n.children.slice();
    while (st.length) {
      var cid = st.pop(), c = nodes[cid];
      c.cost += delta;
      st = st.concat(c.children);
    }
    rewireCount++;
  }
  function gotoFrame(k) {
    resetState();
    for (var i = 0; i <= k && i < DATA.frames.length; i++) {
      var f = DATA.frames[i];
      f.adds.forEach(applyAdd);
      f.rewires.forEach(applyRewire);
    }
    curFrame = k;
    draw();
    updateUI();
  }

  // ---- rendering ----
  function nodeCount() { return Object.keys(nodes).length; }

  function drawObstacles() {
    DATA.obstacles.forEach(function (o) {
      ctx.beginPath();
      o.vertices.forEach(function (v, i) {
        if (i === 0) ctx.moveTo(X(v.x), Y(v.y)); else ctx.lineTo(X(v.x), Y(v.y));
      });
      ctx.closePath();
      ctx.fillStyle = "#1e293f";
      ctx.strokeStyle = "#64748b";
      ctx.lineWidth = 1.5;
      ctx.fill();
      ctx.stroke();
    });
  }

  function line(a, b) { ctx.moveTo(X(a.x), Y(a.y)); ctx.lineTo(X(b.x), Y(b.y)); }

  function bestPathIDs() {
    if (goalID < 0 || !nodes[goalID]) return null;
    var ids = [], id = goalID, guard = 0;
    while (id !== -1 && nodes[id] && guard++ < 100000) { ids.push(id); id = nodes[id].parent; }
    return ids;
  }

  function draw(frame) {
    ctx.clearRect(0, 0, cv.width, cv.height);

    // subtle grid
    ctx.strokeStyle = "rgba(148,163,184,0.07)";
    ctx.lineWidth = 1;
    var grid = 100 * scale;
    ctx.beginPath();
    for (var gx = PAD; gx < cv.width - PAD; gx += grid) { ctx.moveTo(gx, PAD); ctx.lineTo(gx, cv.height - PAD); }
    for (var gy = PAD; gy < cv.height - PAD; gy += grid) { ctx.moveTo(PAD, gy); ctx.lineTo(cv.width - PAD, gy); }
    ctx.stroke();

    // border
    ctx.strokeStyle = "#334155";
    ctx.strokeRect(PAD, PAD, DATA.width * scale, DATA.height * scale);

    drawObstacles();

    var hi = frame ? {adds:{}, rews:{}} : null;
    var oldEdges = [];
    if (frame) {
      frame.adds.forEach(function (a) { hi.adds[a.id] = true; });
      frame.rewires.forEach(function (r) {
        hi.rews[r.node] = true;
        if (nodes[r.oldParent]) oldEdges.push([nodes[r.oldParent], nodes[r.node]]);
      });
    }

    // tree edges
    ctx.strokeStyle = "rgba(59,130,246,0.55)";
    ctx.lineWidth = 1.1;
    ctx.beginPath();
    Object.keys(nodes).forEach(function (k) {
      var n = nodes[k];
      if (n.parent >= 0 && nodes[n.parent]) line(nodes[n.parent], n);
    });
    ctx.stroke();

    // replaced old edges (dashed red)
    if (oldEdges.length) {
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = "rgba(239,68,68,0.9)";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      oldEdges.forEach(function (e) { line(e[0], e[1]); });
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // new rewire edges (bold amber)
    if (frame && frame.rewires.length) {
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      frame.rewires.forEach(function (r) {
        if (nodes[r.newParent]) line(nodes[r.newParent], nodes[r.node]);
      });
      ctx.stroke();
    }

    // best path (purple)
    var path = bestPathIDs();
    if (path && path.length > 1) {
      ctx.strokeStyle = "rgba(168,85,247,0.95)";
      ctx.lineWidth = 3;
      ctx.beginPath();
      for (var i = path.length - 1; i > 0; i--) line(nodes[path[i]], nodes[path[i-1]]);
      ctx.stroke();
    }

    // nodes
    var t = Date.now() / 1000;
    Object.keys(nodes).forEach(function (k) {
      var n = nodes[k], px = X(n.x), py = Y(n.y);
      if (hi && hi.rews[n.id] !== undefined) {
        var pulse = 7 + 2.5 * Math.sin(t * 6);
        ctx.beginPath(); ctx.arc(px, py, pulse + 5, 0, 7);
        ctx.fillStyle = "rgba(245,158,11,0.18)"; ctx.fill();
        ctx.beginPath(); ctx.arc(px, py, pulse, 0, 7);
        ctx.fillStyle = "rgba(245,158,11,0.35)"; ctx.fill();
        ctx.beginPath(); ctx.arc(px, py, 4.2, 0, 7);
        ctx.fillStyle = "#f59e0b"; ctx.fill();
      } else {
        ctx.beginPath(); ctx.arc(px, py, hi && hi.adds[n.id] ? 4 : 2.2, 0, 7);
        ctx.fillStyle = hi && hi.adds[n.id] ? "#22c55e" : "rgba(147,197,253,0.85)";
        ctx.fill();
      }
    });

    // start / goal markers
    marker(DATA.start, "#22c55e", "S");
    marker(DATA.goal, goalID >= 0 && nodes[goalID] ? "#a855f7" : "#94a3b8", "G");

    // sample point(s) for this frame (faint)
    if (frame) frame.adds.forEach(function (a) {
      if (a.sx !== undefined) {
        ctx.beginPath(); ctx.arc(X(a.sx), Y(a.sy), 2.6, 0, 7);
        ctx.strokeStyle = "rgba(230,235,245,0.5)"; ctx.stroke();
      }
    });
  }

  function marker(p, color, label) {
    ctx.beginPath(); ctx.arc(X(p.x), Y(p.y), 7, 0, 7);
    ctx.fillStyle = color; ctx.fill();
    ctx.fillStyle = "#0b1020";
    ctx.font = "bold 9px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(label, X(p.x), Y(p.y) + 0.5);
  }

  // ---- UI ----
  var $ = function (id) { return document.getElementById(id); };
  var scrub = $("scrub"), log = $("eventlog");
  var playing = false, timer = null;
  var speeds = [2000, 1000, 500, 250, 120, 60, 25, 10, 1]; // ms/frame

  function fmt(v) { return v === null || v === undefined ? "∞" : Number(v).toFixed(3); }

  function updateUI() {
    var total = DATA.frames.length;
    var k = curFrame + 1; // 1-based display
    $("frameTxt").textContent = k + " / " + total;
    scrub.max = Math.max(0, total - 1);
    scrub.value = curFrame < 0 ? 0 : curFrame;
    var f = curFrame >= 0 ? DATA.frames[curFrame] : null;
    $("statNodes").textContent = nodeCount();
    $("statRew").textContent = rewireCount;
    $("statAdd").textContent = f ? f.adds.length : 0;
    $("statFrameRew").textContent = f ? f.rewires.length : 0;
    $("statBest").textContent = f && f.best !== null ? fmt(f.best) : "∞";
    $("statFinal").innerHTML = DATA.bestCost === null
      ? "未到达" : fmt(DATA.bestCost) + (DATA.monotonic ? ' <span class="ok">✓</span>' : ' <span class="bad">↑</span>');

    log.innerHTML = f ? frameLog(f) : "";
    log.scrollTop = 0;
    $("btnPrev").disabled = curFrame < 0;
    $("btnFirst").disabled = curFrame < 0;
    $("btnNext").disabled = curFrame >= total - 1;
  }

  function frameLog(f) {
    var s = '<span class="pill goal">iter ' + f.iter + '</span>\n';
    f.adds.forEach(function (a) {
      s += '<span class="pill add">ADD</span> node ' + a.id +
           ' ← parent ' + a.parent + '，cost=' + a.cost.toFixed(3) + '\n';
      if (a.id === goalID) s += '<span class="pill goal">GOAL</span> 首次连接目标\n';
    });
    f.rewires.forEach(function (r) {
      s += '<span class="pill rew">REW</span> node ' + r.node +
           ' 父节点 ' + r.oldParent + '→' + r.newParent +
           '，cost ' + r.oldCost.toFixed(3) + '→' + r.newCost.toFixed(3) +
           ' (Δ=' + (r.newCost - r.oldCost).toFixed(3) + ')\n';
    });
    if (f.best !== null) s += '<span class="pill goal">BEST</span> 当前最优代价 ' + f.best.toFixed(4) + '\n';
    return s || '<span class="pill goal">iter ' + f.iter + '</span>\n（采样被拒绝，无插入）';
  }

  function step(delta) {
    var nxt = Math.max(-1, Math.min(DATA.frames.length - 1, curFrame + delta));
    if (nxt === curFrame) { if (delta > 0) stopPlay(); return; }
    gotoFrame(nxt);
  }

  function stopPlay() {
    playing = false; $("btnPlay").textContent = "▶";
    if (timer) { clearInterval(timer); timer = null; }
  }
  function startPlay() {
    if (curFrame >= DATA.frames.length - 1) gotoFrame(-1);
    playing = true; $("btnPlay").textContent = "⏸";
    var ms = speeds[Number($("speed").value)];
    timer = setInterval(function () {
      if (curFrame >= DATA.frames.length - 1) { stopPlay(); return; }
      step(1);
    }, ms);
  }

  $("btnFirst").onclick = function () { stopPlay(); gotoFrame(-1); };
  $("btnPrev").onclick = function () { stopPlay(); step(-1); };
  $("btnNext").onclick = function () { stopPlay(); step(1); };
  $("btnPlay").onclick = function () { playing ? stopPlay() : startPlay(); };
  scrub.oninput = function () { stopPlay(); gotoFrame(Number(scrub.value)); };
  $("speed").oninput = function () {
    $("speedTxt").textContent = (1000 / speeds[Number($("speed").value)]).toFixed(0) + " 帧/秒";
    if (playing) { stopPlay(); startPlay(); }
  };
  $("speedTxt").textContent = (1000 / speeds[Number($("speed").value)]).toFixed(0) + " 帧/秒";

  document.addEventListener("keydown", function (e) {
    if (e.target.tagName === "INPUT") return;
    if (e.key === "ArrowLeft") { stopPlay(); step(-1); }
    else if (e.key === "ArrowRight") { stopPlay(); step(1); }
    else if (e.key === " ") { e.preventDefault(); playing ? stopPlay() : startPlay(); }
    else if (e.key === "Home") { stopPlay(); gotoFrame(-1); }
    else if (e.key === "End") { stopPlay(); gotoFrame(DATA.frames.length - 1); }
  });

  // pulse animation only while visible; cheap enough to run always
  setInterval(function () { if (curFrame >= 0) draw(DATA.frames[curFrame]); }, 90);

  gotoFrame(DATA.frames.length - 1); // start at final tree
})();
</script>
</body>
</html>`

// WriteHTMLReport runs the planner-independent serialization: it writes a
// single static HTML file embedding all data and replay logic.
func WriteHTMLReport(path string, r *Result) error {
	d := buildReportData(r)
	raw, err := json.Marshal(d)
	if err != nil {
		return fmt.Errorf("rrtstar: marshal report data: %w", err)
	}
	// Prevent </script> breakout inside the JSON payload.
	safe := strings.ReplaceAll(string(raw), "</", "<\\/")

	tmpl, err := template.New("report").Parse(reportTemplate)
	if err != nil {
		return fmt.Errorf("rrtstar: parse report template: %w", err)
	}

	f, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("rrtstar: create %s: %w", path, err)
	}
	defer f.Close()
	if err := tmpl.Execute(f, map[string]any{"Data": d, "JSON": template.JS(safe)}); err != nil {
		return fmt.Errorf("rrtstar: render report: %w", err)
	}
	return nil
}
