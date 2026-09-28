// Command rrtstar-demo 运行一次固定种子的 RRT* 规划，生成单一静态
// report.html（数据与逻辑全部内嵌，无任何第三方库、无需启动服务），
// 在浏览器打开即可逐步回放树的生长与重布线事件。
package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"time"

	"github.com/TommyMarsss/go-rrt"
)

func main() {
	out := flag.String("out", "report.html", "输出 HTML 文件路径")
	iter := flag.Int("iter", 2500, "采样次数上限")
	seed := flag.Int64("seed", 20260928, "随机种子")
	flag.Parse()

	cfg := rrtstar.Config{
		Bounds: rrtstar.Rect{Min: rrtstar.Vec2{X: 0, Y: 0}, Max: rrtstar.Vec2{X: 100, Y: 100}},
		// 两道墙，中间留出可以绕行的通道。
		Obstacles: []rrtstar.Rect{
			{Min: rrtstar.Vec2{X: 30, Y: 0}, Max: rrtstar.Vec2{X: 35, Y: 58}},
			{Min: rrtstar.Vec2{X: 30, Y: 70}, Max: rrtstar.Vec2{X: 35, Y: 100}},
			{Min: rrtstar.Vec2{X: 62, Y: 34}, Max: rrtstar.Vec2{X: 67, Y: 100}},
		},
		Start:       rrtstar.Vec2{X: 8, Y: 50},
		Goal:        rrtstar.Vec2{X: 92, Y: 18},
		GoalRadius:  3,
		StepSize:    5,
		GoalBias:    0.12,
		RadiusGamma: 20,
		RadiusMin:   4.5,
		RadiusMax:   18,
		MaxIter:     *iter,
		Seed:        *seed,
	}

	t0 := time.Now()
	res, err := rrtstar.Run(cfg)
	if err != nil {
		fmt.Fprintln(os.Stderr, "规划失败:", err)
		os.Exit(1)
	}
	elapsed := time.Since(t0)

	data, err := json.Marshal(res)
	if err != nil {
		fmt.Fprintln(os.Stderr, "序列化失败:", err)
		os.Exit(1)
	}

	html := reportHTML(data)
	if err := os.WriteFile(*out, []byte(html), 0o644); err != nil {
		fmt.Fprintln(os.Stderr, "写文件失败:", err)
		os.Exit(1)
	}

	nRewire := 0
	for _, f := range res.Frames {
		for _, ch := range f.Changes {
			if ch.Kind == rrtstar.ChangeRewire {
				nRewire++
			}
		}
	}
	fmt.Printf("已生成 %s\n", *out)
	fmt.Printf("迭代=%d 节点=%d 重布线=%d 到达=%v 最优代价=%.4f 代价回升=%d 耗时=%s\n",
		*iter, len(res.NodeJSON), nRewire, res.Reached, res.BestCost, len(res.Regressions), elapsed)
}
