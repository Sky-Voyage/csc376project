# Assignment 4：Panda 搭桥仿真

当前默认采用竖柱支撑塔：四块小木块竖放成两组立柱，另外两块横放为顶梁，最后将两块桥面上下叠放、跨接两座塔。保留八次抓放，增加水平抓取后的翻转、立柱定位、顶梁对齐和桥面精确叠放。两侧塔底均为 125 mm 高的立方体，桥面保持水平。

原来的单层垫块方案保留在 `scene.flat.json`，可用 `--config assignment4/scene.flat.json` 运行。

## 运行

在仓库根目录使用本次已配置好的独立环境：

```powershell
.venv-assignment4/Scripts/python.exe assignment4/bridge.py
```

Swift 会打开浏览器。点击**侧栏的 Play / Pause** 播放或暂停；**Reset** 回到初始场景。可以调速，也可以拖动 **Seek to time** 跳转并暂停。滑块只负责输入，当前播放时间显示在状态文字中。页面顶部 Swift 自带的暂停按钮控制整个仿真循环；如果顶部显示 ▶，请先恢复该循环。关闭页面即可结束进程，也可以在终端按 Ctrl+C。

只检查轨迹，或在无浏览器环境完整回放：

```powershell
.venv-assignment4/Scripts/python.exe assignment4/bridge.py --check
.venv-assignment4/Scripts/python.exe assignment4/bridge.py --headless
.venv-assignment4/Scripts/python.exe -m pytest assignment4/test_bridge.py -q
```

在已经安装本项目及 Swift 的 devcontainer 中，将命令开头替换成 `python` 即可。

新环境可在仓库根目录安装：

```powershell
uv venv .venv-assignment4 --python 3.12
$env:RTB_PURE_WHEEL = '1'
uv pip install --python .venv-assignment4/Scripts/python.exe -e ./rtb-data -e '.[swift]' pytest
```

这里使用工具箱自带的纯 Python 回退实现，无需在 Windows 上配置 C++ 编译器。不要使用已指向旧仓库路径、或存在 NumPy/SciPy 二进制冲突的系统环境。

## 实测尺寸与待确认项

**当前 `scene.json` 是可运行的布局预览，不是实验室标定结果。页面会显示 EXAMPLE 提示。**

已采用两侧支撑 125 × 125 × 125 mm、两块桥面底面 100 × 50 mm。小木块暂按 7.4 × 2.3 × 1.3 **cm** 换算为 74 × 23 × 13 mm，单位仍需确认。桥面厚度暂用 **18 mm 占位值**，须换成实测厚度后重新规划。

默认竖柱塔的高度为：125 mm 底座 + 74 mm 立柱 + 13 mm 顶梁 = 212 mm。下层桥面底部位于 z=212 mm；上层桥面叠在下层上，总高度为 `212 + 2 × 桥面厚度` mm。18 mm 占位厚度下总高为 248 mm。上下两块桥面共用同一平面投影，不再左右并排。

每座塔的两根立柱底面为 23 × 13 mm，高为 74 mm；底面 23 mm 边沿 x，13 mm 边沿 y。两柱中心沿 y 相隔 44 mm，柱间净空为 31 mm，横向总宽 57 mm。74 mm 长的顶梁沿 y 横跨两柱，顶梁在 x 方向宽 23 mm。

两侧立方体内边缘净距为 60 mm。立柱、顶梁沿 x 的中心偏移为 ±41.5 mm；100 mm 长的桥面在每侧顶梁上搭接 20 mm。桥面宽 50 mm，覆盖两根柱子的中心线，且小于顶梁长 74 mm。

## 放置顺序与精度要求

1. 同一座塔先放后柱，再放前柱，避免前柱挡住水平接近路径；先完成两座塔的四根柱。
2. 供料区的小木块均平躺。夹爪在离木块中心沿长边 20 mm 处抓取，使竖放后的抓取点位于柱顶以下 17 mm，减小手掌与底座干涉的风险。
3. 将木块提升至转运高度，再使用关节轨迹翻成立柱。目标姿态的夹爪轴水平朝 +y，从塔前方 -y 接近。
4. 在塔前 80 mm 处下降至放置高度，再水平接近；最后 12 mm 使用较慢的精确放置段。松爪后向 -y 水平退回，再抬升。
5. 两根顶梁依次从上方放置，随后依次放置下层、上层桥面，机器人撤离并保留十秒观察。

转运参考速度为 200 mm/s，最后放置段不超过 25 mm/s，并按关节速度限值进一步延长时间。抓取余量暂为 6 mm；夹爪指尖尺寸、接触面和平整度需实测。立柱是需要实物验证的精度任务：程序只检查几何与运动，不保证木块能抗扰动站稳。

## 配置

复制 `scene.json` 为自己的配置，再用 `--config assignment4/scene.lab.json` 加载。完成实测标定后将 `example_dimensions` 设为 `false`。

| 字段 | 含义 |
| --- | --- |
| `layout` | 默认 `bank_towers` 竖柱塔；`bank_pads` 单层垫块；`cross_stack` 原中央三层方案 |
| `pier_block_mm` | 小木块的长、宽、高，六块相同 |
| `deck_block_mm` | 桥面块的长、宽、高，两块相同 |
| `pier_centre_mm` | 整体桥梁中心的 x、y |
| `pad_spacing_mm` | 平桥方案同侧相邻垫块的中心距 |
| `tower_column_spacing_mm` | 竖柱塔的前后两柱中心距 |
| `tower_grasp_offset_mm` | 立柱抓取点沿木块长边偏离中心的距离 |
| `tower_standoff_mm` | 立柱水平接近和退出距离 |
| `pier_gap_mm` | 仅原方案使用：同层两块小木块之间的净间距 |
| `bank_size_mm` | 每侧河岸沿 x、y 的尺寸 |
| `bank_height_mm` | 河岸高度；仅原方案要求与三层桥墩总高一致 |
| `bank_inner_gap_mm` | 两侧河岸内边缘间的净距离 |
| `deck_joint_gap_mm` | 仅非竖柱塔布局使用；竖柱塔的两块桥面上下叠放 |
| `supply_xy_mm` | 八个供料位的中心坐标，前六块桥墩、后两块桥面 |
| `safe_tcp_z_mm` | 转运阶段 TCP 高度 |
| `linear_speed_mm_s` | 规划时参考平移速度 |
| `angular_speed_deg_s` | 规划时参考旋转速度 |
| `gripper_opening_mm` | 夹爪最大开度，不超过 80 mm |
| `gripper_margin_mm` | 供料抓取时，开度相对物块宽度的余量 |
| `observe_seconds` | 撤离后的观察阶段，至少十秒 |

坐标系与 Panda 底座一致：z=0 为工作面，x 沿桥梁长度，y 为横向。供料块长边沿 x；立柱长边转到 z 方向；顶梁长边转到 y 方向；桥面长边保持沿 x。配置中的长度为 mm，程序内部统一转换为 m，角度使用 rad。供料位、河岸和木块尺寸改变后，需要重新运行轨迹检查。

## 实现与边界

- 使用 `rtb.models.Panda()`、`fkine()`、`ikine_LM()`、`jtraj()`、`ctraj()` 和 SpatialMath 的 `SE3`。
- 初始接近、立柱翻转转运和最终撤离使用关节轨迹；其余接近、提升、对齐、放置与退回使用笛卡尔插值加连续逆运动学。关节插值中的抓持物轨迹也经过包围盒检查。
- 求解失败、逆运动学分支跳变、关节限位、夹爪开度不合适、供料重叠、结构几何不匹配或总时长超过五分钟时停止规划并报错。
- 通过 Swift 的机器人 handle 驱动关节，物块被抓住后按 TCP 的相对位姿移动，释放后保留在目标位置。
- 每次下降前将夹爪开度调为物块宽度加 6 mm 余量，避免用全开夹爪挤入并排垫块之间。
- 对抓持物块在轨迹采样点和中点的包围盒，检查工作面、河岸、其他物块之间的穿透；这不是整条机械臂的碰撞检测。
- 回放速度只影响显示；不能用加速播放后的墙钟时间作为任务时长。
- 这是运动学排练：没有刚体动力学、摩擦、力控或接触求解。安全高度并不等于整条机械臂经过完整碰撞检测。十秒观察仅是时间轴阶段，不是稳定性证明。
- 不包含向真实 Panda 发送指令的接口。实际工作台坐标、TCP、夹爪指尖形状、碰撞与承重需另行标定和验证。
