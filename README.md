# MUbea-Coiling-spring

## TCP 允许工作区域

按 `坐标/` 中的四个记录点构建侧边界，Z 独立采用用户确认的底座坐标范围：

- 坐标系：底座 `base`。
- 位置单位：毫米。
- 允许范围：P1 → P2 → P3 → P4 围成的四边形内部。
- 高度：`0 <= Z <= 350 mm`，下限是底座 Z=0 平面。
- 四个记录点原本的 Z 不作为底面高度，不拟合倾斜底面。

| 点 | X / mm | Y / mm |
| --- | ---: | ---: |
| P1 | 153.3 | -654.5 |
| P2 | -434.2 | -664.4 |
| P3 | -448.2 | 185.4 |
| P4 | 175.9 | 209.3 |

区域是有斜侧边的立柱，不是按 X/Y 最大最小值扩大的矩形。

### 文件及检查

`safety_zone.json` 保存区域配置；`safety_zone.py` 提供离线点位/直线路径检查，只使用 Python 标准库，不连接机器人。

查看六个边界平面的数学定义：

```powershell
python safety_zone.py
```

检查一个点，输入单位毫米：

```powershell
python safety_zone.py --xyz-mm 24.259 -555.011 67.030
```

检查之前读取程序产生的 A/B 文件及默认抬高 100 mm 的上方点：

```powershell
python safety_zone.py --points-file "完整路径/ur5_points.json" --above-mm 100
```

检查不通过返回非零退出码。文件坐标系和单位必须明确匹配，禁止静默混用米和毫米。

### 在运动程序中使用

将本模块和配置放入可导入的同一项目中，在发送无圆滑过渡的 `moveL` 前检查：

```python
from safety_zone import SafetyZone

zone = SafetyZone.load()
current = receiver.getActualTCPPose()
# UR/RTDE 位姿单位是米、弧度旋转向量，模块会转换为毫米。
zone.require_path_m(current, target_pose, "moveL")
# 仅在检查没有抛出异常时，才允许原运动程序发送该段指令。
```

这只是接入示例。此前的 UR5 运动程序还未导入本模块，不能把生成配置当作原程序已经获得区域限制。也不应直接调用此接口验证 `moveJ`：它会拒绝，避免用起终点错误推断关节运动的中间 TCP 路径。

四边形柱体为凸区域，两端在内可证明理想 TCP 直线位置轨迹在内，但这不包含机器人跟踪误差、停止距离、工具旋转扫掠或其他连杆碰撞。配置中的 `tcp_margin_mm` 当前为 0，忠实保留用户指定的几何边界；它不代表已经预留足够的物理安全余量。需要根据现场工具/工件外伸、速度及停止特性确定内缩值，并重新验证。

### 控制柜安全设置

本仓库没有写入控制器安全配置；下面的位置检测程序也不提供运动拦截。

### 启动位置检测测试

`test_zone_live.py` 只使用 RTDE 接收接口。它不上传脚本，不发运动或停止指令，
也不拦截其他程序或示教器的动作；只能用于观察 TCP 是否处于配置区域内。

使用 Python 3.12，实时检测需要安装 `ur_rtde`：

```powershell
python -m pip install ur_rtde==1.6.5
```

Windows 可双击 `start_monitor.cmd` 启动检测，默认 IP 为 `192.168.1.10`。

先模拟内部点及六个面各自外侧的点，不连接机器人：

```powershell
python test_zone_live.py --self-test
```

连接机器人实时显示位置与最近边界内距，按 Ctrl+C 退出：

```powershell
python test_zone_live.py --ip 192.168.1.10
```

也可使用 `--duration 10` 采集约十秒。`INSIDE` 表示采样点满足区域配置，
`OUTSIDE` 表示采样点越界；负内距表示超出边界。网络断开、时间戳倒退或
一秒未更新时以 `UNKNOWN` 报错退出，不能继续把旧位置视为有效检测。
不要为验证程序而让真实机械臂越界，用 `--self-test` 验证越界判定即可。
有限时长运行的退出码：0 表示所采集点均在区域内，2 表示曾检测越界，
1 表示连接或检测失败，130 表示用户中断。

检测结果不能证明控制柜安全平面已启用，也不能证明全机械臂均在区域内。

### 控制柜安全边界配置

CB3 示教器设置入口为 `Installation > Safety > Boundaries`。用对应特征建立四个侧面，以及 Z=0、Z=350 两个水平面，形成六个限制面；每个面都应选择阻止越界的限制模式，并在示教器中确认允许侧指向区域内部，不能仅配置为触发减速模式。

运行 `python safety_zone.py` 得到的六个平面均使用单位内法向量，数学允许侧为 `n·[X,Y,Z] >= d`。这不是可直接填写到示教器的位姿值，须按现场特征坐标转换并核对预览箭头。由现场负责安全配置的人员结合风险评估、控制器容差和停止距离确认，应用并验证六个面。

CB3 安全平面主要限制 TCP，并不保证整条机械臂或工具外形都在此柱体内。手动拖动和其他程序也不能依赖本 Python 模块获得保护。原点只有关节角记录，尚无此区域下验证过的原点 TCP 与关节路径，不能把之前 A/B 程序中的回原点段视为已验证。

官方参考：

- [CB3 安全边界配置示例](https://www.universal-robots.com/articles/ur/safety/setting-up-safety-boundaries/)
- [UR5 CB3 用户手册](https://s3-eu-west-1.amazonaws.com/ur-support-site/61883/99202_UR5_User_Manual_en_Global.pdf)

### 离线测试

```powershell
python -m unittest discover -s tests -v
```

测试边界内外、Z 上下限、斜边、内缩、输入单位、非法配置，以及拒绝把关节路径当作直线验证。不连接或移动机械臂。
