# 基于高云 FPGA 的旋转倒立摆姿态控制系统

GW5AT-138（小梅哥 BX72 开发板）实现的旋转倒立摆。角度采集、编码器解码、串级控制、
自动起摆、PWM 驱动全部在 FPGA 内部完成，不使用 MCU。

## 功能

| 功能 | 说明 |
|---|---|
| 自动起摆 | 摆杆从下垂自动摆到竖直，用时 4~8 s |
| 自平衡 | 200 Hz 角度环 + 20 Hz 位置环，平衡帧占比 90%~96% |
| 位置保持 | 摆杆保持倒立的同时，旋臂回到设定位置 |

## 目录结构

```
pendulum_fpga/
├── README.md
├── project.json              工程配置（编译文件列表、器件、顶层名）
├── GIT_VERSION.txt           版本记录
├── bitstream/
│   └── pendulum_final.fs     比特流，可直接下载
├── src/                      设计源码
│   ├── pendulum_top.v        顶层：时钟、模块例化、指令路由、遥测组帧
│   ├── balance_ctrl.v        串级控制器：角度环 + 位置环
│   ├── swing_kick.v          起摆状态机：极值检测 + 双脉冲
│   ├── motor_driver.v        PWM 生成 + 死区补偿 + 方向控制
│   ├── uart_cmd_parser.v     串口指令解析
│   ├── firewater_tx8.v       100 Hz 遥测发送
│   └── legacy/               早期版本，不参与编译
├── deps/                     依赖模块
│   ├── ads8361_reader4.v     ADS8361 串行采集时序
│   ├── balance_core.v        quadrature_encoder（四倍频解码）等
│   └── vofa_firewater.v      UART 发送字节模块
├── cst/
│   └── pendulum_bx72.cst     引脚约束
└── tools/
    └── readback_params.py    串口参数回读脚本
```

## 构建

工程共 9 个源文件参与综合，列在 `project.json` 的 `files` 中：

```
src/pendulum_top.v          （顶层）
src/balance_ctrl.v
src/swing_kick.v
src/motor_driver.v
src/uart_cmd_parser.v
src/firewater_tx8.v
deps/ads8361_reader4.v
deps/balance_core.v
deps/vofa_firewater.v
```

用 Gowin 云源软件（V1.9.12 及以上）操作：

1. 新建工程，器件选 **GW5AT-LV138PG484AC1/I0**（器件族 GW5AT-138B）
2. 将上表 9 个文件加入工程
3. 顶层模块设为 `pendulum_top`
4. 物理约束选 `cst/pendulum_bx72.cst`
5. 综合 → 布局布线 → 生成比特流

## 运行

1. 下载 `bitstream/pendulum_final.fs`（SRAM Program）
2. 摆杆自然下垂
3. 按 KEY2 启动，再按一次停止

全部参数已固化在比特流中，上电即生效，无需下发串口指令。

> SRAM 掉电丢失，断电后需重新下载。

## 引脚

| 功能 | 引脚 |
|---|---|
| 时钟 50 MHz | Y18 |
| KEY2（启停） | V17 |
| ADS8361 M0 / M1 / A0 | K13 / K14 / H15 |
| ADS8361 CONVST / CS / CLK | J15 / G15 / H14 |
| ADS8361 BUSY / DOUT | J14 / H13 |
| 电机 PWM / IN1 / IN2 / STBY | N19 / K16 / L18 / M16 |
| 编码器 A / B | L13 / L15 |
| 串口 TX / RX | N14 / N13 |

## 遥测

COM13，115200 8N1，FireWater 协议，100 Hz，8 通道。

| 通道 | 含义 |
|---|---|
| ch0 | 摆杆角度（0~4095，竖直 ≈ 2039） |
| ch1 | 电机指令（−100~+100） |
| ch2 | 编码器速度（计数/10 ms） |
| ch3 | 零点下限 |
| ch4 | 旋臂行程（编码器计数） |
| ch5 | Ki × 10000 |
| ch6 | Kd × 10000 |
| ch7 | 状态位：bit0 使能 / bit1 方向 / bit2 在区间 / bit3 超速 / bit4 起摆中 |

发 `tel=1;` 切换为寄存器回读模式，可核对全部参数的实际值；`tel=0;` 恢复实时遥测。
