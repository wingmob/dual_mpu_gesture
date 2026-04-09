# Dual MPU6050 Gesture Recognition Skeleton

- `Arduino Mega 2560` 负责双传感器采集与串口发送
- `Python` 上位机负责记录数据、训练模型、实时分段与识别
- 方案聚焦 `动态手势`，不是静态手型

## 方案

- 双 `MPU6050` 分别固定在 `手背` 和 `前臂`
- 使用 `相对姿态` 与 `相对角速度`，降低整条手臂平移带来的干扰
- 用 `RandomForest` 做首版分类，速度快、调参简单
- 自带训练、实时识别、数据采集骨架，能快速出准确率和演示效果

## 目录结构

```text
dual_mpu_gesture/
├─ arduino/
│  └─ dual_mpu_gesture/
│     └─ dual_mpu_gesture.ino
├─ docs/
│  └─ gesture_protocol.md
├─ python/
│  ├─ capture_labeled.py
│  ├─ capture_stream.py
│  ├─ features.py
│  ├─ pose.py
│  ├─ protocol.py
│  ├─ realtime_demo.py
│  └─ train_model.py
├─ data/
│  └─ raw/
├─ models/
└─ requirements.txt
```

## 硬件接线

两个模块都接到 `Arduino Mega 2560` 的同一条 I2C 总线：

- `VCC -> 5V` 或 `3.3V`
  - 常见 `GY-521` 可接 `5V`
  - 如果你的模块只支持 `3.3V`，按模块规格走
- `GND -> GND`
- `SDA -> 20`
- `SCL -> 21`

地址区分：

- 手背传感器 `AD0 -> GND`，地址 `0x68`
- 前臂传感器 `AD0 -> VCC`，地址 `0x69`

建议两个传感器安装时 `坐标方向保持一致`，比如箭头都指向手指方向。

## 串口协议

固件以 `230400` 波特率输出 CSV。

注释行以 `#` 开头，数据表头固定为：

```text
ts_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2
```

含义：

- `1` 号传感器：手背
- `2` 号传感器：前臂
- `a*`：加速度原始计数
- `g*`：角速度原始计数

## 手势

- `flexion`：屈腕，手掌向下压
- `extension`：背伸，手背向上抬
- `radial_deviation`：向拇指侧偏
- `ulnar_deviation`：向小指侧偏
- `pronation`：旋前，向内旋
- `supination`：旋后，向外旋

如果时间紧，可以先做前 4 个。

## 快速开始

1. 烧录 Arduino 固件  
   打开 `arduino/dual_mpu_gesture/dual_mpu_gesture.ino`

2. 安装 Python 依赖

```powershell
python -m pip install -r requirements.txt
```

3. 连续查看原始串口流

```powershell
python .\python\capture_stream.py --port COM5
```

4. 采集带标签数据

```powershell
python .\python\capture_labeled.py --port COM5 --label flexion --trials 20
python .\python\capture_labeled.py --port COM5 --label extension --trials 20
```

5. 训练模型

```powershell
python .\python\train_model.py --data-dir .\data\raw --model-out .\models\gesture_model.joblib
```

6. 实时识别

```powershell
python .\python\realtime_demo.py --port COM5 --model .\models\gesture_model.joblib --plot
```


