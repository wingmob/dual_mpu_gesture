# Dual MPU6050 Gesture Recognition Skeleton

- `Arduino Mega 2560` 负责 `MPU6050` 数据采集与串口发送
- `Python` 上位机负责串口记录、训练模型与实时识别

## 方案概览

本仓库当前包含 `5` 套可直接运行的方案：

1. `双 MPU + 手背/前臂`
   适合 `flexion / extension / radial_deviation / ulnar_deviation / pronation / supination`
2. `双 MPU + 拇指/食指`
   适合 `fist / open_palm / thumb_up / victory / point / ok`，也支持把 `6` 个腕部标签一起放进同一个实验模型
3. `三 MPU + 手背/拇指/食指`
   适合同时利用 `手背参考姿态` 和 `拇指/食指相对运动` 的复合手势实验
4. `单 MPU`
   用于单传感器版本的腕部动态手势验证
5. `双 MPU + 摄像头混合`
   双 `MPU` 负责腕部动态，摄像头负责手指静态手势

## 目录结构

```text
dual_mpu_gesture/
├─ arduino/
│  ├─ dual_mpu_gesture/
│  │  └─ dual_mpu_gesture.ino
│  ├─ triple_mpu_gesture/
│  │  └─ triple_mpu_gesture.ino
│  ├─ single_mpu_gesture/
│  │  └─ single_mpu_gesture.ino
│  └─ mpu_whoami_probe/
│     └─ mpu_whoami_probe.ino
├─ data/
│  ├─ dual_wrist/
│  │  └─ raw/
│  ├─ hand_thumb_index/
│  │  └─ raw/
│  ├─ single_imu/
│  │  └─ raw/
│  └─ thumb_index/
│     └─ raw/
├─ docs/
│  └─ gesture_protocol.md
├─ models/
│  ├─ dual_wrist/
│  ├─ hand_thumb_index/
│  ├─ single_imu/
│  └─ thumb_index/
├─ python/
│  ├─ common/
│  │  ├─ dual_protocol.py
│  │  ├─ single_protocol.py
│  │  └─ triple_protocol.py
│  ├─ schemes/
│  │  ├─ dual_wrist/
│  │  │  ├─ capture_labeled.py
│  │  │  ├─ capture_stream.py
│  │  │  ├─ features.py
│  │  │  ├─ pose.py
│  │  │  ├─ realtime_demo.py
│  │  │  └─ train_model.py
│  │  ├─ hand_thumb_index/
│  │  │  ├─ capture_labeled.py
│  │  │  ├─ capture_stream.py
│  │  │  ├─ features.py
│  │  │  ├─ pose.py
│  │  │  ├─ realtime_demo.py
│  │  │  └─ train_model.py
│  │  ├─ hybrid/
│  │  │  ├─ camera_gestures.py
│  │  │  └─ realtime_demo.py
│  │  ├─ single_imu/
│  │  │  ├─ capture_labeled.py
│  │  │  ├─ capture_stream.py
│  │  │  ├─ features.py
│  │  │  ├─ pose.py
│  │  │  ├─ realtime_demo.py
│  │  │  └─ train_model.py
│  │  └─ thumb_index/
│  │     ├─ capture_labeled.py
│  │     ├─ capture_stream.py
│  │     ├─ features.py
│  │     ├─ pose.py
│  │     ├─ realtime_demo.py
│  │     └─ train_model.py
│  └─ tools/
│     └─ probe_whoami.py
├─ requirements.txt
└─ requirements_camera.txt
```

## 统一终端前提

以下命令默认都在仓库根目录执行：

```powershell
cd D:\dual_mpu_gesture
```

建议先准备基础环境：

```powershell
conda create -n dual_mpu python=3.11 -y
conda activate dual_mpu
python -m pip install --upgrade pip
python -m pip install -r .\requirements.txt
```

## 硬件接线

### 双 MPU 方案

两个模块都接到 `Arduino Mega 2560` 的同一条 `I2C` 总线：

- `VCC -> 5V` 或 `3.3V`
- `GND -> GND`
- `SDA -> 20`
- `SCL -> 21`

地址区分：

- `0x68` 传感器：`AD0 -> GND`
- `0x69` 传感器：`AD0 -> VCC`

建议两个传感器安装时 `坐标方向保持一致`。

### 三 MPU 手背 + 拇指 + 食指方案

- `手背` 传感器：硬件 `I2C`，`SDA -> 20`，`SCL -> 21`，`AD0 -> GND`，地址 `0x68`
- `拇指` 传感器：硬件 `I2C`，`SDA -> 20`，`SCL -> 21`，`AD0 -> VCC`，地址 `0x69`
- `食指` 传感器：软件 `I2C`，`SDA -> 16`，`SCL -> 17`，`AD0 -> GND`，地址 `0x68`

说明：

- 第三颗 `MPU6050` 走独立的软件 `I2C`，这样不需要额外多路复用器
- 三个传感器安装时建议尽量 `坐标方向保持一致`
- 如果你想改软件 `I2C` 引脚，可直接修改 `arduino/triple_mpu_gesture/triple_mpu_gesture.ino` 里的 `SOFT_SDA_PIN / SOFT_SCL_PIN`

### 单 MPU 方案

- `VCC -> 5V` 或 `3.3V`
- `GND -> GND`
- `SDA -> 20`
- `SCL -> 21`
- `AD0` 可接 `GND` 或 `VCC`

## 可选排查：WHO_AM_I 自检

如果你怀疑双传感器地址冲突、接线错误，先烧录 `mpu_whoami_probe` 再读取串口摘要：

1. 编译并上传探测固件

```powershell
arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\mpu_whoami_probe
arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\mpu_whoami_probe
```

2. 运行串口解析工具

```powershell
python -m python.tools.probe_whoami --port COM5 --raw
```

## 方案一：双 MPU 手背 + 前臂

默认目录：

- 数据：`data/dual_wrist/raw`
- 模型：`models/dual_wrist/gesture_model.joblib`

标签集合：

- `flexion`
- `extension`
- `radial_deviation`
- `ulnar_deviation`
- `pronation`
- `supination`

### 第 1 步：烧录双 MPU 固件

```powershell
arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
```

### 第 2 步：查看原始串口流

```powershell
python -m python.schemes.dual_wrist.capture_stream --port COM5
```

### 第 3 步：采集带标签数据

```powershell
python -m python.schemes.dual_wrist.capture_labeled --port COM5 --label flexion --trials 20
python -m python.schemes.dual_wrist.capture_labeled --port COM5 --label extension --trials 20
python -m python.schemes.dual_wrist.capture_labeled --port COM5 --label radial_deviation --trials 20
python -m python.schemes.dual_wrist.capture_labeled --port COM5 --label ulnar_deviation --trials 20
python -m python.schemes.dual_wrist.capture_labeled --port COM5 --label pronation --trials 20
python -m python.schemes.dual_wrist.capture_labeled --port COM5 --label supination --trials 20
```

### 第 4 步：训练模型

```powershell
python -m python.schemes.dual_wrist.train_model --data-dir .\data\dual_wrist\raw --model-out .\models\dual_wrist\gesture_model.joblib
```

### 第 5 步：实时识别

```powershell
python -m python.schemes.dual_wrist.realtime_demo --port COM5 --model .\models\dual_wrist\gesture_model.joblib --min-confidence 0.60 --quiet-frames 4 --max-frames 120 --min-frames 12 --min-duration 0.30 --min-peak 150 --cooldown-seconds 0.80
```

## 方案二：双 MPU 拇指 + 食指

默认目录：

- 数据：`data/thumb_index/raw`
- 模型：`models/thumb_index/gesture_model.joblib`

推荐标签：

- 手指类：`fist / open_palm / thumb_up / victory / point / ok`
- 腕部类：`flexion / extension / radial_deviation / ulnar_deviation / pronation / supination`

注意：

- 这一套更像 `neutral -> target gesture -> hold briefly -> relax` 的动作分类
- 如果把 `12` 类都放进一个模型，采集动作的一致性会更重要
- 录腕部动作时，手指尽量保持稳定，不要叠加明显捏合或张指动作

### 第 1 步：烧录双 MPU 固件

```powershell
arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
```

### 第 2 步：查看原始串口流

```powershell
python -m python.schemes.thumb_index.capture_stream --port COM5
```

### 第 3 步：采集带标签数据

```powershell
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label fist --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label open_palm --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label thumb_up --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label victory --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label point --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label ok --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label flexion --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label extension --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label radial_deviation --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label ulnar_deviation --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label pronation --trials 20
python -m python.schemes.thumb_index.capture_labeled --port COM5 --label supination --trials 20
```

### 第 4 步：训练模型

```powershell
python -m python.schemes.thumb_index.train_model --data-dir .\data\thumb_index\raw --model-out .\models\thumb_index\gesture_model.joblib
```

### 第 5 步：实时识别

```powershell
python -m python.schemes.thumb_index.realtime_demo --port COM5 --model .\models\thumb_index\gesture_model.joblib
```

## 方案三：三 MPU 手背 + 大拇指 + 食指

默认目录：

- 数据：`data/hand_thumb_index/raw`
- 模型：`models/hand_thumb_index/gesture_model.joblib`

推荐标签：

- 手指类：`fist / open_palm / thumb_up / victory / point / ok`
- 腕部类：`flexion / extension / radial_deviation / ulnar_deviation / pronation / supination`

这套方案的特点：

- `手背` 传感器提供整手姿态参考
- `拇指 / 食指` 传感器提供手指相对运动细节
- 对同时包含 `腕部运动 + 手指形态` 的动作更友好

### 第 1 步：烧录三 MPU 固件

```powershell
arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\triple_mpu_gesture
arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\triple_mpu_gesture
```

### 第 2 步：查看原始串口流

```powershell
python -m python.schemes.hand_thumb_index.capture_stream --port COM5
```

### 第 3 步：采集带标签数据

```powershell
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label fist --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label open_palm --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label thumb_up --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label victory --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label point --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label ok --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label flexion --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label extension --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label radial_deviation --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label ulnar_deviation --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label pronation --trials 20
python -m python.schemes.hand_thumb_index.capture_labeled --port COM5 --label supination --trials 20
```

### 第 4 步：训练模型

```powershell
python -m python.schemes.hand_thumb_index.train_model --data-dir .\data\hand_thumb_index\raw --model-out .\models\hand_thumb_index\gesture_model.joblib
```

### 第 5 步：实时识别

```powershell
python -m python.schemes.hand_thumb_index.realtime_demo --port COM5 --model .\models\hand_thumb_index\gesture_model.joblib
```

## 方案四：单 MPU

默认目录：

- 数据：`data/single_imu/raw`
- 模型：`models/single_imu/gesture_model.joblib`

标签集合：

- `flexion`
- `extension`
- `radial_deviation`
- `ulnar_deviation`
- `pronation`
- `supination`

### 第 1 步：烧录单 MPU 固件

```powershell
arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\single_mpu_gesture
arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\single_mpu_gesture
```

### 第 2 步：查看原始串口流

```powershell
python -m python.schemes.single_imu.capture_stream --port COM5
```

### 第 3 步：采集带标签数据

```powershell
python -m python.schemes.single_imu.capture_labeled --port COM5 --label flexion --trials 20
python -m python.schemes.single_imu.capture_labeled --port COM5 --label extension --trials 20
python -m python.schemes.single_imu.capture_labeled --port COM5 --label radial_deviation --trials 20
python -m python.schemes.single_imu.capture_labeled --port COM5 --label ulnar_deviation --trials 20
python -m python.schemes.single_imu.capture_labeled --port COM5 --label pronation --trials 20
python -m python.schemes.single_imu.capture_labeled --port COM5 --label supination --trials 20
```

### 第 4 步：训练模型

```powershell
python -m python.schemes.single_imu.train_model --data-dir .\data\single_imu\raw --model-out .\models\single_imu\gesture_model.joblib
```

### 第 5 步：实时识别

```powershell
python -m python.schemes.single_imu.realtime_demo --port COM5 --model .\models\single_imu\gesture_model.joblib --plot
```

## 方案五：双 MPU + 摄像头混合

这套方案依赖：

- 双 `MPU` 腕部动态模型
- 摄像头
- `MediaPipe Hands`

建议单独使用摄像头环境，不要复用基础环境。

### 第 1 步：创建并激活摄像头环境

```powershell
conda create -n hybrid_gesture python=3.11 -y
conda activate hybrid_gesture
python -m pip install --upgrade pip
python -m pip install -r .\requirements.txt
python -m pip install -r .\requirements_camera.txt
```

### 第 2 步：烧录双 MPU 固件

```powershell
arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
```

### 第 3 步：先训练腕部动态模型

```powershell
python -m python.schemes.dual_wrist.train_model --data-dir .\data\dual_wrist\raw --model-out .\models\dual_wrist\gesture_model.joblib
```

### 第 4 步：运行混合实时识别

```powershell
python -m python.schemes.hybrid.realtime_demo --port COM5 --dynamic-model .\models\dual_wrist\gesture_model.joblib --camera-index 0
```

运行逻辑：

- 双 `MPU` 检测到动态动作段时，优先输出腕部标签
- 没有新的动态事件时，摄像头持续输出稳定手指手势标签

## 默认数据与模型目录

### 双 MPU 手背 + 前臂

- 数据目录：`data/dual_wrist/raw`
- 模型路径：`models/dual_wrist/gesture_model.joblib`

### 双 MPU 拇指 + 食指

- 数据目录：`data/thumb_index/raw`
- 模型路径：`models/thumb_index/gesture_model.joblib`

### 三 MPU 手背 + 大拇指 + 食指

- 数据目录：`data/hand_thumb_index/raw`
- 模型路径：`models/hand_thumb_index/gesture_model.joblib`

### 单 MPU

- 数据目录：`data/single_imu/raw`
- 模型路径：`models/single_imu/gesture_model.joblib`

## 说明

- 手势动作定义与采集规则见 `docs/gesture_protocol.md`
- 双 `MPU` 的 CSV 串口协议仍然是：

```text
ts_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2
```

- 三 `MPU` 的 CSV 串口协议是：

```text
ts_ms,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2,ax3,ay3,az3,gx3,gy3,gz3
```

- 单 `MPU` 的 CSV 串口协议是：

```text
ts_ms,ax,ay,az,gx,gy,gz
```
