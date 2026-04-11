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


## 快速开始

1. 烧录 Arduino 固件  
   打开 `arduino/dual_mpu_gesture/dual_mpu_gesture.ino`
   ```powershell
   conda activate dual_mpu
   arduino-cli compile --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
   arduino-cli upload -p COM5 --fqbn arduino:avr:mega D:\dual_mpu_gesture\arduino\dual_mpu_gesture
   ```

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
python .\python\realtime_demo.py --port COM5 --model .\models\gesture_model.joblib --min-confidence 0.60 --quiet-frames 4 --max-frames 120 --min-frames 12 --min-duration 0.30 --min-peak 150 --cooldown-seconds 0.80
```

## 单 MPU6050 版本

如果你当前只有 `1` 个 `MPU6050` 可用，可以直接使用仓库里的单传感器版本：

- 固件：`arduino/single_mpu_gesture/single_mpu_gesture.ino`
- 串口查看：`python/capture_stream_single.py`
- 标签采集：`python/capture_labeled_single.py`
- 模型训练：`python/train_model_single.py`
- 实时识别：`python/realtime_demo_single.py`

### 单传感器接线

- `VCC -> 5V` 或 `3.3V`
- `GND -> GND`
- `SDA -> 20`
- `SCL -> 21`
- `AD0` 可接 `GND` 或 `VCC`

单传感器固件会自动检测 `0x68` 或 `0x69`，不需要你改代码里的地址常量。

### 单传感器串口协议

表头为：

```text
ts_ms,ax,ay,az,gx,gy,gz
```

含义：

- `a*`：加速度原始计数
- `g*`：角速度原始计数

### 单传感器流程

1. 烧录固件  
   打开 `arduino/single_mpu_gesture/single_mpu_gesture.ino`

2. 查看原始串口流

```powershell
python .\python\capture_stream_single.py --port COM5
```

3. 采集带标签数据

```powershell
python .\python\capture_labeled_single.py --port COM5 --label flexion --trials 20
python .\python\capture_labeled_single.py --port COM5 --label extension --trials 20
python .\python\capture_labeled_single.py --port COM5 --label pronation --trials 20
```

4. 训练单传感器模型

```powershell
python .\python\train_model_single.py --data-dir .\data\raw_single --model-out .\models\gesture_model_single.joblib
```

5. 实时识别

```powershell
python .\python\realtime_demo_single.py --port COM5 --model .\models\gesture_model_single.joblib --plot
```

单传感器版本依赖 `手背` 或 `手腕附近` 的绝对姿态和角速度，识别效果通常会弱于双传感器方案，但足够完成 `屈伸`、`桡偏/尺偏`、`旋前/旋后` 这类动态手势的采集与分类验证。

## 双 MPU + 摄像头混合方案

如果你要在保留原有双 `MPU6050` 动态手势的同时，再新增 `fist`、`open_palm`、`thumb_up`、`victory`、`point`、`ok` 这类 `手指静态手势`，推荐使用混合方案：

- 双 `MPU6050` 负责：
  - `flexion`
  - `extension`
  - `radial_deviation`
  - `ulnar_deviation`
  - `pronation`
  - `supination`
- 摄像头 + `MediaPipe Hands` 负责：
  - `fist`
  - `open_palm`
  - `thumb_up`
  - `victory`
  - `point`
  - `ok`

相关文件：

- 摄像头手势模块：`python/camera_gestures.py`
- 混合实时识别：`python/realtime_demo_hybrid.py`
- 摄像头依赖：`requirements_camera.txt`

### 摄像头依赖安装

建议单独创建一个 `Python 3.11` 或 `3.12` 环境安装摄像头依赖，不要复用 `Python 3.13` 环境。

```powershell
conda create -n hybrid_gesture python=3.11 -y
conda activate hybrid_gesture
python -m pip install --upgrade pip
python -m pip install -r .\requirements.txt
python -m pip install -r .\requirements_camera.txt
```

### 先训练双 MPU 动态模型

```powershell
python .\python\train_model.py --data-dir .\data\raw --model-out .\models\gesture_model.joblib
```

### 运行 12 手势混合识别

```powershell
python .\python\realtime_demo_hybrid.py --port COM5 --dynamic-model .\models\gesture_model.joblib --camera-index 0
```

运行逻辑：

- 当双 `MPU` 检测到动态动作段时，优先输出 `6` 个腕部动作标签
- 当没有新的动态动作事件时，摄像头持续输出稳定的手指静态手势标签

注意：

- `point`、`thumb_up`、`victory`、`ok` 的摄像头规则识别默认假设 `手掌大致朝向摄像头`
- `ok` 基于 `拇指-食指捏合` 和其余三指展开的几何规则
- 如果你后续想提升 `手指静态手势` 的稳健性，可以再采集图像数据，把 `camera_gestures.py` 的启发式规则升级成自定义 `MediaPipe` 分类模型


