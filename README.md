# ZZU Hero AutoAim

# 本项目为2026赛季郑州大学中州烛龙英雄视觉自瞄

<div align="center">

![RoboMaster](asset/RoboMaster-mecha-logo.png)

**郑州大学 RoboMaster 英雄步兵机器人自动瞄准系统**

![cover](asset/cover.jpg)

</div>

---

## 目录

- [项目概述](#项目概述)
- [代码架构](#代码架构)
  - [整体架构](#整体架构)
  - [模块详解](#模块详解)
  - [数据流](#数据流)
- [项目环境](#项目环境)
  - [依赖项](#依赖项)
  - [环境搭建](#环境搭建)
  - [构建指南](#构建指南)
- [调试指南](#调试指南)
  - [日志系统](#日志系统)
  - [串口调试](#串口调试)
  - [MCU 仿真](#mcu-仿真)
  - [常见问题](#常见问题)
- [项目展示](#项目展示)

---

## 项目概述

ZZU Hero AutoAim 是郑州大学 RoboMaster 战队为英雄步兵机器人开发的自动瞄准系统。该系统通过计算机视觉技术，实时检测敌方机器人装甲板，解算云台打击角度，实现自主瞄准与跟踪。

### 主要功能

- **装甲板检测**：支持传统视觉算法（灯条检测 + 装甲板匹配）与深度学习算法（YOLOv10）
- **数字分类**：识别装甲板上的数字（1-5），辅助目标选择
- **能量机关检测**：检测大能量机关绿色激活状态
- **目标跟踪**：基于状态估计的目标跟踪，实现平滑跟随
- **串口通信**：与下位机 MCU 通信，传输云台控制数据
- **MCU 仿真**：提供仿真环境，支持无硬件调试

### 效果展示

| 装甲板检测 | 能量机关 |
|:---:|:---:|
| ![demo](asset/demo.gif) | ![demo2](asset/demo2.gif) |

---

## 代码架构

### 整体架构

```
zzu_hero_autoaim/
├── CMakeLists.txt                    # 顶层 CMake 构建配置
├── LICENSE                           # 开源许可证
├── README.md                         # 项目文档
├── INSTALL_WSL.md                    # WSL 安装指南（旧版）
├──
├── src/                              # 核心源代码
│   ├── AutoAim.cpp / .h              # 自动瞄准主模块
│   ├── mcu_simulator.cpp             # MCU 通信仿真器
│   ├── test_serial.cpp               # 串口通信测试
│   ├── trainSVM.cpp / .h             # SVM 模型训练工具
│   │
│   └── armor_detector/               # 装甲板检测模块
│       ├── CMakeLists.txt
│       ├── include/                  # 头文件
│       └── src/                      # 实现文件
│
├── serial_new2/                      # ROS2 串口通信包（colcon 构建产物）
│   └── install/
│       └── serial_def_sdk/           # 串口通信 SDK
│
├── asset/                            # 资源文件
│   ├── structure.png / .pdf / .jpg   # 架构图
│   ├── demo.gif / demo2.gif          # 效果展示
│   ├── cover.jpg / cover.PNG         # 封面图
│   └── show.png                      # 展示图
│
└── logs/                             # 运行日志
```

### 模块详解

#### 1. 自动瞄准主模块 — `AutoAim`

自动瞄准系统的顶层调度模块，负责：

- 初始化各子模块（检测器、分类器、串口通信等）
- 主循环调度：图像采集 → 装甲板检测 → 目标选择 → 角度解算 → 串口发送
- 状态管理与模式切换

#### 2. 装甲板检测模块 — `armor_detector`

该模块是系统的核心视觉模块，包含以下组件：

| 组件 | 文件 | 功能描述 |
|:---|:---|:---|
| **Detector** | `Detector.h/.cpp` | 检测器主类，协调整个检测流程 |
| **LightBarFinder** | `LightBarFinder.h/.cpp` | 灯条检测：对图像进行二值化、轮廓提取、灯条筛选与匹配 |
| **ArmorFinder** | `ArmorFinder.h/.cpp` | 装甲板匹配：将左右灯条配对，计算装甲板位置 |
| **Classifier** | `Classifier.h/.cpp` | 分类器基类，定义分类接口 |
| **NumberClassifier** | `NumberClassifier.h/.cpp` | 数字分类：识别装甲板数字（1-5） |
| **TargetChooser** | `TargetChooser.h/.cpp` | 目标选择：根据优先级策略选择最佳目标 |
| **Tracker** | `Tracker.h/.cpp` | 目标跟踪：基于运动模型（如卡尔曼滤波）预测目标位置 |
| **Score** | `Score.h/.cpp` | 评分模块：评估检测结果的质量 |
| **ROI_Accelerator** | `ROI_Accelerator.h/.cpp` | ROI加速：基于上一帧目标位置动态裁剪检测区域 |
| **GreenLightDetector** | `GreenLightDetector.hpp/.cpp` | 绿色灯条检测：用于能量机关激活检测 |
| **Inference** | `Inference.hpp/.cpp` | 推理引擎封装：ONNX Runtime / OpenCV DNN 等 |
| **YOLOv10Detector** | `YOLOv10Detector.hpp/.cpp` | YOLOv10 深度学习装甲板检测器 |
| **CNN** | `CNN.cpp` | CNN 前向推理实现 |
| **number_classifier** | `number_classifier.hpp` | 数字分类器模板实现 |
| **pyboostcvconverter** | `pyboostcvconverter.h` | Python/C++ 图像数据转换工具 |

#### 3. 串口通信 — `serial_new2`

基于 ROS2 的串口通信包，负责与下位机（C Board / MCU）进行数据交换：

- 发送：目标角度（yaw/pitch）、射击指令等
- 接收：云台姿态、陀螺仪数据、弹速等

#### 4. 工具模块

| 文件 | 功能 |
|:---|:---|
| `mcu_simulator.cpp` | MCU 仿真器，模拟下位机返回数据，便于脱离硬件调试 |
| `test_serial.cpp` | 串口通信测试程序，验证串口收发是否正常 |
| `trainSVM.cpp/.h` | SVM 分类器训练工具，可用于数字识别模型训练 |

### 数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                        自动瞄准数据流                                 │
└─────────────────────────────────────────────────────────────────────┘

  相机采集
     │
     ▼
┌─────────────────┐
│   ROI_Accelerator │  ← 上一帧目标位置（Tracker 提供）
│   (ROI 裁剪加速)   │
└────────┬────────┘
         │ ROI 图像
         ▼
┌─────────────────┐     ┌─────────────────┐
│  LightBarFinder  │────▶│  YOLOv10Detector │  (可选深度学习分支)
│  (灯条检测)       │     │  (YOLOv10 检测)   │
└────────┬────────┘     └────────┬────────┘
         │ 灯条候选                  │ 检测框
         ▼                         ▼
┌─────────────────┐     ┌─────────────────┐
│   ArmorFinder    │     │  Inference / CNN │
│  (装甲板匹配)     │     │  (推理引擎)       │
└────────┬────────┘     └────────┬────────┘
         │ 装甲板候选                │ 分类结果
         ▼                         │
┌──────────────────────────────────┘
│
▼
┌─────────────────┐
│ NumberClassifier │  ← 装甲板数字识别 (1-5)
│  (数字分类)       │
└────────┬────────┘
         │ 带数字的装甲板
         ▼
┌─────────────────┐
│    Score         │  ← 评分筛选
│  (评分筛选)       │
└────────┬────────┘
         │ 最优装甲板
         ▼
┌─────────────────┐     ┌─────────────────┐
│  TargetChooser   │◀───│  GreenLightDetector│
│  (目标选择)       │     │  (能量机关检测)    │
└────────┬────────┘     └─────────────────┘
         │ 选定目标
         ▼
┌─────────────────┐
│    Tracker       │  ← 卡尔曼滤波 / 平滑
│  (目标跟踪)       │
└────────┬────────┘
         │ 预测位置
         ▼
┌─────────────────┐
│  角度解算 & 串口  │  → 发送 yaw/pitch 到 MCU
│  (AutoAim)       │
└─────────────────┘
```

---

## 项目环境

### 依赖项

#### 基础环境

| 依赖 | 版本要求 | 说明 |
|:---|:---|:---|
| Ubuntu (推荐) / Windows + WSL | 20.04+ | 主要开发环境 |
| CMake | 3.10+ | 构建系统 |
| GCC / G++ | 7.5+ | C++ 编译器，需支持 C++17 |

#### 核心依赖

| 依赖 | 版本要求 | 说明 |
|:---|:---|:---|
| OpenCV | 4.5+ | 计算机视觉库，需包含 `imgproc`、`highgui`、`dnn` 模块 |
| Eigen3 | 3.3+ | 线性代数库，用于坐标变换与姿态解算 |
| ONNX Runtime | 可选 | 深度学习推理引擎，用于 YOLOv10 模型部署 |

#### 串口通信

| 依赖 | 说明 |
|:---|:---|
| ROS2 (Humble / Foxy) | 可选，提供串口通信框架 |
| `serial_driver` / `boost::asio` | 串口通信库 |

### 环境搭建

#### 1. 安装 OpenCV

```bash
# Ubuntu
sudo apt install libopencv-dev

# 或源码编译（推荐 4.5+）
git clone https://github.com/opencv/opencv.git
cd opencv
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_GENERATE_PKGCONFIG=ON ..
make -j$(nproc)
sudo make install
```

#### 2. 安装 Eigen3

```bash
sudo apt install libeigen3-dev
```

#### 3. 安装 ONNX Runtime（可选，使用 YOLOv10 时需要）

```bash
# 从 GitHub Release 下载预编译包
# https://github.com/microsoft/onnxruntime/releases
# 或使用包管理器
```

#### 4. 安装 ROS2（可选，串口通信需要）

```bash
# 参考 ROS2 官方安装指南
# https://docs.ros.org/en/humble/Installation.html
```

### 构建指南

#### 纯 CMake 构建

```bash
# 克隆项目
git clone <repo-url> && cd zzu_hero_autoaim

# 创建构建目录
mkdir build && cd build

# 配置
cmake ..

# 编译
make -j$(nproc)

# 可选：安装
sudo make install
```

#### ROS2 + Colcon 构建

```bash
# 确保已安装 ROS2 并 source
source /opt/ros/humble/setup.bash

# 使用 colcon 构建串口包
cd serial_new2
colcon build
```

#### CMake 配置选项

| 选项 | 默认值 | 说明 |
|:---|:---|:---|
| `-DUSE_OPENCV=ON` | ON | 启用 OpenCV |
| `-DUSE_ONNX=OFF` | OFF | 启用 ONNX Runtime |
| `-DBUILD_TESTS=ON` | ON | 构建测试程序 |
| `-DBUILD_SIMULATOR=ON` | ON | 构建 MCU 仿真器 |


## 项目展示

具体效果展示：
https://www.bilibili.com/video/BV1MaDMBwEj6/?spm_id_from=333.337.search-card.all.click&vd_source=6ff10b6b14a249c47ed9714d5f827b72

## 改进方向

后续需要增加对平移靶的检测与跟踪功能

## 致谢
   2026赛季是郑州大学中州烛龙战队近年来表现最好的赛季，26赛季视觉组开拓创新，已研发步兵、哨兵和英雄自瞄，以及哨兵导航系统。感谢26视觉组全体成员近一年来的辛勤付出，感谢兄弟们的夜以继日将中州烛龙的名字写在华北站的舞台上，感谢凡哥、牢涛、甜心等对视觉组一如既往的支持。

   不必再待八载光阴，今朝即是登顶之时。

   各位RM同仁，大家来日方长、未来可期。

   26赛季视觉组：杨成志、何佳衡、贺博宇、熊伟竣、都昊宇、冯艺博
