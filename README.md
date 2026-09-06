# OpenArm Task Studio

面向 OpenArm 双臂机器人的可视化任务编排与动作捕捉工具。它运行在 Isaac Sim / Isaac Lab 中，可创建、编辑和执行 Move、Grasp、Place 任务，并支持 D435 RGB-D 或普通 RGB 摄像头进行人体上肢示教。

> 当前版本主要用于仿真、研究与教学。避障采用 TCP 对场景 AABB 的采样，不是完整机械臂碰撞规划；请勿直接用于无人值守的真实机器人生产环境。

## 功能

- 可视化创建 Move / Grasp / Place 任务点
- 左右臂独立任务与并行执行
- 任务 JSON 保存、加载和路径验证
- D435 RGB-D 与普通摄像头 RGB 动作捕捉切换
- MediaPipe 肩、肘、腕姿态跟随与中立姿态标定
- 动作捕捉监测日志与可选 ESP32-C3 记录按钮
- 参数调节 GUI 和纯 Python 单元测试

## 环境要求

- Ubuntu 22.04
- Python 3.10
- NVIDIA Isaac Sim 对应的 Python 虚拟环境
- [Isaac Lab](https://github.com/isaac-sim/IsaacLab)（已验证 `v2.3.0`）
- [OpenArm Isaac Lab](https://github.com/enactic/openarm_isaac_lab)
- NVIDIA GPU 和满足 Isaac Sim 要求的驱动
- 可选：Intel RealSense D435；没有 D435 时可使用普通 USB/笔记本摄像头

三个上游项目的版本必须互相兼容。若上游仓库后续接口变化，建议先使用 Isaac Lab `v2.3.0` 和与你的 Isaac Sim 版本匹配的 OpenArm Isaac Lab 提交。

## 安装

```bash
git clone https://github.com/YOUR_NAME/openarm-task-studio.git
cd openarm-task-studio
python3 -m pip install -r requirements-camera.txt

# 按实际安装位置设置，不要把个人路径提交到仓库
export ISAACSIM_VENV=/path/to/isaacsim/venv
export ISAACLAB_ROOT=/path/to/IsaacLab
export OPENARM_ISAAC_REPO=/path/to/openarm_isaac_lab

./run_task_studio.sh
```

变量含义：

- `ISAACSIM_VENV`：包含 `bin/activate` 的 Isaac Sim Python 环境
- `ISAACLAB_ROOT`：包含 `isaaclab.sh` 的 Isaac Lab 根目录
- `OPENARM_ISAAC_REPO`：包含 `openarm/` Python 包的 OpenArm Isaac Lab 仓库

## 动作捕捉

1. 启动 Task Studio。
2. 在右下角 `Motion Capture` 面板选择 `RGB-D (D435)` 或 `RGB (Webcam)`。
3. 选择 `TEACH LEFT`、`TEACH RIGHT` 或 `TEACH BOTH`。
4. 点击 `Start Motion Teaching`，让双肩、双肘和双腕进入画面并保持中立姿态约 45 帧。
5. 移动手臂后点击 `Record Current Point`，最后点击 `Stop Motion Teaching`。

普通摄像头默认使用 OpenCV 设备 `0`。命令行验证摄像头：

```bash
./run_d435_teaching_camera.sh --source rgb --camera-index 0
./run_d435_teaching_camera.sh --source rgbd
```

RGB-D 模式使用对齐深度反投影三维关键点；RGB 模式使用 MediaPipe 单目 world landmarks，深度精度和稳定性通常低于 D435。

## 测试

```bash
python3 -m pytest -q tests
OPENARM_TASK_STUDIO_SELF_TEST=1 ./run_task_studio.sh --headless --duration 30
```

## 目录

- `openarm_task_studio/`：Kit 扩展、任务模型、编译器和运行时状态
- `launch_task_studio.py`：Isaac Lab 场景和执行循环
- `studio_scene.py`：OpenArm 双臂、地面与工作台场景
- `d435_rgbd_sender.py`：RGB-D / RGB 姿态捕捉和 UDP 发送器
- `config/`：动作映射与 Kit 扩展配置
- `esp32c3_remote_button/`：可选无线记录按钮固件
- `ARCHITECTURE.md`：任务编译与执行架构

## 隐私

仓库不应包含摄像头画面、动作日志、任务历史、Wi-Fi 密码、用户名或本机绝对路径。`.gitignore` 已排除运行时数据；发布前请执行：

```bash
./scripts/privacy_check.sh
```

## 贡献与许可

欢迎提交 Issue 和 Pull Request。使用前请阅读 `CONTRIBUTING.md`。本项目采用 [Apache License 2.0](LICENSE)。

准备制作演示视频时，可参考 [B 站发布建议](docs/BILIBILI_PUBLISHING.md)。
