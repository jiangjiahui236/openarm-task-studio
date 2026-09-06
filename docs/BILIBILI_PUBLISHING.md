# B 站发布建议

## 推荐视频结构

1. **30 秒效果展示**：创建任务点、RGB-D/RGB 切换、双臂动作、保存任务。
2. **项目解决什么问题**：用可视化编辑和人体示教降低 OpenArm 仿真任务制作门槛。
3. **环境说明**：Ubuntu、GPU、Isaac Sim、Isaac Lab、OpenArm Isaac Lab 的兼容关系。
4. **从零安装**：克隆三个仓库，设置三个环境变量，安装摄像头依赖。
5. **基础操作**：Move / Grasp / Place、路径检查、预览和完整执行。
6. **动作捕捉**：分别演示 D435 与普通摄像头、中立姿态标定和记录点。
7. **边界和安全**：说明当前是仿真工具、TCP/AABB 避障限制、UDP 只应在可信网络使用。
8. **问题反馈**：展示 README、Issues 和运行隐私检查的方法。

建议在单独的新用户账户或干净虚拟机里录制安装过程，能够最真实地发现遗漏依赖。

## 标题示例

`开源｜用 Isaac Sim 做 OpenArm 双臂任务编排，支持 D435/普通摄像头动作示教`

## 简介模板

```text
OpenArm Task Studio 是一个运行在 Isaac Sim / Isaac Lab 中的双臂任务编排工具，
支持 Move、Grasp、Place 可视化编辑、双臂并行执行，以及 D435 RGB-D / 普通摄像头动作捕捉。

项目地址：https://github.com/YOUR_NAME/openarm-task-studio
安装文档：见仓库 README
问题反馈：请提交 GitHub Issue

说明：当前项目面向仿真、研究和教学，避障不是完整机械臂碰撞规划。
```

## 发布前隐私检查

- 终端提示符、浏览器地址栏和窗口标题不要出现真实姓名、用户名或内部仓库地址。
- 不展示 Wi-Fi 名称/密码、GitHub Token、SSH 私钥、局域网设备清单或摄像头中的私人环境。
- 使用示例任务录屏，不使用生产任务、客户物体、动作日志或未经同意的人脸。
- GitHub 推送前运行 `./scripts/privacy_check.sh`，再执行 `git status --ignored` 检查待上传文件。
- 视频导出后从头检查一遍画面和音轨；必要时对人脸、通知、终端路径打码。
