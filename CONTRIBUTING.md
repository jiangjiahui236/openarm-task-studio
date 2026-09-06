# Contributing

感谢你改进 OpenArm Task Studio。

1. 从 `main` 创建功能分支。
2. 保持修改聚焦，并为纯 Python 逻辑补充测试。
3. 运行 `python3 -m pytest -q tests` 和 `./scripts/privacy_check.sh`。
4. Pull Request 中说明测试环境、Isaac Sim / Isaac Lab 版本及硬件。

不要提交摄像头画面、动作日志、真实任务数据、设备地址、Wi-Fi 凭据、本机用户名或绝对路径。安全问题请按 `SECURITY.md` 私下报告，不要创建公开 Issue。
