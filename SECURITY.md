# Security Policy

请勿在公开 Issue 中披露凭据泄漏、远程控制或机器人安全问题。请使用 GitHub 仓库的 **Security > Report a vulnerability** 私下报告，并附上受影响版本、复现步骤和影响范围。

动作捕捉 UDP 接口默认用于可信本机或局域网，不提供认证、加密或公网暴露能力。使用者应通过主机防火墙限制 UDP `5010` 和 `5011`，不要将其转发到互联网。
