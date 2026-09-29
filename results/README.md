# 公开证据索引

公开副本整理于 2026-09-29；测试日期与本次整理日期分开记录。原始结果留在本地，不覆盖、不重复计数。

| 文件 | 含义 |
|---|---|
| [pytest-default.xml](public-20260922/pytest-default.xml) | 2026-09-22：40 collected / 36 passed / 4 skipped / 0 failed / 0 errors |
| [action-transport.xml](public-20260922/action-transport.xml) | 历史独立 Action 通信：1 passed |
| [integration-readonly.xml](public-20260922/integration-readonly.xml) | 历史采样和缺失服务器：2 passed |
| [integration-navigation.xml](public-20260922/integration-navigation.xml) | 历史真实 Nav2 导航：1 passed |
| [navigation.json](public-20260922/navigation.json) | 2026-09-22 成功导航及三组误差 |
| [startup-verification.json](public-20260922/startup-verification.json) | 同日 baseline 就绪与数据链记录 |
| [bag-info.txt](public-20260922/bag-info.txt) / [bag-validation.json](public-20260922/bag-validation.json) | 已录制 rosbag 的数量与离线内容核验，原始 MCAP 不公开 |
| [package-versions.txt](public-20260922/package-versions.txt) | 历史软件版本 |
| [server-timeout.json](public-20260927/server-timeout.json) | 2026-09-27 超时样本，不能单独确定根因 |
| [provenance.json](public-20260922/provenance.json) | 每个原件及公开副本的 SHA-256 与处理记录 |

四项 integration 是默认跳过的那四项，历史上分别执行通过，不与默认 36 项重复相加。JUnit 仅去除主机名和绝对项目路径；详见 [公开证据说明](../docs/公开证据说明.md)。

## 2026-09-29 发布检查

[本次默认 JUnit](publication-20260929/pytest-default.xml)：36 passed / 4 skipped，未启用 integration。公开副本去除了 hostname 和绝对项目路径；原报告留在本次发布工作区。本次结果独立于 9 月 22 日历史结果，不累加用例数。
