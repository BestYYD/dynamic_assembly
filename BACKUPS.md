# 源码备份与恢复

备份日期：2026-10-22（Asia/Hong_Kong）。本次只同步 `dynamic_assembly` 主包，不同步外部依赖、实验数据或完整工作空间。

## 备份点

| 备份 | 保存内容 | 引用 |
| --- | --- | --- |
| GitHub 同步前版本 | 本次操作开始时的远程 main，保留旧 README、旧源码及当时已跟踪的文件 | 分支 `codex/backup-before-sync-20261022-2123`，提交 `f95a39aa3bac322a654cd4d560129e453c54a3e0` |
| 本地主包原始源码快照 | 23 个主包源码 / 配置 / 文档文件，README 修订前逐文件 SHA-256 与本地核对一致；不含缓存、日志和编辑器配置 | 注释标签 `backup/local-source-20261022-2123`，提交 `871bdc3` |
| 完成本次同步后的版本 | 本地源码加修正后的 README、忽略规则和本备份说明 | 注释标签 `backup/synced-20261022-2123` |

备份入口：

- [同步前的 GitHub 版本](https://github.com/BestYYD/dynamic_assembly/tree/codex/backup-before-sync-20261022-2123)
- [README 修订前的本地源码](https://github.com/BestYYD/dynamic_assembly/tree/backup/local-source-20261022-2123)
- [同步完成版本](https://github.com/BestYYD/dynamic_assembly/tree/backup/synced-20261022-2123)

本次不强推、不重写远程历史。main 通过新提交向前更新；旧远程版本、原始本地主包和最终文档版本可分别定位。标签和分支仍受仓库写权限控制，请勿强制移动或删除这些备份引用。

## 在新目录恢复，不覆盖正在工作的文件

```bash
git clone https://github.com/BestYYD/dynamic_assembly.git dynamic_assembly_restore
cd dynamic_assembly_restore
git fetch origin --tags
git switch -c codex/restore-local-source backup/local-source-20261022-2123
```

上例恢复 README 修订前的本地主包。若要恢复本次同步完成的版本，在新克隆中使用：

```bash
git switch -c codex/restore-synced backup/synced-20261022-2123
```

若要检查同步前的远程版本，在新克隆中使用：

```bash
git switch -c codex/restore-before-sync origin/codex/backup-before-sync-20261022-2123
```

恢复出来的是源码快照，不是已经安装好的 ROS 工作空间。AR4 / ArUco / 轨迹工具、系统依赖、相机标定、模型权重和实验数据需要另外准备。不要在同一个 colcon 工作空间中同时放入两份同名 `dynamic_assembly` 包。

## 后续同步注意事项

1. 先确认正在操作的是该仓库的克隆目录，检查 `git remote -v` 和 `git status`。
2. 核对上传范围，不要直接对整个 ROS 工作空间进行全量添加。
3. 保留有意义的源码、配置和文档；构建产物、缓存、日志与敏感凭据不入库。
4. 提交前检查差异，更新 README 中的状态与参数，说明实际执行过的验证。
5. 推送后核对远程分支 / 标签的提交号。当前备份不包含后续尚未提交的本地改动。
