# Git 常用操作指南

## 基础工作流

```bash
# 查看仓库状态
git status

# 查看改动内容
git diff                    # 未暂存的改动
git diff --staged           # 已暂存的改动
git diff HEAD               # 全部改动

# 将文件加入暂存区
git add <file>              # 添加单个文件
git add -A                  # 添加所有改动

# 提交
git commit -m "提交信息"

# 推送到远端
git push origin main

# 拉取远端更新
git pull origin main
```

## 提交信息规范

```bash
# 格式：<类型>: <简短描述>
git commit -m "feat: add motor calibration shell command"
git commit -m "fix: correct DM4340 current limit calculation"
git commit -m "docs: update bringup checklist"
git commit -m "refactor: rename control.c to pid_balance_control.c"
```

## 分支管理

```bash
# 创建并切换分支
git checkout -b feature-xxx

# 切换分支
git checkout main

# 查看所有分支
git branch -a

# 合并分支到 main
git checkout main
git merge feature-xxx

# 删除本地分支
git branch -d feature-xxx
```

## 撤销操作

```bash
# 撤销未暂存的改动（恢复文件）
git checkout -- <file>
git restore <file>

# 取消暂存（保留改动）
git restore --staged <file>

# 撤销最近一次提交（保留改动）
git reset --soft HEAD~1

# 撤销最近一次提交（丢弃改动）
git reset --hard HEAD~1
```

## 查看历史

```bash
# 简洁日志
git log --oneline -10

# 带改动的日志
git log --stat

# 查看某个文件的改动历史
git log -p <file>
```

## SSH 配置

```bash
# 生成密钥
ssh-keygen -t ed25519 -C "your-email@example.com"

# 复制公钥，添加到 GitHub Settings → SSH and GPG keys
cat ~/.ssh/id_ed25519.pub

# 测试连接
ssh -T git@github.com
```

## HTTPS 转 SSH

```bash
# 如果之前用 HTTPS 克隆，可以切换为 SSH
git remote set-url origin git@github.com:123zhengzz/zephyr_ascento_f407_wheel_leg.git
```
