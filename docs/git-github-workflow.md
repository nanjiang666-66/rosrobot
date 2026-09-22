# Git 与 GitHub 简明使用流程

本文档适用于本项目：

- 本地目录：`D:\Projects\ubuntu-workspace\robot_nav_course`
- Ubuntu 共享目录：`/mnt/hgfs/ubuntu-workspace/robot_nav_course`
- GitHub：<https://github.com/nanjiang666-66/rosrobot>
- 稳定分支：`main`
- SLAM 开发分支：`slam-exploration`
- 稳定标签：`stable-sdf-navigation-v1`

## 1. 先理解四个概念

- **工作区**：硬盘上正在编辑的文件。
- **提交（commit）**：一次有说明的本地版本快照。
- **分支（branch）**：一条独立开发路线。`main` 保存稳定版，`slam-exploration` 用来开发 SLAM。
- **标签（tag）**：给重要提交起一个固定名字，适合标记验收版。

`commit` 默认只保存在本机；执行 `git push` 后才会上传到 GitHub。

## 2. 每次开始工作

在 PowerShell 中进入项目：

```powershell
cd D:\Projects\ubuntu-workspace\robot_nav_course
```

确认当前分支和文件状态：

```powershell
git status -sb
```

开发 SLAM 时应看到：

```text
## slam-exploration...origin/slam-exploration
```

如果当前不是 SLAM 分支，切换过去：

```powershell
git switch slam-exploration
```

获取 GitHub 上的新提交：

```powershell
git fetch origin
git pull --ff-only
```

`--ff-only` 可以避免 Git 在不知情时自动制造复杂的合并提交。如果命令拒绝执行，先用 `git status` 检查，不要随意使用 `git reset --hard`。

## 3. 保存并上传一次新版本

修改和测试完成后，先查看变化：

```powershell
git status
git diff
```

把需要保存的变化加入暂存区：

```powershell
git add .
```

再次确认将要提交的内容：

```powershell
git diff --cached --stat
git diff --cached
```

创建本地提交，说明本次做了什么：

```powershell
git commit -m "Add SLAM frontier exploration"
```

上传当前分支：

```powershell
git push
```

如果这是一个刚创建、尚未上传过的新分支，第一次使用：

```powershell
git push -u origin slam-exploration
```

以后在这个分支只需执行 `git push`。

## 4. 查看历史版本

查看简洁提交图：

```powershell
git log --oneline --decorate --graph --all
```

查看某次提交具体改了什么：

```powershell
git show 提交编号
```

例如查看当前稳定版：

```powershell
git show stable-sdf-navigation-v1
```

查看某个文件的历史：

```powershell
git log --oneline -- src/course_bot_planner/src/astar_planner_node.cpp
```

## 5. 获取或恢复以往版本

### 5.1 临时查看旧版本，不修改历史

```powershell
git switch --detach stable-sdf-navigation-v1
```

这时可以查看、构建和运行旧代码，但不要直接在此状态长期开发。查看完成后返回：

```powershell
git switch slam-exploration
```

### 5.2 从旧版本创建一个独立测试分支

如果要基于稳定版做实验：

```powershell
git switch -c recovery-test stable-sdf-navigation-v1
```

实验结束后切回 SLAM 分支：

```powershell
git switch slam-exploration
```

### 5.3 只恢复一个文件

先查看差异：

```powershell
git diff stable-sdf-navigation-v1 -- README.md
```

确认后，把旧版文件恢复到当前工作区：

```powershell
git restore --source=stable-sdf-navigation-v1 -- README.md
```

恢复操作本身还没有成为版本，需要再执行 `git add`、`git commit` 和 `git push`。

### 5.4 撤销一个已经上传的提交

先在历史中找到提交编号：

```powershell
git log --oneline
```

安全撤销该提交：

```powershell
git revert 提交编号
git push
```

`git revert` 会创建一个新的“反向修改”提交，能够保留完整历史，适合已经上传的版本。

> 不要在不清楚后果时使用 `git reset --hard` 或强制推送 `git push --force`，它们可能丢失未保存工作或改写公共历史。

## 6. 稳定分支与开发分支如何配合

日常 SLAM 开发在：

```powershell
git switch slam-exploration
```

`main` 暂时保持稳定。SLAM 功能经过构建、测试和仿真验收后，再把它合并到 `main`：

```powershell
git switch main
git pull --ff-only
git merge --no-ff slam-exploration
git push
```

合并是重要操作。若出现冲突，先执行 `git status` 查看冲突文件并逐个处理；不要在未确认影响前继续提交。

## 7. 创建课程验收标签

当一个阶段通过验收后，可以创建带说明的标签：

```powershell
git tag -a stage7-slam-v1 -m "Stage 7 SLAM navigation verified"
git push origin stage7-slam-v1
```

查看已有标签：

```powershell
git tag -n
```

标签应只用于真正稳定、可复现的版本，不要每天创建。

## 8. 在另一台电脑重新获取项目

```powershell
git clone https://github.com/nanjiang666-66/rosrobot.git robot_nav_course
cd robot_nav_course
git switch slam-exploration
```

Git 只保存源码和文档。本项目的 `build/`、`install/`、`log/` 不上传，需要在 Ubuntu 中重新构建：

```bash
cd /mnt/hgfs/ubuntu-workspace/robot_nav_course
bash scripts/stage2_build.sh
source ~/robot_nav_course_colcon_cpp/install/setup.bash
```

## 9. 常见状态含义

```text
## slam-exploration...origin/slam-exploration
```

本地与 GitHub 同步。

```text
## slam-exploration...origin/slam-exploration [ahead 1]
```

本地多一个提交，需要 `git push`。

```text
## slam-exploration...origin/slam-exploration [behind 1]
```

GitHub 多一个提交，需要先 `git pull --ff-only`。

文件前的常见标记：

- `M`：文件已修改。
- `A`：新文件已加入暂存区。
- `??`：Git 尚未跟踪的新文件。
- `UU`：合并冲突，需要人工处理。

## 10. 本项目推荐的最短日常流程

开始工作：

```powershell
git switch slam-exploration
git pull --ff-only
git status -sb
```

完成并通过测试后：

```powershell
git status
git diff
git add .
git diff --cached --stat
git commit -m "简短说明本次改动"
git push
```

遇到不理解的冲突、分支分叉或文件丢失提示时，先停止并保存 `git status` 输出，再决定下一步。
