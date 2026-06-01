# Branch Management Guide — pocat-lektron-sw

## 仓库结构

```
origin (nanosatlab/pocat-lektron-sw)   ← 公共上游仓库 (只读)
    │
    └── fork ──► personal (HaiyueY/pocat-lektron-sw)  ← 个人 fork
                    │
                    ├── dev-main   ← 功能开发远程分支
                    └── dev-test   ← 测试环境远程分支
```

### 本地分支

| 分支   | 追踪远程              | 用途                         |
| ------ | --------------------- | ---------------------------- |
| `dev`  | `personal/dev-main`   | 功能开发（主要工作分支）      |
| `test` | `personal/dev-test`   | 测试环境、测试脚本开发        |

### 分支关系

`test` 分支始终基于 `dev` 之上（通过 rebase），仅包含测试相关的额外提交：

```
dev:   A ── B ── C ── D          (功能开发)
                       \
test:                   E ── F   (测试环境 + 测试脚本)
```

---

## 日常工作流

### 1. 功能开发 (dev 分支)

```bash
git checkout dev

# 开发新功能...
git add .
git commit -m "feat: 新功能描述"

# 推送到个人仓库
git push personal dev-main
```

### 2. 同步功能到测试分支

当 `dev` 有新功能需要测试时，将 `test` rebase 到最新的 `dev`：

**使用脚本（推荐）：**
```bash
./scripts/sync-test.sh
```

**手动操作：**
```bash
git checkout dev
git pull personal dev-main

git checkout test
git rebase dev

# 如有冲突：解决后 git rebase --continue
# 放弃 rebase：git rebase --abort

git push personal dev-test --force
```

### 3. 测试开发 (test 分支)

```bash
git checkout test

# 开发测试脚本、修改测试环境...
git add .
git commit -m "test: 测试描述"

git push personal dev-test --force   # rebase 后始终需要 force push
```

> **提示：** 测试相关的修改尽量限制在 `test/` 目录内，减少与 dev 分支的冲突。

### 4. 从上游 (nanosatlab) 同步

**使用脚本（推荐）：**
```bash
./scripts/sync-upstream.sh
```

**手动操作：**
```bash
# 1. 拉取上游更新
git fetch origin

# 2. 合并到 dev
git checkout dev
git merge origin/dev-main
git push personal dev-main

# 3. 同步到 test
git checkout test
git rebase dev
git push personal dev-test --force
```

---

## 冲突处理

### Rebase 冲突场景

当 `dev` 和 `test` 修改了相同文件时（如根目录 `CMakeLists.txt`），rebase 会产生冲突。

**解决步骤：**

```bash
# 1. rebase 停在冲突处
git rebase dev
# CONFLICT (content): Merge conflict in <file>

# 2. 查看冲突文件
git status

# 3. 手动编辑冲突文件，解决冲突标记 (<<<<<<<, =======, >>>>>>>)

# 4. 标记为已解决
git add <resolved-files>

# 5. 继续 rebase
git rebase --continue

# 6. 推送
git push personal dev-test --force
```

**放弃 rebase：**
```bash
git rebase --abort   # 回到 rebase 之前的状态
```

### 上游合并冲突

当 `origin/dev-main` 与本地 `dev` 有冲突时：

```bash
git checkout dev
git merge origin/dev-main
# 解决冲突...
git add . && git commit
git push personal dev-main
```

---

## 最佳实践

1. **测试文件隔离** — 所有测试代码、HAL stub、模拟器配置放在 `test/` 目录
2. **同步频率** — 在开始新功能前先运行 `./scripts/sync-upstream.sh`
3. **Force Push 安全** — `test` 分支的 `--force` 推送是正常的（rebase 策略），但确保只有你在使用此分支
4. **提交信息规范** — dev 分支用 `feat:`, `fix:` 等前缀；test 分支用 `test:` 前缀
5. **冲突预防** — 避免在 test 分支修改 dev 分支的核心源码文件

---

## 辅助脚本

| 脚本                         | 功能                                    |
| ---------------------------- | --------------------------------------- |
| `scripts/sync-test.sh`       | 将 dev 最新更改同步到 test（rebase）      |
| `scripts/sync-upstream.sh`   | 从 origin 同步到 dev，再同步到 test       |

使脚本可执行：
```bash
chmod +x scripts/sync-test.sh scripts/sync-upstream.sh
```
