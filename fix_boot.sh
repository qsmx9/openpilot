#!/usr/bin/env bash
# ============================================================
# CarrotPilot 开机报错 / 卡开机动画 一键修复脚本
#
# 适用: 更新后界面停在 "openpilot failed to build" 或卡在开机动画
# 用法: 连上 SSH(或打开终端), 粘贴这一行执行
#   curl -sL https://cdn.jsdelivr.net/gh/qingsimuxue99/openpilot@CP-Dev/fix_boot.sh | bash
#
# 为什么这样设计:
#   - 修复只需要更新 cereal/SConscript 一个文件(约 7KB), 走 CDN 比走 git 快得多,
#     国内网络下 git 直连 GitHub 经常超时, CDN 通常 2 秒内完成
#   - git 直连作为首选(能顺带对齐版本), 失败自动降级到 CDN, 保证一定能开机
#   - 每一步失败立即停止, 绝不带着问题重启; 只有编译完全通过才重启
# ============================================================

DIR="/data/openpilot"
SCONS="/usr/local/venv/bin/scons"
CDN="https://cdn.jsdelivr.net/gh/qingsimuxue99/openpilot@CP-Dev/cereal/SConscript"

say() { echo; echo "=========== $* ==========="; }
die() { echo; echo "!!! 失败: $*"; echo "!!! 已停止, 未重启。请把上面的全部内容截图发回。"; exit 1; }

say "0/5 检查环境"
[ -d "$DIR" ] || die "找不到 $DIR, 这不是一台正常的 openpilot 设备"
cd "$DIR" || die "无法进入 $DIR"
echo "当前版本: $(git rev-parse --short HEAD 2>/dev/null)"

# ---------- 1. 优先用 git 完整更新 ----------
say "1/5 尝试用 git 拉取最新代码 (最多等 90 秒)"
if timeout 90 git fetch https://github.com/qingsimuxue99/openpilot.git CP-Dev >/dev/null 2>&1; then
  git reset --hard FETCH_HEAD >/dev/null 2>&1 && echo "git 方式成功, 已更新到: $(git rev-parse --short HEAD)" || die "git 重置失败"
  GIT_OK=1
else
  echo "git 直连超时/失败(国内网络常见), 改用 CDN 方式修复"
  GIT_OK=0
fi

# ---------- 2. 兜底: CDN 直接替换关键文件 ----------
if [ "$GIT_OK" = "0" ]; then
  say "2/5 从 CDN 下载修复文件"
  cp cereal/SConscript /tmp/SConscript.bak 2>/dev/null
  curl -sL --max-time 60 -o /tmp/SConscript.new "$CDN" || die "CDN 下载失败, 请检查网络"
  [ -s /tmp/SConscript.new ] || die "CDN 下载到空文件"
  grep -q "_capnp_selfheal" /tmp/SConscript.new || die "CDN 下载内容异常(不是有效的 SConscript)"
  cp /tmp/SConscript.new cereal/SConscript || die "写入 SConscript 失败"
  echo "已替换 cereal/SConscript ($(wc -c < cereal/SConscript) 字节)"
  echo "注意: CDN 方式只替换了修复文件, 版本号未完全对齐, 不影响开机"
else
  say "2/5 git 已更新, 跳过 CDN"
fi

# ---------- 3. 清理构建产物 ----------
say "3/5 清理可能残缺的编译产物"
rm -rf cereal/gen
echo "已清理 cereal/gen (会在编译时自动重建)"

# ---------- 4. 编译 ----------
say "4/5 开始编译 (约 5-15 分钟, 期间请勿断电!)"
"$SCONS" -j4 || die "编译失败"

say "5/5 编译成功, 即将重启"
echo "编译全部通过。5 秒后重启设备, 重启后界面应正常进入。"
echo "如果重启后仍报错, 请把本脚本的全部输出截图发回。"
sleep 5
sudo reboot
