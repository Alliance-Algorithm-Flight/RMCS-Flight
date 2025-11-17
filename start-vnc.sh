#!/bin/bash
set -euo pipefail

export USER=root
export HOME=/root
export DISPLAY=:1
export LANG=C.UTF-8
export LC_ALL=C.UTF-8

# 准备 xstartup（优先 openbox，缺失则退化到 xterm）
mkdir -p "$HOME/.vnc"
cat > "$HOME/.vnc/xstartup" <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_RUNTIME_DIR=/tmp/xdg-$(id -u)
mkdir -p "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"
xrdb "$HOME/.Xresources" >/dev/null 2>&1 || true
xsetroot -solid grey
if command -v openbox-session >/dev/null 2>&1; then
  exec dbus-launch --exit-with-session openbox-session
else
  xterm -geometry 120x40+10+10 &
  exec xterm
fi
EOF
chmod +x "$HOME/.vnc/xstartup"

# 清理并启动 VNC :1 (对应 5901)
vncserver -kill :1 > /dev/null 2>&1 || true
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1
vncserver :1 -geometry 1920x1080 -depth 24 -xstartup "$HOME/.vnc/xstartup"

# 启动 noVNC (6080->5901)
websockify --web=/usr/share/novnc/ 6080 localhost:5901 &
echo "noVNC running at http://localhost:6080"

# 前台保持
tail -F /root/.vnc/*.log
