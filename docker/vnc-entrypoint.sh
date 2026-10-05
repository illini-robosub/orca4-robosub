#!/bin/bash
# Used by run.sh on hosts with no GPU passthrough: renders into a virtual
# framebuffer instead of forwarding X11, and serves it over VNC.
set -e

export DISPLAY=:99
# setsid detaches the display stack from this terminal: otherwise Ctrl+C at
# the interactive prompt below reaches Xvfb, which shuts down on SIGINT and
# takes x11vnc and every GUI with it.
setsid Xvfb "$DISPLAY" -screen 0 1280x800x24 &

# Wait for Xvfb's lock file so clients don't race the server on startup.
for _ in $(seq 1 50); do
    [ -e /tmp/.X99-lock ] && break
    sleep 0.2
done

# Without a window manager, Qt apps (rviz2, Gazebo's GUI) often never get
# their windows mapped/placed, so the display stays blank.
setsid fluxbox &
# Paint the root window; otherwise moved/closed windows leave stale pixels.
xsetroot -solid "#2b2b2b"

VNC_PASSWORD="${VNC_PASSWORD:-orca4pass}"
setsid x11vnc -display "$DISPLAY" -forever -shared -passwd "$VNC_PASSWORD" -quiet -bg

echo "VNC server ready on port 5900, password: $VNC_PASSWORD"
echo "Connect from the host to view Gazebo/RViz, e.g. on macOS:"
echo "  Finder > Go > Connect to Server > vnc://localhost:5900"
echo "(macOS Screen Sharing won't accept a blank password, so one is set.)"

exec bash
