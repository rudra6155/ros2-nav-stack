#!/usr/bin/env bash
# Record media/run.gif: ONE real evaluation episode rendered in the Gazebo client.
#
# The evaluation itself is headless; this is a separate, display-bound run of a single episode
# (same seed => same start/goal as in results/episodes.csv). It uses Xvfb + Mesa software GL
# (no GPU), grabs the virtual screen with ffmpeg and converts the episode window to a GIF.
#
# usage: scripts/record_gif.sh [world=cluttered] [episode=1]            (inside the container)
set +e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/lib_stack.sh"
WORLD="${1:-cluttered}"; EP="${2:-1}"
W=1280; H=1024
CROP="${GIF_CROP:-900:840:315:150}"      # w:h:x:y of the 3-D view inside the Gazebo window (measured on a full frame)
OUT="$RESULTS_DIR/tmp/gif"; rm -rf "$OUT"; mkdir -p "$OUT" "$NAV_ROOT/media"

# make the Gazebo client window fill the virtual screen (there is no window manager)
mkdir -p "$HOME/.gazebo"
printf '[geometry]\nx=0\ny=0\nwidth=%s\nheight=%s\n' "$W" "$H" > "$HOME/.gazebo/gui.ini"

Xvfb :99 -screen 0 ${W}x${H}x24 +extension GLX +render -noreset > "$OUT/xvfb.log" 2>&1 &
XVFB=$!
sleep 2
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb
echo "software GL renderer: $(glxinfo 2>/dev/null | grep -m1 'OpenGL renderer')" | tee "$OUT/renderer.txt"

REC_T0=$(date +%s.%N)
ffmpeg -loglevel error -y -f x11grab -framerate 10 -video_size ${W}x${H} -i :99.0 -c:v libx264 -preset ultrafast -pix_fmt yuv420p "$OUT/screen.mp4" &
FF=$!

python3 "$HERE/gif_goal_marker.py" > "$OUT/marker.log" 2>&1 &
MARKER=$!
ros2 run evaluation run_episodes --world "$WORLD" --episode "$EP" --gui --results-dir "$OUT/run" --tag gif --no-bag 2>&1 | tee "$OUT/run.log"
REC_T1=$(date +%s.%N)
sleep 1; kill -INT $FF 2>/dev/null; wait $FF 2>/dev/null
kill $XVFB $MARKER 2>/dev/null; pkill -9 -x gzserver; pkill -9 -x gzclient

# episode window = last <duration_s + margin> seconds of the recording, before the stack shut down
DUR=$(python3 - "$OUT/run/episode_diagnostics.csv" <<'PY'
import csv, sys
r = list(csv.DictReader(open(sys.argv[1])))[0]
print(float(r['duration_s']))
PY
)
python3 - "$REC_T0" "$REC_T1" "$DUR" > "$OUT/window.txt" <<'PY'
import sys
t0, t1, dur = map(float, sys.argv[1:4])
total = t1 - t0
# the stack takes a few seconds to shut down after the episode; the episode ended about 6 s before t1
start = max(0.0, total - dur - 12.0)
print(f"{start:.1f} {dur + 7.0:.1f}")
PY
read SS LEN < "$OUT/window.txt"
echo "recording length $(python3 -c "print(round($REC_T1-$REC_T0,1))") s; GIF window: start $SS s, length $LEN s"
ffmpeg -loglevel error -y -ss "$SS" -t "$LEN" -i "$OUT/screen.mp4" \
  -vf "crop=${CROP},fps=8,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4" \
  "$NAV_ROOT/media/run.gif"
ls -la "$NAV_ROOT/media/run.gif"
ffmpeg -loglevel error -y -ss "$(python3 -c "print($SS + $LEN/2)")" -i "$OUT/screen.mp4" -frames:v 1 "$OUT/full_frame.png"
