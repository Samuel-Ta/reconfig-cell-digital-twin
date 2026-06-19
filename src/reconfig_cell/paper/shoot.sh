#!/usr/bin/env bash
# shoot.sh CX CY CZ TX TY TZ  -> move gz gui camera to look-at, then screenshot.
# Computes a look-at quaternion (gz convention: +X forward, +Z up).
set -e
CX=$1; CY=$2; CZ=$3; TX=$4; TY=$5; TZ=$6
OUT=${7:-$HOME/reconfig_ws/src/reconfig_cell/paper/shots}
read QW QX QY QZ < <(python3 - "$CX" "$CY" "$CZ" "$TX" "$TY" "$TZ" <<'PY'
import sys, math
cx,cy,cz,tx,ty,tz = map(float, sys.argv[1:7])
dx,dy,dz = tx-cx, ty-cy, tz-cz
yaw   = math.atan2(dy, dx)
horiz = math.hypot(dx, dy)
pitch = math.atan2(-dz, horiz)          # +pitch looks down (gz X-fwd/Z-up)
cy_,sy_ = math.cos(yaw/2),   math.sin(yaw/2)
cp,sp   = math.cos(pitch/2), math.sin(pitch/2)
qw =  cp*cy_;  qx = -sp*sy_;  qy = sp*cy_;  qz = cp*sy_
print(qw, qx, qy, qz)
PY
)
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
  --timeout 3000 --req "pose: {position: {x: $CX, y: $CY, z: $CZ}, orientation: {w: $QW, x: $QX, y: $QY, z: $QZ}}" >/dev/null
sleep 2
gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 3000 --req "data: \"$OUT\"" >/dev/null
sleep 1
echo "shot from ($CX,$CY,$CZ) -> ($TX,$TY,$TZ)  quat=($QW,$QX,$QY,$QZ)"
