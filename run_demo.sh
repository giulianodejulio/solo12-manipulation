#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
options=(--rm --init --user "$(id -u):$(id -g)" -e OPENBLAS_NUM_THREADS=1 -e OMP_NUM_THREADS=1)
for arg in "$@"; do
  if [[ "$arg" == --viewer ]]; then
    auth_file="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
    if [[ ! -f "$auth_file" && -f "$HOME/.Xauthority" ]]; then auth_file="$HOME/.Xauthority"; fi
    [[ -f "$auth_file" ]] || { echo 'Set XAUTHORITY to your X11 authentication file.' >&2; exit 1; }
    options+=(-e MUJOCO_GL=glfw -e LIBGL_ALWAYS_SOFTWARE=1 -e "DISPLAY=${DISPLAY:?Set DISPLAY}" -e XAUTHORITY=/tmp/demo.xauth
      --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly
      --mount "type=bind,src=$auth_file,dst=/tmp/demo.xauth,readonly")
    break
  fi
done
mkdir -p "$project_dir/results"
exec docker run "${options[@]}" \
  --mount "type=bind,src=$project_dir/results,dst=/app/results" \
  "${SOLO12_MANIPULATION_IMAGE:-solo12-manipulation:demo}" "$@"
