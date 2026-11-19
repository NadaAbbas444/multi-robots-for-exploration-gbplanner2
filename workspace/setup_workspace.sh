#!/usr/bin/env bash
# Prepare a catkin workspace around this repository:
#   1. clone every upstream dependency at its pinned commit (deps.repos)
#   2. apply this project's patches to voxblox and pci_general
#   3. download the Mask R-CNN COCO weights
#
# Expected layout (the repository must sit somewhere under <ws>/src):
#   mkdir -p ~/icuas2025_ws/src && cd ~/icuas2025_ws/src
#   git clone https://github.com/NadaAbbas444/multi-robots-for-exploration-gbplanner2.git
#   ./multi-robots-for-exploration-gbplanner2/workspace/setup_workspace.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"

# The workspace is the parent of the nearest ancestor directory named "src".
d="$REPO"; while [ "$d" != "/" ] && [ "$(basename "$d")" != "src" ]; do d="$(dirname "$d")"; done
[ "$(basename "$d")" = "src" ] || { echo "error: put this repository inside <workspace>/src" >&2; exit 1; }
SRC="$d"; WS="$(dirname "$SRC")"
echo "workspace: $WS"

python3 - "$HERE/deps.repos" > /tmp/icuas2025_deps.txt <<'EOF'
import re, sys
s = open(sys.argv[1]).read()
for path, url, ver in re.findall(r"  (\S+):\n    type: git\n    url: (\S+)\n    version: (\S+)", s):
    print(path, url, ver)
EOF

while read -r path url ver; do
  dst="$SRC/$path"
  if [ ! -d "$dst/.git" ]; then
    echo "clone  $path"
    git clone -q "$url" "$dst"
  fi
  git -C "$dst" fetch -q origin "$ver" 2>/dev/null || git -C "$dst" fetch -q origin
  git -C "$dst" -c advice.detachedHead=false checkout -q "$ver"
  echo "pinned $path @ ${ver:0:8}"
done < /tmp/icuas2025_deps.txt

apply_patch() {  # repo-relative path, patch file
  if git -C "$SRC/$1" apply --check "$HERE/patches/$2" 2>/dev/null; then
    git -C "$SRC/$1" apply "$HERE/patches/$2" && echo "patched $1"
  elif git -C "$SRC/$1" apply -R --check "$HERE/patches/$2" 2>/dev/null; then
    echo "patch already applied: $1"
  else
    echo "warning: could not apply $2 to $1" >&2
  fi
}
apply_patch mapping/voxblox voxblox.patch
apply_patch exploration/pci_general pci_general.patch

W="$REPO/mask_rcnn_ros/bags/mask_rcnn_coco.h5"
if [ ! -s "$W" ]; then
  echo "download Mask R-CNN COCO weights (246 MB)"
  mkdir -p "$(dirname "$W")"
  curl -L --fail -o "$W" https://github.com/matterport/Mask_RCNN/releases/download/v2.0/mask_rcnn_coco.h5
fi
echo "$(md5sum "$W" | cut -c1-32)  (expected e98aaff6f99e307b5e2a8a3ff741a518)"

cat <<EOF

Workspace ready. Next:
  $HERE/docker/run.sh            # builds the image on first use, opens a shell
  then inside the container:  build_ws && /ws/sim/icuas2025_larics/start.sh
EOF
