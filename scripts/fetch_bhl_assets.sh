#!/usr/bin/env bash
# Fetch the Berkeley Humanoid Lite assets (URDF, MJCF, meshes; CC BY-SA 4.0) into third_party/.
set -euo pipefail
cd "$(dirname "$0")/.."
DEST=third_party/berkeley-humanoid-lite-assets
REV=fc90fedd008b1e56a22e3c5221548d6b24f49707
[ -d "$DEST/.git" ] || git clone https://github.com/HybridRobotics/berkeley-humanoid-lite-assets.git "$DEST"
git -C "$DEST" fetch --depth 1 origin "$REV" 2>/dev/null || true
git -C "$DEST" checkout -q "$REV"
echo "BHL assets at $DEST ($REV)"
