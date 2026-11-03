#!/usr/bin/env bash
# Downloads the 3D assets used by the crosswalk world (cars, walking person,
# trees, lamp posts) from Gazebo Fuel and stores them in a flat, version
# independent layout:
#   <dest>/<key>/{meshes,materials}
# Usage: fetch_assets.sh [dest]   (default /opt/crosswalk_assets; the world
# generator reads $CROSSWALK_ASSETS, default /opt/crosswalk_assets)
# The simulation runs fully offline afterwards. A car/tree model that cannot be
# downloaded is replaced by primitive shapes; the pedestrian model is required.
# Needs the Gazebo CLI (source /opt/ros/jazzy/setup.bash first).
set -u
DEST="${1:-/opt/crosswalk_assets}"
CACHE="$(mktemp -d)"
export GZ_FUEL_CACHE_PATH="$CACHE"
mkdir -p "$DEST"

fetch() {  # key owner "Model Name"
  local key="$1" owner="$2" name="$3"
  local url="https://fuel.gazebosim.org/1.0/${owner}/models/${name// /%20}"
  for attempt in 1 2 3; do
    if timeout 300 gz fuel download -u "$url" >/dev/null 2>&1; then break; fi
    echo "  retry $attempt for $name"; sleep 3
  done
  local lname; lname="$(echo "${name// /%20}" | tr '[:upper:]' '[:lower:]')"
  local lowner; lowner="$(echo "$owner" | tr '[:upper:]' '[:lower:]')"
  local dir; dir="$(ls -d "$CACHE"/fuel.gazebosim.org/"$lowner"/models/"$lname"/* 2>/dev/null | sort -V | tail -1)"
  if [ -z "$dir" ]; then echo "WARNING: could not download $owner/$name"; return; fi
  mkdir -p "$DEST/$key"
  cp -r "$dir"/meshes "$DEST/$key/" 2>/dev/null
  cp -r "$dir"/materials "$DEST/$key/" 2>/dev/null
  echo "OK  $key <- $owner/$name"
}

fetch hatchback       OpenRobotics "Hatchback"
fetch hatchback_blue  OpenRobotics "Hatchback blue"
fetch hatchback_red   OpenRobotics "Hatchback red"
fetch suv             OpenRobotics "SUV"
fetch pickup          OpenRobotics "Pickup"
fetch prius           OpenRobotics "Prius Hybrid"
fetch actor           Mingfei      "actor"
fetch oak_tree        OpenRobotics "Oak tree"
fetch lamp_post       OpenRobotics "Lamp Post"

# Some .mtl files point at textures by URL, model:// URI or a bare file name.
# Rewrite every texture reference to ../materials/textures/<file> and make sure
# the file exists there, so that no network access is needed at run time.
python3 - "$DEST" <<'PYEOF'
import os, re, shutil, sys
dest = sys.argv[1]
textures = {}
for root, _, files in os.walk(dest):
    for f in files:
        if f.lower().endswith(('.png', '.jpg', '.jpeg', '.tga')):
            textures.setdefault(f.lower(), os.path.join(root, f))
for root, _, files in os.walk(dest):
    for f in files:
        if not f.endswith('.mtl'):
            continue
        mtl = os.path.join(root, f)
        tex_dir = os.path.normpath(os.path.join(root, '..', 'materials', 'textures'))
        out = []
        for line in open(mtl):
            m = re.match(r'(\s*map_\w+\s+)(\S+)\s*$', line)
            if m:
                base = os.path.basename(m.group(2))
                target = os.path.join(tex_dir, base)
                if not os.path.isfile(target) and base.lower() in textures:
                    os.makedirs(tex_dir, exist_ok=True)
                    shutil.copy(textures[base.lower()], target)
                line = f'{m.group(1)}../materials/textures/{base}\n'
            out.append(line)
        open(mtl, 'w').writelines(out)
PYEOF

rm -rf "$CACHE"
chmod -R a+rX "$DEST"
du -sh "$DEST"
