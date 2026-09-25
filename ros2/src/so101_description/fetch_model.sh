#!/bin/sh
# fetch_model.sh - download the SO-101 URDFs + meshes from TheRobotStudio/SO-ARM100
# (Apache-2.0) at a pinned commit, and rewrite mesh paths to package:// URIs.
# Run once before `colcon build` (needs internet). Files land in urdf/ and meshes/.
set -e
REPO=https://github.com/TheRobotStudio/SO-ARM100.git
COMMIT=5f6d2b876a53a4872e405b991dd925556c9e38a4
cd "$(dirname "$0")"
TMP=$(mktemp -d)
git clone -q --filter=blob:none --no-checkout "$REPO" "$TMP"
git -C "$TMP" sparse-checkout set Simulation/SO101
git -C "$TMP" checkout -q "$COMMIT"
SRC=$TMP/Simulation/SO101
mkdir -p urdf meshes
for pair in so101_new_calib.urdf:so101_follower.urdf so101_leader_new_calib.urdf:so101_leader.urdf; do
    in=${pair%%:*}; out=${pair##*:}
    sed 's#filename="assets/#filename="package://so101_description/meshes/#g' "$SRC/$in" > "urdf/$out"
done
grep -ho 'package://so101_description/meshes/[^"]*' urdf/*.urdf | sort -u | while read -r uri; do
    f=${uri##*/}; cp "$SRC/assets/$f" meshes/
done
cp "$TMP/LICENSE" meshes/LICENSE.SO-ARM100
rm -rf "$TMP"
echo "SO-101 model: $(ls urdf | tr '\n' ' ')+ $(ls meshes | wc -l) mesh files"
