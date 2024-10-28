git submodule update --init --recursive
cd sjtu_drone
rm -rf build install log
colcon build --symlink-install
cd ..

. params.sh

rm -rf build install log
. bd.sh