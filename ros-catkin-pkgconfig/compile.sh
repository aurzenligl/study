
scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" >/dev/null 2>&1 && pwd  )"
cd $scriptdir

source /opt/ros/melodic/setup.sh
rm -rf build devel
time catkin_make -DCATKIN_ENABLE_TESTING=0 --cmake-args -Wno-dev
