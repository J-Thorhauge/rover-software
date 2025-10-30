source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=50

case $1 in
    --router|-r)
        echo "Starting Zenoh router..."
        export ZENOH_ROUTER_CONFIG_URI=docker/configs/routerconfig.json5
        pkill -9 -f ros && ros2 daemon stop
        ros2 run rmw_zenoh_cpp rmw_zenohd
        ;;
    *)
        echo "Environment set up for zenoh"
        ;;
esac