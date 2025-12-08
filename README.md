# AAU Space Robotics Navigation - System Setup Repository
This repository is a fork from the main AAU Space Robotics repository.
It was made with the intetntion of adding autonomous navigation and SLAM capabilities to the GORM rover while still being modular enough
such that it could be utilized on other systems. 

The original documentation site can be found at:
Documentation site: https://aau-space-robotics.github.io/aau-rover/


## Quick start (get the rover moving)

Start by SSH'ing into the desired jetson orin and ensure that the current branch on it matches this branch.
Run the following commands (note that a first-time build and run will take som time as it has to build some large packages from source)
```bash
cd workspace/p9_dev/rover-software/docker/
./build.sh  # This is only needed if no the containers havent been build before
./run.sh rover --prod -z
./run.sh camgnss --prod -z
./run.sh vslam --prod -z
./run.sh nav2 --prod -z

# Ensure that all of the containers are running
docker ps
```


When the rover is fully set up the next step is setting up the ground control computer.
The only prerequisites are that the ground control computer has ROS2 Humble and Nav2 installed

Open a new terminal that is NOT connected to the rover
First, setup zenoh on the ground control computer:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=50

#You will need to download and unzip the rmw_zenoh_cpp.zip file and unzip it in a known location
# Adapt the to your path:
export ZENOH_ROUTER_CONFIG_URI= PATH/TO/YOUR/rmw_zenoh_cpp/config/routerconfig.json5

# If using a previous DDS implementation, kill the ROS 2 daemon first
pkill -9 -f ros && ros2 daemon stop

# Then start the Zenoh RMW daemon
ros2 run rmw_zenoh_cpp rmw_zenohd
```
In another terminal open up for rviz using the nav2 bringup

```bash
ros2 launch nav2_bringup rviz_launch.py
```
You should now be able to send pose goals to the rover using rviz.




Notes:
- Container name: `rover-deploy` (production).
- Production containers are configured to restart automatically unless stopped.

## Useful commands

- Start development: `./run.sh rover --dev`
- Start production: `./run.sh rover --prod`
- Build production image: `./build.sh`
- Attach to container: `docker exec -it rover bash` (dev) or `docker compose -f docker/docker-compose.yaml exec rover-deploy bash` (prod)
- Tail logs: `docker compose -f docker/docker-compose.yaml logs -f <service>`
- Reset Motors after E-stop: `ros2 topic pub -1 /restart_rover std_msgs/Empty "{}"`

## SSH access to the rover

To access the rover directly (outside Docker), SSH to the rover's IP. The username and IP may vary; example below is for an onboard user account commonly used in our docs:

```bash
ssh gorm@192.168.0.100 # Default IP for the rover is 192.168.0.100
```

Confirm the correct username and password for your hardware image before connecting.

## Simulation

Simulation assets and detailed instructions are not available yet — coming soon. Check the documentation site for updates and the `docs/` directory later for example scenarios and launch files.

---

For more detailed guides, see the `docs/` directory or visit the documentation site linked above.
