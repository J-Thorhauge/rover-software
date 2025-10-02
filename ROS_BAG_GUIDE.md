# AAU Rover ROS2 bag collection guide

## Initial Setup
SSH into the gorm rover 

Launch the docker file that contains the setup you wish to record

for the standard configuration of the rover do the following

cd into the rover_software folder


```bash
cd ~
```

## Collecting the bag


### Select the topics
```bash
ros2 bag record -o <bag_file_name> <topic1> <topic2> ... <topicN>
```
for recording all the topics simply use: 

```bash
ros2 bag record -a
```
It is highly recommended that you use the topics labeled as compressed as you might otherwise face some issues.

For the depth image; use the /zed_front/zed/

### Perform the collection

## Playing the bag


## Common issues 

## No messages received from the depth image



### Discovering topics but cant get data 

To counteract this problem find the 'fastrtps-profiles.xml' file currently present in the Docker/config folder on the dev branch.

Then open the docker-compose.yaml file and add the folowing line to the volumes section for the specific image you are building
```yaml
- ./config/fastrtps-profiles.xml:/home/workspace/fastrtps-profiles.xml
```

Then Ensure that this line is present in the environement of the docker compose file for the specific image you are building:
```yaml
- FASTRTPS_DEFAULT_PROFILES_FILE=/home/workspace/fastrtps-profiles.xml
```
Rebuild the docker container and reboot the orin (reboot might not be necessary but it did not work for us until we did)
