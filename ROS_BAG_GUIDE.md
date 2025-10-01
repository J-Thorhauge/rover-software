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

### Perform the collection

## Playing the bag


## Common issues 

### Discovering topics but cant get data 

```yaml
- ./config/fastrtps-profiles.xml:/home/workspace/fastrtps-profiles.xml
```

Then Ensure that this line is present in the environement of the docker compose file for the specific image you are building:
```yaml
- FASTRTPS_DEFAULT_PROFILES_FILE=/home/workspace/fastrtps-profiles.xml
```

