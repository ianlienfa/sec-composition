# Build
To run this, first enter the src/composition directory and run
`colcon build --symlink-install --packages-select mycomposition`

# Set up remote listener
The remote listener listens to the topic initiated from the composition process and also publishes a topic called 'remote'
```
source install/setup.bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE={REPLACE WITH THE DIRECTORY PATH}/src/mycomposition/mycomposition_keystore
ros2 run mycomposition remote_listener --ros-args --enclave /remote_listener
```

# Set up the composition process
```
source install/setup.bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE={REPLACE WITH THE DIRECTORY PATH}/src/mycomposition/mycomposition_keystore
ros2 run mycomposition manual_compose --ros-args --enclave /listener
```

# Run
If successfully ran, you'll see something like
```
[ERROR] [1746315370.676532018] [attacker]: Attcker heard from topic[shout]: 'Hello World: 5'
[ERROR] [1746315370.678557398] [attacker]: Attcker heard from topic[shout]: 'SECRET | Hello World: 5'
[ERROR] [1746315370.680082944] [attacker]: Attcker heard from topic[shout]: 'Remote Publisher: 3'
[ERROR] [1746315370.680644862] [attacker]: Attcker heard from topic[shout]: 'Remote Publisher: 3'
[ERROR] [1746315370.681184155] [attacker]: Attcker heard from topic[shout]: 'Remote Publisher: 3'
[ERROR] [1746315370.681662906] [attacker]: Attcker heard from topic[shout]: 'Remote Publisher: 3'
```
Meaning the attacker has successfully subscribed to the remote topic

