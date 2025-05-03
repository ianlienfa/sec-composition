# Exploitation
- Checkout exploitations in these branches: memory_corruption, memory_consumption, secrecy, secrecy_remote

# Build
Enter the src/mycompostion directory
`colcon build --packages-select mycomposition`

# Source environment
`source install/setup.bash`

# For the memory_consumption exploitation, set memory limitation
`ulimit -v 2000000  # 2GB virtual memory limit`

# Run composition
`ros2 run mycomposition manual_compose`

# If switching between branches...
Remove the previous build by running this command and do the rebuild again
`rm -rf build/ install/ log/`