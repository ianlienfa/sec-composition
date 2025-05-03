run `ulimit -v 1000000` to emulate the behavior of memory limited system here

# Build
Enter the src/mycompostion directory
`colcon build --packages-select mycomposition`

# Source environment
`source install/setup.bash`

# Set memory limitation
`ulimit -v 2000000  # 2GB virtual memory limit`

# Run composition
`ros2 run mycomposition manual_compose`


# If switching between branches...
Remove the previous build by running this command and do the rebuild again
`rm -rf build/ install/ log/`