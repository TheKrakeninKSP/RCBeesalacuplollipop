# Maze Pathfinding in ROS2

This repository contains a ROS2-based obstacle avoidance and pathfinding method for a robot navigating a maze using laser sensors. The robot makes movement decisions based on front, left, and right distance readings from a LiDAR sensor.

## Pathfinding Logic

The robot follows these steps:
1. **Obstacle Detection**: Reads laser scan data to determine distances in front, left, and right directions.
2. **Initial Rotation**: Starts with a small rotation before moving forward.
3. **Avoidance Maneuvers**:
   - If an obstacle is detected in front (distance < 0.5m), the robot turns left at a random angle.
   - If no obstacle is detected, the robot moves forward with a zigzag motion, favoring the direction with more open space.

## Code Implementation

The key logic is implemented in the `laser_callback` function:
```cpp
void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float front = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + 10);
    float left = *std::min_element(msg->ranges.begin() + 80, msg->ranges.begin() + 100);
    float right = *std::min_element(msg->ranges.begin() + 260, msg->ranges.begin() + 280);
    
    RCLCPP_INFO(this->get_logger(), "Front: %.2f, Left: %.2f, Right: %.2f", front, left, right);

    geometry_msgs::msg::Twist cmd;

    if (state_ == "INIT_ROTATION") {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.30;
        state_ = "FORWARD";
    } else if (front < 0.5) {
        cmd.linear.x = 0.0;
        float random_angle = (std::rand() % 91 + 30) * (M_PI / 180.0);
        cmd.angular.z = random_angle;
        state_ = "TURNING_LEFT";
    } else {
        float max_dist = std::max({front, left, right});
        zigzag_dir_ = (max_dist == left) ? 1 : (max_dist == right) ? -1 : zigzag_dir_;
        
        cmd.linear.x = 1.1;
        cmd.angular.z = 0.3 * zigzag_dir_;
        state_ = "FORWARD";
    }
    
    cmd_pub_->publish(cmd);
}
```

## Setup Instructions

1. Install ROS2 and set up a workspace.
2. Clone this repository into the `src` directory of your ROS2 workspace.
3. Build the package:
   ```bash
   colcon build
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```
5. Run the node

## Example Image

Below is a top-down visualization of the maze with the robot using laser sensors for navigation:

![Maze Pathfinding](A_top-down_view_of_a_simple_maze_with_walls_and_a_.png)

