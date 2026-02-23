#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_node");

// --- CONFIGURATION ---
const std::string GROUP_ARM = "hand"; 
const std::string GROUP_GRIPPER = "gripper";

// --- COORDINATES (Meters) ---

// 1. PICK (Front/Right)
const double PICK_X = 0.25;
const double PICK_Y = 0.00;
const double PICK_Z_HOVER = 0.15;
const double PICK_Z_GRIP  = 0.00; 

// 2. PLACE (Back/Left) - MODIFIED FOR SAFETY
const double PLACE_X = -0.25;
// CRITICAL FIX: Y=0.02 prevents the robot from hitting exactly 180 degrees.
// This stops the "360 spin" or "extra movement" during the return trip.
const double PLACE_Y = 0.02; 
const double PLACE_Z_HOVER = 0.15;
const double PLACE_Z_GRIP  = 0.00; 

// 3. ARCH (The Bridge)
// We force the arch to be on the Positive Y side so the cable doesn't tangle
const double ARCH_Y_OFFSET = 0.15; 
const double ARCH_HEIGHT   = 0.26;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

bool moveToPose(moveit::planning_interface::MoveGroupInterface& move_group,
                geometry_msgs::msg::Pose target_pose,
                std::string pose_name) 
{
    RCLCPP_INFO(LOGGER, ">> %s", pose_name.c_str());
    move_group.setPoseTarget(target_pose);
    
    // Looser tolerances = Smoother motion
    move_group.setGoalPositionTolerance(0.01); 
    move_group.setGoalOrientationTolerance(0.15); 

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.execute(my_plan);
        return true;
    } else {
        RCLCPP_WARN(LOGGER, "!! Failed to plan: %s", pose_name.c_str());
        return false;
    }
}

geometry_msgs::msg::Pose getCompatiblePose(double x, double y, double z) 
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    // Calculate Yaw to face the target radially
    double yaw = atan2(y, x);
    
    // Orientation: Pitch = -90 deg (Gripper pointing down)
    tf2::Quaternion q;
    q.setRPY(0, 1.57, yaw); 
    
    pose.orientation.w = q.getW();
    pose.orientation.x = q.getX();
    pose.orientation.y = q.getY();
    pose.orientation.z = q.getZ();

    return pose;
}

void setGripper(moveit::planning_interface::MoveGroupInterface& gripper, std::string name) {
    gripper.setNamedTarget(name);
    gripper.move();
    // Short wait for gripper to physically move
    rclcpp::sleep_for(std::chrono::milliseconds(300));
}

// ============================================================================
// MAIN LOOP
// ============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pick_place_node");
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface arm(node, GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface gripper(node, GROUP_GRIPPER);

    // Speed up slightly to make the motion more fluid
    arm.setMaxVelocityScalingFactor(0.8);
    arm.setMaxAccelerationScalingFactor(0.6);
    arm.setPlanningTime(1.0);

    // DEFINE THE ARCH TOP POINT (SAFE RETURN POINT)
    auto arch_top_pose = getCompatiblePose(0.0, ARCH_Y_OFFSET, ARCH_HEIGHT);
    // Force top orientation to face 'Forward' (Yaw=90) to bridge the gap
    tf2::Quaternion q_top; q_top.setRPY(0, 1.57, 1.57); 
    arch_top_pose.orientation.x = q_top.getX(); arch_top_pose.orientation.y = q_top.getY();
    arch_top_pose.orientation.z = q_top.getZ(); arch_top_pose.orientation.w = q_top.getW();

    RCLCPP_INFO(LOGGER, "Moving Home...");
    arm.setNamedTarget("home");
    arm.move();
    setGripper(gripper, "open");

    while (rclcpp::ok()) {
        RCLCPP_INFO(LOGGER, "\n--- NEW CYCLE START ---");

        // 1. Move to Pick Hover
        auto pick_hover = getCompatiblePose(PICK_X, PICK_Y, PICK_Z_HOVER);
        if(!moveToPose(arm, pick_hover, "1. Hover Pick")) continue;

        // 2. Dive & Grip
        auto pick_grip = getCompatiblePose(PICK_X, PICK_Y, PICK_Z_GRIP);
        moveToPose(arm, pick_grip, "2. Grip Low");
        setGripper(gripper, "close");

        // 3. Lift Back to Hover
        moveToPose(arm, pick_hover, "3. Lift Up");

        // 4. ARCH TRANSFER (Right -> Left)
        moveToPose(arm, arch_top_pose, "4. Arch Over");

        // 5. Place Hover
        auto place_hover = getCompatiblePose(PLACE_X, PLACE_Y, PLACE_Z_HOVER);
        moveToPose(arm, place_hover, "5. Hover Place");

        // 6. Dive & Release
        auto place_grip = getCompatiblePose(PLACE_X, PLACE_Y, PLACE_Z_GRIP);
        moveToPose(arm, place_grip, "6. Place Low");
        setGripper(gripper, "open");

        // 7. Retract Up
        moveToPose(arm, place_hover, "7. Retract");

        // 8. ARCH RETURN (Left -> Right)
        // This is the CRITICAL step to prevent extra movements.
        // We force it to go back via the same bridge.
        moveToPose(arm, arch_top_pose, "8. Arch Return");

        // Cycle repeats...
    }

    rclcpp::shutdown();
    return 0;
}