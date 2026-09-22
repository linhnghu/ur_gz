#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using namespace std::chrono_literals;


class DrawLetter
{
public:

  // ============================================================
  // Constructor
  // ============================================================

  explicit DrawLetter(
    const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "=========================================="
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "        UR3e DRAW LETTER NODE"
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "=========================================="
    );


    // ==========================================================
    // Publisher for RViz LINE_STRIP
    // ==========================================================

    marker_pub_ =
      node_->create_publisher<
        visualization_msgs::msg::Marker
      >(
        "/letter_path",
        10
      );


    // ==========================================================
    // Create MoveGroupInterface
    // ==========================================================

    move_group_ =
      std::make_shared<
        moveit::planning_interface::MoveGroupInterface
      >(
        node_,
        "ur_manipulator"
      );


    // ==========================================================
    // MoveIt configuration
    // ==========================================================

    move_group_->setPlanningTime(10.0);

    move_group_->setNumPlanningAttempts(10);

    move_group_->setMaxVelocityScalingFactor(0.15);

    move_group_->setMaxAccelerationScalingFactor(0.15);

    move_group_->setPoseReferenceFrame("world");

    move_group_->setEndEffectorLink("tool0");


    // ==========================================================
    // Print MoveIt information
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "Planning frame: %s",
      move_group_->getPlanningFrame().c_str()
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "End effector: %s",
      move_group_->getEndEffectorLink().c_str()
    );


    // ==========================================================
    // Start current state monitor
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "Waiting for current robot state..."
    );

    if (!move_group_->startStateMonitor(5.0))
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Could not start current state monitor."
      );
    }

    std::this_thread::sleep_for(2s);


    RCLCPP_INFO(
      node_->get_logger(),
      "MoveGroupInterface initialized."
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "RViz marker publisher: /letter_path"
    );
  }


  // ============================================================
  // Create downward tool orientation
  // ============================================================

  geometry_msgs::msg::Quaternion downwardOrientation()
  {
    geometry_msgs::msg::Quaternion q;

    /*
     * 180-degree rotation around X axis.
     *
     * x = 1
     * y = 0
     * z = 0
     * w = 0
     */

    q.x = 1.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 0.0;

    return q;
  }


  // ============================================================
  // Create Cartesian pose
  // ============================================================

  geometry_msgs::msg::Pose createPose(
    double x,
    double y,
    double z)
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    pose.orientation =
      downwardOrientation();

    return pose;
  }


  // ============================================================
  // Publish LINE_STRIP to RViz
  // ============================================================

  void publishLetterPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    visualization_msgs::msg::Marker marker;


    // ==========================================================
    // Header
    // ==========================================================

    marker.header.frame_id = "world";

    marker.header.stamp =
      node_->now();


    // ==========================================================
    // Marker identification
    // ==========================================================

    marker.ns = "letter";

    marker.id = 0;


    // ==========================================================
    // LINE_STRIP
    // ==========================================================

    marker.type =
      visualization_msgs::msg::Marker::LINE_STRIP;

    marker.action =
      visualization_msgs::msg::Marker::ADD;


    // ==========================================================
    // Line thickness
    // ==========================================================

    marker.scale.x = 0.008;


    // ==========================================================
    // Marker lifetime
    //
    // 0 = keep marker permanently
    // ==========================================================

    marker.lifetime =
      rclcpp::Duration::from_seconds(0.0);


    // ==========================================================
    // Add points
    // ==========================================================

    for (const auto& pose : waypoints)
    {
      geometry_msgs::msg::Point point;

      point.x = pose.position.x;
      point.y = pose.position.y;
      point.z = pose.position.z;

      marker.points.push_back(point);
    }


    // ==========================================================
    // Publish
    // ==========================================================

    marker_pub_->publish(marker);


    RCLCPP_INFO(
      node_->get_logger(),
      "Published %zu points to /letter_path",
      marker.points.size()
    );
  }


  // ============================================================
  // Move to single Cartesian pose
  // ============================================================

  bool moveToPose(
    const geometry_msgs::msg::Pose& target)
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "------------------------------------------"
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "Planning to target:"
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "  x = %.3f m",
      target.position.x
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "  y = %.3f m",
      target.position.y
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "  z = %.3f m",
      target.position.z
    );


    // ==========================================================
    // Use current state as start state
    // ==========================================================

    move_group_->setStartStateToCurrentState();


    // ==========================================================
    // Set target
    // ==========================================================

    move_group_->setPoseTarget(target);


    // ==========================================================
    // Plan
    // ==========================================================

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto plan_result =
      move_group_->plan(plan);


    if (
      plan_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Planning to target FAILED."
      );

      move_group_->clearPoseTargets();

      return false;
    }


    RCLCPP_INFO(
      node_->get_logger(),
      "Planning successful."
    );


    // ==========================================================
    // Execute
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "Executing trajectory..."
    );

    auto execute_result =
      move_group_->execute(plan);


    move_group_->clearPoseTargets();


    if (
      execute_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Trajectory execution FAILED."
      );

      return false;
    }


    RCLCPP_INFO(
      node_->get_logger(),
      "Trajectory execution successful."
    );

    return true;
  }


  // ============================================================
  // Cartesian motion
  // ============================================================

  bool cartesianMove(
    const std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    if (waypoints.size() < 2)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cartesian path requires at least 2 waypoints."
      );

      return false;
    }


    RCLCPP_INFO(
      node_->get_logger(),
      "------------------------------------------"
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "Computing Cartesian path..."
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "Number of waypoints: %zu",
      waypoints.size()
    );


    // ==========================================================
    // Cartesian interpolation
    // ==========================================================

    const double eef_step = 0.005;

    const double jump_threshold = 0.0;


    // ==========================================================
    // Robot trajectory
    // ==========================================================

    moveit_msgs::msg::RobotTrajectory trajectory;


    // ==========================================================
    // Compute Cartesian path
    // ==========================================================

    const double fraction =
      move_group_->computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory,
        true
      );


    RCLCPP_INFO(
      node_->get_logger(),
      "Cartesian path: %.2f%% achieved",
      fraction * 100.0
    );


    // ==========================================================
    // Check Cartesian path
    // ==========================================================

    if (fraction < 0.99)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cartesian path incomplete."
      );

      RCLCPP_ERROR(
        node_->get_logger(),
        "Only %.2f%% achieved.",
        fraction * 100.0
      );

      return false;
    }


    // ==========================================================
    // Convert trajectory to MoveGroup plan
    // ==========================================================

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    plan.trajectory_ = trajectory;


    // ==========================================================
    // Execute
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "Executing Cartesian trajectory..."
    );

    auto result =
      move_group_->execute(plan);


    if (
      result !=
      moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cartesian trajectory execution FAILED."
      );

      return false;
    }


    RCLCPP_INFO(
      node_->get_logger(),
      "Cartesian trajectory executed successfully."
    );


    return true;
  }


  // ============================================================
  // Draw letter L
  // ============================================================

  bool drawLetterL()
  {
    RCLCPP_INFO(
      node_->get_logger(),
      "=========================================="
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "            DRAWING LETTER L"
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "==========================================");


    /*
     *
     *                  P1
     *                  |
     *                  |
     *                  |
     *                  |
     *                  |
     *                  P2 -------- P3
     *
     *
     * P1 = (0.35,  0.15, 0.25)
     * P2 = (0.35, -0.15, 0.25)
     * P3 = (0.50, -0.15, 0.25)
     *
     */


    // ==========================================================
    // Define points
    // ==========================================================

    geometry_msgs::msg::Pose p1 =
      createPose(
        0.35,
        0.15,
        0.25
      );


    geometry_msgs::msg::Pose p2 =
      createPose(
        0.35,
        -0.15,
        0.25
      );


    geometry_msgs::msg::Pose p3 =
      createPose(
        0.50,
        -0.15,
        0.25
      );


    // ==========================================================
    // Publish letter shape to RViz
    // ==========================================================

    std::vector<
      geometry_msgs::msg::Pose
    > letter_path;

    letter_path.push_back(p1);
    letter_path.push_back(p2);
    letter_path.push_back(p3);


    publishLetterPath(letter_path);


    // ==========================================================
    // STEP 1
    // Move to P1
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "STEP 1: Moving to P1..."
    );


    if (!moveToPose(p1))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Cannot reach P1."
      );

      return false;
    }


    // ==========================================================
    // STEP 2
    // Draw P1 -> P2
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "STEP 2: Drawing vertical stroke P1 -> P2..."
    );


    std::vector<
      geometry_msgs::msg::Pose
    > vertical_stroke;


    vertical_stroke.push_back(p1);
    vertical_stroke.push_back(p2);


    if (!cartesianMove(vertical_stroke))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to draw P1 -> P2."
      );

      return false;
    }


    // ==========================================================
    // STEP 3
    // Draw P2 -> P3
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "STEP 3: Drawing horizontal stroke P2 -> P3..."
    );


    std::vector<
      geometry_msgs::msg::Pose
    > horizontal_stroke;


    horizontal_stroke.push_back(p2);
    horizontal_stroke.push_back(p3);


    if (!cartesianMove(horizontal_stroke))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to draw P2 -> P3."
      );

      return false;
    }


    // ==========================================================
    // STEP 4
    // Lift tool
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "STEP 4: Lifting tool..."
    );


    geometry_msgs::msg::Pose p3_lift = p3;

    p3_lift.position.z += 0.08;


    std::vector<
      geometry_msgs::msg::Pose
    > lift_path;


    lift_path.push_back(p3);
    lift_path.push_back(p3_lift);


    if (!cartesianMove(lift_path))
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to lift tool."
      );

      return false;
    }


    // ==========================================================
    // Finished
    // ==========================================================

    RCLCPP_INFO(
      node_->get_logger(),
      "=========================================="
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "        LETTER L COMPLETED"
    );

    RCLCPP_INFO(
      node_->get_logger(),
      "==========================================");


    return true;
  }


private:

  // ============================================================
  // ROS node
  // ============================================================

  rclcpp::Node::SharedPtr node_;


  // ============================================================
  // MoveIt
  // ============================================================

  std::shared_ptr<
    moveit::planning_interface::MoveGroupInterface
  > move_group_;


  // ============================================================
  // RViz marker publisher
  // ============================================================

  rclcpp::Publisher<
    visualization_msgs::msg::Marker
  >::SharedPtr marker_pub_;
};


// =================================================================
// MAIN
// =================================================================

int main(
  int argc,
  char** argv)
{
  rclcpp::init(
    argc,
    argv
  );


  // ==============================================================
  // Node options
  //
  // Allows robot_description_kinematics from launch file
  // ==============================================================

  auto node =
    rclcpp::Node::make_shared(
      "draw_letter",

      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(
          true
        )
    );


  try
  {
    // ============================================================
    // Create drawer
    // ============================================================

    DrawLetter drawer(node);


    std::this_thread::sleep_for(2s);


    // ============================================================
    // Draw L
    // ============================================================

    const bool success =
      drawer.drawLetterL();


    // ============================================================
    // Check result
    // ============================================================

    if (!success)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Drawing failed."
      );

      rclcpp::shutdown();

      return 1;
    }


    RCLCPP_INFO(
      node->get_logger(),
      "Drawing finished successfully."
    );
  }


  catch (
    const std::exception& e)
  {
    RCLCPP_FATAL(
      node->get_logger(),
      "Exception: %s",
      e.what()
    );

    rclcpp::shutdown();

    return 1;
  }


  // ==============================================================
  // Shutdown
  // ==============================================================

  rclcpp::shutdown();

  return 0;
}
