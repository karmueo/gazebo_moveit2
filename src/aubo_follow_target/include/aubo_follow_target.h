#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

class MoveItFollowTarget : public rclcpp::Node
{
public:
    /// Constructor
    MoveItFollowTarget();

private:
    /// 回调函数，每次目标位姿改变时规划并执行
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

public:
    /// Move group 接口
    moveit::planning_interface::MoveGroupInterface move_group_;
    /// 订阅目标位姿
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    /// 用于检测位姿变化的目标位姿
    geometry_msgs::msg::Pose previous_target_pose_;
    // 障碍
    moveit_msgs::msg::CollisionObject collision_object_;
    // 碰撞检测接口
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    // 规划次数
    int plan_counts_;
};