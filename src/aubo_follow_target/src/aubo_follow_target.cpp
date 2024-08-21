#include "aubo_follow_target.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

const std::string MOVE_GROUP = "aubo_arm";

MoveItFollowTarget::MoveItFollowTarget() : Node("aubo_follow_target"),
                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
    plan_counts_ = 0;
    // 设置关节速度和加速度上限
    this->move_group_.setMaxAccelerationScalingFactor(1);
    this->move_group_.setMaxVelocityScalingFactor(1);

    // this->move_group_.setPlanningTime(10.0);      // 设置规划时间更长，以便找到更优的路径
    // this->move_group_.setNumPlanningAttempts(10); // 尝试更多次规划

    // 确保碰撞对象的参考坐标系与运动规划的参考坐标系一致
    collision_object_.header.frame_id = move_group_.getPlanningFrame();
    // 定义障碍物
    collision_object_.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    // 障碍物尺寸
    primitive.dimensions[primitive.BOX_X] = 10.0;
    primitive.dimensions[primitive.BOX_Y] = 10.0;
    primitive.dimensions[primitive.BOX_Z] = 0.01;
    // 障碍物位置
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = 0;
    // 将障碍物添加到碰撞对象中
    collision_object_.primitives.push_back(primitive);
    collision_object_.primitive_poses.push_back(box_pose);
    collision_object_.operation = collision_object_.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object_);

    RCLCPP_INFO(this->get_logger(), "Add an object into the world");
    planning_scene_interface_.addCollisionObjects(collision_objects);

    // 订阅目标位姿
    // PoseStamped 是一个包含位姿（位置和方向）信息的消息类型
    // QoS(1)，1 表示队列的深度，即订阅者在处理消息时可以缓存的最大消息数量。
    // std::placeholders::_1 作为占位符，表示回调函数将接收一个参数
    // 订阅话题为 /target_pose，当有消息到达时，调用 target_pose_callback 函数
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose", rclcpp::QoS(1), std::bind(&MoveItFollowTarget::target_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void MoveItFollowTarget::target_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    // 获取当前末端执行器的位姿
    if (abs(msg->pose.position.x - previous_target_pose_.position.x) < 0.01 && abs(msg->pose.position.y - previous_target_pose_.position.y) < 0.01 && abs(msg->pose.position.z - previous_target_pose_.position.z) < 0.01)

    {
        return;
    }

    // 定义路径点数组
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(previous_target_pose_); // 当前位姿作为路径的起点

    // 目标位姿
    geometry_msgs::msg::Pose target_pose = msg->pose;
    waypoints.push_back(target_pose); // 目标位姿作为路径的终点

    // 增加更多路径点
    geometry_msgs::msg::Pose intermediate_pose = previous_target_pose_;
    intermediate_pose.position.x += (target_pose.position.x - previous_target_pose_.position.x) / 2;
    intermediate_pose.position.y += (target_pose.position.y - previous_target_pose_.position.y) / 2;
    intermediate_pose.position.z += (target_pose.position.z - previous_target_pose_.position.z) / 2;
    waypoints.insert(waypoints.begin() + 1, intermediate_pose);

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;    // 末端执行器的步长
    const double jump_threshold = 0; // 跳跃阈值

    move_group_.setGoalTolerance(0.01); // 设置目标位姿的容差
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // 检查路径规划是否成功
    if (fraction < 0.9) // 若成功率低于 %，视为失败
    {
        RCLCPP_WARN(this->get_logger(), "笛卡尔路径规划成功率低于预期: %f", fraction);
        // 设置目标位姿并执行
        this->move_group_.setPoseTarget(msg->pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (this->move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "执行规划: %s", success ? "成功" : "失败");
        if (success)
        {
            this->move_group_.execute(plan);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划成功: %f", fraction);

        // 执行规划的轨迹
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_.execute(plan);
    }
    // 更新用于下次回调的位姿
    previous_target_pose_ = msg->pose;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto target_follower = std::make_shared<MoveItFollowTarget>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(target_follower);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
