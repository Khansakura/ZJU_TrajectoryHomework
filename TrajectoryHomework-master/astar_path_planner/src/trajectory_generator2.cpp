#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef std::vector<Eigen::Vector2d> Path;

// 轨迹生成器类
class TrajectoryGenerator {
public:
    // 构造函数：初始化地图尺寸、分辨率等参数
    TrajectoryGenerator(int width, int height, double min_val, double max_val, double resolution)
        : width_(width), height_(height), min_val_(min_val), max_val_(max_val), resolution_(resolution) {
        if (width_ <= 0 || height_ <= 0) {
            ROS_ERROR("Invalid map dimensions: %d x %d", width_, height_);
            throw std::invalid_argument("Invalid map dimensions");
        }
    }

    // 生成轨迹：基于输入路径生成贝塞尔曲线轨迹
    std::vector<Eigen::Vector2d> generateTrajectory(const Path& path) {
        if (path.size() < 2) {
            ROS_WARN("Path has insufficient points to generate a trajectory.");
            return {};  // 如果路径点少于两个，无法生成轨迹
        }

        std::vector<Eigen::Vector2d> trajectory;

        // 遍历路径，生成贝塞尔曲线
        for (size_t i = 0; i < path.size() - 1; ++i) {
            Eigen::Vector2d start = path[i];
            Eigen::Vector2d end = path[i + 1];

            // 生成贝塞尔曲线的控制点
            Eigen::Vector2d control1 = start + 0.4 * (end - start);  // 控制点1
            Eigen::Vector2d control2 = end - 0.4 * (end - start);    // 控制点2

            // 生成轨迹点：通过控制参数调整贝塞尔曲线的平滑度
            for (double t = 0; t <= 1.0; t += 0.05) {  // 更精细的轨迹生成（步长更小）
                Eigen::Vector2d point = bezierCurve(start, control1, control2, end, t);
                trajectory.push_back(point);
            }
        }

        return trajectory;
    }

private:
    int width_, height_;
    double min_val_, max_val_, resolution_;

    // 计算贝塞尔曲线的点（4个控制点的三次贝塞尔曲线）
    Eigen::Vector2d bezierCurve(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
        const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, double t) {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        // 贝塞尔曲线公式
        Eigen::Vector2d point = uuu * p0 + 3 * uu * t * p1 + 3 * u * tt * p2 + ttt * p3;
        return point;
    }
};

// ROS节点类，负责订阅路径并生成轨迹
class TrajectoryNode {
public:
    TrajectoryNode(ros::NodeHandle& nh) : nh_(nh), path_received_(false) {
        // 初始化发布器
        trajectory_pub_ = nh_.advertise<nav_msgs::Path>("generated_trajectory", 1);
        // 订阅路径消息
        path_sub_ = nh_.subscribe("path", 1, &TrajectoryNode::pathCallback, this);

        // 创建轨迹生成器对象
        generator_ = std::make_shared<TrajectoryGenerator>(100, 100, -5.0, 5.0, 0.1);
    }

    // 回调函数：接收外部输入的路径
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        received_path_.clear();
        for (const auto& pose : msg->poses) {
            received_path_.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
        }
        path_received_ = true;
        ROS_INFO("Path received with %zu points.", received_path_.size());
    }

    // 运行节点：根据接收到的路径生成轨迹并发布
    void run() {
        ros::Rate loop_rate(10);  // 设置ROS循环频率为10Hz

        while (ros::ok()) {
            ros::spinOnce();  // 调用回调函数，处理ROS消息

            if (path_received_) {
                // 调用生成轨迹的方法
                std::vector<Eigen::Vector2d> trajectory_points = generator_->generateTrajectory(received_path_);
                publishTrajectory(trajectory_points);  // 发布生成的轨迹
                path_received_ = false;  // 重置路径接收标志
            }

            loop_rate.sleep();  // 按照设定的频率休眠
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub_;
    ros::Subscriber path_sub_;
    std::shared_ptr<TrajectoryGenerator> generator_;
    Path received_path_;
    bool path_received_;

    // 发布生成的轨迹
    void publishTrajectory(const std::vector<Eigen::Vector2d>& trajectory_points) {
        if (trajectory_points.empty()) {
            ROS_WARN("No trajectory points to publish.");
            return;
        }

        nav_msgs::Path trajectory_msg;
        trajectory_msg.header.frame_id = "map";  // 设定轨迹的坐标系为map
        trajectory_msg.header.stamp = ros::Time::now();  // 设置时间戳

        // 将轨迹点转换为消息并添加到轨迹消息中
        for (const auto& point : trajectory_points) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0;  // 轨迹是平面路径，z坐标设为0
            trajectory_msg.poses.push_back(pose);
        }

        trajectory_pub_.publish(trajectory_msg);  // 发布轨迹消息
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh;

    // 创建ROS节点对象并运行
    TrajectoryNode trajectory_node(nh);
    trajectory_node.run();

    return 0;
}
