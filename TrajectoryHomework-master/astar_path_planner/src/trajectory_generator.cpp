#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseStamped.h>

typedef std::vector<Eigen::Vector2d> Path;

class TrajectoryGenerator {
public:
    // 构造函数：初始化地图尺寸、分辨率等参数
    TrajectoryGenerator(int width, int height, double m_min, double m_max, double res)
        : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res) {
    }

    // 生成轨迹：基于输入路径生成六次多项式轨迹
    std::vector<Eigen::Vector2d> GenerateTrajectory(const std::vector<Eigen::Vector2d>& path) {
        std::vector<Eigen::Vector2d> trajectory;
        if (path.size() < 2) {
            return trajectory; 
        }

        // 假定每段轨迹时间为1秒
        double dt = 1.0;

        for (size_t i = 0; i < path.size() - 1; ++i) {
            Eigen::Vector2d start = path[i];  // 当前路径点
            Eigen::Vector2d end = path[i + 1];  // 下一个路径点

            // 计算六次多项式的系数
            Eigen::VectorXd x_coeffs = computeSixthOrderPolynomial(start.x(), 0.0, 0.0, 0.0, end.x(), 0.0, 0.0, 0.0, dt);
            Eigen::VectorXd y_coeffs = computeSixthOrderPolynomial(start.y(), 0.0, 0.0, 0.0, end.y(), 0.0, 0.0, 0.0, dt);

            // 生成轨迹点：根据时间间隔逐步计算轨迹
            for (double t = 0; t <= dt; t += 0.1) {
                double x = 0.0;
                double y = 0.0;
                for (int j = 0; j < 6; ++j) {
                    x += x_coeffs[j] * std::pow(t, j);  // 计算 x 坐标
                    y += y_coeffs[j] * std::pow(t, j);  // 计算 y 坐标
                }
                trajectory.push_back(Eigen::Vector2d(x, y));  // 保存轨迹点
            }
        }

        return trajectory;  // 返回生成的完整轨迹
    }

private:
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;

    // 计算六次多项式系数：利用边界条件（位置、速度、加速度等）
    Eigen::VectorXd computeSixthOrderPolynomial(double start_pos, double start_vel, double start_acc, double start_jerk,
        double end_pos, double end_vel, double end_acc, double end_jerk, double dt) {
        Eigen::MatrixXd M(6, 6);  // 六次多项式的系数矩阵
        M << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, dt, std::pow(dt, 2), std::pow(dt, 3), std::pow(dt, 4), std::pow(dt, 5),
            0, 1, 2 * dt, 3 * std::pow(dt, 2), 4 * std::pow(dt, 3), 5 * std::pow(dt, 4),
            0, 0, 2, 6 * dt, 12 * std::pow(dt, 2), 20 * std::pow(dt, 3);

        Eigen::VectorXd boundary_conditions(6);  // 存储起始和结束的边界条件
        boundary_conditions << start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;

        // 求解线性方程组，得到六次多项式系数
        return M.colPivHouseholderQr().solve(boundary_conditions);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    ros::Publisher trajectory_pub = nh.advertise<nav_msgs::Path>("generated_trajectory", 1);
    ros::Rate loop_rate(10);  // 设置ROS循环频率为10Hz

    // 初始化路径变量和标志位
    Path received_path;
    bool path_received = false;

    // 订阅路径消息：接收外部输入的路径
    ros::Subscriber path_subscriber = nh.subscribe<nav_msgs::Path>("path", 1,
        [&received_path, &path_received](const nav_msgs::Path::ConstPtr& msg) {
            received_path.clear();  // 清空已有路径
            for (const auto& pose : msg->poses) {
                received_path.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));  // 存储路径点
            }
            path_received = true;  // 标记路径已接收
        });

    // 创建轨迹生成器对象
    TrajectoryGenerator generator(100, 100, -5.0, 5.0, 0.1);
    std::vector<Eigen::Vector2d> trajectory_points;

    while (ros::ok()) {
        ros::spinOnce();  // 调用回调函数，处理ROS消息

        if (path_received) {
            ROS_INFO("New path received. Path length: %zu", received_path.size());  // 输出路径点的数量
            trajectory_points = generator.GenerateTrajectory(received_path);  // 生成轨迹
            path_received = false;  // 重置路径接收标志
        }

        // 发布生成的轨迹
        if (!trajectory_points.empty()) {
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

            trajectory_pub.publish(trajectory_msg);  // 发布轨迹消息
        }

        loop_rate.sleep();  // 按照设定的频率休眠
    }

    return 0;
}

