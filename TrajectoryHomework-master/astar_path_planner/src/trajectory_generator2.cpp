#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef std::vector<Eigen::Vector2d> Path;

// �켣��������
class TrajectoryGenerator {
public:
    // ���캯������ʼ����ͼ�ߴ硢�ֱ��ʵȲ���
    TrajectoryGenerator(int width, int height, double min_val, double max_val, double resolution)
        : width_(width), height_(height), min_val_(min_val), max_val_(max_val), resolution_(resolution) {
        if (width_ <= 0 || height_ <= 0) {
            ROS_ERROR("Invalid map dimensions: %d x %d", width_, height_);
            throw std::invalid_argument("Invalid map dimensions");
        }
    }

    // ���ɹ켣����������·�����ɱ��������߹켣
    std::vector<Eigen::Vector2d> generateTrajectory(const Path& path) {
        if (path.size() < 2) {
            ROS_WARN("Path has insufficient points to generate a trajectory.");
            return {};  // ���·���������������޷����ɹ켣
        }

        std::vector<Eigen::Vector2d> trajectory;

        // ����·�������ɱ���������
        for (size_t i = 0; i < path.size() - 1; ++i) {
            Eigen::Vector2d start = path[i];
            Eigen::Vector2d end = path[i + 1];

            // ���ɱ��������ߵĿ��Ƶ�
            Eigen::Vector2d control1 = start + 0.4 * (end - start);  // ���Ƶ�1
            Eigen::Vector2d control2 = end - 0.4 * (end - start);    // ���Ƶ�2

            // ���ɹ켣�㣺ͨ�����Ʋ����������������ߵ�ƽ����
            for (double t = 0; t <= 1.0; t += 0.05) {  // ����ϸ�Ĺ켣���ɣ�������С��
                Eigen::Vector2d point = bezierCurve(start, control1, control2, end, t);
                trajectory.push_back(point);
            }
        }

        return trajectory;
    }

private:
    int width_, height_;
    double min_val_, max_val_, resolution_;

    // ���㱴�������ߵĵ㣨4�����Ƶ�����α��������ߣ�
    Eigen::Vector2d bezierCurve(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1,
        const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, double t) {
        double u = 1.0 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;

        // ���������߹�ʽ
        Eigen::Vector2d point = uuu * p0 + 3 * uu * t * p1 + 3 * u * tt * p2 + ttt * p3;
        return point;
    }
};

// ROS�ڵ��࣬������·�������ɹ켣
class TrajectoryNode {
public:
    TrajectoryNode(ros::NodeHandle& nh) : nh_(nh), path_received_(false) {
        // ��ʼ��������
        trajectory_pub_ = nh_.advertise<nav_msgs::Path>("generated_trajectory", 1);
        // ����·����Ϣ
        path_sub_ = nh_.subscribe("path", 1, &TrajectoryNode::pathCallback, this);

        // �����켣����������
        generator_ = std::make_shared<TrajectoryGenerator>(100, 100, -5.0, 5.0, 0.1);
    }

    // �ص������������ⲿ�����·��
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        received_path_.clear();
        for (const auto& pose : msg->poses) {
            received_path_.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
        }
        path_received_ = true;
        ROS_INFO("Path received with %zu points.", received_path_.size());
    }

    // ���нڵ㣺���ݽ��յ���·�����ɹ켣������
    void run() {
        ros::Rate loop_rate(10);  // ����ROSѭ��Ƶ��Ϊ10Hz

        while (ros::ok()) {
            ros::spinOnce();  // ���ûص�����������ROS��Ϣ

            if (path_received_) {
                // �������ɹ켣�ķ���
                std::vector<Eigen::Vector2d> trajectory_points = generator_->generateTrajectory(received_path_);
                publishTrajectory(trajectory_points);  // �������ɵĹ켣
                path_received_ = false;  // ����·�����ձ�־
            }

            loop_rate.sleep();  // �����趨��Ƶ������
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub_;
    ros::Subscriber path_sub_;
    std::shared_ptr<TrajectoryGenerator> generator_;
    Path received_path_;
    bool path_received_;

    // �������ɵĹ켣
    void publishTrajectory(const std::vector<Eigen::Vector2d>& trajectory_points) {
        if (trajectory_points.empty()) {
            ROS_WARN("No trajectory points to publish.");
            return;
        }

        nav_msgs::Path trajectory_msg;
        trajectory_msg.header.frame_id = "map";  // �趨�켣������ϵΪmap
        trajectory_msg.header.stamp = ros::Time::now();  // ����ʱ���

        // ���켣��ת��Ϊ��Ϣ����ӵ��켣��Ϣ��
        for (const auto& point : trajectory_points) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0;  // �켣��ƽ��·����z������Ϊ0
            trajectory_msg.poses.push_back(pose);
        }

        trajectory_pub_.publish(trajectory_msg);  // �����켣��Ϣ
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh;

    // ����ROS�ڵ��������
    TrajectoryNode trajectory_node(nh);
    trajectory_node.run();

    return 0;
}
