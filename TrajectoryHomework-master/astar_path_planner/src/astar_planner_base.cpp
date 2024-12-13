#include <ros/ros.h>
#include <utility>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"


struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值
};

// 比较器，用于优先队列
struct cmp {
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        return a->f() > b->f();
    }
};

struct GridMap {
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res) : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius) {
        // 将世界坐标(cx, cy)转换为网格坐标
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution + 1);

        // Step 1: 将圆形区域标记为占用
        for (int i = -grid_radius; i <= grid_radius; ++i) {
            for (int j = -grid_radius; j <= grid_radius; ++j) {
                // 计算当前网格的坐标
                int nx = grid_cx + i;
                int ny = grid_cy + j;

                // 检查当前网格是否在地图范围内
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // 计算当前网格与圆心的距离
                    if (i * i + j * j <= grid_radius * grid_radius) {
                        grid[nx][ny] = 1; // 标记为占用
                    }
                }
            }
        }
    }
};

class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res)
            : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res),
              grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) {}

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    void printGridMap() {
        for (int i = 0; i < width_; i++) {
            for (int j = 0; j < height_; j++) {
                std::cout << grid_map_.grid[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "num of obstacles: " << num_of_obs_ << std::endl;
    }

    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
	ros::Time start_time = ros::Time::now();
	if (num_of_obs_ == 0) {
            return {};
        }

        // 起点和终点转换为网格坐标
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        // 开放列表和关闭列表
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        // 起点加入开放列表
        open_list.push(std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, 0.0)));

        // Step 3： 实现 A* 算法，搜索结束调用 reconstructPath 返回路径
        while (!open_list.empty()) {
            auto current = open_list.top();
            open_list.pop();

            if (closed_list[current->x][current->y]) {
                continue;
            }

            closed_list[current->x][current->y] = true;

            if (current->x == gridGoal.first && current->y == gridGoal.second) {
		ros::Time end_time = ros::Time::now();
		ros::Duration time_taken = end_time - start_time;
		ROS_INFO("FindPath took:%f seconds", time_taken.toSec());
                return reconstructPath(current);
            }

            for (const auto& neighbor : getNeighbors(*current)) {
                if (closed_list[neighbor.x][neighbor.y]) {
                    continue;
                }

                // 计算新的g_cost
                double new_g_cost = current->g_cost + distance(*current, Node(neighbor.x, neighbor.y, 0, 0));
                double new_h_cost = heuristic({neighbor.x, neighbor.y}, gridGoal);

                // 创建新的邻居节点
                auto neighbor_node = std::make_shared<Node>(neighbor.x, neighbor.y, new_g_cost, new_h_cost, current);

                // 如果该邻居已经在开放列表中，且发现一个更短的路径，更新该邻居的代价
                open_list.push(neighbor_node);
            }
        }

        // 如果没有找到路径，返回空路径
        return {};
    }

    void reset() {
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }

private:
    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    // 获取当前节点的所有邻居节点
    std::vector<Node> getNeighbors(const Node& current) {
        std::vector<Node> neighbors;

        // 八连通邻居
        std::vector<std::pair<int, int>> directions = {
                {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        
        // 遍历每一个方向
        for (const auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            // 检查邻居是否在地图范围内
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                // 检查该位置是否是障碍物
                if (grid_map_.grid[nx][ny] == 0) {  // 空闲区域
                    double g_cost = current.g_cost + distance(current, Node(nx, ny, 0, 0)); // 更新代价
                    neighbors.push_back(Node(nx, ny, g_cost, 0, std::make_shared<Node>(current)));
                }
            }
        }

        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset();
        return path;
    }

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);

    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);
    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });

    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
        ros::spinOnce();
        // 执行路径搜索
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);
	
	//路径长度输出，但随机性导致路径长度没有对比意义
	//double path_length = 0.0;
	//for (size_t i = 1; i < path.size(); ++i){
	//	path_length += (path[i] - path[i-1]).norm();
	//}
	//ROS_INFO("Path Length: %f", path_length);
        // 路径可视化
        if (path.empty()) {
            continue;
        }
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}