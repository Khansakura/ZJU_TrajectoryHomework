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
    if (num_of_obs_ == 0) {
        return {};
    }
    // 开始计时
    ros::Time start_time = ros::Time::now();
    // 将起点和终点转换为网格坐标
    auto gridStart = worldToGrid(start);
    auto gridGoal = worldToGrid(goal);

    // 开放列表和关闭列表
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list_start;
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list_goal;

    std::vector<std::vector<bool>> closed_list_start(width_, std::vector<bool>(height_, false));
    std::vector<std::vector<bool>> closed_list_goal(width_, std::vector<bool>(height_, false));

    // 起点和终点分别加入开放列表
    auto start_node = std::make_shared<Node>(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal));
    auto goal_node = std::make_shared<Node>(gridGoal.first, gridGoal.second, 0.0, heuristic(gridGoal, gridStart));

    open_list_start.push(start_node);
    open_list_goal.push(goal_node);

    // 存储两个方向的节点访问情况，用于合并路径
    std::map<std::pair<int, int>, std::shared_ptr<Node>> visited_from_start;
    std::map<std::pair<int, int>, std::shared_ptr<Node>> visited_from_goal;

    visited_from_start[{gridStart.first, gridStart.second}] = start_node;
    visited_from_goal[{gridGoal.first, gridGoal.second}] = goal_node;

    std::pair<int, int> meeting_point;
    bool path_found = false;

    while (!open_list_start.empty() && !open_list_goal.empty()) {
        // 正向搜索一步
        if (!open_list_start.empty()) {
            auto current_start = open_list_start.top();
            open_list_start.pop();

            auto current_pos = std::make_pair(current_start->x, current_start->y);

            if (closed_list_start[current_start->x][current_start->y]) {
                continue;
            }
            closed_list_start[current_start->x][current_start->y] = true;

            // 检查是否在反向搜索中遇到
            if (visited_from_goal.find(current_pos) != visited_from_goal.end()) {
                meeting_point = current_pos;
                path_found = true;
                break;
            }

            // 遍历邻居节点
            for (const auto& neighbor : getNeighbors(*current_start)) {
                if (closed_list_start[neighbor.x][neighbor.y]) {
                    continue;
                }

                double new_g_cost = current_start->g_cost + distance(*current_start, neighbor);
                auto neighbor_node = std::make_shared<Node>(neighbor.x, neighbor.y, new_g_cost, heuristic({neighbor.x, neighbor.y}, gridGoal), current_start);

                if (visited_from_start.find({neighbor.x, neighbor.y}) == visited_from_start.end() ||
                    new_g_cost < visited_from_start[{neighbor.x, neighbor.y}]->g_cost) {
                    open_list_start.push(neighbor_node);
                    visited_from_start[{neighbor.x, neighbor.y}] = neighbor_node;
                }
            }
        }

        // 反向搜索一步
        if (!open_list_goal.empty()) {
            auto current_goal = open_list_goal.top();
            open_list_goal.pop();

            auto current_pos = std::make_pair(current_goal->x, current_goal->y);

            if (closed_list_goal[current_goal->x][current_goal->y]) {
                continue;
            }
            closed_list_goal[current_goal->x][current_goal->y] = true;

            // 检查是否在正向搜索中遇到
            if (visited_from_start.find(current_pos) != visited_from_start.end()) {
                meeting_point = current_pos;
                path_found = true;
                break;
            }

            // 遍历邻居节点
            for (const auto& neighbor : getNeighbors(*current_goal)) {
                if (closed_list_goal[neighbor.x][neighbor.y]) {
                    continue;
                }

                double new_g_cost = current_goal->g_cost + distance(*current_goal, neighbor);
                auto neighbor_node = std::make_shared<Node>(neighbor.x, neighbor.y, new_g_cost, heuristic({neighbor.x, neighbor.y}, gridStart), current_goal);

                if (visited_from_goal.find({neighbor.x, neighbor.y}) == visited_from_goal.end() ||
                    new_g_cost < visited_from_goal[{neighbor.x, neighbor.y}]->g_cost) {
                    open_list_goal.push(neighbor_node);
                    visited_from_goal[{neighbor.x, neighbor.y}] = neighbor_node;
                }
            }
        }
    }

    if (path_found) {
	    // 打印路径时间
	    ros::Time end_time = ros::Time::now();
            ros::Duration time_taken = end_time - start_time;
            ROS_INFO("FindPath took:%f seconds", time_taken.toSec());
	    return reconstructBidirectionalPath(visited_from_start[meeting_point], visited_from_goal[meeting_point]);
    }

    // 如果未找到路径，返回空路径
    return {};
}



    void reset() {
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }

private:
    // 欧几里得距离启发函数
    double euclideanHeuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 曼哈顿距离启发函数
    double manhattanHeuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::abs(from.first - to.first) + std::abs(from.second - to.second);
    }

    // 切比雪夫距离启发函数
    double chebyshevHeuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::max(std::abs(from.first - to.first), std::abs(from.second - to.second));
    }

    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
    //    return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
        double w_euclidean = 0.5; // 欧几里得距离权重
        double w_manhattan = 0.3; // 曼哈顿距离权重
        double w_chebyshev = 0.2; // 切比雪夫距离权重

        double h_euclidean = euclideanHeuristic(from, to);
        double h_manhattan = manhattanHeuristic(from, to);
        double h_chebyshev = chebyshevHeuristic(from, to);

        return w_euclidean * h_euclidean + w_manhattan * h_manhattan + w_chebyshev * h_chebyshev;
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

    // 计算距离（用于路径距离统计）
    double calculateDistance(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2){
    	return (point1 - point2).norm();
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

    std::vector<Eigen::Vector2d> reconstructBidirectionalPath(
    std::shared_ptr<Node> meeting_node_from_start,
    std::shared_ptr<Node> meeting_node_from_goal) {

    std::vector<Eigen::Vector2d> path;

    // 从正向搜索的相遇点回溯到起点
    std::vector<Eigen::Vector2d> path_from_start;
    auto current = meeting_node_from_start;
    while (current != nullptr) {
        path_from_start.push_back(gridToWorld(current->x, current->y));
        current = current->parent;
    }
    std::reverse(path_from_start.begin(), path_from_start.end());

    // 从反向搜索的相遇点回溯到终点
    std::vector<Eigen::Vector2d> path_from_goal;
    current = meeting_node_from_goal->parent;  // 注意：跳过 meeting_node_from_goal，避免重复相遇点
    while (current != nullptr) {
        path_from_goal.push_back(gridToWorld(current->x, current->y));
        current = current->parent;
    }

    // 合并两部分路径
    path.insert(path.end(), path_from_start.begin(), path_from_start.end());
    path.insert(path.end(), path_from_goal.begin(), path_from_goal.end());
	
    // 打印路径长度
    double total_length = 0.0;
    for (size_t i = 1; i < path.size(); ++i){
    	total_length += calculateDistance(path[i-1], path[i]);
    }
    ROS_INFO("Total Path Length = %f", total_length);
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
