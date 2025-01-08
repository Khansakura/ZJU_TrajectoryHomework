# ZJU_TrajectoryHomework
2024-2025冬智能移动机器人课程大作业
### 小组成员：
周克涵、贾文骏、蒋益斌  
### 分工：
周克涵：任务一A\*算法框架Step1、Step2、Step3撰写完善、改进A\*算法的实现。  
蒋益斌：任务二轨迹优化方法实现，六阶多项式轨迹规划方法实现，三阶贝塞尔轨迹规划方法实现。
贾文骏：实验报告撰写。
### 主要工作：
调研了相关文献，完善基本A*框架，在此基础上实现改进A\*算法，添加双向A\*算法，重构reconstruction函数，在原先启发式函数的基础上添加了多启发式函数加权融合，实现了六阶多项式轨迹规划方法实现和三阶贝塞尔轨迹规划方法实现,分析了机器人路径运动的速度和加速度曲线分析，分析总结了实验结果，撰写了实验报告。  
### 运行方法：
clone本仓库到工作空间src文件夹下  
编译工作空间
```
catkin_make
source devel/setup.sh
```
运行代码，将打开rviz显示结果
```
roslaunch astar_path_planner astar_planner.launch
```
编译无报错、rviz显示随机生成圆柱障碍物和绿色直线初始路径时成功
### 结果展示
如Result.md所示。  
A\*算法  
![Image text](https://github.com/Khansakura/ZJU_TrajectoryHomework/blob/main/png/Astar.jpg)  
优化的双向A\*算法  
![Image text](https://github.com/Khansakura/ZJU_TrajectoryHomework/blob/main/png/ImprovedAstar.jpg)  
六阶多项式
![Image text](https://github.com/Khansakura/ZJU_TrajectoryHomework/blob/main/png/1.jpg)  
三阶贝塞尔
![Image text](https://github.com/Khansakura/ZJU_TrajectoryHomework/blob/main/png/2.jpg)  
