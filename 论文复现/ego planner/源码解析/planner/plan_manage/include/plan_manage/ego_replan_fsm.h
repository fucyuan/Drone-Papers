#ifndef _REBO_REPLAN_FSM_H_  // 如果没有定义_REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_  // 定义_REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>  // 引入Eigen库，用于矩阵和向量运算
#include <algorithm>  // 引入算法库
#include <iostream>  // 引入输入输出流库
#include <nav_msgs/Path.h>  // 引入nav_msgs下的Path消息类型，用于导航路径
#include <sensor_msgs/Imu.h>  // 引入sensor_msgs下的Imu消息类型，用于惯性测量单元数据
#include <ros/ros.h>  // 引入ROS主要头文件
#include <std_msgs/Empty.h>  // 引入std_msgs下的Empty消息类型
#include <vector>  // 引入向量库

#include <visualization_msgs/Marker.h>  // 引入可视化消息类型

#include <bspline_opt/bspline_optimizer.h>  // 引入B样条优化器
#include <plan_env/grid_map.h>  // 引入网格地图
#include <ego_planner/Bspline.h>  // 引入ego_planner下的Bspline类型
#include <ego_planner/DataDisp.h>  // 引入ego_planner下的DataDisp类型
#include <plan_manage/planner_manager.h>  // 引入规划管理器
#include <traj_utils/planning_visualization.h>  // 引入轨迹规划可视化工具

using std::vector;  // 使用std命名空间下的vector

namespace ego_planner
{

  class EGOReplanFSM  // EGO重规划飞行器状态机类
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE  // FSM执行状态枚举
    {
      INIT,  // 初始化
      WAIT_TARGET,  // 等待目标
      GEN_NEW_TRAJ,  // 生成新轨迹
      REPLAN_TRAJ,  // 重规划轨迹
      EXEC_TRAJ,  // 执行轨迹
      EMERGENCY_STOP  // 紧急停止
    };
    enum TARGET_TYPE  // 目标类型枚举
    {
      MANUAL_TARGET = 1,  // 手动目标
      PRESET_TARGET = 2,  // 预设目标
      REFENCE_PATH = 3  // 参考路径
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;  // 规划管理器指针
    PlanningVisualization::Ptr visualization_;  // 规划可视化指针
    ego_planner::DataDisp data_disp_;  // 数据显示

    /* parameters */
    int target_type_; // 目标类型：1手动选择，2硬编码
    double no_replan_thresh_, replan_thresh_;  // 无需重规划和需重规划的阈值
    double waypoints_[50][3];  // 路径点数组
    int waypoint_num_;  // 路径点数量
    double planning_horizen_, planning_horizen_time_;  // 规划视界和规划视界时间
    double emergency_time_;  // 紧急情况时间

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;  // 触发器，有无目标，有无里程计，有无新目标标志
    FSM_EXEC_STATE exec_state_;  // 执行状态
    int continously_called_times_{0};  // 连续调用次数

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // 里程计位置，速度，加速度
    Eigen::Quaterniond odom_orient_;  // 里程计方向

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // 初始点，起始点，起始速度，起始加速度，起始偏航
    Eigen::Vector3d end_pt_, end_vel_;  // 结束点，结束速度
    Eigen::Vector3d local_target_pt_, local_target_vel_;  // 本地目标点，本地目标速度
    int current_wp_;  // 当前路径点

    bool flag_escape_emergency_;  // 标志是否逃逸紧急情况

    /* ROS utils */
    ros::NodeHandle node_;  // ROS节点句柄
    ros::Timer exec_timer_, safety_timer_;  // 执行定时器，安全定时器
    ros::Subscriber waypoint_sub_, odom_sub_;  // 路径点订阅者，里程计订阅者
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;  // 重规划发布者，新发布者，B样条发布者，数据显示发布者

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // 前端和后端方法，调用反弹重规划
    bool callEmergencyStop(Eigen::Vector3d stop_pos);  // 前端和后端方法，调用紧急停止
    bool planFromCurrentTraj();  // 从当前轨迹规划

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);  // 改变FSM执行状态
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();  // 返回连续状态调用的次数
    void printFSMExecState();  // 打印FSM执行状态

    void planGlobalTrajbyGivenWps();  // 通过给定路径点规划全局轨迹
    void getLocalTarget();  // 获取本地目标

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);  // 执行FSM回调
    void checkCollisionCallback(const ros::TimerEvent &e);  // 检查碰撞回调
    void waypointCallback(const nav_msgs::PathConstPtr &msg);  // 路径点回调
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);  // 里程计回调

    bool checkCollision();  // 检查碰撞

  public:
    EGOReplanFSM(/* args */)  // 构造函数
    {
    }
    ~EGOReplanFSM()  // 析构函数
    {
    }

    void init(ros::NodeHandle &nh);  // 初始化函数

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏
  };

} // namespace ego_planner

#endif
