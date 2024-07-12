#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

 class EGOPlannerManager
{
    // SECTION stable
  public:
    EGOPlannerManager();  // 构造函数
    ~EGOPlannerManager();  // 析构函数

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 使用Eigen库时，保证适当的内存对齐

    /* 主要规划接口 */
    // 弹性重规划接口
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // 紧急停止接口
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    // 规划全局轨迹
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // 通过一系列路径点规划全局轨迹
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    // 初始化规划模块
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

    // 规划参数、本地和全局轨迹数据、网格地图
    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;

  private:
    /* 主要规划算法和模块 */
    // 可视化模块
    PlanningVisualization::Ptr visualization_;

    // Bspline优化器（反弹）
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;

    // 连续失败计数
    int continous_failures_count_{0};

    // 更新轨迹信息
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

    // 重新参数化Bspline
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    // 精细化轨迹算法
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    // 类型定义，用于创建EGOPlannerManager的唯一指针
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
};

} // namespace ego_planner

#endif