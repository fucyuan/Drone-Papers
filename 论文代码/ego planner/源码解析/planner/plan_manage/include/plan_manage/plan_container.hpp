#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace ego_planner
{

 class GlobalTrajData
{
private:
public:
    PolynomialTraj global_traj_;  // 全局轨迹对象
    vector<UniformBspline> local_traj_;  // 本地轨迹对象的数组

    double global_duration_;  // 全局轨迹的持续时间
    ros::Time global_start_time_;  // 全局轨迹的开始时间
    double local_start_time_, local_end_time_;  // 本地轨迹的开始和结束时间
    double time_increase_;  // 时间增量
    double last_time_inc_;  // 上一个时间增量
    double last_progress_time_;  // 上一次进度时间

    GlobalTrajData(/* args */) {}  // 构造函数

    ~GlobalTrajData() {}  // 析构函数

    // 检查本地轨迹是否达到目标
    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    // 设置全局轨迹
    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
{
    global_traj_ = traj; // 将传入的轨迹赋值给成员变量 global_traj_
    global_traj_.init(); // 初始化 global_traj_，可能涉及到计算或设置内部状态

    global_duration_ = global_traj_.getTimeSum(); // 计算并设置全局轨迹的总持续时间

    global_start_time_ = time; // 设置全局轨迹的开始时间

    // 清空并重置与局部轨迹相关的成员变量
    local_traj_.clear(); // 清空局部轨迹
    local_start_time_ = -1; // 将局部轨迹的开始时间重置为-1，表示未设置
    local_end_time_ = -1; // 将局部轨迹的结束时间重置为-1，表示未设置
    time_increase_ = 0.0; // 将时间增量重置为0
    last_time_inc_ = 0.0; // 将上一次时间增量重置为0
    last_progress_time_ = 0.0; // 将上一次进展时间重置为0
}


    // 设置本地轨迹
    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    // 获取指定时间的位置
    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // 获取指定时间的速度
    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // 获取指定时间的加速度
    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // 根据半径获取Bspline参数化数据
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // 截断段的长度
      double seg_time = 0.0;   // 截断段的持续时间
      double radius = 0.0;     // 到第一个点的距离

      double delt = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // 段的第一个点
      Eigen::Vector3d prev_pt = first_pt;              // 前一个点
      Eigen::Vector3d cur_pt;                          // 当前点

      // 向前遍历直到轨迹超过半径或全局时间
      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // 通过所需点的密度获取参数化dt
      int seg_num = floor(seg_length / dist_pt);

      // 获取输出
      seg_duration = seg_time; // 截断段的持续时间
      dt = seg_time / seg_num; // 两点间的时间差

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // 获取固定持续时间的Bspline参数化数据
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
};

  struct PlanParameters
{
    /* 规划算法参数 */
    double max_vel_, max_acc_, max_jerk_; // 物理限制：最大速度、最大加速度、最大加加速度（急动度）
    double ctrl_pt_dist;                  // B-spline控制点之间的距离
    double feasibility_tolerance_;        // 允许超出速度/加速度限制的比率
    double planning_horizen_;             // 规划视野（范围）

    /* 处理时间 */
    double time_search_ = 0.0;            // 搜索时间
    double time_optimize_ = 0.0;          // 优化时间
    double time_adjust_ = 0.0;            // 调整时间
};

struct LocalTrajData
{
    /* 生成轨迹的信息 */

    int traj_id_;                         // 轨迹ID
    double duration_;                     // 轨迹持续时间
    double global_time_offset;            // 全局时间偏移。当本地轨迹完成并即将切换回全局轨迹时，全局轨迹的时间不再与世界时间匹配。
    ros::Time start_time_;                // 开始时间
    Eigen::Vector3d start_pos_;           // 起始位置
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_; // 位置、速度、加速度的Uniform Bspline轨迹
};


} // namespace ego_planner

#endif