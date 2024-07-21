#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"

// 梯度和弹性带优化

// 输入：带符号的距离场和一系列点
// 输出：优化后的点序列
// 点的格式：N x 3 矩阵，每行一个点
namespace ego_planner
{

  class ControlPoints
  {
  public:
    double clearance; // 清除距离
    int size; // 控制点的数量
    Eigen::MatrixXd points; // 控制点的矩阵
    std::vector<std::vector<Eigen::Vector3d>> base_point; // 方向向量起点的点（碰撞点）
    std::vector<std::vector<Eigen::Vector3d>> direction;  // 方向向量，必须归一化
    std::vector<bool> flag_temp; // 在许多地方使用的临时标志，在每次使用前初始化

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
    }
  };

  class BsplineOptimizer
  {
  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* 主要 API */
    void setEnvironment(const GridMap::Ptr &env);
    void setParam(ros::NodeHandle &nh);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* 辅助函数 */

    // 必需的输入
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setCostFunction(const int &cost_function);
    void setTerminateCond(const int &max_num_id, const int &max_time_id);

    // 可选的输入
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // 最多 N-2 个约束

    void optimize();

    Eigen::MatrixXd getControlPoints();

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // 必须在 initControlPoints() 之后调用
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }

  private:
    GridMap::Ptr grid_map_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // 主要输入
    double bspline_interval_; // B样条节点间隔
    Eigen::Vector3d end_pt_;  // 轨迹的终点

    vector<Eigen::Vector3d> guide_pts_; // 几何引导路径点，N-6
    vector<Eigen::Vector3d> waypoints_; // 约束路径点
    vector<int> waypt_idx_; // 路径点约束的索引

    int max_num_id_, max_time_id_; // 停止条件
    int cost_function_; // 用于确定目标函数
    double start_time_; // 用于动态障碍物的全局时间

    /* 优化参数 */
    int order_; // B样条的阶数
    double lambda1_; // 跃度平滑权重
    double lambda2_, new_lambda2_; // 距离权重
    double lambda3_; // 可行性权重
    double lambda4_; // 曲线拟合权重

    double dist0_; // 安全距离
    double max_vel_, max_acc_; // 动态限制

    int variable_num_; // 优化变量的数量
    int iter_num_; // 求解器的迭代次数
    Eigen::VectorXd best_variable_; // 最佳变量
    double min_cost_; // 最小代价

    ControlPoints cps_; // 控制点

    /* 代价函数 */
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif
