#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  // 实现不同维度的非均匀B样条曲线，同时也可表示均匀B样条曲线
  class UniformBspline
  {
  private:
    // 不同维度的B样条曲线的控制点
    // 每行代表一个控制点
    // 维度由列数确定
    // 例如：在3D空间中有N个点的B样条 -> Nx3矩阵
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // B样条的阶数p，控制点数量n+1，m = n+p+1
    Eigen::VectorXd u_; // 节点向量
    double interval_;   // 节点间距\delta t

    Eigen::MatrixXd getDerivativeControlPoints(); // 获取导数控制点

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // 物理限制和时间调整比率

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // 初始化为均匀B样条曲线
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // 获取/设置基本B样条信息

    void setKnot(const Eigen::VectorXd &knot); // 设置节点
    Eigen::VectorXd getKnot(); // 获取节点
    Eigen::MatrixXd getControlPoint(); // 获取控制点
    double getInterval(); // 获取间隔
    bool getTimeSpan(double &um, double &um_p); // 获取时间跨度

    // 计算位置/导数

    Eigen::VectorXd evaluateDeBoor(const double &u); // 使用u \in [up, u_mp]进行评估
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // 使用t \in [0, duration]进行评估
    UniformBspline getDerivative(); // 获取导数

    // 对点集进行3D B样条插值，包括边界速度和加速度约束
    // 输入：(K+2)个点和边界速度/加速度；时间间隔ts
    // 输出：(K+6)个控制点
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* 检查可行性，调整时间 */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance); // 设置物理限制
    bool checkFeasibility(double &ratio, bool show = false); // 检查可行性
    void lengthenTime(const double &ratio); // 延长时间

    /* 性能评估 */

    double getTimeSum(); // 获取总时间
    double getLength(const double &res = 0.01); // 获取长度
    double getJerk(); // 获取急动度
    void getMeanAndMaxVel(double &mean_v, double &max_v); // 获取平均和最大速度
    void getMeanAndMaxAcc(double &mean_a, double &max_a); // 获取平均和最大加速度

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner

#endif