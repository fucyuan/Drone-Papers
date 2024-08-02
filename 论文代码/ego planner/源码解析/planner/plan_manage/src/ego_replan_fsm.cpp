
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

 void EGOReplanFSM::init(ros::NodeHandle &nh)  // EGOReplanFSM的初始化函数
{
  current_wp_ = 0;  // 当前路径点索引设置为0
  exec_state_ = FSM_EXEC_STATE::INIT;  // 将执行状态设置为初始化
  have_target_ = false;  // 设置是否有目标的标志为false
  have_odom_ = false;  // 设置是否有里程计数据的标志为false

  /* fsm param */
  // nh.param 是 ROS（机器人操作系统）中的一个函数，用于从ROS的参数服务器中获取参数值。
  // 这个函数的第一个参数是参数服务器中的参数名，
  // 第二个参数是用来存储获取到的参数值的变量，
  // 第三个参数是一个默认值，在参数服务器中没有找到对应参数时使用。
  nh.param("fsm/flight_type", target_type_, -1);  // 从ROS参数服务器读取飞行类型
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);  // 从ROS参数服务器读取重规划阈值
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);  // 从ROS参数服务器读取无需重规划阈值
  nh.param("fsm/planning_horizon", planning_horizen_, -1.0);  // 从ROS参数服务器读取规划视界
  nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);  // 从ROS参数服务器读取规划视界时间
  nh.param("fsm/emergency_time_", emergency_time_, 1.0);  // 从ROS参数服务器读取紧急情况时间

  nh.param("fsm/waypoint_num", waypoint_num_, -1);  // 从ROS参数服务器读取路径点数量
  for (int i = 0; i < waypoint_num_; i++)  // 遍历所有路径点
  {
    // 从ROS参数服务器读取每个路径点的x，y，z坐标
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  visualization_.reset(new PlanningVisualization(nh));  // 初始化规划可视化模块
  planner_manager_.reset(new EGOPlannerManager);  // 初始化规划管理器
  planner_manager_->initPlanModules(nh, visualization_);  // 初始化规划模块

  /* callback */
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)  // 如果目标类型为手动目标
    waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);//第一步
  else if (target_type_ == TARGET_TYPE::PRESET_TARGET)  // 如果目标类型为预设目标
  {
    ros::Duration(1.0).sleep();  // 等待1秒
    while (ros::ok() && !have_odom_)  // 在有里程计数据之前循环等待
      ros::spinOnce();
    planGlobalTrajbyGivenWps();  // 根据给定的路径点规划全局轨迹
  }
  else
    cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;  // 如果目标类型错误，则输出错误信息
  // 设置执行FSM的回调函数，定时器频率为0.01秒
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this); 
  //this 关键字代表当前类（在这个案例中是 EGOReplanFSM 类）的实例自身。
  // 在C++中，this 是一个指针，指向调用成员函数的对象的实例。


  // 设置检查碰撞的回调函数，定时器频率为0.05秒
  safety_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::checkCollisionCallback, this); 

  // 订阅里程计数据
  odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

  // 设置B样条和数据显示的发布者
  bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
  data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);


}


 void EGOReplanFSM::planGlobalTrajbyGivenWps()
{
    std::vector<Eigen::Vector3d> wps(waypoint_num_); // 创建一个存储航点的向量
    for (int i = 0; i < waypoint_num_; i++)
    {
        wps[i](0) = waypoints_[i][0]; // 设置航点的X坐标
        wps[i](1) = waypoints_[i][1]; // 设置航点的Y坐标
        wps[i](2) = waypoints_[i][2]; // 设置航点的Z坐标

        end_pt_ = wps.back(); // 设置最后一个航点为目标点
    }

    // 调用规划器，规划从当前位置到目标点经过给定航点的全局轨迹
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 可视化所有航点
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        ros::Duration(0.001).sleep(); // 短暂休眠以确保可视化正常
    }

    // 如果轨迹规划成功
    if (success)
    {
        // 可视化全局轨迹
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        std::vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }

        end_vel_.setZero(); // 重置结束速度
        have_target_ = true; // 设置有目标标志
        have_new_target_ = true; // 设置有新目标标志

        // 更新状态机状态
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 更改状态为生成新轨迹

        // 再次可视化全局路径
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
        ros::Duration(0.001).sleep(); // 短暂休眠以确保可视化正常
    }
    else
    {
        ROS_ERROR("Unable to generate global trajectory!"); // 如果规划失败，输出错误信息
    }
}

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
{
    // 如果目标点的z坐标小于-0.1，直接返回
    if (msg->poses[0].pose.position.z < -0.1)
        return;

    cout << "Triggered!" << endl; // 打印触发信息
    trigger_ = true; // 设置触发标志为真
    init_pt_ = odom_pos_; // 将当前位置设置为初始点

    bool success = false;
    // 从消息中获取目标点的位置，并设置z坐标为1.0
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    // 规划从当前位置到目标点的全局轨迹
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 可视化目标点
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    // 如果规划成功
    if (success)
    {
        // 可视化全局轨迹
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }

        // 重置结束速度，设置有目标标志
        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        // 更新状态机
        if (exec_state_ == WAIT_TARGET)
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 如果当前状态是等待目标，则切换到生成新轨迹
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "TRIG"); // 如果当前状态是执行轨迹，则切换到重规划轨迹

        // 可视化全局路径
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        // 如果规划失败，打印错误信息
        ROS_ERROR("Unable to generate global trajectory!");
    }
}

 void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // 提取并设置里程计消息中的位置信息
    odom_pos_(0) = msg->pose.pose.position.x; // 设置X坐标
    odom_pos_(1) = msg->pose.pose.position.y; // 设置Y坐标
    odom_pos_(2) = msg->pose.pose.position.z; // 设置Z坐标

    // 提取并设置里程计消息中的速度信息
    odom_vel_(0) = msg->twist.twist.linear.x; // 设置线速度X分量
    odom_vel_(1) = msg->twist.twist.linear.y; // 设置线速度Y分量
    odom_vel_(2) = msg->twist.twist.linear.z; // 设置线速度Z分量

    // 注释掉的代码可能是用于估计加速度，目前未被使用
    //odom_acc_ = estimateAcc(msg);

    // 提取并设置里程计消息中的姿态信息（四元数格式）
    odom_orient_.w() = msg->pose.pose.orientation.w; // 四元数W分量
    odom_orient_.x() = msg->pose.pose.orientation.x; // 四元数X分量
    odom_orient_.y() = msg->pose.pose.orientation.y; // 四元数Y分量
    odom_orient_.z() = msg->pose.pose.orientation.z; // 四元数Z分量

    // 标记已接收到里程计数据
    have_odom_ = true;
}

void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)  // 改变FSM执行状态的方法
{
    if (new_state == exec_state_)  // 如果新状态与当前状态相同
        continously_called_times_++;  // 连续被调用的次数增加
    else
        continously_called_times_ = 1;  // 否则，重置连续调用次数为1

    // 状态字符串数组，用于打印状态名
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);  // 记录之前的状态
    exec_state_ = new_state;  // 更新当前状态为新状态

    // 打印状态变化的信息
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}


std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
{
    // 返回一个std::pair对象，第一个元素是连续被调用的次数（continously_called_times_），
    // 第二个元素是当前执行的状态（exec_state_）
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
}


  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

 void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)  // 定时器回调函数
{
    static int fsm_num = 0;  // 静态变量，用于计数回调被调用的次数
    fsm_num++;
    if (fsm_num == 100)  // 每100次回调打印一次状态信息
    {
        printFSMExecState();  // 打印当前的FSM执行状态
        if (!have_odom_)
            cout << "no odom." << endl;  // 如果没有里程计数据，则打印提示
        if (!trigger_)
            cout << "wait for goal." << endl;  // 如果没有触发，则等待目标
        fsm_num = 0;
    }

    switch (exec_state_)  // 根据当前的执行状态决定下一步操作
    {
    case INIT:  // 初始化状态
    {
        if (!have_odom_ || !trigger_)  // 如果没有里程计数据或没有触发，则返回
        {
            return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");  // 改变状态到等待目标
        //exec_state_会发生变化重新执行Switch语句
        break;
    }

    case WAIT_TARGET:  // 等待目标状态
    {
        if (!have_target_)
            return;  // 如果没有目标，则返回
        else
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");  // 改变状态到生成新轨迹
        }
        break;
    }

    case GEN_NEW_TRAJ:  // 生成新轨迹状态
    {
        // 设置起始位置、速度和加速度
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        bool flag_random_poly_init = timesOfConsecutiveStateCalls().first != 1;//判断是否第一次调用
        /*
                std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
        {
            // 返回一个std::pair对象，第一个元素是连续被调用的次数（continously_called_times_），
            // 第二个元素是当前执行的状态（exec_state_）
            return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
        }
        第一次调用为false；

        */


        // 如果当前状态不是第一次被连续调用（也就是至少是第二次调用或更多），则设置 flag_random_poly_init 为 true

        bool success = callReboundReplan(true, flag_random_poly_init);  // 调用重规划方法
        if (success)
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");  // 如果成功，则改变状态到执行轨迹
            flag_escape_emergency_ = true;
        }
        else
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");  // 如果失败，保持在当前状态
        }
        break;
    }

    case REPLAN_TRAJ:  // 重规划轨迹状态
    {
        if (planFromCurrentTraj())  // 从当前轨迹进行规划
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");  // 如果规划成功，改变状态到执行轨迹
        }
        else
        {
            changeFSMExecState(REPLAN_TRAJ, "FSM");  // 如果失败，保持在当前状态
        }
        break;
    }

      case EXEC_TRAJ:  // 执行轨迹状态
    {
        // 以下是判断是否需要重规划的逻辑
        LocalTrajData *info = &planner_manager_->local_data_; // 获取局部轨迹信息
        ros::Time time_now = ros::Time::now(); // 获取当前时间
        double t_cur = min(info->duration_, (time_now - info->start_time_).toSec()); // 计算从轨迹开始到现在的时间

        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur); // 在当前时间点评估轨迹位置

        // 判断轨迹是否接近结束或机器人是否接近目标点
        if (t_cur > info->duration_ - 1e-2 || (end_pt_ - pos).norm() < no_replan_thresh_)
        {
            have_target_ = false; // 清除目标标志
            changeFSMExecState(WAIT_TARGET, "FSM"); // 改变状态机的状态到等待目标状态
            return; // 退出当前状态处理
        }
        else if ((info->start_pos_ - pos).norm() < replan_thresh_)
        {
            return; // 如果机器人离起始点太近，不进行任何操作
        }
        else
        {
            changeFSMExecState(REPLAN_TRAJ, "FSM"); // 如果不满足以上条件，改变状态到重规划轨迹状态
        }
        break; // 结束当前状态处理
    }

    case EMERGENCY_STOP:  // 紧急停止状态
    {
        if (flag_escape_emergency_)  // 如果是逃逸紧急情况
        {
            callEmergencyStop(odom_pos_);  // 调用紧急停止方法
        }
        else
        {
            if (odom_vel_.norm() < 0.1)  // 如果速度小于0.1
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");  // 改变状态到生成新轨迹
        }
        flag_escape_emergency_ = false;
        break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);  // 发布数据显示信息
}


  bool EGOReplanFSM::planFromCurrentTraj()
  {
      LocalTrajData *info = &planner_manager_->local_data_; // 获取当前轨迹的局部数据引用
      ros::Time time_now = ros::Time::now(); // 获取当前时间
      double t_cur = (time_now - info->start_time_).toSec(); // 计算当前时间相对于轨迹开始时间的差值

      // 获取当前时间点上的位置、速度和加速度
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // 尝试进行重规划，不使用多项式初始化和随机轨迹
      bool success = callReboundReplan(false, false);

      // 如果重规划失败，尝试使用多项式初始化但不使用随机轨迹
      if (!success)
      {
          success = callReboundReplan(true, false);
          // 如果再次失败，尝试使用多项式初始化和随机轨迹
          if (!success)
          {
              success = callReboundReplan(true, true);
              // 如果所有重规划尝试均失败，返回false
              if (!success)
              {
                  return false;
              }
          }
      }

      // 如果任一重规划尝试成功，返回true
      return true;
  }


 void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{
    LocalTrajData *info = &planner_manager_->local_data_; // 获取当前局部轨迹信息
    auto map = planner_manager_->grid_map_; // 获取用于检测障碍物的地图

    // 如果当前状态是等待目标或轨迹开始时间非常短，直接返回
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
        return;

    /* ---------- 检查轨迹 ---------- */
    constexpr double time_step = 0.01; // 设定检查时间间隔
    double t_cur = (ros::Time::now() - info->start_time_).toSec(); // 计算当前时间与轨迹开始时间的差
    double t_2_3 = info->duration_ * 2 / 3; // 计算轨迹时长的2/3

    // 按时间步长检查轨迹的每一点
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
        // 如果只考虑轨迹的前2/3部分，且已经检查到这个部分之后，结束循环
        if (t_cur < t_2_3 && t >= t_2_3)
            break;

        // 如果在当前时间点的轨迹位置检测到障碍物
        if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
        {
            // 尝试从当前轨迹位置开始重规划
            if (planFromCurrentTraj())
            {
                changeFSMExecState(EXEC_TRAJ, "SAFETY"); // 如果重规划成功，改变状态为执行轨迹
                return;
            }
            else
            {
                // 如果检测到障碍物的时间距离当前时间小于紧急时间
                if (t - t_cur < emergency_time_) // 紧急时间为0.8秒
                {
                    ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
                    changeFSMExecState(EMERGENCY_STOP, "SAFETY"); // 紧急停止
                }
                else
                {
                    changeFSMExecState(REPLAN_TRAJ, "SAFETY"); // 否则，改变状态为重规划轨迹
                }
                return;
            }
            break;
        }
    }
}


  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{
    getLocalTarget();  // 获取局部目标点
    cout<<"局部目标点"<<endl;

    // 执行重规划，并返回规划是否成功的布尔值
    bool plan_success = planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
   //这里没有找到 have_new_target_ 的初始化
   
   
    have_new_target_ = false;  // 重置新目标标志

    cout << "final_plan_success=" << plan_success << endl;  // 打印规划成功与否的信息

    if (plan_success)  // 如果规划成功
    {
        auto info = &planner_manager_->local_data_;  // 获取规划信息

        /* publish traj */
        ego_planner::Bspline bspline;  // 创建B样条对象
        bspline.order = 3;  // 设置B样条的阶数
        bspline.start_time = info->start_time_;  // 设置起始时间
        bspline.traj_id = info->traj_id_;  // 设置轨迹ID

        // 将控制点转换为消息格式并添加到B样条消息中
        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        // 将节点添加到B样条消息中
        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        bspline_pub_.publish(bspline);  // 发布B样条轨迹

        visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);  // 可视化最优控制点
    }

    return plan_success;  // 返回规划成功与否
}


  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

void EGOReplanFSM::getLocalTarget()
{
  // 主要功能是计算局部目标点（local_target_pt_）和相应的速度（local_target_vel_）

    double t;

    // 计算时间步长
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
        // 获取时间点t对应的全局路径位置
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
        // 计算该点与起始点的距离
        double dist = (pos_t - start_pt_).norm();

        // 检查是否在规划视界范围内
        if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
        {
            ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");  // 如果不在规划视界内，打印错误信息
            return;
        }
        if (dist < dist_min)  // 更新最小距离及对应的时间点
        {
            dist_min = dist;
            dist_min_t = t;
        }
        if (dist >= planning_horizen_)  // 如果达到规划视界范围，设置局部目标点并退出循环
        {
            local_target_pt_ = pos_t;
            planner_manager_->global_data_.last_progress_time_ = dist_min_t;
            break;
        }
    }
    if (t > planner_manager_->global_data_.global_duration_)  // 如果到达全局路径的末端
    {
        local_target_pt_ = end_pt_;  // 将局部目标点设置为全局路径的结束点
    }

    // 设置局部目标点的速度
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
        local_target_vel_ = Eigen::Vector3d::Zero();  // 如果局部目标点接近结束点，将速度设置为零
    }
    else
    {
        local_target_vel_ = planner_manager_->global_data_.getVelocity(t);  // 否则，设置为对应时间点的速度
    }
}


} // namespace ego_planner
