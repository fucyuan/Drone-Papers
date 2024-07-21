为了帮助您理解这个有限状态机 (FSM) 的工作流程，让我们通过一个流程图来表示状态机的状态切换。下面是对代码的每个状态的描述以及状态转换条件：

### 状态描述

1. **INIT**: 初始化状态
   - 条件：如果没有里程计数据或没有触发事件，则保持在 INIT 状态。否则，转到 WAIT_TARGET 状态。

2. **WAIT_TARGET**: 等待目标状态
   - 条件：如果没有目标，则保持在 WAIT_TARGET 状态。否则，转到 GEN_NEW_TRAJ 状态。

3. **GEN_NEW_TRAJ**: 生成新轨迹状态
   - 条件：调用重规划方法 `callReboundReplan`。如果成功，转到 EXEC_TRAJ 状态，否则保持在 GEN_NEW_TRAJ 状态。

4. **REPLAN_TRAJ**: 重规划轨迹状态
   - 条件：从当前轨迹进行规划。如果成功，转到 EXEC_TRAJ 状态，否则保持在 REPLAN_TRAJ 状态。

5. **EXEC_TRAJ**: 执行轨迹状态
   - 条件：如果轨迹接近结束或机器人接近目标点，转到 WAIT_TARGET 状态。如果机器人离起始点太近，不进行任何操作。如果不满足以上条件，转到 REPLAN_TRAJ 状态。

6. **EMERGENCY_STOP**: 紧急停止状态
   - 条件：如果是逃逸紧急情况，调用紧急停止方法。如果速度小于0.1，转到 GEN_NEW_TRAJ 状态。否则，保持在 EMERGENCY_STOP 状态。

### 流程图

下面是这个有限状态机的流程图表示：

```plaintext
INIT
 |
 v
[!have_odom_ || !trigger_] -----> [WAIT_TARGET]
 |                                    |
 |                                    v
[WAIT_TARGET] <------------------ [!have_target_]
 |                                    |
 v                                    v
[GEN_NEW_TRAJ] -------> [callReboundReplan()成功] ---> [EXEC_TRAJ]
 |                                   |
 v                                   v
[GEN_NEW_TRAJ] <----------------- [callReboundReplan()失败]
 |                                    |
 v                                    v
[REPLAN_TRAJ] -------> [planFromCurrentTraj()成功] ---> [EXEC_TRAJ]
 |                                    |
 v                                    v
[REPLAN_TRAJ] <----------------- [planFromCurrentTraj()失败]
 |
 v
[EXEC_TRAJ] --> [t_cur > info->duration_ - 1e-2 || (end_pt_ - pos).norm() < no_replan_thresh_] --> [WAIT_TARGET]
 |                                    |
 v                                    v
[EXEC_TRAJ] --> [(info->start_pos_ - pos).norm() < replan_thresh_] --> [RETURN]
 |                                    |
 v                                    v
[EXEC_TRAJ] --> [!满足以上条件] --> [REPLAN_TRAJ]
 |
 v
[EMERGENCY_STOP] --> [flag_escape_emergency_] --> [callEmergencyStop(odom_pos_)]
 |                                    |
 v                                    v
[EMERGENCY_STOP] --> [!flag_escape_emergency_ && odom_vel_.norm() < 0.1] --> [GEN_NEW_TRAJ]
```

### 解释

1. **INIT** 状态：
   - 如果没有里程计数据 (`have_odom_ == false`) 或者没有触发 (`trigger_ == false`)，则保持在 `INIT` 状态。
   - 如果有里程计数据且触发事件，状态转到 `WAIT_TARGET`。

2. **WAIT_TARGET** 状态：
   - 如果没有目标 (`have_target_ == false`)，则保持在 `WAIT_TARGET` 状态。
   - 如果有目标，则状态转到 `GEN_NEW_TRAJ`。

3. **GEN_NEW_TRAJ** 状态：
   - 调用重规划方法 `callReboundReplan`。
   - 如果调用成功，状态转到 `EXEC_TRAJ`。
   - 如果调用失败，保持在 `GEN_NEW_TRAJ` 状态。

4. **REPLAN_TRAJ** 状态：
   - 从当前轨迹进行规划。
   - 如果规划成功，状态转到 `EXEC_TRAJ`。
   - 如果规划失败，保持在 `REPLAN_TRAJ` 状态。

5. **EXEC_TRAJ** 状态：
   - 判断轨迹是否接近结束或机器人是否接近目标点，如果是，状态转到 `WAIT_TARGET`。
   - 判断机器人是否离起始点太近，如果是，不进行任何操作，返回。
   - 如果不满足以上条件，状态转到 `REPLAN_TRAJ`。

6. **EMERGENCY_STOP** 状态：
   - 如果是逃逸紧急情况，调用紧急停止方法。
   - 如果速度小于0.1，状态转到 `GEN_NEW_TRAJ`。
   - 否则，保持在 `EMERGENCY_STOP` 状态。

### 总结

通过流程图，可以清楚地看到状态机的各个状态以及它们之间的转换条件。这个有限状态机通过在不同状态之间转换，处理无人驾驶汽车的路径规划和执行任务。