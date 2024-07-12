#include <ros/ros.h>
#include "plan_env/grid_map.h"

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh;

    // 创建 GridMap 对象
    GridMap grid_map;

    // 调用 initMap 函数
    grid_map.initMap(nh);

    // 其他逻辑（如进入 ROS 循环）
    ros::spin();

    return 0;
}
