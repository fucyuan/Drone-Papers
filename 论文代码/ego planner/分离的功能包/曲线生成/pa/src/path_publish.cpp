#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv) {
   ros::init(argc, argv, "spiral_path_with_ellipses_and_arrows");
    ros::NodeHandle nh;

    ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    visualization_msgs::MarkerArray marker_array;

    int points = 100; // 轨迹上的点数
    float height = 5.0; // 螺旋线的高度
    float turns = 5.0; // 螺旋线的圈数

    for (int i = 0; i < points; ++i) {
        // 为每个点创建一个椭圆球
        visualization_msgs::Marker ellipse;
        ellipse.header.frame_id = "world";
        ellipse.header.stamp = ros::Time::now();
        ellipse.ns = "ellipse";
        ellipse.id = i;
        ellipse.type = visualization_msgs::Marker::SPHERE;
        ellipse.action = visualization_msgs::Marker::ADD;
        ellipse.color.a = 1.0; // 不透明度
        ellipse.color.r = 1.0; // 红色分量
        ellipse.color.g = 0.0; // 绿色分量
        ellipse.color.b = 0.0; // 蓝色分量

        // 设置椭圆球的大小
        ellipse.scale.x = 0.3; // 在X方向的直径
        ellipse.scale.y = 0.2; // 在Y方向的直径
        ellipse.scale.z = 0.1; // 在Z方向的直径

        float angle = (turns * 2.0 * M_PI) * (i / static_cast<float>(points));
        float z = (height * i) / static_cast<float>(points);
        float x = 0.5 * cos(angle);
        float y = 0.5 * sin(angle);

        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;

        ellipse.pose.position = p;

        // 为每个点创建一个箭头
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "world";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "arrow";
        arrow.id = i + points; // 确保ID不会与椭圆球重复
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.color.a = 1.0;
        arrow.color.r = 1.0;
        arrow.color.g = 1.0;
        arrow.color.b = 1.0;

        arrow.scale.x = 0.2; // 箭头长度
        arrow.scale.y = 0.02; // 箭头宽度
        arrow.scale.z = 0.02; // 箭头高度

        arrow.pose.position = p;

        // 设置箭头的方向
        tf::Vector3 axis_vector = tf::Vector3(-sin(angle), cos(angle), height / (turns * 2.0 * M_PI));
        tf::Vector3 up_vector = tf::Vector3(0.0, 0.0, 1.0);
        tf::Vector3 cross_product = axis_vector.cross(up_vector);
        tf::Quaternion q;
        q.setRotation(cross_product, angle);



        ellipse.pose.orientation.x = q.x();
        ellipse.pose.orientation.y = q.y();
        ellipse.pose.orientation.z = q.z();
        ellipse.pose.orientation.w = q.w();
        
        arrow.pose.orientation.x = q.x();
        arrow.pose.orientation.y = q.y();
        arrow.pose.orientation.z = q.z();
        arrow.pose.orientation.w = q.w();

        // 将椭圆球和箭头添加到标记数组
        marker_array.markers.push_back(ellipse);            
        marker_array.markers.push_back(arrow);
    }
    // 发布MarkerArray
    while (ros::ok()) {
        marker_array_pub.publish(marker_array);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    return 0;
}
