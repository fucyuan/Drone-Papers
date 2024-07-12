// #include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_datatypes.h> // 包含tf数据类型转换工具

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "path_publisher");
//     ros::NodeHandle nh;

//     // 修复：使用nav_msgs::Path而不是nav_msgs/Path
//     ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10, true);

//     // 初始化nav_msgs::Path消息
//     nav_msgs::Path path;
//     path.header.stamp = ros::Time::now();
//     path.header.frame_id = "world"; // 或者是 "map" 或者你想要的其他frame

//     for (int i = 0; i < 10; ++i) {
//         geometry_msgs::PoseStamped pose_stamped;
//         pose_stamped.header.stamp = path.header.stamp;
//         pose_stamped.header.frame_id = path.header.frame_id;

//         // 设置一个假的位姿 (x, y, z, 和方向)
//         pose_stamped.pose.position.x = i; // 例如，一个简单的直线
//         pose_stamped.pose.position.y = i;
//         pose_stamped.pose.position.z = i;

//         // 设置一个假的方向（仅作为示例）
//         tf::Quaternion q = tf::createQuaternionFromYaw(i * M_PI / 20); // 增量旋转
//         pose_stamped.pose.orientation.x = q.x();
//         pose_stamped.pose.orientation.y = q.y();
//         pose_stamped.pose.orientation.z = q.z();
//         pose_stamped.pose.orientation.w = q.w();

//         path.poses.push_back(pose_stamped);
//     }

//     while (ros::ok()) {
//         path_pub.publish(path);
//         ros::spinOnce();
//         ros::Duration(1.0).sleep(); // 休眠一秒
//     }

//     return 0;
// }
// #include <ros/ros.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/Point.h>
// #include <tf/tf.h>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "pose_array_publisher");
//     ros::NodeHandle nh;

//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

//     visualization_msgs::MarkerArray markers;
//     markers.markers.resize(10); // 创建10个标记

//     for (int i = 0; i < 10; ++i) {
//         markers.markers[i].header.frame_id = "world";
//         markers.markers[i].header.stamp = ros::Time::now();
//         markers.markers[i].ns = "arrows";
//         markers.markers[i].id = i;
//         markers.markers[i].type = visualization_msgs::Marker::
//         markers.markers[i].action = visualization_msgs::Marker::ADD;
//         markers.markers[i].pose.position.x = i;
//         markers.markers[i].pose.position.y = i;
//         markers.markers[i].pose.position.z = i;

//         tf::Quaternion q = tf::createQuaternionFromYaw(i * M_PI / 20);
//         markers.markers[i].pose.orientation.x = q.x();
//         markers.markers[i].pose.orientation.y = q.y();
//         markers.markers[i].pose.orientation.z = q.z();
//         markers.markers[i].pose.orientation.w = q.w();

//         markers.markers[i].scale.x = 1.0; // 箭头长度
//         markers.markers[i].scale.y = 0.1; // 箭头宽度
//         markers.markers[i].scale.z = 0.1; // 箭头高度
//         markers.markers[i].color.a = 1.0; // 不透明度
//         markers.markers[i].color.r = 0.0;
//         markers.markers[i].color.g = 1.0;
//         markers.markers[i].color.b = 0.0;
//     }

//     // 发布MarkerArray
//     while (ros::ok()) {
//         marker_pub.publish(markers);
//         ros::spinOnce();
//         ros::Duration(1.0).sleep();
//     }

//     return 0;
// }
// #include <ros/ros.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_datatypes.h>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "ellipse_sphere_publisher");
//     ros::NodeHandle nh;

//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

//     visualization_msgs::Marker spheres;
//     spheres.header.frame_id = "world";
//     spheres.header.stamp = ros::Time::now();
//     spheres.ns = "spheres";
//     spheres.id = 0;
//     spheres.type = visualization_msgs::Marker::SPHERE_LIST;
//     spheres.action = visualization_msgs::Marker::ADD;
//     spheres.pose.orientation.w = 1.0;

//     spheres.scale.x = 0.1; // 球体的直径（X方向）
//     spheres.scale.y = 0.2; // 球体的直径（Y方向）
//     spheres.scale.z = 0.1; // 球体的直径（Z方向）

//     spheres.color.a = 1.0; // 不透明度
//     spheres.color.r = 1.0;
//     spheres.color.g = 0.0;
//     spheres.color.b = 0.0;

//     // 创建并填充位置点
//     for (int i = 0; i < 10; ++i) {
//         geometry_msgs::Point p;
//         p.x = static_cast<float>(i);
//         p.y = 0.0;
//         p.z = 0.0;
//         spheres.points.push_back(p);
//     }

//     // 发布Marker
//     while (ros::ok()) {
//         marker_pub.publish(spheres);
//         ros::spinOnce();
//         ros::Duration(1.0).sleep();
//     }

//     return 0;
// }

// #include <ros/ros.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_datatypes.h>

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "pose_and_orientation_publisher");
//     ros::NodeHandle nh;

//     ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

//     visualization_msgs::MarkerArray marker_array;

//     // 创建球体标记
//     visualization_msgs::Marker sphere;
//     sphere.header.frame_id = "world";
//     sphere.header.stamp = ros::Time::now();
//     sphere.ns = "pose";
//     sphere.id = 0; // 确保每个标记的ID是唯一的
//     sphere.type = visualization_msgs::Marker::SPHERE;
//     sphere.action = visualization_msgs::Marker::ADD;
//     sphere.pose.position.x = 1.0; // 设置球体的位置
//     sphere.pose.position.y = 1.0;
//     sphere.pose.position.z = 1.0;
//     sphere.scale.x = 0.2; // 球体直径
//     sphere.scale.y = 0.2;
//     sphere.scale.z = 0.2;
//     sphere.color.a = 1.0; // 不透明度
//     sphere.color.r = 0.0;
//     sphere.color.g = 1.0;
//     sphere.color.b = 0.0;

//     // 创建箭头标记
//     visualization_msgs::Marker arrow;
//     arrow.header.frame_id = "world";
//     arrow.header.stamp = ros::Time::now();
//     arrow.ns = "orientation";
//     arrow.id = 1; // 确保每个标记的ID是唯一的
//     arrow.type = visualization_msgs::Marker::ARROW;
//     arrow.action = visualization_msgs::Marker::ADD;
//     arrow.pose.position = sphere.pose.position; // 箭头从球体的中心开始
//     // 设置箭头的方向，例如指向x轴正方向
//     tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
//     arrow.pose.orientation.x = q.x();
//     arrow.pose.orientation.y = q.y();
//     arrow.pose.orientation.z = q.z();
//     arrow.pose.orientation.w = q.w();
//     arrow.scale.x = 0.4; // 箭头长度
//     arrow.scale.y = 0.05; // 箭头宽度
//     arrow.scale.z = 0.05; // 箭头高度
//     arrow.color.a = 1.0; // 不透明度
//     arrow.color.r = 1.0;
//     arrow.color.g = 0.0;
//     arrow.color.b = 0.0;

//     // 将球体和箭头添加到标记数组
//     marker_array.markers.push_back(sphere);
//     marker_array.markers.push_back(arrow);

//     // 发布MarkerArray
//     while (ros::ok()) {
//         marker_pub.publish(marker_array);
//         ros::spinOnce();
//         ros::Duration(1.0).sleep();
//     }

//     return 0;
// }

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
