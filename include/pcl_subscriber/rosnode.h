#ifndef ROSNODE_H
#define ROSNODE_H

#include <vector>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

class RosNode{
private:
    pcl::PointCloud<pcl::PointXYZ> pcl_points;
    ros::Publisher pcl_pub;
    std::string topic_name_sub;
    float distance;
public:
    RosNode(int argc, char **argv, std::string &topic_name_sub, float distance=10);
    void cloudCallback(const pcl::PCLPointCloud2 &cloud_msg);
    float calculateDistance(const float &x, const float &y, const float &z);
};


#endif // ROSNODE_H