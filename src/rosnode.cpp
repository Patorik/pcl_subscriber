#include "rosnode.h"

RosNode::RosNode(int argc, char **argv, std::string &topic_name_sub, float distance){
    this->topic_name_sub = topic_name_sub;
    this->distance = distance;

    ros::init(argc, argv, "pointcloud_listener");
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName());

    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> server;
    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;

    f = boost::bind(&RosNode::paramsCallback, this, _1, _2);
    server.setCallback(f);

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(topic_name_sub, 1, &RosNode::cloudCallback, this);
    pcl_pub = n.advertise<pcl::PCLPointCloud2>("pcl_published", 1);

    ros::spin();
}

void RosNode::paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level)
{
    // UPDATE GLOBAL VARIABLES
    this->distance = config.distance;
    
    ROS_INFO_STREAM("Updated params :\n" << "Distance:\t" << this->distance);
}

void RosNode::cloudCallback(const pcl::PCLPointCloud2 &cloud_msg){
    // Convert PCL2 to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud, filtered_points;
    filtered_points.header = cloud_msg.header;
    pcl::fromPCLPointCloud2(cloud_msg, pcl_points);

    std::string target_frame_pcl = "base_link";

    cloud_filtered = pcl_points.makeShared();

    // INIT PUBLISHED topics
    pcl::PointCloud<pcl::PointXYZL> cloud_filtered_pub;
    pcl::PointXYZL c_point;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud_filtered);
    voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    voxel_grid.filter(*cloud_filtered);

    cloud_filtered_pub.header = cloud_msg.header;
    
    // GET NECESSARRY PCL POINTS
    for (auto &cloud_point : *cloud_filtered)
    {
        if(calculateDistance(cloud_point.x, cloud_point.y, cloud_point.z)>this->distance){
            c_point.x = cloud_point.x;
            c_point.y = cloud_point.y;
            c_point.z = cloud_point.z;
            cloud_filtered_pub.push_back(c_point);
        }
    }
    
    pcl_pub.publish(cloud_filtered_pub);
    cloud_filtered_pub.clear();
}

float RosNode::calculateDistance(const float &x, const float &y, const float &z){
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}