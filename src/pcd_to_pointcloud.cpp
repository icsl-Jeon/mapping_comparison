//
// Created by jbs on 20. 12. 17..
//



#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
using namespace pcl;
using namespace std;

PointCloud<PointXYZ>::Ptr  cloud_ptr;
PointCloud<PointXYZ>::Ptr  cloud_filtered;

int main(int argc, char** argv){

    cloud_ptr = static_cast<boost::shared_ptr<PointCloud<PointXYZ>>>(new PointCloud<PointXYZ>);
    cloud_filtered = static_cast<boost::shared_ptr<PointCloud<PointXYZ>>>(new PointCloud<PointXYZ>);
    ros::init(argc,argv,"pcd_to_pointcloud");
    ros::NodeHandle nh("~");

    std::string pcd_file = "/home/jbs/catkin_ws/src/multi_chaser/map/factoryMappingTransformed.pcd";
    nh.param<string>("pcd_file",pcd_file,pcd_file);
    pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud_ptr );
    cout<<"have loaded ground pcl of size "<< cloud_ptr->points.size()<<endl;
    double minX,maxX,minY,maxY,minZ,maxZ;

    nh.param("x_min",minX,-10.0);
    nh.param("x_max",maxX,10.0);
    nh.param("y_min",minY,-10.0);
    nh.param("y_max",maxY,10.0);
    nh.param("z_min",minZ,-2.0);
    nh.param("z_max",maxZ,5.0);
    string pcl_topic1;
    string pcl_topic2;

    nh.param<string>("pcl_topic1",pcl_topic1,"/cloud_in");
    nh.param<string>("pcl_topic2",pcl_topic2,"/pointcloud");

    ros::Publisher pubPCL1 = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic1,1);
    ros::Publisher pubPCL2 = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic2,1);


    CropBox<PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY , minZ , 1));
    boxFilter.setMax(Eigen::Vector4f(maxX,maxY,maxZ, 1));
    boxFilter.setInputCloud(cloud_ptr);
    boxFilter.filter(*cloud_filtered);

    cloud_ptr->header.frame_id = "/map";
    cloud_filtered->header.frame_id = "/map";
    sensor_msgs::PointCloud2 rosPCL;
    toROSMsg(*cloud_filtered,rosPCL);

    while(ros::ok()){

        rosPCL.header.stamp = ros::Time::now(); // to voxblox, it is important
        rosPCL.header.frame_id = "map";
        pubPCL1.publish(rosPCL);
        pubPCL2.publish(rosPCL);

        ros::Rate(30).sleep();
        ros::spinOnce();
    }


}
