#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/octomap_types.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <chrono>

using namespace octomap;
using namespace std;

shared_ptr<DynamicEDTOctomap> edf_ptr;
AbstractOcTree* abs_octree;
shared_ptr<OcTree> octree;

void octomap_callback(const octomap_msgs::Octomap& msg){
    // we receive only once from octoamp server

        // octomap subscribing 

        abs_octree=octomap_msgs::fullMsgToMap(msg);
        octree.reset(dynamic_cast<octomap::OcTree*>(abs_octree));
        
        ROS_INFO_ONCE("[Objects handler] octomap received.");
        double x,y,z;
        octree.get()->getMetricMin(x,y,z);
		cout<<"metric min : "<<x<<" , "<<y << ", " << z << endl;
        octomap::point3d boundary_min(x,y,z); 
        octree.get()->getMetricMax(x,y,z);
		cout<<"metric max : "<<x<<" , "<<y << ", " << z << endl;
        octomap::point3d boundary_max(x,y,z); 


		
		bool unknownAsOccupied = false;

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Euclidean distance transform  
        edf_ptr.reset (new DynamicEDTOctomap(1.2,octree.get(),
            boundary_min,
            boundary_max,unknownAsOccupied));
        edf_ptr.get()->update();          
        point3d eval_point(1,0,0);        
        cout<<"eval dist val of [1,0,0] = "<<edf_ptr.get()->getDistance(eval_point)<<endl;  

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double diff = std::chrono::duration_cast<chrono::nanoseconds>( end - begin ).count()*1e-9;
        ROS_INFO("[EDT build] dynamic EDT computed in %f [sec]",diff);
};





int main(int argc,char** argv){
    ros::init(argc,argv,"edf_cal_test_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_octo = nh.subscribe("/octomap_full",1,octomap_callback);
    ros::spin();
    return 0;
}
