//
// Created by jbs on 20. 12. 16..
//
#include "voxblox_ros/esdf_server.h"
#include <octomap_server/TrackingOctomapServer.h>
#include <octomap_msgs/GetOctomap.h>
#include <chrono>
#include <ros/ros.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
using namespace std::chrono;
using namespace std;

int main (int argc,char** argv){
    ros::init(argc,argv,"vsf_mapping_comparison");

    std::string filename = "/home/jbs/catkin_ws/src/multi_chaser/map/factoryMapping.bt";

    ros::NodeHandle nh("/");
    ros::NodeHandle nh_private("~");

    double maxDist,height,resolution;
    nh.param("cutoff_distance",maxDist,3.0);
    nh.param("edf_slice_leve",height,0.5);
    nh.param("resolution",resolution,0.5);


    auto octomapTrackingServer =new octomap_server::TrackingOctomapServer();
    // Access to octomap
    octomap_msgs::GetOctomap OctomapSrv;
    octomap::OcTree* ot = new octomap::OcTree(resolution);
    // EDF computation
    double xmin,ymin,zmin,xmax,ymax,zmax; bool unknownAsOccupied = false;
    DynamicEDTOctomap* distmap_ptr;



    // EDF visualization


    pcl::PointCloud<pcl::PointXYZI> edf_slice;
    edf_slice.header.frame_id = "/map";
    ros::Publisher pubEDFMarker = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("edf_marker",1);

    // Voxblox
    voxblox::EsdfServer voxbloxServer(nh,nh_private);

    auto t0 = steady_clock::now();
    auto t1 = steady_clock::now();
    bool isOctomapValid = false;

    double queryTimeOctomap,queryTimeVoxblox;

    while (ros::ok()) {


        // octomap data processing
        octomapTrackingServer->octomapBinarySrv(OctomapSrv.request, OctomapSrv.response);
        ot = dynamic_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(OctomapSrv.response.map));

        if (ot->getNumLeafNodes() > 0) {

            cout << "Received octomap:" << ot->getNumLeafNodes() << endl;
            ot->getMetricMin(xmin, ymin, zmin);
            ot->getMetricMax(xmax, ymax, zmax);

            if (not isOctomapValid)
                distmap_ptr = new DynamicEDTOctomap(maxDist, ot, octomap::point3d(xmin, ymin, zmin),
                                                    octomap::point3d(xmax, ymax, zmax), unknownAsOccupied);

            isOctomapValid = true;
        }

        if (isOctomapValid) {
            ROS_INFO_ONCE("starting distance map update");

            t0 = steady_clock::now();
            distmap_ptr->update(true);
            t1 = steady_clock::now();

            printf("[octomap] dist field update took %ld[ms]\n", duration_cast<microseconds>(t1 - t0).count());

            int bbx_nx = floor((xmax - xmin) / resolution);
            int bbx_ny = floor((ymax - ymin) / resolution);


            edf_slice.points.clear();
            for (int nx = 0; nx <= bbx_nx; nx++)
                for (int ny = 0; ny <= bbx_ny; ny++) {
                    octomap::point3d pnt(xmin + nx * resolution, ymin + ny * resolution, height);
                    geometry_msgs::Point point;
                    point.x = pnt.x();
                    point.y = pnt.y();
                    point.z = pnt.z();
                    double dist = distmap_ptr->getDistance(pnt);
                    if (dist > 0) {
                        pcl::PointXYZI pntPcl;
                        pntPcl.intensity = dist;
                        pntPcl.x = pnt.x();
                        pntPcl.y = pnt.y();
                        pntPcl.z = pnt.z();
                        edf_slice.points.push_back(pntPcl);
//                        auto curColor = getColour(dist, 0, maxDist, 0.7);
                    }
                }
            edf_slice.header.stamp = pcl_conversions::toPCL(ros::Time::now());
            pubEDFMarker.publish(edf_slice);


            // query test

//        int bbx_nx = floor((xmax - xmin) / resolution);
//        int bbx_ny = floor((ymax - ymin) / resolution);
            float curX, curY, curZ;
            curZ = height;
            octomap::point3d curPnt;
            // octomap
            t0 = steady_clock::now();
            for (int nx = 0; nx <= bbx_nx; nx++)
                for (int ny = 0; ny <= bbx_ny; ny++) {
                    float x = xmin + resolution * nx;
                    float y = ymin + resolution * ny;
                    curPnt.x() = x;
                    curPnt.y() = y;
                    curPnt.z() = height;
                    distmap_ptr->getDistance(curPnt);
                }
            t1 = steady_clock::now();
            // voxblox
            queryTimeOctomap = duration_cast<nanoseconds>(t1 - t0).count();
            t0 = steady_clock::now();
            double dist;
            for (int nx = 0; nx <= bbx_nx; nx++)
                for (int ny = 0; ny <= bbx_ny; ny++) {
                    float x = xmin + resolution * nx;
                    float y = ymin + resolution * ny;
                    curPnt.x() = x;
                    curPnt.y() = y;
                    curPnt.z() = height;
                    voxbloxServer.getEsdfMapPtr()->getDistanceAtPosition(Eigen::Vector3d(x, y, height), &dist);
                }
            t1 = steady_clock::now();
            queryTimeVoxblox = duration_cast<nanoseconds>(t1 - t0).count();
            ROS_INFO("query time (octomap vs voxblox) for %d block [ms] = %f vs %f \n ", bbx_nx * bbx_ny,
                     queryTimeOctomap, queryTimeVoxblox);
        }
        ros::spinOnce();
        ros::Rate(30).sleep();

    }
    return 0;
}
