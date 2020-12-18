# mapping_comparison

In this package, we compare [octomap](https://octomap.github.io/) and [voxblox](https://voxblox.readthedocs.io/en/latest/api/library_root.html) 
with respect to Euclidean distance field (EDF).
Specifically, we compare the time taken to compute EDF from octomap or TSDT along with the query time when traversing the EDF. 
The nodes included are twofold: 1) a pointcloud publisher from pcd file, and a node where 
 each server for mapping is included, and we query by traversing in a grid way.
As the EDF from octomap does not include visualization for rviz, I coded :) 
 


<p align = "center">
<img src= "https://github.com/icsl-Jeon/mapping_comparison/blob/master/img/Screenshot from 2020-12-18 11-35-48.png" width="600">
</p>


For the former node can crop the pcd for the pointcloud published with the following paramters;
```html
    <param name="x_max" value="20"/>
    <param name="y_max" value="20"/>
    <param name="z_max" value="5"/>
    <param name="x_min" value="-10"/>
    <param name="y_min" value="-10"/>
    <param name="z_min" value="-2"/>
```
 
The tunable arguments for the second node are mainly: 
```html
    <arg name = "resolution" value = "0.2" />
    <arg name = "max_range" value = "20.0"/>
    <arg name = "max_obstacle_distance" value = "5"/>
```

Disclaimer:
I am not an expert in mapping. In this package, I just used the out-of-box API provided by the two libraries. 
Also, I just did not include the pointcloud insertion load from pointcloud (TODO). Currently, the time is tracke only in voxblox package. 

Launch the below: 

```html
   roslaunch multi_chaser octomap_vs_voxblox.launch 
```








 



