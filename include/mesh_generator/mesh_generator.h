#ifndef MESH_GENERATOR_H_INCLUDED
#define MESH_GENERATOR_H_INCLUDED

//headers in PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl_conversions/pcl_conversions.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

//headers in boost
#include <boost/shared_ptr.hpp>

//headers in STL
#include <mutex>

class mesh_generator
{
public:
    mesh_generator();
    ~mesh_generator();
    std_msgs::Float32MultiArray get_mesh_data();
private:
    ros::Subscriber _pointcloud_sub;
    ros::Publisher _mesh_pub;
    ros::NodeHandle _nh;
    void _pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr msg);
    boost::shared_ptr<pcl::PolygonMesh> _triangles;
    std::mutex _mtx;
    double _search_radius;
    double _mu;
    double _maximum_nearest_neighbors;
    double _maximum_surface_angle;
    double _minimum_angle;
    double _maximum_angle;
};

#endif  //MESH_GENERATOR_H_INCLUDED