#include <mesh_generator/mesh_generator.h>

mesh_generator::mesh_generator()
{
    _mesh_pub = _nh.advertise<std_msgs::Float32MultiArray>(ros::this_node::getName()+"/mesh", 1);
    _nh.param<double>(ros::this_node::getName()+"/search_radius", _search_radius, 0.025);
    _nh.param<double>(ros::this_node::getName()+"/mu", _mu, 2.5);
    _nh.param<double>(ros::this_node::getName()+"/maximum_nearest_neighbors",_maximum_nearest_neighbors,100);
    _nh.param<double>(ros::this_node::getName()+"/maximum_surface_angle", _maximum_surface_angle, M_PI/4);
    _nh.param<double>(ros::this_node::getName()+"/minimum_angle", _minimum_angle, M_PI/18);
    _nh.param<double>(ros::this_node::getName()+"/maximum_angle", _maximum_angle, 2*M_PI/3);
    _triangles = boost::make_shared<pcl::PolygonMesh>();
    _pointcloud_sub = _nh.subscribe(ros::this_node::getName()+"/pointcloud",1,&mesh_generator::_pointcloud_callback,this);
}

mesh_generator::~mesh_generator()
{

}

std_msgs::Float32MultiArray mesh_generator::get_mesh_data()
{
    std::lock_guard<std::mutex> lock(_mtx);
    std_msgs::Float32MultiArray ret;
    std::vector<pcl::Vertices> vertices = _triangles->polygons;
    pcl::PCLPointCloud2 pcl_pc2 = _triangles->cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_pc);
    pcl::PointCloud<pcl::PointXYZ> cloud = *pcl_pc;
    //ret.data.resize(vertices.size()*3);
    for(int i=0;i<vertices.size(); i++)
    {
        std::vector<uint32_t> vertices_in_single_polygon;
        try
        {
            pcl::PointXYZ p0 = cloud[vertices_in_single_polygon[0]];
            ret.data.push_back(p0.x);
            ret.data.push_back(p0.y);
            ret.data.push_back(p0.z);
            pcl::PointXYZ p1 = cloud[vertices_in_single_polygon[1]];
            ret.data.push_back(p1.x);
            ret.data.push_back(p1.y);
            ret.data.push_back(p1.z);
            pcl::PointXYZ p2 = cloud[vertices_in_single_polygon[2]];
            ret.data.push_back(p2.x);
            ret.data.push_back(p2.y);
            ret.data.push_back(p2.z);
        }
        catch(...)
        {
            continue;
        }
    }
    return ret;
}

void mesh_generator::_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(_mtx);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(_search_radius);

    // Set typical values for the parameters
    gp3.setMu(_mu);
    gp3.setMaximumNearestNeighbors(_maximum_nearest_neighbors);
    gp3.setMaximumSurfaceAngle(_maximum_surface_angle); // 45 degrees
    gp3.setMinimumAngle(_minimum_angle); // 10 degrees
    gp3.setMaximumAngle(_maximum_angle); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(*_triangles);

    // Additional vertex information
    //std::vector<int> parts = gp3.getPartIDs();
    //std::vector<int> states = gp3.getPointStates();
    _mesh_pub.publish(get_mesh_data());
    return;
}