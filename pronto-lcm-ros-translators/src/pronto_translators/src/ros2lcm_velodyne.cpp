#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>


namespace pcl
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

typedef pcl::PointCloud<pcl::PointXYZIR> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

  pcl::io::savePCDFileASCII ("test_pcd.pcd", *msg);

//  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/velodyne_points", 1, callback);
  ros::spin();
}
