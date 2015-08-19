// ros2lcm translator for point clouds
// mfallon

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <map>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>

#include <lcmtypes/pronto_lcmtypes.hpp>
/*
using namespace std;
namespace pcl
{
  // Euclidean Velodyne coordinate, including intensity and ring number.
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
*/
class App {
public:
    App(ros::NodeHandle node_);
    ~App();

private:
    lcm::LCM lcmPublish_ ;
    ros::NodeHandle node_;
    velodyne_rawdata::VPointCloud packet_;
    ros::Subscriber headLeftImageSub_;
    ros::Subscriber velodyne_scan_sub_;
    velodyne_rawdata::RawData rd_;
    int counter;
    double downsample_factor_;
    int modulo;
    int vid[32]; // velodyne index lookup table

    void headLeftImageCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void velodyneScanCallback(const velodyne_msgs::VelodyneScanConstPtr& msg);

};

App::App(ros::NodeHandle node_) :
    node_(node_), counter(0), downsample_factor_(0.5) {


    modulo = (int)ceil(1.0 / downsample_factor_);
    for(int i = 0; i < 16; i++) {
        vid[i * 2] = i * 2;
        vid[i * 2 + 1] = i * 2 + 1 + 15;
    }


    ROS_INFO("Initializing Translator");
    if(!lcmPublish_.good()) {
        std::cerr << "ERROR: lcm is not good()" << std::endl;
    }
    int sc = rd_.setup(node_);
    if(sc == 0) {
        std::cout << "SETUP successful" << std::endl;
    } else {
        std::cout << "SETUP unsuccessful (code " << sc << ")" << std::endl;
    }

    rd_.setParameters(std::numeric_limits<double>::min(),
                      std::numeric_limits<double>::max(),
                      0.0,
                      2 * M_PI);

    std::string pointcloudTopic = "velodyne_points";
    std::string velodyne_scan_topic = "velodyne_packets";
    //ros::NodeHandle nh_("~");
    // nh_.getParam("pointcloud_topic", pointcloudTopic);
    std::cout << "Subscribing to " << pointcloudTopic << std::endl;
    //headLeftImageSub_ = node_.subscribe(pointcloudTopic, 1, &App::headLeftImageCallback,this);
    velodyne_scan_sub_ = node_.subscribe(velodyne_scan_topic, 1, &App::velodyneScanCallback, this);
};

App::~App()  {
}

int pc_counter = 0;
void App::headLeftImageCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (pc_counter % 80 == 0) {
        ROS_ERROR("CLOUD [%d]", pc_counter );
    }
    pc_counter++;

    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

//  pcl::io::savePCDFileASCII ("test_pcd.pcd", *msg);

    int64_t current_utime = (int64_t) floor(msg->header.stamp.toNSec() / 1000);
    pronto::pointcloud2_t lcm_msg;
    lcm_msg.frame_id = msg->header.frame_id;
    lcm_msg.seq = msg->header.seq;
    lcm_msg.utime = current_utime;
    lcm_msg.width = msg->width;
    lcm_msg.height = msg->height;

    lcm_msg.nfields = msg->fields.size();
    for (size_t i = 0; i < msg->fields.size(); i++) {
        pronto::pointfield_t field;
        field.name = msg->fields[i].name;
        field.offset = msg->fields[i].offset;
        field.datatype = msg->fields[i].datatype;
        field.count = msg->fields[i].count;
        lcm_msg.fields.push_back( field );
    }

    lcm_msg.is_bigendian = msg->is_bigendian;
    lcm_msg.point_step = msg->point_step;
    lcm_msg.row_step = msg->row_step;
    lcm_msg.data_nbytes = msg->data.size();
    lcm_msg.data = msg->data;
    lcm_msg.is_dense = msg->is_dense;

    lcmPublish_.publish("VELODYNE", &lcm_msg);
}

void App::velodyneScanCallback(const velodyne_msgs::VelodyneScanConstPtr &msg) {
    pronto::pointcloud2_t lcm_msg;

    for(int i = 0; i < msg->packets.size(); i++) {
        std::cout << "scan size: " << msg->packets.size() << std::endl;

        rd_.unpack(msg->packets[i], packet_);


        for(int j = 0; j < packet_.size(); j++) {

        }

        packet_.clear();

    }
    counter++;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ros2lcm_camera");
    ros::NodeHandle nh;
    new App(nh);
    ROS_ERROR("ROS2LCM Camera Translator Ready");
    ros::spin();
    return 0;
}
