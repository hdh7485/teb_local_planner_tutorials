#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <lcm_to_ros/hyundai_mission.h>

// pcl
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// velodyne
#include <velodyne_pointcloud/colors.h>
#include <velodyne_pointcloud/point_types.h>

struct point_2d {
  double x;
  double y;
};

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXYZRGB RGBPoint;
typedef pcl::PointCloud<RGBPoint> RGBPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CollisionChecker {
private:
  double collision_distance_;
  nav_msgs::Path subscribed_local_path_;
  sensor_msgs::PointCloud2 subscribed_map_;
  std::vector<point_2d> obs_vec_;
  std::vector<point_2d> path_vec_;
  std_msgs::Bool collision_result_;
  int mission_num_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber mission_num_sub_;
  ros::Publisher collision_pub_;

public:
  CollisionChecker():nh_("~") {
    obstacle_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_obstacles", 10, &CollisionChecker::obsCallback, this);
    path_sub_ = nh.subscribe<nav_msgs::Path>("/local_coor_wpt_l2r", 10, &CollisionChecker::pathCallback, this);
    mission_num_sub_ = nh.subscribe<lcm_to_ros::hyundai_mission>("/LCM2ROS_mission", 10, &CollisionChecker::missionNumCallback, this);

    collision_pub_ = nh.advertise<std_msgs::Bool>("/collision_check", 10);
  }
  
  void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    subscribed_local_path_ = *path_msg;
    collision_result_.data = (isCollision() && mission_num_ == 8);
    collision_pub_.publish(collision_result_);
  }

  void missionNumCallback(const lcm_to_ros::hyundai_mission::ConstPtr& mission_msg) {
    mission_num_ = mission_msg->mission_number;
  }

  void obsCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    obs_vec_.clear();

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *temp_cloud);

    size_t npoints = temp_cloud->points.size();

    for(int i =0; i < npoints; i++) {
      point_2d tmp;

      tmp.x = temp_cloud->points[i].x;
      tmp.y = temp_cloud->points[i].y;

      obs_vec_.push_back(tmp);
    }
  }

  bool isCollision() {
    int path_length = sizeof(subscribed_local_path_.poses)/sizeof(subscribed_local_path_.poses[0]);

    for(int path_index = 0; path_index < path_length; path_index++) {
      double path_point_x = subscribed_local_path_.poses[path_index].pose.position.x;
      double path_point_y = subscribed_local_path_.poses[path_index].pose.position.y;

      for(int obstacle_index = 0; obstacle_index < obs_vec_.size(); obstacle_index++) {
        double obstacle_point_x = obs_vec_.at(obstacle_index).x;
        double obstacle_point_y = obs_vec_.at(obstacle_index).y;

        double distance = std::sqrt(std::pow(path_point_x - obstacle_point_x, 2) + std::pow(path_point_y - obstacle_point_y, 2));
        if(distance < collision_distance_) {
          return true;
        }
      }
    }
    return false;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "collision_check");

  CollisionChecker cc;

  ros::spin();

  return 0;
}
