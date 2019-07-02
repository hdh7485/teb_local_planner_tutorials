//
//  highpass.cpp
//  
//
//  Created by USRG on 01/07/2019.
//

// standard
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

// velodyne
#include <velodyne_pointcloud/colors.h>
#include <velodyne_pointcloud/point_types.h>

//lcm
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include "eurecar/local_path_cmd.hpp"
#include <lcm_to_ros/hyundai_mission.h>
#include <lcm_to_ros/local_coor_wpt.h>

using namespace std;

#define PI 3.14159265359
#define COLLISION_DIST_MARGIN 1.1
#define CHECKING_DIST 5.0
#define HIGH_PASS_MISSION_NUM 8

struct point_2d {
    double x;
    double y;
};

struct goal_info{
    bool is_feasible;
    int goal_pt_idx;
};

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXYZRGB RGBPoint;
typedef pcl::PointCloud<RGBPoint> RGBPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#define LOOK_AHEAD_IDX_MIN 10
#define LOOK_AHEAD_IDX_MAX 20

sensor_msgs::PointCloud2 cloud_in, cloud_out;

class HighPass {
private:
    ros::NodeHandle nh_;
    
    // ros msg
    nav_msgs::Path subscribed_wpt_bodycoor_;
    sensor_msgs::PointCloud2 subscribed_height_map_;
    
    // ros subscriber
    ros::Subscriber wpt_bodycoor_sub_; // from wpt_republisher node
    ros::Subscriber obs_sub_; // from wpt_republisher nodes
    ros::Subscriber mission_num_sub_; // from lcm_to_ros
    
    // ros publisher
    ros::Publisher goal_pub_;
    ros::Publisher obstacle_left_publisher_ = nh_.advertise<VPointCloud>("velodyne_left_obstacles",1);
    ros::Publisher obstacle_right_publisher_ = nh_.advertise<VPointCloud>("velodyne_right_obstacles",1);;
    
    // ros msg
    geometry_msgs::PoseStamped goal_msg_;
    
    // variables
    vector<point_2d> wpt_xy_vec_;
    vector<point_2d> obs_xy_vec_;
    vector<point_2d> obs_right_xy_vec_;
    vector<point_2d> obs_left_xy_vec_;
    
    VPointCloud obstacle_left_cloud_;
    VPointCloud obstacle_right_cloud_;
    
    double cur_pos_x_;
    double cur_pos_y_;
    int cur_idx_;
    int cur_mission_num_;
    
    bool is_collision_ = false;
    bool is_arrived_ = false;
    bool collision_first_call_ = false;
    int collision_first_call_idx_;
    
    point_2d goal_pt_;
    int goal_idx_;
    int default_goal_idx_;
    bool is_goal_choose = false;
    goal_info goal_info_;
    
    vector<float> waypoint_global_x_;
    vector<float> waypoint_global_y_;
    
    int waypoint_global_length_;
    
public:
    
    HighPass():nh_("~") {
        
        // initialize
        //        goal_info_.is_feasible = false;
        
        //        std::ifstream file("/home/usrg/catkin_ws/src/teb_local_planner_tutorials/waypoint/K_CITY_MAIN.lvm");
        
        //        std::string line;
        //        std::string partial;
        
        //        std::vector<std::string> tokens;
        
        //        this->waypoint_global_x_.clear();
        //        this->waypoint_global_y_.clear();
        
        //        while(std::getline(file, line)) {     // '\n' is the default delimiter
        
        //            std::vector<std::string> strs;
        //            boost::split(strs, line, boost::is_any_of("\t"));
        
        //            this->waypoint_global_x_.push_back(atof(strs[4].c_str()));
        //            this->waypoint_global_y_.push_back(atof(strs[5].c_str()));
        //        }
        
        //        this->waypoint_global_length_ = this->waypoint_global_x_.size();
        
        this->wpt_bodycoor_sub_ = nh_.subscribe<nav_msgs::Path>("/lcm_to_ros/LCM2ROS_local_coor_wpt", 10, &HighPass::wptCallback, this);
        this->obs_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_obstacles", 10, &HighPass::obsCallback, this);
        this->mission_num_sub_ = nh_.subscribe<lcm_to_ros::hyundai_mission>("/lcm_to_ros/LCM2ROS_mission", 10, &HighPass::missionNumCallback, this);
    }
    
    bool isLeftObs(point_2d check_pt) {
        point_2d start_vec = this->wpt_xy_vec_[this->cur_idx_+LOOK_AHEAD_IDX_MIN];
        point_2d end_vec = this->wpt_xy_vec_[this->cur_idx_+LOOK_AHEAD_IDX_MAX];
        
        bool sign; // left = true / right = false
        
        point_2d vec_ref,vec_obs;
        vec_ref.x = end_vec.x - start_vec.x;
        vec_ref.y = end_vec.y - start_vec.y;
        
        vec_obs.x = check_pt.x - start_vec.x;
        vec_obs.y = check_pt.y - start_vec.y;
        
        if(vec_ref.x*vec_obs.y - vec_ref.y*vec_obs.x < 0)
            return false;
        else
            return true;
    }
    
    bool isLeftObs(point_2d check_pt, int obs_near_idx ) {
        
        point_2d start_vec = this->wpt_xy_vec_[obs_near_idx + 2];
        point_2d end_vec = this->wpt_xy_vec_[obs_near_idx + 3];
        
        bool sign; // left = true / right = false
        
        point_2d vec_ref,vec_obs;
        vec_ref.x = end_vec.x - start_vec.x;
        vec_ref.y = end_vec.y - start_vec.y;
        
        vec_obs.x = check_pt.x - start_vec.x;
        vec_obs.y = check_pt.y - start_vec.y;
        
        if(vec_ref.x*vec_obs.y - vec_ref.y*vec_obs.x < 0)
            return false;
        else
            return true;
        /*
         else
         {
         point_2d start_vec = this->wpt_xy_vec_[obs_near_idx-1];
         point_2d end_vec = this->wpt_xy_vec_[obs_near_idx];
         
         bool sign; // left = true / right = false
         
         point_2d vec_ref,vec_obs;
         vec_ref.x = end_vec.x - start_vec.x;
         vec_ref.y = end_vec.y - start_vec.y;
         
         vec_obs.x = check_pt.x - start_vec.x;
         vec_obs.y = check_pt.y - start_vec.y;
         
         if(vec_ref.x*vec_obs.y - vec_ref.y*vec_obs.x < 0)
         return false;
         else
         return true;
         }*/
    }
    
    int nearIdx(point_2d check_pt) {
        
        int near_idx = -1;
        double min_dist = 10000000;
        
        for(int i = 0; i < wpt_xy_vec_.size(); i++)
        {
            double dist = sqrt(pow(check_pt.x - wpt_xy_vec_[i].x,2) + pow(check_pt.y - wpt_xy_vec_[i].y,2));
            
            if(min_dist > dist) {
                near_idx = i;
                min_dist = dist;
            }
        }
        
        //        std::cout << near_idx << std::endl;
        return near_idx;
    }
    
    double minDist_LeftObs() {
        double left_min_dist = 1000000;
        
        for(int i = 0; i < this->obs_left_xy_vec_.size(); i++) {
            double dist = sqrt(pow(this->obs_left_xy_vec_[i].x - wpt_xy_vec_[i].x,2) + pow(this->obs_left_xy_vec_[i].y - wpt_xy_vec_[i].y,2));
            
            if(left_min_dist > dist) {
                //                near_idx = i;
                left_min_dist = dist;
            }
        }
        
        return left_min_dist;
    }
    
    double minDist_RightObs() {
        double right_min_dist = 1000000;
        
        for(int i = 0; i < this->obs_right_xy_vec_.size(); i++) {
            double dist = sqrt(pow(this->obs_right_xy_vec_[i].x - wpt_xy_vec_[i].x,2) + pow(this->obs_right_xy_vec_[i].y - wpt_xy_vec_[i].y,2));
            
            if(right_min_dist > dist) {
                //                near_idx = i;
                right_min_dist = dist;
            }
        }
        
        return right_min_dist;
    }
    
    void calcShift() {
        
        double left_dist = minDist_LeftObs();
        double right_dist = minDist_RightObs();
        
        lcm::LCM lcm;
        if (!lcm.good()) {
            ROS_INFO("LCM init error");
            return;
        }
        
        eurecar::local_path_cmd highPassLcmMsg; // temporal lcm msg
        //highPassLcmMsg.timestamp = int64_t(std::chrono::system_clock::now());
        highPassLcmMsg.timestamp = 0;
        
        std::cout << "Left min : " << left_dist << " Right min : " << right_dist << std::endl;

	highPassLcmMsg.d_steer_angle = 0.0;
        
        if(left_dist < right_dist) {
            if(left_dist < COLLISION_DIST_MARGIN) {
                // waypoint to the right
                std::cout << "to the RIGHT, LEFT dist is " << left_dist << std::endl;
                highPassLcmMsg.d_steer_angle = -1.0;
            }
            else { // do not shifting
                highPassLcmMsg.d_steer_angle = 0.0;
            }
        }
        else {
            if(right_dist < COLLISION_DIST_MARGIN) {
                // waypoint to the left
                std::cout << "to the LEFT, RIGHT dist is " << right_dist << std::endl;
                highPassLcmMsg.d_steer_angle = 1.0;
            }
            else { // do not shifting
                highPassLcmMsg.d_steer_angle = 0.0;
            }
        }
        lcm.publish("LOCAL_PATH_CMD", &highPassLcmMsg);
        
    }
    
    void missionNumCallback(const lcm_to_ros::hyundai_mission::ConstPtr& mission_msg) {
        
        ROS_DEBUG("hyundai mission callback");
        
        this->cur_pos_x_ = mission_msg->pos_x;
        this->cur_pos_y_ = mission_msg->pos_y;
        
        this->cur_idx_ = mission_msg->current_idx;
        this->cur_mission_num_ = mission_msg->mission_number;
        
    }
    
    void wptCallback(const nav_msgs::Path::ConstPtr& wpt_msg) {
        
        this->subscribed_wpt_bodycoor_ = *wpt_msg;
        
        this->wpt_xy_vec_.clear();
        
        int size = wpt_msg->poses.size();
        
        for(int i = 0; i < size; i++) {
            
            point_2d tmp;
            tmp.x = wpt_msg->poses[i].pose.position.x;
            tmp.y = wpt_msg->poses[i].pose.position.y;
            
            this->wpt_xy_vec_.push_back(tmp);
        }
        
        if(this->cur_mission_num_ == HIGH_PASS_MISSION_NUM)
            calcShift();
    }
    
    void obsCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        
        this->obs_xy_vec_.clear();
        this->obs_left_xy_vec_.clear();
        this->obs_right_xy_vec_.clear();
        this->obstacle_left_cloud_.clear();
        this->obstacle_right_cloud_.clear();
        
        this->obstacle_left_cloud_.header.frame_id = cloud_msg->header.frame_id;
        this->obstacle_right_cloud_.header.frame_id = cloud_msg->header.frame_id;
        
        // Container for original & filtered data
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 cloud_filtered;
        
        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*cloud,*temp_cloud);
        
        size_t npoints = temp_cloud->points.size();
        
        for(int i =0; i < npoints; i++) {
            point_2d tmp;
            VPoint tmp_c;
            
            tmp.x = temp_cloud->points[i].x;
            tmp.y = temp_cloud->points[i].y;
            
            double obs_dist = sqrt(tmp.x*tmp.x + tmp.y*tmp.y);
            
            if(this->wpt_xy_vec_.size() != 0)
            {
                if( tmp.x > 0 && tmp.x < this->wpt_xy_vec_[this->wpt_xy_vec_.size()-1].x
                   && tmp.x > this->wpt_xy_vec_[0].x
                   && obs_dist <= CHECKING_DIST)  {
                    
                    tmp_c.x = temp_cloud->points[i].x;
                    tmp_c.y = temp_cloud->points[i].y;
                    tmp_c.z = 0;
                    
                    int obs_near_idx = nearIdx(tmp);
                    
                    if(isLeftObs(tmp,obs_near_idx)) {
                        
                        this->obs_left_xy_vec_.push_back(tmp);
                        this->obstacle_left_cloud_.push_back(tmp_c);
                        
                    }
                    else {
                        
                        this->obs_right_xy_vec_.push_back(tmp);
                        this->obstacle_right_cloud_.push_back(tmp_c);
                        
                    }
                    
                    this->obs_xy_vec_.push_back(tmp);
                    
                }
            }
            
        }
        obstacle_left_publisher_.publish(obstacle_left_cloud_);
        obstacle_right_publisher_.publish(obstacle_right_cloud_);
    }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "highpass");
    
    ros::NodeHandle node;
    
    ros::Rate rate(20.0);
    
    HighPass gg;
    
    while(node.ok()) {
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
