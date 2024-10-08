#include <queue>
#include <deque>

#include <ros/ros.h>

#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Customized header files
#include <definitions.h>
#include <pagslamNode.h>

enum
{
    CLOUD_TOO_OLD,  // 0
    CLOUD_TOO_NEW,  // 1
    CLOUD_FOUND     // 2
};


class InputManager
{
    public:
        explicit InputManager(ros::NodeHandle nh);
        bool Run();

    private:
        void OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg);
        void h_PCCb_(const sensor_msgs::PointCloud2ConstPtr &h_cloudMsg);
        void v_PCCb_(const sensor_msgs::PointCloud2ConstPtr &v_cloudMsg);

        int FindPC(const ros::Time stamp, CloudT::Ptr h_cloud, CloudT::Ptr v_cloud);
        // bool callPAGSLAM(SE3 relativeMotion, ros::Time stamp);
        bool callPAGSLAM(SE3 relativeMotion, StampedSE3 odom);

        void Odom2SlamTf(ros::Time stamp);
        void PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                            const std::string &parent_frame_id,
                            const std::string &child_frame_id);

        void se3ToRosOdom_(const SE3 &tf, const ros::Time stamp, nav_msgs::Odometry &odom_msg);
        void se3ToRosPoseStamped_(const SE3 &tf, const ros::Time stamp, geometry_msgs::PoseStamped &pose_msg);
        
        ros::NodeHandle nh_;

        ros::Publisher pubPose_;
        
        ros::Subscriber OdomSub_;
        ros::Subscriber h_PCSub_;
        ros::Subscriber v_PCSub_;

        std::queue<sensor_msgs::PointCloud2ConstPtr> h_pcQueue_;    
        std::queue<sensor_msgs::PointCloud2ConstPtr> v_pcQueue_; 
        std::deque<StampedSE3> odomQueue_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        tf2_ros::TransformBroadcaster broadcaster_;

        // params
        std::string map_frame_id_;
        std::string odom_frame_id_;
        std::string robot_frame_id_;
        
        std::string h_cloud_topic_;
        std::string v_cloud_topic_;
        std::string odom_topic_;
        float minOdomDistance_;
        size_t maxQueueSize_;

        float maxTimeDifference_;

        // vars
        boost::shared_ptr<pagslam::PAGSLAMNode> pagslam_ = nullptr;
        std::vector<SE3> keyPoses_;
        StampedSE3 latestOdom;
        bool firstOdom_;
        float firstOdomOrientation_;
        bool publishTf_;
        bool success_;
        size_t odomCounter_;
        size_t odomFreqFilter_;
};


InputManager::InputManager(ros::NodeHandle nh) : nh_(nh), tf_listener_{tf_buffer_}
{
 
    // (1) Simulation
    nh_.param<float>("min_odom_distance", minOdomDistance_, 0.1); //0.10
    nh_.param<float>("max_time_difference", maxTimeDifference_, 0.1); //0.05

    // (2) ACRE-1
    // nh_.param<float>("min_odom_distance", minOdomDistance_, 0.05); //0.05
    // nh_.param<float>("max_time_difference", maxTimeDifference_, 0.05); //0.05
    // (2) ACRE-2
    // nh_.param<float>("min_odom_distance", minOdomDistance_, 0.1); //0.05
    // nh_.param<float>("max_time_difference", maxTimeDifference_, 0.05); //0.05
    // (3) ACRE-3
    // nh_.param<float>("min_odom_distance", minOdomDistance_, 0.01); //0.05
    // nh_.param<float>("max_time_difference", maxTimeDifference_, 0.05); //0.05
    // (2) new-ACRE-long
    // nh_.param<float>("min_odom_distance", minOdomDistance_, 0.2); //0.1, 0.05
    // nh_.param<float>("max_time_difference", maxTimeDifference_, 0.05); //0.05

    odomFreqFilter_ = nh_.param("odom_freq_filter", 1);

    maxQueueSize_ = nh_.param("maxQueueSize", 30);
    // odomFreqFilter_ = nh_.param("odom_freq_filter", 20);

    publishTf_ = nh_.param("publish_tf", true);

    nh_.param<float>("first_odom_orientation", firstOdomOrientation_,  0 * (M_PI / 180)); //0.1, 0.05

    nh_.param<std::string>("robot_frame_id", robot_frame_id_, "base_link");
    nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    nh_.param<std::string>("map_frame_id", map_frame_id_, "map");

    // (1) ACRE
    nh_.param<std::string>("h_cloud_topic", h_cloud_topic_, "/ns1/velodyne_points");
    nh_.param<std::string>("v_cloud_topic", v_cloud_topic_, "/ns2/velodyne_points");
    nh_.param<std::string>("odom_topic", odom_topic_, "/odometry/filtered");

    // (2) Simulation
    // nh_.param<std::string>("h_cloud_topic", h_cloud_topic_, "/velodyne1_points");
    // nh_.param<std::string>("v_cloud_topic", v_cloud_topic_, "/velodyne2_points");
    // nh_.param<std::string>("odom_topic", odom_topic_, "/odometry/noise");

    // pubPose_ = nh_.advertise<geometry_msgs::PoseStamped>("debug/ref_frame_pagslam_pose", 10);
    pubPose_ = nh_.advertise<nav_msgs::Odometry>("debug/ref_frame_pagslam_pose_", 10);

    h_PCSub_ = nh_.subscribe(h_cloud_topic_, 10, &InputManager::h_PCCb_, this);
    v_PCSub_ = nh_.subscribe(v_cloud_topic_, 10, &InputManager::v_PCCb_, this);
    OdomSub_ = nh_.subscribe(odom_topic_, 10, &InputManager::OdomCb_, this);

    auto pagslam_ptr = boost::make_shared<pagslam::PAGSLAMNode>(nh_);
    pagslam_ = std::move(pagslam_ptr);

    firstOdom_ = true;
    odomCounter_ = 0;

    ROS_INFO("P-AgSLAM initialized");
}


// Odometry Callback function
void InputManager::OdomCb_(const nav_msgs::OdometryConstPtr &odom_msg)
{
    // Count the number of subscribed odometry data
    odomCounter_++;
    if (odomCounter_ % odomFreqFilter_ != 0){
        return;
    }
    odomCounter_ = 0;

    auto pose = odom_msg->pose.pose;

    ros::Time odomStamp = odom_msg->header.stamp;
    Quat rot(pose.orientation.w, pose.orientation.x,
             pose.orientation.y, pose.orientation.z);
    Vector3 pos(pose.position.x, pose.position.y, pose.position.z);

    SE3 odom = SE3();
    odom.setQuaternion(rot);
    odom.translation() = pos;

    // Add odom into odom queue
    odomQueue_.emplace_back(odom, odomStamp);
    if (odomQueue_.size() > 10 * maxQueueSize_){
        odomQueue_.pop_front();
    }
}


// Pointcloud Callback function
// (1) Horizontal LiDAR
void InputManager::h_PCCb_(const sensor_msgs::PointCloud2ConstPtr &h_cloudMsg)
{
    h_pcQueue_.push(h_cloudMsg);
    if (h_pcQueue_.size() > maxQueueSize_){
        h_pcQueue_.pop();
    }
}


// (2) Vertical LiDAR
void InputManager::v_PCCb_(const sensor_msgs::PointCloud2ConstPtr &v_cloudMsg)
{
    v_pcQueue_.push(v_cloudMsg);
    if (v_pcQueue_.size() > maxQueueSize_){
        v_pcQueue_.pop();
    }
}


bool InputManager::Run()
{
    if (odomQueue_.empty() || h_pcQueue_.empty() || v_pcQueue_.empty()){
        return false;
    }

    for (auto i = 0; i < odomQueue_.size(); ++i){
        auto odom = odomQueue_[i];
        // Use odom to estimate motion since last key frame
        SE3 currRelativeMotion = latestOdom.pose.inverse() * odom.pose;

        if (firstOdom_){
            ROS_DEBUG_STREAM("First pagslam call");
            if (callPAGSLAM(currRelativeMotion, odom)){
                firstOdom_ = false;
                latestOdom.pose = odom.pose;
                latestOdom.stamp = odom.stamp;
                if(publishTf_){
                    Odom2SlamTf(latestOdom.stamp);
                }
                return true;
            }
        }
        else{
            double accumMovement = currRelativeMotion.translation().norm();
            if (accumMovement > minOdomDistance_){
                ROS_DEBUG_STREAM("Pagslam call" << accumMovement);
                int k = 0;
                if (callPAGSLAM(currRelativeMotion, odom)){
                    latestOdom.pose = odom.pose;
                    latestOdom.stamp = odom.stamp;
                    if(publishTf_){
                        Odom2SlamTf(latestOdom.stamp);
                    }
                    k = 1;
                    return true;
                }
                ROS_DEBUG_STREAM("Distance between frames: " << accumMovement);
            }
        }
    }
    return false;
}


// bool InputManager::callPAGSLAM(SE3 relativeMotion, ros::Time stamp)
bool InputManager::callPAGSLAM(SE3 relativeMotion, StampedSE3 odom)
{
    CloudT::Ptr h_cloud(new CloudT);
    CloudT::Ptr v_cloud(new CloudT);

    auto r = FindPC(odom.stamp, h_cloud, v_cloud);

    if (r == CLOUD_FOUND){
        odomQueue_.pop_front();
        SE3 keyPose = SE3();
        // SE3 prevKeyPose = firstOdom_ ? SE3() : keyPoses_[keyPoses_.size() - 1];
        
        // Initialize prevKeyPose with a 90-degree yaw rotation if firstOdom_ is true
        SE3 prevKeyPose;
        if (firstOdom_) {
            Eigen::Vector3d translation(0, 0, 0);
            Eigen::Quaterniond rotation(Eigen::AngleAxisd(firstOdomOrientation_, Eigen::Vector3d::UnitZ())); // 90 degrees around Z axis
            prevKeyPose = SE3(rotation, translation);
        } else {
            prevKeyPose = keyPoses_.back();
        }

        pcl_conversions::toPCL(odom.stamp, h_cloud->header.stamp);
        pcl_conversions::toPCL(odom.stamp, v_cloud->header.stamp);
        
        // std::cout << "prevKeyPose matrix:\n" << prevKeyPose.matrix() << std::endl;
        // std::cout << "relativeMotion matrix:\n" << relativeMotion.matrix() << std::endl;

        bool success_ = pagslam_->run(relativeMotion, prevKeyPose, h_cloud, v_cloud, odom, keyPose);
        if (success_){
            keyPoses_.push_back(keyPose);
            return true;
        }
    }
    else{
        if (r == CLOUD_TOO_NEW){
            odomQueue_.pop_front();
        }

        ROS_DEBUG_STREAM("Corresponding point cloud not found. Skipping.");
    }
    return false;
}


int InputManager::FindPC(const ros::Time stamp, CloudT::Ptr h_cloud, CloudT::Ptr v_cloud)
{
    if (h_pcQueue_.empty() || v_pcQueue_.empty()){
        return false;
    }
        
    while (!h_pcQueue_.empty() && !v_pcQueue_.empty()){
        if (h_pcQueue_.front()->header.stamp.toSec() < stamp.toSec() - maxTimeDifference_){
            // ROS_DEBUG_STREAM("H_PCLOUD MSG TOO OLD");
            h_pcQueue_.pop();
        }
        else if (h_pcQueue_.front()->header.stamp.toSec() > stamp.toSec() + maxTimeDifference_){
            // ROS_DEBUG_STREAM("H_PCLOUD MSG TOO NEW");
            return CLOUD_TOO_NEW;
        }
        else{
            if (v_pcQueue_.front()->header.stamp.toSec() < stamp.toSec() - maxTimeDifference_){
                // ROS_DEBUG_STREAM("V_PCLOUD MSG TOO OLD");
                v_pcQueue_.pop();
            }
            else if (v_pcQueue_.front()->header.stamp.toSec() > stamp.toSec() + maxTimeDifference_){
                // ROS_DEBUG_STREAM("V_PCLOUD MSG TOO NEW");
                return CLOUD_TOO_NEW;
            }
            else{
                ROS_DEBUG_STREAM("Calling PAGSLAM:\n h_cloud: " << h_pcQueue_.front()->header.stamp.toSec() << "\n v_cloud: " << v_pcQueue_.front()->header.stamp.toSec() << "\n odom:" << stamp.toSec());
                
                pcl::fromROSMsg(*h_pcQueue_.front(), *h_cloud);
                h_pcQueue_.pop();

                pcl::fromROSMsg(*v_pcQueue_.front(), *v_cloud);
                v_pcQueue_.pop();

                return CLOUD_FOUND;
            }
        }
    }
    return CLOUD_TOO_OLD;
}


void InputManager::Odom2SlamTf(ros::Time stamp)
{
    if (keyPoses_.size() == 0){
        return;
    }
        
    auto slam_pose = keyPoses_[keyPoses_.size()-1];

    // compute the tf based on odom when the graph slam optimization is called
    SE3 wheel_vio_odom = latestOdom.pose;
    SE3 odom2slam = slam_pose * (wheel_vio_odom.inverse());   // SE3 for correction

    std::string parent_frame_id = map_frame_id_;
    std::string child_frame_id = odom_frame_id_;

    nav_msgs::Odometry odom_msg, odom_msg2;
    se3ToRosOdom_(odom2slam, stamp, odom_msg);
    se3ToRosOdom_(slam_pose, stamp, odom_msg2);
    
    float cov_position, cov_orientation;
    if (success_){
        cov_position = 1e-6;
        cov_orientation = 1e-4;
    }
    else{
        cov_position = 1e-2;
        cov_orientation = 1e-2;
    }

    odom_msg2.pose.covariance[0] = cov_position;
    odom_msg2.pose.covariance[7] = cov_position;
    odom_msg2.pose.covariance[14] = cov_position;
    odom_msg2.pose.covariance[21] = cov_orientation;
    odom_msg2.pose.covariance[28] = cov_orientation;
    odom_msg2.pose.covariance[35] = cov_orientation;

    odom_msg2.header.frame_id = map_frame_id_;
    odom_msg2.child_frame_id = "base_link2";
    
    pubPose_.publish(odom_msg2);

    PublishOdomAsTf(odom_msg, parent_frame_id, child_frame_id);
    
    // Convert the SE3 transform to a PoseStamped message
    geometry_msgs::PoseStamped pose_msg;
    se3ToRosPoseStamped_(slam_pose, stamp, pose_msg);
    
    // Publish the pose on the topic
    // pubPose_.publish(pose_msg);
}


void InputManager::PublishOdomAsTf(const nav_msgs::Odometry &odom_msg,
                                   const std::string &parent_frame_id,
                                   const std::string &child_frame_id)
{
    geometry_msgs::TransformStamped tf;
    tf.header = odom_msg.header;
    // note that normally parent_frame_id should be the same as odom_msg.header.frame_id
    tf.header.frame_id = parent_frame_id;
    tf.child_frame_id = child_frame_id;
    tf.transform.translation.x = odom_msg.pose.pose.position.x;
    tf.transform.translation.y = odom_msg.pose.pose.position.y;
    tf.transform.translation.z = odom_msg.pose.pose.position.z;
    tf.transform.rotation = odom_msg.pose.pose.orientation;
    broadcaster_.sendTransform(tf);
}


void InputManager::se3ToRosOdom_(const SE3 &tf, const ros::Time stamp, nav_msgs::Odometry &odom_msg)
{
    geometry_msgs::Pose pose_msg;
        
    Eigen::Quaterniond orientation(tf.so3().matrix());
    pose_msg.orientation.x = orientation.x();
    pose_msg.orientation.y = orientation.y();
    pose_msg.orientation.z = orientation.z();
    pose_msg.orientation.w = orientation.w();

    Eigen::Vector3d translation(tf.translation());
    pose_msg.position.x = translation.x();
    pose_msg.position.y = translation.y();
    pose_msg.position.z = translation.z();

    // Create an example nav_msgs::Odometry message and assign the pose
    odom_msg.pose.pose = pose_msg;

    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_id_;
}


void InputManager::se3ToRosPoseStamped_(const SE3 &tf, const ros::Time stamp, geometry_msgs::PoseStamped &pose_msg){
    // Set the position
    pose_msg.pose.position.x = tf.translation().x();
    pose_msg.pose.position.y = tf.translation().y();
    pose_msg.pose.position.z = tf.translation().z();

    // Set the orientation
    Eigen::Quaterniond quat(tf.unit_quaternion());
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();

    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    
    // Set the header of the message
    pose_msg.header.frame_id = map_frame_id_; // Set the frame of the pose
    pose_msg.header.stamp = stamp; // Set the time of the pose
}

/////////////////////////////////////////////////////
// main function //
/////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pagslam");

    ros::NodeHandle n("pagslam");

    InputManager in(n);

    ros::Rate r(20); // 10 hz
    while (ros::ok()){
        for (auto i = 0; i < 10; ++i){
            ros::spinOnce();    // spinOnce(): it allows ROS to process any incoming messages and call any callbacks that are associated with the node's subscribers.   
            if (i % 5 == 0){
                in.Run();
            }
            r.sleep();
        }
    }
    return 0;
}
