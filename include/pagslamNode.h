#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <map>
#include <vector>

// Others
#include <definitions.h>
#include <pagslam.h>
#include <extraction.h>
#include <mapManager.h>
#include <visualization_msgs/MarkerArray.h>


namespace pagslam
{
    class PAGSLAMNode : public pagslam
    {
        public:
            explicit PAGSLAMNode(const ros::NodeHandle &nh);
            PAGSLAMNode(const PAGSLAMNode &) = delete;
            PAGSLAMNode operator=(const PAGSLAMNode &) = delete;
            
            using Ptr = boost::shared_ptr<PAGSLAMNode>;
            using ConstPtr = boost::shared_ptr<const PAGSLAMNode>;

            // bool groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn);
            // bool groundExtraction(CloudT::Ptr& v_cloud, PagslamInput& pagslamIn);
            bool groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn, const SE3 initialGuess);
            bool rowExtraction(CloudT::Ptr& h_cloud, std::vector<std::array<float, 2>> twoRowsYRange, PagslamInput& pagslamIn, const SE3 initialGuess);  
            bool stalkExtraction(CloudT::Ptr& v_cloud, PagslamInput& pagslamIn);
            
            visualization_msgs::Marker visualizeHistogramInRviz(const std::map<int, int>& histogram, float binWidth, float minY);
            float computeYAxisDensityHistogram(const CloudT::Ptr& cloud, std::map<int, int>& histogram, float binWidth);
            std::vector<std::pair<int, int>> extractLocalMaxima(const std::map<int, int>& histogram);
            void splitPointCloudByDensity(const CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& cloudSegments, float binWidth, int minDensity);
    
            // bool run(const SE3 initialGuess, const SE3 prevKeyPose, CloudT::Ptr h_cloud, CloudT::Ptr v_cloud, ros::Time stamp, SE3 &outPose);
            // bool run(const SE3 initialGuess, const SE3 prevKeyPose, CloudT::Ptr CloudT::Ptr v_cloud, StampedSE3 odom, SE3 &outPose);
            bool run(const SE3 initialGuess, const SE3 prevKeyPose, CloudT::Ptr h_cloud, CloudT::Ptr v_cloud, StampedSE3 odom, SE3 &outPose);
            bool transformFrame(const std::string source_frame, const std::string target_frame, tf2::Transform& tf_sourceToTarget_);
            // void groundPlaneVisualization(const pcl::ModelCoefficients::Ptr groundCoefficients);
            visualization_msgs::Marker groundPlaneVisualization(const GroundFeature groundFeature, int groundType);
            visualization_msgs::MarkerArray stalkCloudClustersVisualization(const std::vector<CloudT::Ptr> outCloudClusters);
            void stalkLinesVisualization(const std::vector<StalkFeature::Ptr>& stalkFeatures);
            // void trajectoryVisualization(const std::vector<SE3> &poses);
            visualization_msgs::MarkerArray trajectoryVisualization(const std::vector<SE3> &poses, const int pose_type);
            visualization_msgs::MarkerArray mapCloudVisualization(const std::vector<StalkFeature::Ptr> stalkVector);
            visualization_msgs::MarkerArray mapTopCloudVisualization(const std::vector<StalkFeature::Ptr> stalkVector);
            
            visualization_msgs::MarkerArray mapCloudVisualization2(const std::vector<StalkFeature::Ptr>& stalkVector, int type);
            void projectStalk(const SE3 &tf, StalkFeature::Ptr &stalk);
            visualization_msgs::Marker rowPlaneVisualization(pcl::ModelCoefficients::Ptr planeCoefficients, int groundType);
            float calculateYawFromNormal(const pcl::ModelCoefficients::Ptr rowCoefficients); 
        private:
            void initParams_();

            ros::NodeHandle nh_;

            // DEBUG TOPICS
            ros::Publisher pubGroundCloud_;

            ros::Publisher pubHCloud_;
            ros::Publisher pubVCloud_;

            ros::Publisher pubRow1Cloud_;
            ros::Publisher pubRow2Cloud_;
            
            ros::Publisher pubStalkCloud_;
            ros::Publisher pubGroundMarkerIn_;
            ros::Publisher pubGroundMarkerOut1_;
            ros::Publisher pubGroundMarkerOut2_;
            
            ros::Publisher pubRow1MarkerIn_;
            ros::Publisher pubRow2MarkerIn_;

            ros::Publisher pubStalkCloudClustersMarker_;
            ros::Publisher pubStalkSeedClustersMarker_;
            ros::Publisher pubStalkLinesMarker_;
            
            ros::Publisher pubTrajectory_;
            ros::Publisher pubOriginalTrajectory_;
            
            ros::Publisher pubMapPose_;
            ros::Publisher pubMapCloudMarker_;
            ros::Publisher pubMapTopPointMarker_;
            ros::Publisher pubMapTotalCloud_;

            ros::Publisher pubMapCloudMarkerBefore_;
            ros::Publisher pubMapCloudMarkerAfter_;

            ros::Publisher pubYAxisHistogram_;

            // Transform
            tf2_ros::Buffer tf_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
            tf2::Transform tf_groundSourceToTarget_;
            tf2::Transform tf_rowSourceToTarget_;
            tf2::Transform tf_stalkSourceToTarget_;

            // Submodule objects
            boost::shared_ptr<ext::Extraction> extractor_ = nullptr;
            MapManager semanticMap_;

            std::vector<SE3> trajectory_;
            std::vector<SE3> originalTrajectory_;
            std::vector<CloudT::Ptr> mapCloud_;

            bool firstScan_;
            bool debugMode_;
            bool bool_groundTransformFrame_;
            bool bool_rowTransformFrame_;
            bool bool_stalkTransformFrame_;
            bool bool_stalkCloud_;
            bool bool_stalkCloudClusters_;
            bool bool_stalkSeedClusters_;


            bool bool_ground_;
            bool bool_row_;
            bool bool_stalk_;

            std::string map_frame_id_;
            std::string robot_frame_id_;
            std::string h_lidar_frame_id_;
            std::string v_lidar_frame_id_;
            // Params
            // Plane
            int ransacMaxIterations_;

            // Stalk
            double eps_;  // maximum distance between points in a cluster
            int minDbscanPts_;   // minimum number of points in a cluster            
              
            GroundFeature prev_ground_;
    };
}
