#include <pagslamNode.h>
#include <pcl/common/io.h>
#include <chrono>


namespace pagslam
{
    PAGSLAMNode::PAGSLAMNode(const ros::NodeHandle &nh) : nh_(nh)
    {
        debugMode_ = nh_.param("debug_mode", true);

        if (debugMode_){
            ROS_DEBUG_STREAM("Running PAGSLAM in Debug Mode" << std::endl);
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
                ros::console::notifyLoggerLevelsChanged();
            }
        }
        else{
            if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)){
                ros::console::notifyLoggerLevelsChanged();
            }
        }

        // Debugging Publishers
        // pubGroundCloud_ = nh_.advertise<CloudT>("debug/ground_cloud", 1);
        // // pubVCloud_ = nh_.advertise<CloudT>("debug/v_cloud", 1);
        // pubStalkCloud_ = nh_.advertise<CloudT>("debug/stalk_cloud", 1);

        // pubHCloud_ = nh_.advertise<CloudT>("debug/h_cloud", 1);
        // pubVCloud_ = nh_.advertise<CloudT>("debug/v_cloud", 1);

        // pubGroundMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/ground_marker", 1);
        // pubStalkCloudClustersMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/stalk_cloud_clusters_marker", 1);
        // pubStalkSeedClustersMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/stalk_seed_clusters_marker", 1);
        // pubStalkLinesMarker_ = nh_.advertise<visualization_msgs::Marker>("debug/stalk_lines_marker", 1);
    
        // pubTrajectory_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/trajectory", 1, true);
        // pubOriginalTrajectory_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/original_trajectory", 1, true);
        
        // // pubMapPose_ = nh_.advertise<geometry_msgs::PoseStamped>("map_pose", 1);
        // pubMapPose_ = nh_.advertise<nav_msgs::Odometry>("map_pose", 1);
        // pubMapCloudMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_cloud_marker", 1);
        // pubMapTotalCloud_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_total_cloud", 1);
        // pubMapTopPointMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_top_point_marker", 1);


        pubGroundCloud_ = nh_.advertise<CloudT>("debug/ground_cloud_", 1);
        // pubVCloud_ = nh_.advertise<CloudT>("debug/v_cloud", 1);
        pubStalkCloud_ = nh_.advertise<CloudT>("debug/stalk_cloud_", 1);

        pubHCloud_ = nh_.advertise<CloudT>("debug/h_cloud_", 1);
        pubVCloud_ = nh_.advertise<CloudT>("debug/v_cloud_", 1);

        pubRow1Cloud_ = nh_.advertise<CloudT>("debug/row_cloud1_", 1);
        pubRow2Cloud_ = nh_.advertise<CloudT>("debug/row_cloud2_", 1);

        // pubGroundMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/ground_marker_", 1);
        pubGroundMarkerIn_ = nh_.advertise<visualization_msgs::Marker>("debug/ground_marker_in_", 1);
        pubGroundMarkerOut1_ = nh_.advertise<visualization_msgs::Marker>("debug/ground_marker_out1_", 1);
        pubGroundMarkerOut2_ = nh_.advertise<visualization_msgs::Marker>("debug/ground_marker_out2_", 1);

        pubRow1MarkerIn_ = nh_.advertise<visualization_msgs::Marker>("debug/row_marker_in1_", 1);
        pubRow2MarkerIn_ = nh_.advertise<visualization_msgs::Marker>("debug/row_marker_in2_", 1);
        
        pubStalkCloudClustersMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/stalk_cloud_clusters_marker_", 1);
        pubStalkSeedClustersMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/stalk_seed_clusters_marker_", 1);
        pubStalkLinesMarker_ = nh_.advertise<visualization_msgs::Marker>("debug/stalk_lines_marker_", 1);
    
        pubTrajectory_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/trajectory_", 1, true);
        pubOriginalTrajectory_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/original_trajectory_", 1, true);
        
        // pubMapPose_ = nh_.advertise<geometry_msgs::PoseStamped>("map_pose", 1);
        pubMapPose_ = nh_.advertise<nav_msgs::Odometry>("map_pose_", 1);
        pubMapCloudMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_cloud_marker_", 1);
        pubMapTotalCloud_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_total_cloud_", 1);
        pubMapTopPointMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_top_point_marker_", 1);

        pubMapCloudMarkerBefore_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_cloud_marker_before_", 1);
        pubMapCloudMarkerAfter_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/map_cloud_marker_after_", 1);

        pubYAxisHistogram_ = nh_.advertise<visualization_msgs::Marker>("debug/y_axis_histogram", 1);
        
        firstScan_ = nh_.param("first_scan", true);
        bool_groundTransformFrame_ = nh_.param("bool_ground_transform_frame", false);
        bool_rowTransformFrame_ = nh_.param("bool_row_transform_frame", false);
        bool_stalkTransformFrame_ = nh_.param("bool_stalk_transform_frame", false);
        bool_stalkCloud_ = nh_.param("bool_stalk_cloud_extraction", false);
        bool_stalkCloudClusters_ = nh_.param("bool_stalk_cloud_cluster_extraction", false);
        bool_stalkSeedClusters_ = nh_.param("bool_stalk_seed_cluster_extraction", false);

        bool_ground_ = nh_.param("bool_ground_extraction", false);
        bool_row_ = nh_.param("bool_row_extraction", false);
        bool_stalk_ = nh_.param("bool_stalk_extraction", false);
        
        tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
        
        initParams_();
    }


    void PAGSLAMNode::initParams_()
    {
        // (1) Simulation
        // ransacMaxIterations_ = nh_.param("ransac_max_iterations", 80);
        // eps_= nh_.param("dbscan_epsilon", 0.03); // 0.03
        // minDbscanPts_ = nh_.param("dbscan_min_num_points", 20); // 80

        // // (2) ACRE-1
        // ransacMaxIterations_ = nh_.param("ransac_max_iterations", 80);
        // eps_= nh_.param("dbscan_epsilon", 0.03); // 0.03
        // minDbscanPts_ = nh_.param("dbscan_min_num_points", 50); // 80

        // (2) ACRE-2
        // ransacMaxIterations_ = nh_.param("ransac_max_iterations", 100);
        // eps_= nh_.param("dbscan_epsilon", 0.008); // 0.006
        // minDbscanPts_ = nh_.param("dbscan_min_num_points", 80); // 80

        // (3) ACRE-3
        // ransacMaxIterations_ = nh_.param("ransac_max_iterations", 100);
        // eps_= nh_.param("dbscan_epsilon", 0.008); // 0.006
        // minDbscanPts_ = nh_.param("dbscan_min_num_points", 100); // 80


        // // (2) // new-ACRE-long
        // ransacMaxIterations_ = nh_.param("ransac_max_iterations", 100);
        // eps_= nh_.param("dbscan_epsilon", 0.05); // 0.006
        // minDbscanPts_ = nh_.param("dbscan_min_num_points", 2); // 80

        // TEST: 2024-08
        ransacMaxIterations_ = nh_.param("ransac_max_iterations", 10);
        eps_= nh_.param("dbscan_epsilon", 0.05); // 0.05
        minDbscanPts_ = nh_.param("dbscan_min_num_points", 2); // 80


        tf_groundSourceToTarget_.getIdentity();
        tf_rowSourceToTarget_.getIdentity();
        tf_stalkSourceToTarget_.getIdentity();

        // Frame Ids        
        nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
        nh_.param<std::string>("robot_frame_id", robot_frame_id_, "base_link");
        nh_.param<std::string>("h_lidar_frame_id_", h_lidar_frame_id_, "velodyne1");
        nh_.param<std::string>("v_lidar_frame_id_", v_lidar_frame_id_, "velodyne2");

        auto temp_ext = boost::make_shared<ext::Extraction>(ransacMaxIterations_, robot_frame_id_, eps_, minDbscanPts_);
        extractor_ = std::move(temp_ext);       

        semanticMap_ = MapManager(); 
    }


    // (1) For dual LiDARs
    // bool PAGSLAMNode::groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn)
    // {
    //     CloudT::Ptr groundCloud(new CloudT());
    //     CloudT::Ptr groundCloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr groundCoefficients (new pcl::ModelCoefficients);
               
    //     // ******** (1) Ground plane *********        
    //     // Filter the point cloud to use points with negative z-values
    //     pcl::PassThrough<PointT> pass;
    //     pass.setInputCloud(h_cloud);
    //     pass.setFilterFieldName("z");
    //     // (1) SIM
    //     // pass.setFilterLimits(-1.0, -0.2);  // Set the filter limits for negative z-values
    //     // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
    //     // (2) ACRE
    //     pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
    //     // pass.setFilterLimitsNegative(false);  // Keep points inside the limits
    //     pass.filter(*h_cloud);

    //     extractor_->ransac(h_cloud, groundCloud, groundCloud_outlier, groundCoefficients);
        
    //     // Transform the point cloud and model coefficients to robot_frame
    //     bool_groundTransformFrame_ = transformFrame(h_lidar_frame_id_, robot_frame_id_, tf_groundSourceToTarget_);

    //     if (bool_groundTransformFrame_){
    //         extractor_->transformGroundPlane(tf_groundSourceToTarget_, groundCloud, groundCoefficients, pagslamIn);

    //         // if (debugMode_){   
    //             pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
    //             groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
    //         // }

    //         if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
    //         abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //             return false;
    //         }            
    //     }

    //     // if (debugMode_){   
    //     //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
    //     //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
    //     // }
    //     if (debugMode_){   
    //         pubHCloud_.publish(h_cloud);
    //     }

    //     return true;
    // }

    
    // // (2) For a single LiDAR (vertical LiDAR)
    // bool PAGSLAMNode::groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn, const SE3 initialGuess)
    // {
    //     CloudT::Ptr groundCloud(new CloudT());
    //     CloudT::Ptr groundCloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr groundCoefficients (new pcl::ModelCoefficients);
               
    //     // ******** (1) Ground plane *********        
    //     // Filter the point cloud to use points with negative z-values
    //     pcl::PassThrough<PointT> pass;
    //     pass.setInputCloud(h_cloud);
    //     pass.setFilterFieldName("y");
    //     // pass.setFilterFieldName("z");
    //     // (1) SIM
    //     // pass.setFilterLimits(-1.0, -0.2);  // Set the filter limits for negative z-values
    //     // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
    //     // (2) ACRE
    //     // pass.setFilterLimits(0.3, 0.5);  // Set the filter limits for positive y-values
    //     pass.setFilterLimits(0.3, 0.8);  // Set the filter limits for positive y-values
    //     // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
    //     // pass.setFilterLimitsNegative(false);  // Keep points inside the limits
    //     pass.filter(*h_cloud);

    //     extractor_->ransac(h_cloud, groundCloud, groundCloud_outlier, groundCoefficients);
        
    //     // Transform the point cloud and model coefficients to robot_frame
    //     bool_groundTransformFrame_ = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_groundSourceToTarget_);
    //     // bool_groundTransformFrame_ = transformFrame(robot_frame_id_, v_lidar_frame_id_, tf_groundSourceToTarget_);
        
    //     // ROS_DEBUG_STREAM("Before Tf Ground: " << groundCoefficients->values[0] << " "
    //     //     << groundCoefficients->values[1] << " "
    //     //     << groundCoefficients->values[2] << " "
    //     //     << groundCoefficients->values[3]);

    //     if (bool_groundTransformFrame_){
    //         extractor_->transformGroundPlane(tf_groundSourceToTarget_, groundCloud, groundCoefficients, pagslamIn, initialGuess);

    //         // if (debugMode_){   
    //         //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
    //         //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
    //         // }
            
    //         // ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
    //         // << pagslamIn.groundFeature.coefficients->values[1] << " "
    //         // << pagslamIn.groundFeature.coefficients->values[2] << " "
    //         // << pagslamIn.groundFeature.coefficients->values[3]);

    //         // if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
    //         // abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //         //     return false;
    //         // }      

    //         if (abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //             return false;
    //         }         
    //     }
    //     else{
    //         return false;
    //     }

    //     if (debugMode_){   
    //         pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            
    //         // visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);
    //         // pubGroundMarkerIn_.publish(plane_marker_in);
    //     }
    //     // if (debugMode_){   
    //     //     pubHCloud_.publish(h_cloud);
    //     // }

    //     // // Add visualization marker
    //     // visualization_msgs::Marker plane_marker;
    //     // plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame
    //     // plane_marker.header.stamp = ros::Time::now();
    //     // plane_marker.ns = "ground_plane";
    //     // plane_marker.id = 0;
    //     // plane_marker.type = visualization_msgs::Marker::CUBE;
    //     // plane_marker.action = visualization_msgs::Marker::ADD;

    //     // // Calculate the normal and centroid of the plane
    //     // Eigen::Vector3f normal(pagslamIn.groundFeature.coefficients->values[0],
    //     //                        pagslamIn.groundFeature.coefficients->values[1],
    //     //                        pagslamIn.groundFeature.coefficients->values[2]);

    //     // float d = pagslamIn.groundFeature.coefficients->values[3];
    //     // Eigen::Vector3f centroid(0, 0, -d / normal.norm());

    //     // plane_marker.pose.position.x = centroid.x();
    //     // plane_marker.pose.position.y = centroid.y();
    //     // plane_marker.pose.position.z = centroid.z();

    //     // // Calculate the orientation from the normal vector
    //     // Eigen::Quaternionf q;
    //     // q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
    //     // plane_marker.pose.orientation.x = q.x();
    //     // plane_marker.pose.orientation.y = q.y();
    //     // plane_marker.pose.orientation.z = q.z();
    //     // plane_marker.pose.orientation.w = q.w();

    //     // // Set the size of the plane
    //     // plane_marker.scale.x = 10.0;
    //     // plane_marker.scale.y = 10.0;
    //     // plane_marker.scale.z = 0.01;  // Thickness of the plane

    //     // // Set the color and transparency
    //     // plane_marker.color.a = 0.5;  // Transparency
    //     // plane_marker.color.r = 0.0;
    //     // plane_marker.color.g = 1.0;
    //     // plane_marker.color.b = 0.0;

    //     // // Publish the marker
    //     // pubGroundMarker_.publish(plane_marker);

    //     return true;
    // }

    // (3) For a single LiDAR (horizontal LiDAR)
    bool PAGSLAMNode::groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn, const SE3 initialGuess)
    {
        CloudT::Ptr groundCloud(new CloudT());
        CloudT::Ptr groundCloud_outlier(new CloudT());
        pcl::ModelCoefficients::Ptr groundCoefficients (new pcl::ModelCoefficients);
               
        // ******** (1) Ground plane *********        
        // Filter the point cloud to use points with negative z-values
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(h_cloud);
        // pass.setFilterFieldName("y");
        pass.setFilterFieldName("z");
        // (1) SIM
        // pass.setFilterLimits(-1.0, -0.2);  // Set the filter limits for negative z-values
        // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
        // (2) ACRE
        // pass.setFilterLimits(0.3, 0.5);  // Set the filter limits for positive y-values
        // pass.setFilterLimits(0.3, 0.8);  // Set the filter limits for positive y-values
        pass.setFilterLimits(-0.8, -0.5);  // Set the filter limits for negative z-values
        
        // pass.setFilterLimitsNegative(false);  // Keep points inside the limits
        pass.filter(*h_cloud);

        extractor_->ransac(h_cloud, groundCloud, groundCloud_outlier, groundCoefficients);
        
        // Transform the point cloud and model coefficients to robot_frame
        bool_groundTransformFrame_ = transformFrame(h_lidar_frame_id_, robot_frame_id_, tf_groundSourceToTarget_);
        // bool_groundTransformFrame_ = transformFrame(robot_frame_id_, v_lidar_frame_id_, tf_groundSourceToTarget_);
        
        // ROS_DEBUG_STREAM("Before Tf Ground: " << groundCoefficients->values[0] << " "
        //     << groundCoefficients->values[1] << " "
        //     << groundCoefficients->values[2] << " "
        //     << groundCoefficients->values[3]);

        if (bool_groundTransformFrame_){
            extractor_->transformGroundPlane(tf_groundSourceToTarget_, groundCloud, groundCoefficients, pagslamIn, initialGuess);

            // if (debugMode_){   
            //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
            // }
            
            // ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
            // << pagslamIn.groundFeature.coefficients->values[1] << " "
            // << pagslamIn.groundFeature.coefficients->values[2] << " "
            // << pagslamIn.groundFeature.coefficients->values[3]);

            // if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
            // abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
            //     return false;
            // }      

            if (abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
                return false;
            }         
        }
        else{
            return false;
        }

        if (debugMode_){   
            pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            
            // visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);
            // pubGroundMarkerIn_.publish(plane_marker_in);
        }
        // if (debugMode_){   
        //     pubHCloud_.publish(h_cloud);
        // }

        // // Add visualization marker
        // visualization_msgs::Marker plane_marker;
        // plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame
        // plane_marker.header.stamp = ros::Time::now();
        // plane_marker.ns = "ground_plane";
        // plane_marker.id = 0;
        // plane_marker.type = visualization_msgs::Marker::CUBE;
        // plane_marker.action = visualization_msgs::Marker::ADD;

        // // Calculate the normal and centroid of the plane
        // Eigen::Vector3f normal(pagslamIn.groundFeature.coefficients->values[0],
        //                        pagslamIn.groundFeature.coefficients->values[1],
        //                        pagslamIn.groundFeature.coefficients->values[2]);

        // float d = pagslamIn.groundFeature.coefficients->values[3];
        // Eigen::Vector3f centroid(0, 0, -d / normal.norm());

        // plane_marker.pose.position.x = centroid.x();
        // plane_marker.pose.position.y = centroid.y();
        // plane_marker.pose.position.z = centroid.z();

        // // Calculate the orientation from the normal vector
        // Eigen::Quaternionf q;
        // q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
        // plane_marker.pose.orientation.x = q.x();
        // plane_marker.pose.orientation.y = q.y();
        // plane_marker.pose.orientation.z = q.z();
        // plane_marker.pose.orientation.w = q.w();

        // // Set the size of the plane
        // plane_marker.scale.x = 10.0;
        // plane_marker.scale.y = 10.0;
        // plane_marker.scale.z = 0.01;  // Thickness of the plane

        // // Set the color and transparency
        // plane_marker.color.a = 0.5;  // Transparency
        // plane_marker.color.r = 0.0;
        // plane_marker.color.g = 1.0;
        // plane_marker.color.b = 0.0;

        // // Publish the marker
        // pubGroundMarker_.publish(plane_marker);

        return true;
    }

    // // (2) For a single LiDAR (vertical LiDAR)
    // bool PAGSLAMNode::groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn)
    // {
    //     CloudT::Ptr groundCloud(new CloudT());
    //     CloudT::Ptr groundCloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr groundCoefficients (new pcl::ModelCoefficients);
               
    //     // ******** (1) Ground plane *********        
    //     // Filter the point cloud to use points with negative z-values
    //     pcl::PassThrough<PointT> pass;
    //     pass.setInputCloud(h_cloud);
    //     pass.setFilterFieldName("y");
    //     // pass.setFilterFieldName("z");
    //     // (1) SIM
    //     // pass.setFilterLimits(-1.0, -0.2);  // Set the filter limits for negative z-values
    //     // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
    //     // (2) ACRE
    //     // pass.setFilterLimits(0.3, 0.5);  // Set the filter limits for positive y-values
    //     pass.setFilterLimits(0.3, 0.8);  // Set the filter limits for positive y-values
    //     // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
    //     // pass.setFilterLimitsNegative(false);  // Keep points inside the limits
    //     pass.filter(*h_cloud);

    //     extractor_->ransac(h_cloud, groundCloud, groundCloud_outlier, groundCoefficients);
        
    //     // Transform the point cloud and model coefficients to robot_frame
    //     bool_groundTransformFrame_ = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_groundSourceToTarget_);
    //     // bool_groundTransformFrame_ = transformFrame(v_lidar_frame_id_, map_frame_id_, tf_groundSourceToTarget_);
        
    //     ROS_DEBUG_STREAM("Before Tf Ground: " << groundCoefficients->values[0] << " "
    //         << groundCoefficients->values[1] << " "
    //         << groundCoefficients->values[2] << " "
    //         << groundCoefficients->values[3]);

    //     if (bool_groundTransformFrame_){
    //         extractor_->transformGroundPlane(tf_groundSourceToTarget_, groundCloud, groundCoefficients, pagslamIn);

    //         // if (debugMode_){   
    //         //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
    //         //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
    //         // }
            
    //         ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
    //         << pagslamIn.groundFeature.coefficients->values[1] << " "
    //         << pagslamIn.groundFeature.coefficients->values[2] << " "
    //         << pagslamIn.groundFeature.coefficients->values[3]);

    //         // if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
    //         // abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //         //     return false;
    //         // }      

    //         if (abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //             return false;
    //         }         
    //     }
    //     else{
    //         return false;
    //     }

    //     if (debugMode_){   
    //         pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            
    //         // visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);
    //         // pubGroundMarkerIn_.publish(plane_marker_in);
    //     }
    //     // if (debugMode_){   
    //     //     pubHCloud_.publish(h_cloud);
    //     // }

    //     // // Add visualization marker
    //     // visualization_msgs::Marker plane_marker;
    //     // plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame
    //     // plane_marker.header.stamp = ros::Time::now();
    //     // plane_marker.ns = "ground_plane";
    //     // plane_marker.id = 0;
    //     // plane_marker.type = visualization_msgs::Marker::CUBE;
    //     // plane_marker.action = visualization_msgs::Marker::ADD;

    //     // // Calculate the normal and centroid of the plane
    //     // Eigen::Vector3f normal(pagslamIn.groundFeature.coefficients->values[0],
    //     //                        pagslamIn.groundFeature.coefficients->values[1],
    //     //                        pagslamIn.groundFeature.coefficients->values[2]);

    //     // float d = pagslamIn.groundFeature.coefficients->values[3];
    //     // Eigen::Vector3f centroid(0, 0, -d / normal.norm());

    //     // plane_marker.pose.position.x = centroid.x();
    //     // plane_marker.pose.position.y = centroid.y();
    //     // plane_marker.pose.position.z = centroid.z();

    //     // // Calculate the orientation from the normal vector
    //     // Eigen::Quaternionf q;
    //     // q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
    //     // plane_marker.pose.orientation.x = q.x();
    //     // plane_marker.pose.orientation.y = q.y();
    //     // plane_marker.pose.orientation.z = q.z();
    //     // plane_marker.pose.orientation.w = q.w();

    //     // // Set the size of the plane
    //     // plane_marker.scale.x = 10.0;
    //     // plane_marker.scale.y = 10.0;
    //     // plane_marker.scale.z = 0.01;  // Thickness of the plane

    //     // // Set the color and transparency
    //     // plane_marker.color.a = 0.5;  // Transparency
    //     // plane_marker.color.r = 0.0;
    //     // plane_marker.color.g = 1.0;
    //     // plane_marker.color.b = 0.0;

    //     // // Publish the marker
    //     // pubGroundMarker_.publish(plane_marker);

    //     return true;
    // }

    
    // (3) For a single LiDAR (horizontal LiDAR)
    // bool PAGSLAMNode::rowExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn, const SE3 initialGuess)
    // {
    //     CloudT::Ptr rowCloud(new CloudT());
    //     CloudT::Ptr rowCloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr rowCoefficients (new pcl::ModelCoefficients);
               
    //     // ******** (1) Ground plane *********        
    //     // Filter the point cloud to use points with negative z-values
    //     pcl::PassThrough<PointT> pass;
    //     pass.setInputCloud(h_cloud);
    //     // pass.setFilterFieldName("y");
    //     pass.setFilterFieldName("z");
    //     // (1) SIM
    //     // pass.setFilterLimits(-1.0, -0.2);  // Set the filter limits for negative z-values
    //     // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
    //     // (2) ACRE
    //     // pass.setFilterLimits(0.3, 0.5);  // Set the filter limits for positive y-values
    //     // pass.setFilterLimits(0.3, 0.8);  // Set the filter limits for positive y-values
    //     pass.setFilterLimits(-0.5, 1.5);  // Set the filter limits for negative z-values
        
    //     // pass.setFilterLimitsNegative(false);  // Keep points inside the limits
    //     pass.filter(*h_cloud);

    //     // extractor_->ransac(h_cloud, rowCloud, rowCloud_outlier, rowCoefficients);
    //     extractor_->rowRansac(h_cloud, rowCloud, rowCloud_outlier, rowCoefficients);
        
    //     // Transform the point cloud and model coefficients to robot_frame
    //     bool_rowTransformFrame_ = transformFrame(h_lidar_frame_id_, robot_frame_id_, tf_rowSourceToTarget_);
    //     // bool_groundTransformFrame_ = transformFrame(robot_frame_id_, v_lidar_frame_id_, tf_groundSourceToTarget_);
        
    //     // ROS_DEBUG_STREAM("Before Tf Ground: " << groundCoefficients->values[0] << " "
    //     //     << groundCoefficients->values[1] << " "
    //     //     << groundCoefficients->values[2] << " "
    //     //     << groundCoefficients->values[3]);

    //     if (bool_rowTransformFrame_){
    //         extractor_->transformGroundPlane(tf_rowSourceToTarget_, rowCloud, rowCoefficients, pagslamIn, initialGuess);

    //         // if (debugMode_){   
    //         //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
    //         //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
    //         // }
            
    //         // ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
    //         // << pagslamIn.groundFeature.coefficients->values[1] << " "
    //         // << pagslamIn.groundFeature.coefficients->values[2] << " "
    //         // << pagslamIn.groundFeature.coefficients->values[3]);

    //         // if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
    //         // abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //         //     return false;
    //         // }      

    //         if (abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //             return false;
    //         }         
    //     }
    //     else{
    //         return false;
    //     }

    //     if (debugMode_){   
    //         pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            
    //         // visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);
    //         // pubGroundMarkerIn_.publish(plane_marker_in);
    //     }
    //     // if (debugMode_){   
    //     //     pubHCloud_.publish(h_cloud);
    //     // }

    //     // // Add visualization marker
    //     // visualization_msgs::Marker plane_marker;
    //     // plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame
    //     // plane_marker.header.stamp = ros::Time::now();
    //     // plane_marker.ns = "ground_plane";
    //     // plane_marker.id = 0;
    //     // plane_marker.type = visualization_msgs::Marker::CUBE;
    //     // plane_marker.action = visualization_msgs::Marker::ADD;

    //     // // Calculate the normal and centroid of the plane
    //     // Eigen::Vector3f normal(pagslamIn.groundFeature.coefficients->values[0],
    //     //                        pagslamIn.groundFeature.coefficients->values[1],
    //     //                        pagslamIn.groundFeature.coefficients->values[2]);

    //     // float d = pagslamIn.groundFeature.coefficients->values[3];
    //     // Eigen::Vector3f centroid(0, 0, -d / normal.norm());

    //     // plane_marker.pose.position.x = centroid.x();
    //     // plane_marker.pose.position.y = centroid.y();
    //     // plane_marker.pose.position.z = centroid.z();

    //     // // Calculate the orientation from the normal vector
    //     // Eigen::Quaternionf q;
    //     // q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
    //     // plane_marker.pose.orientation.x = q.x();
    //     // plane_marker.pose.orientation.y = q.y();
    //     // plane_marker.pose.orientation.z = q.z();
    //     // plane_marker.pose.orientation.w = q.w();

    //     // // Set the size of the plane
    //     // plane_marker.scale.x = 10.0;
    //     // plane_marker.scale.y = 10.0;
    //     // plane_marker.scale.z = 0.01;  // Thickness of the plane

    //     // // Set the color and transparency
    //     // plane_marker.color.a = 0.5;  // Transparency
    //     // plane_marker.color.r = 0.0;
    //     // plane_marker.color.g = 1.0;
    //     // plane_marker.color.b = 0.0;

    //     // // Publish the marker
    //     // pubGroundMarker_.publish(plane_marker);

    //     return true;
    // }

    // // (3) For a single LiDAR (horizontal LiDAR)
    // bool PAGSLAMNode::rowExtraction(CloudT::Ptr& h_cloud, std::vector<std::array<float, 2>> twoRowsYRange, PagslamInput& pagslamIn, const SE3 initialGuess)
    // {
    //     // Downsample the input cloud to reduce the number of points
    //     pcl::VoxelGrid<PointT> sor;
    //     float leafSize = 0.01f; // Adjust this value based on the desired resolution
    //     // float leafSize = 0.08f; // Adjust this value based on the desired resolution
    //     sor.setInputCloud(h_cloud);
    //     sor.setLeafSize(leafSize, leafSize, leafSize); // Set the voxel size (leaf size)
    //     CloudT::Ptr downsampledCloud(new CloudT);
    //     sor.filter(*downsampledCloud);

    //     CloudT::Ptr row1Cloud(new CloudT());
    //     CloudT::Ptr row1Cloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr row1Coefficients (new pcl::ModelCoefficients);
    //     CloudT::Ptr row2Cloud(new CloudT());
    //     CloudT::Ptr row2Cloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr row2Coefficients (new pcl::ModelCoefficients);
        
    //     // CloudT::Ptr h_cloud1(new CloudT);  // Initialize h_cloud1 as a new point cloud
    //     // pcl::copyPointCloud(*downsampledCloud, *h_cloud1);  // Copy the points from h_cloud to h_cloud1
    //     // CloudT::Ptr h_cloud2(new CloudT);  // Initialize h_cloud1 as a new point cloud
    //     // pcl::copyPointCloud(*downsampledCloud, *h_cloud2);  // Copy the points from h_cloud to h_cloud1
    //     CloudT::Ptr h_cloud_tmp(new CloudT);  // Initialize h_cloud1 as a new point cloud
    //     pcl::copyPointCloud(*downsampledCloud, *h_cloud_tmp);  // Copy the points from h_cloud to h_cloud1

    //     // ******** (1) Ground plane *********        
    //     // Filter the point cloud to use points with negative z-values
    //     pcl::PassThrough<PointT> pass;
    //     pass.setInputCloud(h_cloud_tmp);
    //     pass.setFilterFieldName("y");

    //     if (twoRowsYRange[0][1] < twoRowsYRange[1][0]){
    //         pass.setFilterLimits(twoRowsYRange[0][0], twoRowsYRange[1][1]);  // Set the filter limits for negative z-values
    //     }
    //     else{
    //         pass.setFilterLimits(twoRowsYRange[1][0], twoRowsYRange[0][1]);  // Set the filter limits for negative z-values
    //     }
    //     // pass.setFilterLimits(twoRowsYRange[0][0], twoRowsYRange[0][1]);  // Set the filter limits for negative z-values
    //     pass.filter(*h_cloud_tmp);
    //     pubRow1Cloud_.publish(h_cloud_tmp);

    //     // Setup for DBSCAN clustering
    //     pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    //     tree->setInputCloud(h_cloud_tmp);

    //     std::vector<pcl::PointIndices> clusterIndices;
    //     pcl::EuclideanClusterExtraction<PointT> ec;
    //     ec.setClusterTolerance(0.1); // Set the tolerance to the desired cluster distance (adjust based on your environment)
    //     ec.setMinClusterSize(200);   // Minimum number of points that constitute a valid cluster (adjust based on your data)
    //     ec.setMaxClusterSize(25000); // Maximum number of points in a cluster (adjust based on your data)
    //     ec.setSearchMethod(tree);
    //     ec.setInputCloud(h_cloud_tmp);
    //     ec.extract(clusterIndices);
        
    //     // Initialize containers for clusters
    //     CloudT::Ptr h_cloud1(new CloudT());
    //     CloudT::Ptr h_cloud2(new CloudT());

    //     // Count the number of clusters
    //     int numberOfClusters = clusterIndices.size();
    //     std::cout << "Number of clusters detected: " << numberOfClusters << std::endl;

    //     // Process the clusters
    //     int clusterIdx = 0;
    //     for (const auto& indices : clusterIndices) {
    //         CloudT::Ptr cluster(new CloudT);
    //         pcl::copyPointCloud(*h_cloud_tmp, indices, *cluster);

    //         // For simplicity, let's assume we are looking for the first two clusters only
    //         if (clusterIdx == 0) {
    //             *h_cloud1 += *cluster;
    //         } else if (clusterIdx == 1) {
    //             *h_cloud2 += *cluster;
    //         } else {
    //             break; // Only process the first two clusters for now
    //         }
    //         clusterIdx++;
    //     }

    //     // If not enough clusters were found, return false
    //     if (clusterIdx < 2) {
    //         ROS_WARN("Not enough clusters found to represent two rows.");
    //         return false;
    //     }

    //     // // Store the Y ranges of each row in twoRowsYRange for future use
    //     // if (h_cloud1->points.size() > 0) {
    //     //     float minY1 = h_cloud1->points[0].y;
    //     //     float maxY1 = h_cloud1->points[0].y;
    //     //     for (const auto& point : h_cloud1->points) {
    //     //         if (point.y < minY1) minY1 = point.y;
    //     //         if (point.y > maxY1) maxY1 = point.y;
    //     //     }
    //     //     twoRowsYRange.push_back({minY1, maxY1});
    //     // }

    //     // if (h_cloud2->points.size() > 0) {
    //     //     float minY2 = h_cloud2->points[0].y;
    //     //     float maxY2 = h_cloud2->points[0].y;
    //     //     for (const auto& point : h_cloud2->points) {
    //     //         if (point.y < minY2) minY2 = point.y;
    //     //         if (point.y > maxY2) maxY2 = point.y;
    //     //     }
    //     //     twoRowsYRange.push_back({minY2, maxY2});
    //     // }

    //     // Set the header and timestamp correctly
    //     h_cloud1->header.frame_id = h_lidar_frame_id_;
    //     // h_cloud1->header.stamp = ros::Time::now().toNSec();  // Convert ROS time to nanoseconds
    //     h_cloud2->header.frame_id = h_lidar_frame_id_;
    //     // h_cloud2->header.stamp = ros::Time::now().toNSec();  // Convert ROS time to nanoseconds


    //     // Publish row clouds for visualization
    //     // pubRow1Cloud_.publish(h_cloud1);
    //     pubRow2Cloud_.publish(h_cloud2);


    //     // extractor_->ransac(h_cloud, rowCloud, rowCloud_outlier, rowCoefficients);
    //     extractor_->rowRansac(h_cloud1, row1Cloud, row1Cloud_outlier, row1Coefficients);
        
    //     ROS_DEBUG_STREAM("Before Tf Row1: " << row1Coefficients->values[0] << " "
    //         << row1Coefficients->values[1] << " "
    //         << row1Coefficients->values[2] << " "
    //         << row1Coefficients->values[3]);

    //     extractor_->rowRansac(h_cloud2, row2Cloud, row2Cloud_outlier, row2Coefficients);
        
    //     ROS_DEBUG_STREAM("Before Tf Row2: " << row2Coefficients->values[0] << " "
    //         << row2Coefficients->values[1] << " "
    //         << row2Coefficients->values[2] << " "
    //         << row2Coefficients->values[3]);

        


    //     visualization_msgs::Marker row1_plane = rowPlaneVisualization(row1Coefficients, 0);        
    //     pubRow1MarkerIn_.publish(row1_plane);

    //     visualization_msgs::Marker row2_plane = rowPlaneVisualization(row2Coefficients, 0);        
    //     pubRow2MarkerIn_.publish(row2_plane);

    //     // Transform the point cloud and model coefficients to robot_frame
    //     bool_rowTransformFrame_ = transformFrame(h_lidar_frame_id_, robot_frame_id_, tf_rowSourceToTarget_);
    //     // bool_groundTransformFrame_ = transformFrame(robot_frame_id_, v_lidar_frame_id_, tf_groundSourceToTarget_);
        
        

    //     if (bool_rowTransformFrame_){
    //         // extractor_->transformGroundPlane(tf_rowSourceToTarget_, rowCloud, rowCoefficients, pagslamIn, initialGuess);

    //         // // if (debugMode_){   
    //         // //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
    //         // //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
    //         // // }
            
    //         // // ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
    //         // // << pagslamIn.groundFeature.coefficients->values[1] << " "
    //         // // << pagslamIn.groundFeature.coefficients->values[2] << " "
    //         // // << pagslamIn.groundFeature.coefficients->values[3]);

    //         // // if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
    //         // // abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //         // //     return false;
    //         // // }      

    //         // if (abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
    //         //     return false;
    //         // }         
    //     }
    //     else{
    //         return false;
    //     }

    //     // if (debugMode_){   
    //     //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            
    //     //     // visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);
    //     //     // pubGroundMarkerIn_.publish(plane_marker_in);
    //     // }
    //     // if (debugMode_){   
    //     //     pubHCloud_.publish(h_cloud);
    //     // }

    //     // // Add visualization marker
    //     // visualization_msgs::Marker plane_marker;
    //     // plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame
    //     // plane_marker.header.stamp = ros::Time::now();
    //     // plane_marker.ns = "ground_plane";
    //     // plane_marker.id = 0;
    //     // plane_marker.type = visualization_msgs::Marker::CUBE;
    //     // plane_marker.action = visualization_msgs::Marker::ADD;

    //     // // Calculate the normal and centroid of the plane
    //     // Eigen::Vector3f normal(pagslamIn.groundFeature.coefficients->values[0],
    //     //                        pagslamIn.groundFeature.coefficients->values[1],
    //     //                        pagslamIn.groundFeature.coefficients->values[2]);

    //     // float d = pagslamIn.groundFeature.coefficients->values[3];
    //     // Eigen::Vector3f centroid(0, 0, -d / normal.norm());

    //     // plane_marker.pose.position.x = centroid.x();
    //     // plane_marker.pose.position.y = centroid.y();
    //     // plane_marker.pose.position.z = centroid.z();

    //     // // Calculate the orientation from the normal vector
    //     // Eigen::Quaternionf q;
    //     // q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
    //     // plane_marker.pose.orientation.x = q.x();
    //     // plane_marker.pose.orientation.y = q.y();
    //     // plane_marker.pose.orientation.z = q.z();
    //     // plane_marker.pose.orientation.w = q.w();

    //     // // Set the size of the plane
    //     // plane_marker.scale.x = 10.0;
    //     // plane_marker.scale.y = 10.0;
    //     // plane_marker.scale.z = 0.01;  // Thickness of the plane

    //     // // Set the color and transparency
    //     // plane_marker.color.a = 0.5;  // Transparency
    //     // plane_marker.color.r = 0.0;
    //     // plane_marker.color.g = 1.0;
    //     // plane_marker.color.b = 0.0;

    //     // // Publish the marker
    //     // pubGroundMarker_.publish(plane_marker);

    //     return true;
    // }

    // (3) For a single LiDAR (horizontal LiDAR)
    bool PAGSLAMNode::rowExtraction(CloudT::Ptr& h_cloud, std::vector<std::array<float, 2>> twoRowsYRange, PagslamInput& pagslamIn, const SE3 initialGuess)
    {
        // Downsample the input cloud to reduce the number of points
        pcl::VoxelGrid<PointT> sor;
        float leafSize = 0.02f; // Adjust this value based on the desired resolution
        // float leafSize = 0.08f; // Adjust this value based on the desired resolution
        sor.setInputCloud(h_cloud);
        sor.setLeafSize(leafSize, leafSize, leafSize); // Set the voxel size (leaf size)
        CloudT::Ptr downsampledCloud(new CloudT);
        sor.filter(*downsampledCloud);

        CloudT::Ptr row1Cloud(new CloudT());
        CloudT::Ptr row1Cloud_outlier(new CloudT());
        pcl::ModelCoefficients::Ptr row1Coefficients (new pcl::ModelCoefficients);
        CloudT::Ptr row2Cloud(new CloudT());
        CloudT::Ptr row2Cloud_outlier(new CloudT());
        pcl::ModelCoefficients::Ptr row2Coefficients (new pcl::ModelCoefficients);
        
        CloudT::Ptr h_cloud1(new CloudT);  // Initialize h_cloud1 as a new point cloud
        pcl::copyPointCloud(*h_cloud, *h_cloud1);  // Copy the points from h_cloud to h_cloud1
        CloudT::Ptr h_cloud2(new CloudT);  // Initialize h_cloud1 as a new point cloud
        pcl::copyPointCloud(*h_cloud, *h_cloud2);  // Copy the points from h_cloud to h_cloud1

        // ******** (1) Ground plane *********        
        // Filter the point cloud to use points with negative z-values

        std::array<float, 2> RowYRange1;
        std::array<float, 2> RowYRange2;

        if (twoRowsYRange[0][0] < twoRowsYRange[1][1]){
            RowYRange1 = twoRowsYRange[0];
            RowYRange2 = twoRowsYRange[1];
        }
        else{
            RowYRange1 = twoRowsYRange[1];
            RowYRange2 = twoRowsYRange[0];
        }
        pcl::PassThrough<PointT> pass1;
        pass1.setInputCloud(h_cloud1);
        pass1.setFilterFieldName("y");
        pass1.setFilterLimits(RowYRange1[0], RowYRange1[1]);  // Set the filter limits for negative z-values
        pass1.filter(*h_cloud1);
        pubRow1Cloud_.publish(h_cloud1);

        pcl::PassThrough<PointT> pass2;
        pass2.setInputCloud(h_cloud2);
        pass2.setFilterFieldName("y");
        pass2.setFilterLimits(RowYRange2[0], RowYRange2[1]);  // Set the filter limits for negative z-values
        pass2.filter(*h_cloud2);
        pubRow2Cloud_.publish(h_cloud2);

        // extractor_->ransac(h_cloud, rowCloud, rowCloud_outlier, rowCoefficients);
        extractor_->rowRansac(h_cloud1, row1Cloud, row1Cloud_outlier, row1Coefficients);
        extractor_->rowRansac(h_cloud2, row2Cloud, row2Cloud_outlier, row2Coefficients);
        
        ROS_DEBUG_STREAM("Before Tf Row1: " << row1Coefficients->values[0] << " "
            << row1Coefficients->values[1] << " "
            << row1Coefficients->values[2] << " "
            << row1Coefficients->values[3]);
        
        ROS_DEBUG_STREAM("Before Tf Row2: " << row2Coefficients->values[0] << " "
            << row2Coefficients->values[1] << " "
            << row2Coefficients->values[2] << " "
            << row2Coefficients->values[3]);

        visualization_msgs::Marker row1_plane = rowPlaneVisualization(row1Coefficients, 0);        
        pubRow1MarkerIn_.publish(row1_plane);
        visualization_msgs::Marker row2_plane = rowPlaneVisualization(row2Coefficients, 0);        
        pubRow2MarkerIn_.publish(row2_plane);

        
        // Transform the point cloud and model coefficients to robot_frame
        bool_rowTransformFrame_ = transformFrame(h_lidar_frame_id_, robot_frame_id_, tf_rowSourceToTarget_);
        // bool_groundTransformFrame_ = transformFrame(robot_frame_id_, v_lidar_frame_id_, tf_groundSourceToTarget_);
        
        

        if (bool_rowTransformFrame_){
            extractor_->transformRowPlane(tf_rowSourceToTarget_, row1Cloud, row2Cloud, row1Coefficients, row2Coefficients, pagslamIn, initialGuess);
            // extractor_->transformRowPlane(tf_rowSourceToTarget_, row2Cloud, pagslamIn, initialGuess);

            // // Construct normal vectors from the coefficients
            // Eigen::Vector3f normal1(row1Coefficients->values[0], row1Coefficients->values[1], row1Coefficients->values[2]);
            // Eigen::Vector3f normal2(row2Coefficients->values[0], row2Coefficients->values[1], row2Coefficients->values[2]);

            float yaw1 = calculateYawFromNormal(row1Coefficients);
            float yaw2 = calculateYawFromNormal(row2Coefficients);

            // Convert to degrees if necessary
            float yaw1_deg = yaw1 * 180.0 / M_PI;
            float yaw2_deg = yaw2 * 180.0 / M_PI;

            std::cout << "Yaw angle for row 1 (radians): " << yaw1 << " (degrees): " << yaw1_deg << std::endl;
            std::cout << "Yaw angle for row 2 (radians): " << yaw2 << " (degrees): " << yaw2_deg << std::endl;

            float row1_y_intercept = -row1Coefficients->values[3]/row1Coefficients->values[1];
            float row2_y_intercept = -row2Coefficients->values[3]/row2Coefficients->values[1];
            
            cout << "row1 y intercept: " << row1_y_intercept << endl;
            cout << "row2 y intercept: " << row2_y_intercept << endl;

              
            // // if (debugMode_){   
            // //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            // //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
            // // }
            
            // // ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
            // // << pagslamIn.groundFeature.coefficients->values[1] << " "
            // // << pagslamIn.groundFeature.coefficients->values[2] << " "
            // // << pagslamIn.groundFeature.coefficients->values[3]);

            // // if (pagslamIn.groundFeature.coefficients->values[3] < 0 ||
            // // abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
            // //     return false;
            // // }      

            // if (abs(pagslamIn.groundFeature.coefficients->values[3]/pagslamIn.groundFeature.coefficients->values[2]) > 1){    // If the extracted ground plane is not accurate
            //     return false;
            // }         
        }
        else{
            return false;
        }

        // if (debugMode_){   
        //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            
        //     // visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);
        //     // pubGroundMarkerIn_.publish(plane_marker_in);
        // }
        // if (debugMode_){   
        //     pubHCloud_.publish(h_cloud);
        // }

        // // Add visualization marker
        // visualization_msgs::Marker plane_marker;
        // plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame
        // plane_marker.header.stamp = ros::Time::now();
        // plane_marker.ns = "ground_plane";
        // plane_marker.id = 0;
        // plane_marker.type = visualization_msgs::Marker::CUBE;
        // plane_marker.action = visualization_msgs::Marker::ADD;

        // // Calculate the normal and centroid of the plane
        // Eigen::Vector3f normal(pagslamIn.groundFeature.coefficients->values[0],
        //                        pagslamIn.groundFeature.coefficients->values[1],
        //                        pagslamIn.groundFeature.coefficients->values[2]);

        // float d = pagslamIn.groundFeature.coefficients->values[3];
        // Eigen::Vector3f centroid(0, 0, -d / normal.norm());

        // plane_marker.pose.position.x = centroid.x();
        // plane_marker.pose.position.y = centroid.y();
        // plane_marker.pose.position.z = centroid.z();

        // // Calculate the orientation from the normal vector
        // Eigen::Quaternionf q;
        // q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
        // plane_marker.pose.orientation.x = q.x();
        // plane_marker.pose.orientation.y = q.y();
        // plane_marker.pose.orientation.z = q.z();
        // plane_marker.pose.orientation.w = q.w();

        // // Set the size of the plane
        // plane_marker.scale.x = 10.0;
        // plane_marker.scale.y = 10.0;
        // plane_marker.scale.z = 0.01;  // Thickness of the plane

        // // Set the color and transparency
        // plane_marker.color.a = 0.5;  // Transparency
        // plane_marker.color.r = 0.0;
        // plane_marker.color.g = 1.0;
        // plane_marker.color.b = 0.0;

        // // Publish the marker
        // pubGroundMarker_.publish(plane_marker);

        return true;
    }

    // Function to calculate the yaw angle from a normal vector
    float PAGSLAMNode::calculateYawFromNormal(const pcl::ModelCoefficients::Ptr rowCoefficients) {
        Eigen::Vector3f normal(rowCoefficients->values[0], rowCoefficients->values[1], rowCoefficients->values[2]);

        // Normalize the normal vector to get the direction
        Eigen::Vector3f normalized_normal = normal.normalized();

        // Project the normal vector onto the xy-plane (ignore z-component)
        float a = normalized_normal.x();
        float b = normalized_normal.y();

        // Calculate the yaw angle using atan2 to get the correct quadrant
        float yaw = std::atan2(b, a);

        return yaw; // Return yaw angle in radians
    }


    float PAGSLAMNode::computeYAxisDensityHistogram(const CloudT::Ptr& cloud, std::map<int, int>& histogram, float binWidth) {
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.2, 1.5);  // Set the filter limits for negative z-values
        pass.filter(*cloud);
    
        // Downsample the input cloud to reduce the number of points
        pcl::VoxelGrid<PointT> sor;
        float leafSize = 0.02f; // Adjust this value based on the desired resolution
        // float leafSize = 0.08f; // Adjust this value based on the desired resolution
        sor.setInputCloud(cloud);
        sor.setLeafSize(leafSize, leafSize, leafSize); // Set the voxel size (leaf size)
        CloudT::Ptr downsampledCloud(new CloudT);
        sor.filter(*downsampledCloud);
        
        // pubHCloud_.publish(downsampledCloud);

        // Initialize the histogram map
        histogram.clear();
        
        // Find the min and max y-values to determine the range
        float minY = cloud->points[0].y;
        float maxY = cloud->points[0].y;
        for (const auto& point : cloud->points) {
            if (point.y < minY) minY = point.y;
            if (point.y > maxY) maxY = point.y;
        }

        // Fill histogram based on bin width
        for (const auto& point : cloud->points) {
            int binIndex = static_cast<int>((point.y - minY) / binWidth);
            histogram[binIndex]++;
        }

        return minY;
    }

    // Function to detect local maxima from the histogram
    std::vector<std::pair<int, int>> PAGSLAMNode::extractLocalMaxima(const std::map<int, int>& histogram) {
        std::vector<std::pair<int, int>> localMaxima; // Store pairs of (bin index, count)

        // Ensure the histogram has enough bins to compare
        if (histogram.size() < 3) {
            return localMaxima;  // Not enough bins to have local maxima
        }

        // Initialize iterators properly
        auto prev_it = histogram.begin();  // First element
        auto it = prev_it;                 // Start from the first element
        ++it;                              // Move to the second element
        auto next_it = it;                 // Start from the second element
        ++next_it;                         // Move to the third element

        while (next_it != histogram.end()) {
            if (prev_it->second < it->second && it->second > next_it->second) {
                // Local maximum found, store the (bin index, count) pair
                localMaxima.emplace_back(it->first, it->second);
            }
            ++prev_it;
            ++it;
            ++next_it;
        }

        // Sort the local maxima by count in descending order
        std::sort(localMaxima.begin(), localMaxima.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            return a.second > b.second;
        });

        // // Print the two largest values of the local maxima
        // if (!localMaxima.empty()) {
        //     std::cout << "Top two local maxima:" << std::endl;
        //     for (size_t i = 0; i < std::min(localMaxima.size(), static_cast<size_t>(2)); ++i) {
        //         std::cout << "Bin: " << localMaxima[i].first << ", Y value: " << (minY + localMaxima[i].first * 0.1f) << ", Count: " << localMaxima[i].second << std::endl;
        //     }
        // } else {
        //     std::cout << "No local maxima found." << std::endl;
        // }
        
        return localMaxima;
    }


    visualization_msgs::Marker PAGSLAMNode::visualizeHistogramInRviz(const std::map<int, int>& histogram, float binWidth, float minY) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne1";
        marker.header.stamp = ros::Time::now();
        marker.ns = "histogram";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST; // Using LINE_LIST for histogram bars
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05; // Width of the lines
        marker.color.a = 1.0; // Alpha (transparency)
        marker.color.r = 1.0; // Red color

        for (const auto& bin : histogram) {
            float y_value = minY + bin.first * binWidth;
            float height = static_cast<float>(bin.second) / 100.0; // Scale height for visualization

            // Create two points for each bin (start and end of the line)
            geometry_msgs::Point p_start, p_end;
            p_start.x = 0.0;
            p_start.y = y_value;
            p_start.z = 0.0;
            p_end.x = 0.0;
            p_end.y = y_value;
            p_end.z = height;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);
        }

        return marker;
        // marker_pub.publish(marker);
        // // Publish the marker
        // ros::Rate r(1); // Loop rate for visualization
        // while (ros::ok()) {
            
        //     ros::spinOnce();
        //     r.sleep();
        // }
    }


    void PAGSLAMNode::splitPointCloudByDensity(const CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& cloudSegments, float binWidth, int minDensity) {
        // Step 1: Downsample the point cloud
        pcl::VoxelGrid<PointT> vg;
        CloudT::Ptr downsampledCloud(new CloudT);
        vg.setInputCloud(inCloud);
        vg.setLeafSize(0.1f, 0.1f, 0.1f); // Adjust leaf size as needed
        vg.filter(*downsampledCloud);

        // Step 2: Remove noise using Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(downsampledCloud);
        sor.setMeanK(50); // Number of neighbors to analyze
        sor.setStddevMulThresh(1.0); // Standard deviation multiplier threshold
        CloudT::Ptr filteredCloud(new CloudT);
        sor.filter(*filteredCloud);

        // Step 3: Compute point density along the y-axis
        std::map<int, int> densityMap; // Map to hold bin index and point count
        std::map<int, std::vector<int>> pointIndexMap; // Map to hold bin index and corresponding point indices

        // Determine the min and max y values
        float minY = filteredCloud->points[0].y;
        float maxY = filteredCloud->points[0].y;
        for (const auto& point : filteredCloud->points) {
            if (point.y < minY) minY = point.y;
            if (point.y > maxY) maxY = point.y;
        }

        // Create bins along the y-axis
        for (int i = 0; i < filteredCloud->points.size(); ++i) {
            int binIndex = static_cast<int>((filteredCloud->points[i].y - minY) / binWidth);
            densityMap[binIndex]++;
            pointIndexMap[binIndex].push_back(i);
        }

        // Step 4: Identify high-density regions and split the cloud
        for (const auto& bin : densityMap) {
            if (bin.second >= minDensity) { // Check if bin has enough points to be considered dense
                CloudT::Ptr segment(new CloudT);
                for (int idx : pointIndexMap[bin.first]) {
                    segment->points.push_back(filteredCloud->points[idx]);
                }
                cloudSegments.push_back(segment);
            }
        }
    }

    bool PAGSLAMNode::stalkExtraction(CloudT::Ptr& v_cloud, PagslamInput& pagslamIn)
    {
        CloudT::Ptr stalkCloud(new CloudT());

        //////////////////////////////////////////////////////////////////////////////////////////
        CloudT::Ptr tfm_v_cloud(new CloudT());
        std::vector<CloudT::Ptr> outCloudClusters;
        std::vector<CloudT::Ptr> tfm_outCloudClusters;
        std::vector<CloudT::Ptr> seedClusters;

        // Transform the point cloud and model coefficients to robot_frame
        bool_stalkTransformFrame_ = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_stalkSourceToTarget_);
        
        if (!bool_stalkTransformFrame_){
            return false;
        }
        
        geometry_msgs::Transform transform;
        tf2::convert(tf_stalkSourceToTarget_, transform);
        pcl_ros::transformPointCloud(*v_cloud, *tfm_v_cloud, transform);
        tfm_v_cloud->header.frame_id = robot_frame_id_;

        pubVCloud_.publish(tfm_v_cloud);

        bool_stalkCloudClusters_ =  extractor_->stalkCloudClustersExtraction(tfm_v_cloud, outCloudClusters, pagslamIn.groundFeature.coefficients);
        
        if (!bool_stalkCloudClusters_){
            return false;
        }

        // visualization_msgs::MarkerArray viz_stalkCloudClusters = stalkCloudClustersVisualization(outCloudClusters);
        // pubStalkCloudClustersMarker_.publish(viz_stalkCloudClusters);

        bool_stalkSeedClusters_ = extractor_->stalkSeedClustersExtraction(tfm_v_cloud, outCloudClusters, seedClusters);
        
        visualization_msgs::MarkerArray viz_stalkCloudClusters = stalkCloudClustersVisualization(outCloudClusters);
        pubStalkCloudClustersMarker_.publish(viz_stalkCloudClusters);

        // bool_stalkCloud_ = extractor_->stalkCloudExtraction(v_cloud, stalkCloud); // NEED TO MODIFY
        // bool_stalkCloud_ = extractor_->stalkCloudExtraction(v_cloud, stalkCloud); // NEED TO MODIFY
        
        // pubStalkCloud_.publish(stalkCloud);
        
        // if (!bool_stalkCloud_){
        //     return false;
        // }

        ////////////////////////////////////////////////////////////////////////////////////////////

        // std::vector<CloudT::Ptr> outCloudClusters;
        // std::vector<CloudT::Ptr> tfm_outCloudClusters;
        // std::vector<CloudT::Ptr> seedClusters;

        // bool_stalkCloudClusters_ =  extractor_->stalkCloudClustersExtraction(stalkCloud, outCloudClusters);
        // bool_stalkCloudClusters_ =  extractor_->stalkCloudClustersExtraction(tfm_v_cloud, outCloudClusters, pagslamIn.groundFeature.coefficients);
        
        // if (!bool_stalkCloudClusters_){
        //     return false;
        // }
        
        // // Transform the point cloud and model coefficients to robot_frame
        // bool_stalkTransformFrame_ = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_stalkSourceToTarget_);
    
        // if (!bool_stalkTransformFrame_){
        //     return false;
        // }
        
        // extractor_->transformStalkCloud(tf_stalkSourceToTarget_, outCloudClusters, tfm_outCloudClusters);
        
        // visualization_msgs::MarkerArray viz_stalkCloudClusters = stalkCloudClustersVisualization(tfm_outCloudClusters);
        // pubStalkCloudClustersMarker_.publish(viz_stalkCloudClusters);
       
        
        // bool_stalkSeedClusters_ = extractor_->stalkSeedClustersExtraction(tfm_outCloudClusters, seedClusters);
        
        if (!bool_stalkSeedClusters_){
            ROS_DEBUG_STREAM("No stalk seed clusters");
            return false;
        }

        // if (debugMode_){
            // Publish the marker array
            visualization_msgs::MarkerArray viz_seedClusters = stalkCloudClustersVisualization(seedClusters);
            pubStalkSeedClustersMarker_.publish(viz_seedClusters);
        // }

        std::vector<StalkFeature::Ptr> stalkFeatures;
        extractor_->representativeLine(seedClusters, stalkFeatures);

        pagslamIn.stalkFeatures = stalkFeatures;

        // if (debugMode_){
            stalkLinesVisualization(stalkFeatures);
        // }

        // if (debugMode_){   
            // pubVCloud_.publish(tfm_v_cloud);
        // }
        return true;
    }


    bool PAGSLAMNode::run(const SE3 initialGuess, const SE3 prevKeyPose, CloudT::Ptr h_cloud, CloudT::Ptr v_cloud, StampedSE3 odom, SE3 &outPose)
    {
        PagslamInput pagslamIn = PagslamInput();
        PagslamOutput pagslamOut = PagslamOutput();

        SE3 poseEstimate = prevKeyPose * initialGuess;
        SE3 originalPoseEstimate = odom.pose;

        pagslamIn.poseEstimate = poseEstimate;
        pagslamIn.distance = initialGuess.translation().norm();

        // stalkCloud initialization
        semanticMap_.getSubmap(poseEstimate, pagslamIn.mapStalkFeatures);

        // if (!firstScan_ && pagslamIn.mapStalkFeatures.size() == 0){
        //     ROS_DEBUG("Discarding msg");
        //     return false;
        // }

        // Assuming 'poseEstimate' is of type Sophus::SE3d
        // std::cout << "poseEstimate (SE3):" << std::endl;
        // std::cout << "Rotation Matrix:" << std::endl;
        // std::cout << poseEstimate.rotationMatrix() << std::endl;
        // std::cout << "Translation Vector:" << std::endl;
        // std::cout << poseEstimate.translation().transpose() << std::endl;
        
        ROS_DEBUG_STREAM("Entering Callback. Lidar data stamp: " << odom.stamp);

        // (1) Ground Extraction
        // SEGMENTATION
        // bool_ground_ = groundExtraction(v_cloud, pagslamIn);
        // bool_ground_ = groundExtraction(h_cloud, pagslamIn);
        CloudT::Ptr h_cloud_tmp(new CloudT);  // Initialize h_cloud_tmp as a new point cloud
        pcl::copyPointCloud(*h_cloud, *h_cloud_tmp);  // Copy the points from h_cloud to h_cloud_tmp

        bool_ground_ = groundExtraction(h_cloud, pagslamIn, initialGuess);


        // (2) Row Extraction
        // Compute histogram
        std::map<int, int> histogram;
        float binWidth = 0.1f; // Define bin width
        float minY = computeYAxisDensityHistogram(h_cloud_tmp, histogram, binWidth);
        
        std::vector<std::pair<int, int>> localMaxima = extractLocalMaxima(histogram);
        std::vector<std::array<float, 2>> twoRowsYRange;
        // Print the two largest values of the local maxima
        if (!localMaxima.empty()) {
            std::cout << "Top two local maxima:" << std::endl;
            for (size_t i = 0; i < std::min(localMaxima.size(), static_cast<size_t>(2)); ++i) {
                float yValue = minY + localMaxima[i].first * binWidth;
                std::cout << "Bin: " << localMaxima[i].first << ", Count: " << localMaxima[i].second << ", Y-Value: " << yValue << std::endl;
                
                // Store the y-range for this local maximum
                twoRowsYRange.push_back({yValue - 0.15f, yValue + 0.15f});
                // std::cout << "Bin: " << localMaxima[i].first << ", Y value: " << (minY + localMaxima[i].first * binWidth) - 0.2 << " " << (minY + localMaxima[i].first * binWidth) + 0.2 <<  ", Count: " << localMaxima[i].second << std::endl;
            }
        } 
        else {
            std::cout << "No local maxima found." << std::endl;
        }

        // Visualize histogram in RViz
        visualization_msgs::Marker marker_YHistogram = visualizeHistogramInRviz(histogram, binWidth, minY);
        pubYAxisHistogram_.publish(marker_YHistogram);

        bool_row_ = rowExtraction(h_cloud_tmp, twoRowsYRange, pagslamIn, initialGuess);
        // bool_ground_ = groundExtraction(h_cloud, pagslamIn, poseEstimate);
        
        // if (!bool_ground_){
        //     cout << "********" << endl;
        //     return false;
        // }

        // (3) Stalk Extraction
        bool_stalk_ = stalkExtraction(v_cloud, pagslamIn);  

        visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);        
        pubGroundMarkerIn_.publish(plane_marker_in);

        if (!firstScan_){
            visualization_msgs::Marker plane_marker_out2 = groundPlaneVisualization(prev_ground_, 2);
            pubGroundMarkerOut2_.publish(plane_marker_out2);
        }
        
        PagslamInput in_tmp(pagslamIn);

        for (auto &cf : in_tmp.stalkFeatures){
            // int m = 0;
            // StalkFeature::Ptr cf_proj = std::make_shared<StalkFeature>(*cf);
            // projectStalk(tf, cf_proj, false);

            // cout << "??????????" << cf->root << endl;
            projectStalk(in_tmp.poseEstimate, cf);
            // cout << "!!!!!!!!!!" << cf->root << endl;
        }

        visualization_msgs::MarkerArray viz_stalkVector_before = mapCloudVisualization2(in_tmp.stalkFeatures, 0);
        pubMapCloudMarkerBefore_.publish(viz_stalkVector_before);

        // bool success = runPagslam(pagslamIn, pagslamOut);
        bool success = runPagslam(pagslamIn, pagslamOut, initialGuess);

        visualization_msgs::MarkerArray viz_stalkVector_after = mapCloudVisualization2(pagslamOut.stalks, 1);
        pubMapCloudMarkerAfter_.publish(viz_stalkVector_after);

        visualization_msgs::Marker plane_marker_out1 = groundPlaneVisualization(pagslamOut.ground, 1);
        pubGroundMarkerOut1_.publish(plane_marker_out1);

        prev_ground_ = pagslamOut.ground;


        semanticMap_.updateMap(pagslamOut.stalks, pagslamOut.matches);

        // if (success){
            if (firstScan_ && pagslamOut.stalks.size() > 0){
                firstScan_ = false;
            }

            auto semantic_map = semanticMap_.getMap();
            trajectory_.push_back(pagslamOut.T_Map_Curr);
            outPose = pagslamOut.T_Map_Curr;

            originalTrajectory_.push_back(originalPoseEstimate);

            std::vector<CloudT::Ptr> v_cloud_vec;
            std::vector<CloudT::Ptr> tfm_v_cloud_vec;
            std::vector<CloudT::Ptr> tfm2_v_cloud_vec;
            v_cloud_vec.push_back(v_cloud);
            extractor_->transformStalkCloud(tf_stalkSourceToTarget_, v_cloud_vec, tfm_v_cloud_vec);

            tf2::Transform tf2_transform;  // resulting tf2::Transform object

            auto slam_pose = outPose;

            // // compute the tf based on odom when the graph slam optimization is called
            // SE3 wheel_vio_odom = odom.pose;
            // SE3 odom2slam = slam_pose * (wheel_vio_odom.inverse());   // SE3 for correction
            
            // Extract the rotation matrix and translation vector from the Sophus::SE3d object
            Eigen::Matrix3d rotation_matrix = slam_pose.rotationMatrix();
            Eigen::Vector3d translation_vector = slam_pose.translation();

            // Create a tf2::Matrix3x3 object from the rotation matrix
            tf2::Matrix3x3 tf2_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                                rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                                rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));

            // Create the tf2::Vector3 object from the translation vector
            tf2::Vector3 tf2_translation_vector(translation_vector(0), translation_vector(1), translation_vector(2));

            // Create the tf2::Transform object from the rotation matrix and translation vector
            tf2_transform.setBasis(tf2_rotation_matrix);
            tf2_transform.setOrigin(tf2_translation_vector);

            extractor_->transformStalkCloud(tf2_transform, tfm_v_cloud_vec, tfm2_v_cloud_vec);

            mapCloud_.push_back(tfm2_v_cloud_vec[0]);
            // mapCloud_.push_back(tfm_v_cloud_vec[0]);

            // if(debugMode_){
                visualization_msgs::MarkerArray original_tMarkerArray = trajectoryVisualization(originalTrajectory_, 0);
                pubOriginalTrajectory_.publish(original_tMarkerArray);

                visualization_msgs::MarkerArray tMarkerArray = trajectoryVisualization(trajectory_, 1);
                pubTrajectory_.publish(tMarkerArray);

                // visualization_msgs::MarkerArray viz_mapTotalCloud = stalkCloudClustersVisualization(mapCloud_);
                // pubMapTotalCloud_.publish(viz_mapTotalCloud);

                visualization_msgs::MarkerArray viz_stalkVector = mapCloudVisualization(pagslamIn.mapStalkFeatures);

                // Comment when it visualization is not required
                // visualization_msgs::MarkerArray viz_stalkVector = mapCloudVisualization(semantic_map);
                pubMapCloudMarker_.publish(viz_stalkVector);

                // visualization_msgs::MarkerArray viz_mapTopCloud = mapTopCloudVisualization(semantic_map);
                // pubMapTopPointMarker_.publish(viz_mapTopCloud);
                
                // Convert the SE3 transform to a PoseStamped message
                // geometry_msgs::PoseStamped pose_msg;
                nav_msgs::Odometry pose_msg;
                
                // Set the position
                pose_msg.pose.pose.position.x = pagslamOut.T_Map_Curr.translation().x();
                pose_msg.pose.pose.position.y = pagslamOut.T_Map_Curr.translation().y();
                pose_msg.pose.pose.position.z = pagslamOut.T_Map_Curr.translation().z();

                // Set the orientation
                Eigen::Quaterniond orientation(pagslamOut.T_Map_Curr.so3().matrix());

                pose_msg.pose.pose.orientation.x = orientation.x();
                pose_msg.pose.pose.orientation.y = orientation.y();
                pose_msg.pose.pose.orientation.z = orientation.z();
                pose_msg.pose.pose.orientation.w = orientation.w();

                
                // Set the header of the message
                pose_msg.header.frame_id = map_frame_id_; // Set the frame of the pose
                ros::Time pose_msg_stamp = pcl_conversions::fromPCL(pagslamOut.ground.coefficients->header).stamp;
                pose_msg.header.stamp = pose_msg_stamp; // Set the time of the pose

                // Publish the pose on the topic
                pubMapPose_.publish(pose_msg);
            // }
        // }
        return true;
        // return success;
    }

    void PAGSLAMNode::projectStalk(const SE3 &tf, StalkFeature::Ptr &stalk){     
        stalk->root = (tf * stalk->root.cast<double>()).cast<float>();
        stalk->top = (tf * stalk->top.cast<double>()).cast<float>();
        stalk->centroid = (tf * stalk->centroid.cast<double>()).cast<float>();
        stalk->direction = (stalk->centroid-stalk->root)/(stalk->centroid-stalk->root).norm();

        for (auto& point : stalk->cloud.points){
            Eigen::Vector3f pt (point.x, point.y, point.z);
            // cout << "Before : " << point.x << " " << point.y << " " << point.z << endl;
            pt = (tf * pt.cast<double>()).cast<float>();
            
            point.x = pt[0];
            point.y = pt[1];
            point.z = pt[2];

            // if (firstStalk){
            // cout << "After: " << point.x << " " << point.y << " " << point.z << endl;
            // }
            
        }
    }

    bool PAGSLAMNode::transformFrame(const std::string source_frame, const std::string target_frame, tf2::Transform& tf_sourceToTarget_)
    {
        // Get the transform from the source frame to the target frame
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            // ROS_WARN("%s",ex.what());
            return false;
        }
        // Convert the transform to an Eigen transform
        tf2::fromMsg(transformStamped.transform, tf_sourceToTarget_);
        
        // // Get the translation part
        // tf2::Vector3 translation = tf_sourceToTarget_.getOrigin();

        // // Get the rotation part (as a quaternion)
        // tf2::Quaternion rotation = tf_sourceToTarget_.getRotation();

        // // Print the translation
        // cout << source_frame << " " << target_frame << endl;
        // std::cout << "Translation: " << std::endl;
        // std::cout << "x: " << translation.x() << ", "
        //         << "y: " << translation.y() << ", "
        //         << "z: " << translation.z() << std::endl;

        // // Print the rotation
        // std::cout << "Rotation (quaternion): " << std::endl;
        // std::cout << "x: " << rotation.x() << ", "
        //         << "y: " << rotation.y() << ", "
        //         << "z: " << rotation.z() << ", "
        //         << "w: " << rotation.w() << std::endl;
                
        return true;
    }


    visualization_msgs::Marker PAGSLAMNode::groundPlaneVisualization(const GroundFeature groundFeature, int groundType)
    {
        // int num_markers = 360;
        // visualization_msgs::MarkerArray markers;
        // markers.markers.resize(num_markers);

        // // Loop over all markers
        // for (int i = 0; i < num_markers; i++) {
        //     markers.markers[i].header.frame_id = groundCoefficients->header.frame_id;

        //     ros::Time stamp = pcl_conversions::fromPCL(groundCoefficients->header).stamp;
        //     markers.markers[i].header.stamp = stamp;    
        //     markers.markers[i].ns = "plane_markers";
        //     markers.markers[i].id = i;
        //     markers.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
        //     markers.markers[i].action = visualization_msgs::Marker::ADD;
            
        //     float radian = float(i) * M_PI / 180.0;
        //     markers.markers[i].pose.position.x = 2*cos(radian);
        //     markers.markers[i].pose.position.y = 2*sin(radian);
        //     markers.markers[i].pose.position.z = (-groundCoefficients->values[3] - groundCoefficients->values[0] * markers.markers[i].pose.position.x - groundCoefficients->values[1] * markers.markers[i].pose.position.y) / groundCoefficients->values[2];
            
        //     markers.markers[i].pose.orientation.w = 1.0;
        //     markers.markers[i].scale.x = 0.05;
        //     markers.markers[i].scale.y = 0.05;
        //     markers.markers[i].scale.z = 0.05;
        //     markers.markers[i].color.a = 1.0;
        //     markers.markers[i].color.r = 0.0;
        //     markers.markers[i].color.g = 1.0;
        //     markers.markers[i].color.b = 0.0;
        // }

        // pubGroundMarker_.publish(markers);

        // Add visualization marker
        visualization_msgs::Marker plane_marker;
        plane_marker.header.frame_id = robot_frame_id_;  // Replace with your frame

        if (groundType == 3){
            plane_marker.header.frame_id = h_lidar_frame_id_;  // Replace with your frame
        }

        // plane_marker.header.frame_id = "map";  // Replace with your frame
        plane_marker.header.stamp = ros::Time::now();
        plane_marker.ns = "ground_plane";
        plane_marker.id = 0;
        plane_marker.type = visualization_msgs::Marker::CUBE;
        plane_marker.action = visualization_msgs::Marker::ADD;

        // Calculate the normal and centroid of the plane
        Eigen::Vector3f normal(groundFeature.coefficients->values[0],
                               groundFeature.coefficients->values[1],
                               groundFeature.coefficients->values[2]);

        float d = groundFeature.coefficients->values[3];
        
        // Calculate the centroid of the point cloud
        // Eigen::Vector4f ground_centroid;
        // pcl::compute3DCentroid(*groundFeature.cloud, ground_centroid);
        Eigen::Vector3f centroid(0, 0, -d / normal.norm());

        plane_marker.pose.position.x = centroid.x();
        plane_marker.pose.position.y = centroid.y();
        plane_marker.pose.position.z = centroid.z();

        // Calculate the orientation from the normal vector
        Eigen::Quaternionf q;
        q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
        plane_marker.pose.orientation.x = q.x();
        plane_marker.pose.orientation.y = q.y();
        plane_marker.pose.orientation.z = q.z();
        plane_marker.pose.orientation.w = q.w();

        // Set the size of the plane
        plane_marker.scale.x = 2.0;
        plane_marker.scale.y = 2.0;
        plane_marker.scale.z = 0.01;  // Thickness of the plane

        // Set the color and transparency
        plane_marker.color.a = 0.5;  // Transparency

        if (groundType == 0){
            plane_marker.color.r = 0.0;
            plane_marker.color.g = 1.0;
            plane_marker.color.b = 0.0;
            // cout << "Before : " << normal << endl;
        }
        else if (groundType == 1){
            plane_marker.color.r = 1.0;
            plane_marker.color.g = 0.0;
            plane_marker.color.b = 0.0;
            // cout << "After : " << normal << endl;
        }
        else if (groundType == 2){
            plane_marker.color.r = 0.0;
            plane_marker.color.g = 0.0;
            plane_marker.color.b = 1.0;
            // cout << "Prev : " << normal << endl;
        }

        return plane_marker;
        // Publish the marker
        // pubGroundMarker_.publish(plane_marker);
    }

    visualization_msgs::Marker PAGSLAMNode::rowPlaneVisualization(pcl::ModelCoefficients::Ptr planeCoefficients, int groundType)
    {
        visualization_msgs::Marker plane_marker;
        plane_marker.header.frame_id = h_lidar_frame_id_;  // Replace with your frame

        // plane_marker.header.frame_id = "map";  // Replace with your frame
        plane_marker.header.stamp = ros::Time::now();
        plane_marker.ns = "row_plane";
        plane_marker.id = 0;
        plane_marker.type = visualization_msgs::Marker::CUBE;
        plane_marker.action = visualization_msgs::Marker::ADD;

        // Calculate the normal and centroid of the plane
        Eigen::Vector3f normal(planeCoefficients->values[0],
                               planeCoefficients->values[1],
                               planeCoefficients->values[2]);

        float d = planeCoefficients->values[3];
        
        // cout << "Normal: " << normal << " " << d << endl;
        // Calculate the centroid of the point cloud
        // Eigen::Vector4f ground_centroid;
        // pcl::compute3DCentroid(*groundFeature.cloud, ground_centroid);
        // Eigen::Vector3f centroid(0, 0, -d / normal.norm());
        Eigen::Vector3f centroid(0, -d / planeCoefficients->values[1], 0);

        plane_marker.pose.position.x = centroid.x();
        plane_marker.pose.position.y = centroid.y();
        plane_marker.pose.position.z = centroid.z();

        // Calculate the orientation from the normal vector
        Eigen::Quaternionf q;
        q.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), normal.normalized());
        plane_marker.pose.orientation.x = q.x();
        plane_marker.pose.orientation.y = q.y();
        plane_marker.pose.orientation.z = q.z();
        plane_marker.pose.orientation.w = q.w();

        // Set the size of the plane
        plane_marker.scale.x = 4.0;
        plane_marker.scale.y = 0.5;
        plane_marker.scale.z = 0.01;  // Thickness of the plane

        // Set the color and transparency
        plane_marker.color.a = 0.5;  // Transparency

        if (groundType == 0){
            plane_marker.color.r = 0.0;
            plane_marker.color.g = 1.0;
            plane_marker.color.b = 0.0;
            // cout << "Before : " << normal << endl;
        }
        else if (groundType == 1){
            plane_marker.color.r = 1.0;
            plane_marker.color.g = 0.0;
            plane_marker.color.b = 0.0;
            // cout << "After : " << normal << endl;
        }
        else if (groundType == 2){
            plane_marker.color.r = 0.0;
            plane_marker.color.g = 0.0;
            plane_marker.color.b = 1.0;
            // cout << "Prev : " << normal << endl;
        }

        return plane_marker;
        // Publish the marker
        // pubGroundMarker_.publish(plane_marker);
    }

    visualization_msgs::MarkerArray PAGSLAMNode::stalkCloudClustersVisualization(const std::vector<CloudT::Ptr> outCloudClusters)
    {
        visualization_msgs::MarkerArray marker_array;
        // Loop over each point cloud and create a marker for it
        int id = 0;

        for (const auto& outCloudCluster : outCloudClusters){
            float num_r = static_cast<float>(rand()) / RAND_MAX;
            float num_g = static_cast<float>(rand()) / RAND_MAX;
            float num_b = static_cast<float>(rand()) / RAND_MAX;
            
            // Create a marker for the point cloud
            visualization_msgs::Marker marker;
            marker.header.frame_id = outCloudCluster->header.frame_id;
            // marker.header.frame_id = map_frame_id_;

            ros::Time stamp = pcl_conversions::fromPCL(outCloudCluster->header).stamp;
            marker.header.stamp = stamp;

            marker.id = id++;
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "point_cloud";
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = num_r;
            marker.color.g = num_g;
            marker.color.b = num_b;

            marker.pose.orientation.w = 1.0;  // Identity Quaternion
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;


            for (const auto& point : outCloudCluster->points) {
            geometry_msgs::Point p;

            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            marker.points.push_back(p);
            }

            // Add the marker to the array
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }


    void PAGSLAMNode::stalkLinesVisualization(const std::vector<StalkFeature::Ptr>& stalkFeatures)
    {
        // Create a marker message
        visualization_msgs::Marker marker;

        marker.header.frame_id = stalkFeatures[0]->header.frame_id;
        
        ros::Time stamp = pcl_conversions::fromPCL(stalkFeatures[0]->header).stamp;
        marker.header.stamp = stamp;        

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        
        // lines_marker color
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // lines_marker orientaiton
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // lines_marker position
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;

        for (const auto& stalkFeature : stalkFeatures){
            // Set the marker points to represent the line
            geometry_msgs::Point point;
            point.x = stalkFeature->root(0);
            point.y = stalkFeature->root(1);
            point.z = stalkFeature->root(2);
            marker.points.push_back(point);

            point.x = stalkFeature->root(0) + stalkFeature->direction(0);
            point.y = stalkFeature->root(1) + stalkFeature->direction(1);
            point.z = stalkFeature->root(2) + stalkFeature->direction(2);
            marker.points.push_back(point);
        }
        
        // Publish the marker array
        pubStalkLinesMarker_.publish(marker);
    }


    // void PAGSLAMNode::trajectoryVisualization(const std::vector<SE3> &poses)
    visualization_msgs::MarkerArray PAGSLAMNode::trajectoryVisualization(const std::vector<SE3> &poses, const int pose_type)
    {
        visualization_msgs::MarkerArray tMarkerArray;
        visualization_msgs::Marker points, line_strip;
        points.header.stamp = line_strip.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = "points_and_lines";
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

        points.id = 1000;
        line_strip.id = 1001;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.05;
        points.scale.y = 0.05;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.01;

        if (pose_type == 0){  // pose_type: 0 (original trajectory)
            points.header.frame_id = line_strip.header.frame_id = "map";
            points.color.r = 1.0;
            points.color.g = 0.0;
            points.color.b = 0.0;
            points.color.a = 1.0;

            // Line strip is blue
            line_strip.color.r = 0.0;
            line_strip.color.g = 0.0;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;
        } 
        else if (pose_type == 1){  // pose_type: 1 (trajectory)
            points.header.frame_id = line_strip.header.frame_id = "map";
            points.color.r = 1.0;
            points.color.g = 1.0;
            points.color.b = 0.0;
            points.color.a = 1.0;

            // Line strip is blue
            line_strip.color.r = 1.0;
            line_strip.color.g = 0.6471;
            line_strip.color.b = 0.0;
            line_strip.color.a = 1.0;
        }

        for (auto o : poses)
        {
            // Between odom line strips
            geometry_msgs::Point pt;
            auto obs_posit = o.translation();
            pt.x = obs_posit[0];
            pt.y = obs_posit[1];
            pt.z = obs_posit[2];
            points.points.push_back(pt);
            line_strip.points.push_back(pt);
        }
        tMarkerArray.markers.push_back(points);
        tMarkerArray.markers.push_back(line_strip);

        return tMarkerArray;
    }


    visualization_msgs::MarkerArray PAGSLAMNode::mapCloudVisualization(const std::vector<StalkFeature::Ptr> stalkVector)
    {
        // int num_markers = 360;
        visualization_msgs::MarkerArray marker_array;
        // Loop over each point cloud and create a marker for it
        int id = 0;

        for (const auto& stalk : stalkVector){
            float num_r = static_cast<float>(rand()) / RAND_MAX;
            float num_g = static_cast<float>(rand()) / RAND_MAX;
            float num_b = static_cast<float>(rand()) / RAND_MAX;
            
            // Create a marker for the point cloud
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            
            // ros::Time stamp = pcl_conversions::fromPCL(stalk->header).stamp;
            // marker.header.stamp = stamp;
            marker.header.stamp = ros::Time::now();


            marker.id = id++;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "point_cloud";
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = num_r;
            marker.color.g = num_g;
            marker.color.b = num_b;

            for (const auto& point : stalk->cloud.points) {
                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                marker.points.push_back(p);
            }

            // Add the marker to the array
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }

    visualization_msgs::MarkerArray PAGSLAMNode::mapCloudVisualization2(const std::vector<StalkFeature::Ptr>& stalkVector, int type)
    {
        visualization_msgs::MarkerArray marker_array;
        // Loop over each point cloud and create a marker for it
        int id = 0;

        for (const auto& stalk : stalkVector){
            float num_r;
            float num_g;
            float num_b;
            
            // Create a marker for the point cloud
            visualization_msgs::Marker marker;

            if (type == 0){
                num_r = 1.0;
                num_g = 0.0;
                num_b = 0.0;

                // marker.header.frame_id = map_frame_id_;
            }
            else if (type == 1){
                num_r = 0.0;
                num_g = 1.0;
                num_b = 0.0;

                // marker.header.frame_id = map_frame_id_;
            }

            // Create a marker for the point cloud
            // visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            
            // ros::Time stamp = pcl_conversions::fromPCL(stalk->header).stamp;
            // marker.header.stamp = stamp;
            marker.header.stamp = ros::Time::now();


            marker.id = id++;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "point_cloud";
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = num_r;
            marker.color.g = num_g;
            marker.color.b = num_b;

            for (const auto& point : stalk->cloud.points) {
                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                marker.points.push_back(p);

                // if (type == 0){
                //     cout << p.x << " " << p.y << " " << p.z << endl;
                // }
            }

            // Add the marker to the array
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }

    visualization_msgs::MarkerArray PAGSLAMNode::mapTopCloudVisualization(const std::vector<StalkFeature::Ptr> stalkVector)
    {
        // int num_markers = 360;
        visualization_msgs::MarkerArray marker_array;
        // Loop over each point cloud and create a marker for it
        int id = 0;

        for (const auto& stalk : stalkVector){
            // float num_r = static_cast<float>(rand()) / RAND_MAX;
            // float num_g = static_cast<float>(rand()) / RAND_MAX;
            // float num_b = static_cast<float>(rand()) / RAND_MAX;
            
            // Create a marker for the point cloud
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame_id_;
            
            ros::Time stamp = pcl_conversions::fromPCL(stalk->header).stamp;
            marker.header.stamp = stamp;

            marker.id = id++;
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "top_point";
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            
            geometry_msgs::Point p;

            p.x = stalk->top[0];
            p.y = stalk->top[1];
            p.z = stalk->top[2];

            marker.points.push_back(p);
            
            // for (const auto& point : stalk->cloud.points) {
            //     geometry_msgs::Point p;

            //     p.x = point.x;
            //     p.y = point.y;
            //     p.z = point.z;
            //     marker.points.push_back(p);
            // }

            // Add the marker to the array
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }
} // namespace pagslam

