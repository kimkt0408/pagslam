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

        // pubGroundMarker_ = nh_.advertise<visualization_msgs::MarkerArray>("debug/ground_marker_", 1);
        pubGroundMarkerIn_ = nh_.advertise<visualization_msgs::Marker>("debug/ground_marker_in_", 1);
        pubGroundMarkerOut1_ = nh_.advertise<visualization_msgs::Marker>("debug/ground_marker_out1_", 1);
        pubGroundMarkerOut2_ = nh_.advertise<visualization_msgs::Marker>("debug/ground_marker_out2_", 1);
        
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


        firstScan_ = nh_.param("first_scan", true);
        bool_groundTransformFrame_ = nh_.param("bool_ground_transform_frame", false);
        bool_stalkTransformFrame_ = nh_.param("bool_stalk_transform_frame", false);
        bool_stalkCloud_ = nh_.param("bool_stalk_cloud_extraction", false);
        bool_stalkCloudClusters_ = nh_.param("bool_stalk_cloud_cluster_extraction", false);
        bool_stalkSeedClusters_ = nh_.param("bool_stalk_seed_cluster_extraction", false);

        bool_ground_ = nh_.param("bool_ground_extraction", false);
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
        eps_= nh_.param("dbscan_epsilon", 0.05); // 0.006
        minDbscanPts_ = nh_.param("dbscan_min_num_points", 2); // 80


        tf_groundSourceToTarget_.getIdentity();
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

    
    // (2) For a single LiDAR (vertical LiDAR)
    bool PAGSLAMNode::groundExtraction(CloudT::Ptr& h_cloud, PagslamInput& pagslamIn, const SE3 initialGuess)
    {
        CloudT::Ptr groundCloud(new CloudT());
        CloudT::Ptr groundCloud_outlier(new CloudT());
        pcl::ModelCoefficients::Ptr groundCoefficients (new pcl::ModelCoefficients);
               
        // ******** (1) Ground plane *********        
        // Filter the point cloud to use points with negative z-values
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(h_cloud);
        pass.setFilterFieldName("y");
        // pass.setFilterFieldName("z");
        // (1) SIM
        // pass.setFilterLimits(-1.0, -0.2);  // Set the filter limits for negative z-values
        // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
        // (2) ACRE
        // pass.setFilterLimits(0.3, 0.5);  // Set the filter limits for positive y-values
        pass.setFilterLimits(0.3, 0.8);  // Set the filter limits for positive y-values
        // pass.setFilterLimits(-1.0, -0.5);  // Set the filter limits for negative z-values
        
        // pass.setFilterLimitsNegative(false);  // Keep points inside the limits
        pass.filter(*h_cloud);

        extractor_->ransac(h_cloud, groundCloud, groundCloud_outlier, groundCoefficients);
        
        // Transform the point cloud and model coefficients to robot_frame
        bool_groundTransformFrame_ = transformFrame(v_lidar_frame_id_, robot_frame_id_, tf_groundSourceToTarget_);
        // bool_groundTransformFrame_ = transformFrame(robot_frame_id_, v_lidar_frame_id_, tf_groundSourceToTarget_);
        
        ROS_DEBUG_STREAM("Before Tf Ground: " << groundCoefficients->values[0] << " "
            << groundCoefficients->values[1] << " "
            << groundCoefficients->values[2] << " "
            << groundCoefficients->values[3]);

        if (bool_groundTransformFrame_){
            extractor_->transformGroundPlane(tf_groundSourceToTarget_, groundCloud, groundCoefficients, pagslamIn, initialGuess);

            // if (debugMode_){   
            //     pubGroundCloud_.publish(pagslamIn.groundFeature.cloud);
            //     groundPlaneVisualization(pagslamIn.groundFeature.coefficients);
            // }
            
            ROS_DEBUG_STREAM("After Tf Ground: " << pagslamIn.groundFeature.coefficients->values[0] << " "
            << pagslamIn.groundFeature.coefficients->values[1] << " "
            << pagslamIn.groundFeature.coefficients->values[2] << " "
            << pagslamIn.groundFeature.coefficients->values[3]);

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
        std::cout << "poseEstimate (SE3):" << std::endl;
        std::cout << "Rotation Matrix:" << std::endl;
        std::cout << poseEstimate.rotationMatrix() << std::endl;
        std::cout << "Translation Vector:" << std::endl;
        std::cout << poseEstimate.translation().transpose() << std::endl;
        
        ROS_DEBUG_STREAM("Entering Callback. Lidar data stamp: " << odom.stamp);

        // SEGMENTATION
        // bool_ground_ = groundExtraction(v_cloud, pagslamIn);
        // bool_ground_ = groundExtraction(h_cloud, pagslamIn);
        bool_ground_ = groundExtraction(h_cloud, pagslamIn, initialGuess);
        // bool_ground_ = groundExtraction(h_cloud, pagslamIn, poseEstimate);
        
        // if (!bool_ground_){
        //     cout << "********" << endl;
        //     return false;
        // }

        bool_stalk_ = stalkExtraction(v_cloud, pagslamIn);  

        visualization_msgs::Marker plane_marker_in = groundPlaneVisualization(pagslamIn.groundFeature, 0);        
        pubGroundMarkerIn_.publish(plane_marker_in);

        if (!firstScan_){
            visualization_msgs::Marker plane_marker_out2 = groundPlaneVisualization(prev_ground_, 2);
            pubGroundMarkerOut2_.publish(plane_marker_out2);
        }

        bool success = runPagslam(pagslamIn, pagslamOut);

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

                // visualization_msgs::MarkerArray viz_stalkVector = mapCloudVisualization(pagslamIn.mapStalkFeatures);

                // Comment when it visualization is not required
                // visualization_msgs::MarkerArray viz_stalkVector = mapCloudVisualization(semantic_map);
                // pubMapCloudMarker_.publish(viz_stalkVector);

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
            cout << "Before : " << normal << endl;
        }
        else if (groundType == 1){
            plane_marker.color.r = 1.0;
            plane_marker.color.g = 0.0;
            plane_marker.color.b = 0.0;
            cout << "After : " << normal << endl;
        }
        else if (groundType == 2){
            plane_marker.color.r = 0.0;
            plane_marker.color.g = 0.0;
            plane_marker.color.b = 1.0;
            cout << "Prev : " << normal << endl;
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
            
            ros::Time stamp = pcl_conversions::fromPCL(stalk->header).stamp;
            marker.header.stamp = stamp;

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

