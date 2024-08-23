#include <extraction.h>

namespace ext
{
    // (1) Simulation
    // Extraction::Extraction(int ransacMaxIterations, std::string robot_frame_id, double eps, int minDbscanPts) 
    // : ransacMaxIterations_(ransacMaxIterations),
    //   robot_frame_id_(robot_frame_id),
    //   eps_(eps),
    //   minDbscanPts_(minDbscanPts),
    //   maxSeedZLimit_(0.1),
    //   maxGrowthSearch_(30),
    // //   seedSearchRadius_(0.12),
    //   seedSearchRadius_(0.4),
    //   minSeedPts_(6),
    //   nIterations_(20),
    //   minInliers_(100)
    // {}

    // (2) ACRE-1/2
    Extraction::Extraction(int ransacMaxIterations, std::string robot_frame_id, double eps, int minDbscanPts) 
    : ransacMaxIterations_(ransacMaxIterations),
      robot_frame_id_(robot_frame_id),
      eps_(eps),
      minDbscanPts_(minDbscanPts),
    //   maxSeedZLimit_(0.1),
      maxGrowthSearch_(30),
    //   seedSearchRadius_(0.20),
      tan_vec_threshold_(0.1),
    //   minSeedPts_(3),
    //   nIterations_(20),
    //   minInliers_(100),   // Sim


    //   minSeedPts_(4),
    //   nIterations_(100),
    //   minInliers_(30)      // ACRE

      
    //   maxSeedZLimit_(0.1),
    //   minSeedPts_(4),
    //   nIterations_(100),
    //   minInliers_(30)      // ACRE


    //   seedSearchRadius_(0.12),
    //   maxSeedZLimit_(0.5),
    //   minSeedPts_(4),
    //   nIterations_(100),
    //   minInliers_(30),      // new-ACRE-long
    //   toleranceR_(0.10),
    //   min_z_addition_(0.10),
    //   max_z_addition_(0.16),
    //   offsetSearchRadius_(0.04)

      // TEST: 2024-08
      seedSearchRadius_(0.12),
      maxSeedZLimit_(0.5),
      minSeedPts_(4),
      nIterations_(10),
      minInliers_(30),      
      toleranceR_(0.10),
      min_z_addition_(0.10),
      max_z_addition_(0.16),
      offsetSearchRadius_(0.04)
    {}
    
    // (3) ACRE-3
    // Extraction::Extraction(int ransacMaxIterations, std::string robot_frame_id, double eps, int minDbscanPts) 
    // : ransacMaxIterations_(ransacMaxIterations),
    //   robot_frame_id_(robot_frame_id),
    //   eps_(eps),
    //   minDbscanPts_(minDbscanPts),
    //   maxSeedZLimit_(0.1),
    //   maxGrowthSearch_(30),
    //   seedSearchRadius_(0.20),
    // //   minSeedPts_(3),
    // //   nIterations_(20),
    // //   minInliers_(100),   // Sim


    //   minSeedPts_(4),
    //   nIterations_(100),
    //   minInliers_(30)      // ACRE
    // {}
    
    // void Extraction::downsamplePointCloud(CloudT::Ptr cloud, float leafSize)
    // {
    //     pcl::VoxelGrid<PointT> sor;
    //     sor.setInputCloud(cloud);
    //     sor.setLeafSize(leafSize, leafSize, leafSize); // Set the voxel size (leaf size)
    //     CloudT::Ptr cloud_filtered(new CloudT);
    //     sor.filter(*cloud_filtered);
    //     cloud = cloud_filtered; // Update the input cloud with the downsampled cloud
    // }


    // Added downsampling function to reduce the computational latency of the system
    void Extraction::ransac(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud_inlier, CloudT::Ptr& outCloud_outlier, pcl::ModelCoefficients::Ptr& groundCoefficients)
    {
        // Downsample the input cloud to reduce the number of points
        pcl::VoxelGrid<PointT> sor;
        float leafSize = 0.15f; // Adjust this value based on the desired resolution
        sor.setInputCloud(inCloud);
        sor.setLeafSize(leafSize, leafSize, leafSize); // Set the voxel size (leaf size)
        CloudT::Ptr downsampledCloud(new CloudT);
        sor.filter(*downsampledCloud);
        
        pcl::PointIndices inliers;
        pcl::ModelCoefficients groundCoeffs;
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (ransacMaxIterations_);
        seg.setInputCloud (downsampledCloud);

        // Set the distance threshold only once
        if (seg.getDistanceThreshold() != 0.06) {
            seg.setDistanceThreshold (0.06);
        }

        #pragma omp parallel for
        for (int i = 0; i < nIterations_; ++i)
        {
            // Perform segmentation
            seg.segment (inliers, groundCoeffs);

            // cout << "!!!!!!!!!" << inliers.indices.size() << endl;
            // If enough inliers were found
            if (inliers.indices.size () > minInliers_) {
                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<PointT> extract;
                extract.setInputCloud(downsampledCloud);
                extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
                extract.setNegative(false);
                extract.filter(*outCloud_inlier);

                extract.setNegative(true);
                extract.filter(*outCloud_outlier);

                // Update the ground coefficients
                groundCoefficients = boost::make_shared<pcl::ModelCoefficients>(groundCoeffs);

                // cout << "Model coefficients: " << groundCoefficients->values[0] << " " 
                //                     << groundCoefficients->values[1] << " "
                //                     << groundCoefficients->values[2] << " " 
                //                     << groundCoefficients->values[3] << endl;

                return;
            }
        }

        // No valid segmentation found
        outCloud_inlier->clear();
        outCloud_outlier = downsampledCloud;
        groundCoefficients.reset(new pcl::ModelCoefficients);
        ROS_WARN("No valid segmentation found in %d iterations", nIterations_);
    }
   

    // void Extraction::ransac(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud_inlier, CloudT::Ptr& outCloud_outlier, pcl::ModelCoefficients::Ptr& groundCoefficients)
    // {
    //     // Downsample the input cloud to reduce the number of points
    //     float leafSize = 1.0f; // Adjust this value based on the desired resolution
    //     downsamplePointCloud(inCloud, leafSize);
        
    //     pcl::PointIndices inliers;
    //     pcl::ModelCoefficients groundCoeffs;
    //     // Create the segmentation object
    //     pcl::SACSegmentation<PointT> seg;
    //     // Optional
    //     seg.setOptimizeCoefficients (true);
    //     // Mandatory
    //     seg.setModelType (pcl::SACMODEL_PLANE);
    //     seg.setMethodType (pcl::SAC_RANSAC);
    //     seg.setMaxIterations (ransacMaxIterations_);
    //     seg.setInputCloud (inCloud);

    //     // Set the distance threshold only once
    //     if (seg.getDistanceThreshold() != 0.06) {
    //         seg.setDistanceThreshold (0.06);
    //     }

    //     #pragma omp parallel for
    //     for (int i = 0; i < nIterations_; ++i)
    //     {
    //         // Perform segmentation
    //         seg.segment (inliers, groundCoeffs);

    //         // cout << "!!!!!!!!!" << inliers.indices.size() << endl;
    //         // If enough inliers were found
    //         if (inliers.indices.size () > minInliers_) {
    //             // Extract the planar inliers from the input cloud
    //             pcl::ExtractIndices<PointT> extract;
    //             extract.setInputCloud(inCloud);
    //             extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
    //             extract.setNegative(false);
    //             extract.filter(*outCloud_inlier);

    //             extract.setNegative(true);
    //             extract.filter(*outCloud_outlier);

    //             // Update the ground coefficients
    //             groundCoefficients = boost::make_shared<pcl::ModelCoefficients>(groundCoeffs);

    //             // cout << "Model coefficients: " << groundCoefficients->values[0] << " " 
    //             //                     << groundCoefficients->values[1] << " "
    //             //                     << groundCoefficients->values[2] << " " 
    //             //                     << groundCoefficients->values[3] << endl;

    //             return;
    //         }
    //     }

    //     // No valid segmentation found
    //     outCloud_inlier->clear();
    //     outCloud_outlier = inCloud;
    //     groundCoefficients.reset(new pcl::ModelCoefficients);
    //     ROS_WARN("No valid segmentation found in %d iterations", nIterations_);
    // }
    
    // Function to convert geometry_msgs::Transform to Eigen::Isometry3d
    Eigen::Isometry3d Extraction::transformToEigen(const geometry_msgs::Transform& transform_msg) {
        Eigen::Translation3d translation(transform_msg.translation.x,
                                        transform_msg.translation.y,
                                        transform_msg.translation.z);
        Eigen::Quaterniond rotation(transform_msg.rotation.w,
                                    transform_msg.rotation.x,
                                    transform_msg.rotation.y,
                                    transform_msg.rotation.z);

        // Combine translation and rotation into an Isometry3d
        Eigen::Isometry3d transform = translation * rotation;
        return transform;
    }

    // Function to convert Eigen::Isometry3d back to geometry_msgs::Transform
    geometry_msgs::Transform Extraction::eigenToTransform(const Eigen::Isometry3d& transform_eigen) {
        geometry_msgs::Transform transform_msg;

        transform_msg.translation.x = transform_eigen.translation().x();
        transform_msg.translation.y = transform_eigen.translation().y();
        transform_msg.translation.z = transform_eigen.translation().z();

        Eigen::Quaterniond rotation(transform_eigen.rotation());
        transform_msg.rotation.x = rotation.x();
        transform_msg.rotation.y = rotation.y();
        transform_msg.rotation.z = rotation.z();
        transform_msg.rotation.w = rotation.w();

        return transform_msg;
    }

    // Function to multiply two geometry_msgs::Transform objects
    geometry_msgs::Transform Extraction::multiplyTransforms(const geometry_msgs::Transform& t1,
                                                const geometry_msgs::Transform& t2) {
        // Convert geometry_msgs::Transform to Eigen::Isometry3d
        Eigen::Isometry3d eigen_t1 = transformToEigen(t1);
        Eigen::Isometry3d eigen_t2 = transformToEigen(t2);

        // Perform the multiplication to get A->C
        Eigen::Isometry3d result = eigen_t1 * eigen_t2;

        // Convert the result back to geometry_msgs::Transform
        return eigenToTransform(result);
    }

    // Function to convert SE3 to geometry_msgs::Transform
    geometry_msgs::Transform Extraction::SE3ToTransform(const Sophus::SE3d& se3)
    {
        geometry_msgs::Transform transform_msg;

        // Extract translation
        Eigen::Vector3d translation = se3.translation();
        transform_msg.translation.x = translation.x();
        transform_msg.translation.y = translation.y();
        transform_msg.translation.z = translation.z();

        // Extract rotation (as a quaternion)
        Eigen::Quaterniond quaternion = se3.unit_quaternion();
        transform_msg.rotation.x = quaternion.x();
        transform_msg.rotation.y = quaternion.y();
        transform_msg.rotation.z = quaternion.z();
        transform_msg.rotation.w = quaternion.w();

        return transform_msg;
    }

    void Extraction::transformGroundPlane(const tf2::Transform tf, const CloudT::Ptr& groundCloud, pcl::ModelCoefficients::Ptr& groundCoefficients, PagslamInput &pagslamIn, const SE3 initialGuess)
    {
        CloudT::Ptr tfm_groundCloud(new CloudT());
        pcl::ModelCoefficients::Ptr tfm_groundCoefficients (new pcl::ModelCoefficients);
        
        geometry_msgs::Transform transform1;
        tf2::convert(tf, transform1);

        geometry_msgs::Transform transform2 = SE3ToTransform(initialGuess);
        
        geometry_msgs::Transform transform = multiplyTransforms(transform1, transform2);
        // geometry_msgs::Transform transform = transform1;
        // (1) Point Cloud
        pcl_ros::transformPointCloud(*groundCloud, *tfm_groundCloud, transform);

        // (2) Plane Coefficient
        Eigen::Affine3d H_affine = tf2::transformToEigen(transform);
        Eigen::Matrix4d H = H_affine.matrix();
        Eigen::Vector4d coeff_vec;

        groundCoefficients->values.resize(4);
        coeff_vec[0] = groundCoefficients->values[0];
        coeff_vec[1] = groundCoefficients->values[1];
        coeff_vec[2] = groundCoefficients->values[2];
        coeff_vec[3] = groundCoefficients->values[3];

        Eigen::Matrix4d HInv = H.inverse();  
        Eigen::Matrix4d HInvTrans = HInv.transpose(); 
        Eigen::Vector4d tfm_coeff_vec = HInvTrans * coeff_vec.cast<double>();

        tfm_groundCoefficients->values.resize(4);
        tfm_groundCoefficients->values[0] = tfm_coeff_vec[0];
        tfm_groundCoefficients->values[1] = tfm_coeff_vec[1];
        tfm_groundCoefficients->values[2] = tfm_coeff_vec[2];
        tfm_groundCoefficients->values[3] = tfm_coeff_vec[3];

        // cout << "Model coefficients: " << groundCoefficients->values[0] << " " 
        //                             << groundCoefficients->values[1] << " "
        //                             << groundCoefficients->values[2] << " " 
        //                             << groundCoefficients->values[3] << " " 
        //                             << -groundCoefficients->values[3]/groundCoefficients->values[2] << endl;


        tfm_groundCoefficients->header.seq = groundCloud->header.seq;
        tfm_groundCoefficients->header.stamp = groundCloud->header.stamp;
        tfm_groundCoefficients->header.frame_id = robot_frame_id_;

        pagslamIn.groundFeature.cloud = tfm_groundCloud;
        pagslamIn.groundFeature.cloud->header.frame_id = robot_frame_id_;

        pagslamIn.groundFeature.coefficients = tfm_groundCoefficients;  
    }


    // void Extraction::transformGroundPlane(const tf2::Transform tf, const CloudT::Ptr& groundCloud, pcl::ModelCoefficients::Ptr& groundCoefficients, PagslamInput &pagslamIn)
    // {
    //     CloudT::Ptr tfm_groundCloud(new CloudT());
    //     pcl::ModelCoefficients::Ptr tfm_groundCoefficients (new pcl::ModelCoefficients);
        
    //     geometry_msgs::Transform transform1;
    //     tf2::convert(tf, transform1);

    //     // geometry_msgs::Transform transform2 = SE3ToTransform(pagslamIn.poseEstimate);
        
    //     // geometry_msgs::Transform transform = multiplyTransforms(transform1, transform2);
    //     geometry_msgs::Transform transform = transform1;
    //     // (1) Point Cloud
    //     pcl_ros::transformPointCloud(*groundCloud, *tfm_groundCloud, transform);

    //     // (2) Plane Coefficient
    //     Eigen::Affine3d H_affine = tf2::transformToEigen(transform);
    //     Eigen::Matrix4d H = H_affine.matrix();
    //     Eigen::Vector4d coeff_vec;

    //     groundCoefficients->values.resize(4);
    //     coeff_vec[0] = groundCoefficients->values[0];
    //     coeff_vec[1] = groundCoefficients->values[1];
    //     coeff_vec[2] = groundCoefficients->values[2];
    //     coeff_vec[3] = groundCoefficients->values[3];

    //     Eigen::Matrix4d HInv = H.inverse();  
    //     Eigen::Matrix4d HInvTrans = HInv.transpose(); 
    //     Eigen::Vector4d tfm_coeff_vec = HInvTrans * coeff_vec.cast<double>();

    //     tfm_groundCoefficients->values.resize(4);
    //     tfm_groundCoefficients->values[0] = tfm_coeff_vec[0];
    //     tfm_groundCoefficients->values[1] = tfm_coeff_vec[1];
    //     tfm_groundCoefficients->values[2] = tfm_coeff_vec[2];
    //     tfm_groundCoefficients->values[3] = tfm_coeff_vec[3];

    //     // cout << "Model coefficients: " << groundCoefficients->values[0] << " " 
    //     //                             << groundCoefficients->values[1] << " "
    //     //                             << groundCoefficients->values[2] << " " 
    //     //                             << groundCoefficients->values[3] << " " 
    //     //                             << -groundCoefficients->values[3]/groundCoefficients->values[2] << endl;


    //     tfm_groundCoefficients->header.seq = groundCloud->header.seq;
    //     tfm_groundCoefficients->header.stamp = groundCloud->header.stamp;
    //     tfm_groundCoefficients->header.frame_id = robot_frame_id_;

    //     pagslamIn.groundFeature.cloud = tfm_groundCloud;
    //     pagslamIn.groundFeature.cloud->header.frame_id = robot_frame_id_;

    //     pagslamIn.groundFeature.coefficients = tfm_groundCoefficients;  
    // }


    bool Extraction::stalkCloudExtraction(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud)
    {
        CloudT::Ptr stalkCloud_outlier(new CloudT());
        pcl::ModelCoefficients::Ptr outlierCoefficients (new pcl::ModelCoefficients);
        
        ransac(inCloud, stalkCloud_outlier, outCloud, outlierCoefficients);
        
        if (std::abs(outlierCoefficients->values[1]) < 0.5){ // If the outlier is not correctly filtered out
            return false;
        }
        else{
            // // Create a voxelgrid filter for downsampling
            // pcl::VoxelGrid<PointT> voxelGrid;
            // voxelGrid.setInputCloud(outCloud);        // Set the input point cloud
            // voxelGrid.setLeafSize(0.02f, 0.02f, 0.02f); // Set the voxel size
            // // voxelGrid.setLeafSize(0.04f, 0.04f, 0.04f); // Set the voxel size
            // voxelGrid.filter(*outCloud);                // Apply the filter

            stalkCloud_outlier.reset();
        }

        return true;
    }  

    // bool Extraction::stalkCloudExtraction(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud)
    // {
    //     CloudT::Ptr stalkCloud_outlier(new CloudT());
    //     pcl::ModelCoefficients::Ptr outlierCoefficients (new pcl::ModelCoefficients);
        
    //     ransac(inCloud, stalkCloud_outlier, outCloud, outlierCoefficients);
        
    //     if (std::abs(outlierCoefficients->values[1]) < 0.5){ // If the outlier is not correctly filtered out
    //         return false;
    //     }
    //     else{
    //         // // Create a voxelgrid filter for downsampling
    //         // pcl::VoxelGrid<PointT> voxelGrid;
    //         // voxelGrid.setInputCloud(outCloud);        // Set the input point cloud
    //         // voxelGrid.setLeafSize(0.02f, 0.02f, 0.02f); // Set the voxel size
    //         // // voxelGrid.setLeafSize(0.04f, 0.04f, 0.04f); // Set the voxel size
    //         // voxelGrid.filter(*outCloud);                // Apply the filter

    //         stalkCloud_outlier.reset();
    //     }

    //     return true;
    // }  

    void Extraction::filterByZ(CloudT::Ptr inCloud, CloudT::Ptr& filteredCloud, float lowerLimit, float upperLimit) {
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(inCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(lowerLimit, upperLimit); // only keep points with z-values between A and B
        // Use setFilterLimitsNegative(true) to keep points outside the range A to B and discard points inside it
        pass.filter(*filteredCloud);
    }

    void Extraction::printPointCloud(CloudT::Ptr cloud) {
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            cout << "Point " << i << ": x=" << cloud->points[i].x 
                                << " y=" << cloud->points[i].y 
                                << " z=" << cloud->points[i].z << endl;
        }
    }

    bool Extraction::stalkCloudClustersExtraction(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& outCloudClusters, pcl::ModelCoefficients::Ptr& groundCoefficients)
    {
        // input: tfm_v_cloud (CloudT::Ptr), groundCoefficients
        // output: outCloudClusters (std::vector<CloudT::Ptr>)

        // (1)-1 Filter tfm_v_cloud w.r.t. ground
        CloudT::Ptr filteredZInCloud(new CloudT);

        float ground_z_value = -groundCoefficients->values[3] / groundCoefficients->values[2];
        float minZLimit = ground_z_value + min_z_addition_;
        float maxZLimit = ground_z_value + max_z_addition_;

        // ROS_DEBUG_STREAM("MIN_MAX: " << ground_z_value << " " << minZLimit << " " << maxZLimit); 
        filterByZ(inCloud, filteredZInCloud, minZLimit, maxZLimit);
        // cout << "FILTERED: " << filteredZInCloud->size() << endl;
        // printPointCloud(filteredZInCloud);

        // (1)-2 Find initial seed points (DBSCAN)
        std::vector<pcl::PointIndices> cluster_indices;
        dbscan(filteredZInCloud, eps_, minDbscanPts_, cluster_indices);

        if (cluster_indices.empty()){
            return false;
        } 
        else {
            // Save the points corresponding to the indices of each cluster in an unordered_map
            std::unordered_map<int, CloudT::Ptr> clusters;
            int cluster_id = 0;
            
            for (const pcl::PointIndices& indices : cluster_indices) {
                if (clusters.count(cluster_id) == 0) {
                    clusters[cluster_id] = CloudT::Ptr(new CloudT);
                }
                for (int pit : indices.indices) {
                    clusters[cluster_id]->points.push_back(filteredZInCloud->points[pit]);
                }
                ++cluster_id;
            }

            // Copy the clusters from the unordered_map to the output vector
            for (const auto& cluster : clusters) {
                CloudT::Ptr cloud = cluster.second;
                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->is_dense = true;
                cloud->header = inCloud->header;
                outCloudClusters.push_back(cloud);

                // cout << clusters.size() << " " << cloud->width << " FILTERED: " << ground_z_value << endl;
            }
            // cout << clusters.size() << " FILTERED: " << ground_z_value << endl;
        }
        return true;

        // (1)-3 Find stalkCloud
        


        // (2)-1 Search seed points per each initial seed point
        // input: outCloudClusters (std::vector<CloudT::Ptr>)
        // output: seedClusters (std::vector<CloudT::Ptr>)

        /////////////////////////////////////////////////////////////
        // // Project point cloud onto XY plane
        // CloudT::Ptr tmpInCloud(new CloudT);
        // pcl::copyPointCloud(*inCloud, *tmpInCloud);
        // for (auto& point : tmpInCloud->points){
        //     point.y = 0;
        // }

        // // Perform DBSCAN
        // std::vector<pcl::PointIndices> cluster_indices;
        // dbscan(tmpInCloud, eps_, minDbscanPts_, cluster_indices);

        // if (cluster_indices.empty()){
        //     return false;
        // } 
        // else {
        //     // Save the points corresponding to the indices of each cluster in an unordered_map
        //     std::unordered_map<int, CloudT::Ptr> clusters;
        //     int cluster_id = 0;
            
        //     for (const pcl::PointIndices& indices : cluster_indices) {
        //         if (clusters.count(cluster_id) == 0) {
        //             clusters[cluster_id] = CloudT::Ptr(new CloudT);
        //         }
        //         for (int pit : indices.indices) {
        //             clusters[cluster_id]->points.push_back(inCloud->points[pit]);
        //         }
        //         ++cluster_id;
        //     }

        //     // Copy the clusters from the unordered_map to the output vector
        //     for (const auto& cluster : clusters) {
        //         CloudT::Ptr cloud = cluster.second;
        //         cloud->width = cloud->points.size();
        //         cloud->height = 1;
        //         cloud->is_dense = true;
        //         cloud->header = tmpInCloud->header;
        //         outCloudClusters.push_back(cloud);
        //     }
        // }
        // return true;
    }


    // bool Extraction::stalkCloudClustersExtraction(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& outCloudClusters)
    // {
    //     // Project point cloud onto XY plane
    //     CloudT::Ptr tmpInCloud(new CloudT);
    //     pcl::copyPointCloud(*inCloud, *tmpInCloud);
    //     for (auto& point : tmpInCloud->points){
    //         point.y = 0;
    //     }

    //     // Perform DBSCAN
    //     std::vector<pcl::PointIndices> cluster_indices;
    //     dbscan(tmpInCloud, eps_, minDbscanPts_, cluster_indices);

    //     if (cluster_indices.empty()){
    //         return false;
    //     } 
    //     else {
    //         // Save the points corresponding to the indices of each cluster in an unordered_map
    //         std::unordered_map<int, CloudT::Ptr> clusters;
    //         int cluster_id = 0;
            
    //         for (const pcl::PointIndices& indices : cluster_indices) {
    //             if (clusters.count(cluster_id) == 0) {
    //                 clusters[cluster_id] = CloudT::Ptr(new CloudT);
    //             }
    //             for (int pit : indices.indices) {
    //                 clusters[cluster_id]->points.push_back(inCloud->points[pit]);
    //             }
    //             ++cluster_id;
    //         }

    //         // Copy the clusters from the unordered_map to the output vector
    //         for (const auto& cluster : clusters) {
    //             CloudT::Ptr cloud = cluster.second;
    //             cloud->width = cloud->points.size();
    //             cloud->height = 1;
    //             cloud->is_dense = true;
    //             cloud->header = tmpInCloud->header;
    //             outCloudClusters.push_back(cloud);
    //         }
    //     }
    //     return true;
    // }


    bool Extraction::stalkSeedClustersExtraction(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters)
    {        
        std::vector<int> idx_vec;
        
        initialSeedPoint(inCloud, inCloudClusters, seedClusters, idx_vec);
        findGrowthDirection(inCloudClusters, seedClusters, idx_vec);

        if (!seedClusters.size()){
            return false;
        }

        return true;
    }


    void Extraction::initialSeedPoint(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec)
    {
        // int idx = 0;

        // pcl::PassThrough<PointT> pass;
        // pass.setFilterFieldName("z");
        // pass.setFilterLimitsNegative(false);
        // pass.setFilterLimits(-1.0, maxSeedZLimit_);

        // float toleranceR = 0.05;
        
        pcl::KdTreeFLANN<PointT> kdtree;
        // CloudT::Ptr inCloudCluster_filtered(new CloudT);

        for (int idx = 0; idx < inCloudClusters.size(); ++idx) {
            CloudT::Ptr inCloudCluster = inCloudClusters[idx];
            CloudT::Ptr seedCluster(new CloudT);

            // pass.setInputCloud(inCloudCluster);
            // pass.filter(*inCloudCluster_filtered);

            // if (!inCloudCluster_filtered->size()) {
            //     continue;
            // }

            kdtree.setInputCloud(inCloudCluster);

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*inCloudCluster, centroid);

            PointT searchPoint;
            searchPoint.x = centroid[0];
            searchPoint.y = centroid[1];
            searchPoint.z = centroid[2];

            int k = 1;      // Only need to find the single nearest point
            std::vector<int> pointIdxNKNSearch(k);
            std::vector<float> pointNKNSquaredDistance(k);
            kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);

            PointT nearestPoint = inCloudCluster->points[pointIdxNKNSearch[0]];
            seedCluster->push_back(nearestPoint);
            seedClusters.push_back(seedCluster);

            idx_vec.push_back(idx);


            // Find points with tolerances in x, y values from the initial seed points
            for (const auto& point : inCloud->points) {
                float deltaX = std::abs(point.x - nearestPoint.x);
                float deltaY = std::abs(point.y - nearestPoint.y);

                float deltaR = std::sqrt(deltaX*deltaX+ deltaY*deltaY);

                if (deltaR <= toleranceR_) {
                    inCloudCluster->push_back(point);
                }
            }
        }
    }

    // void Extraction::initialSeedPoint(std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec)
    // {
    //     // int idx = 0;

    //     pcl::PassThrough<PointT> pass;
    //     pass.setFilterFieldName("z");
    //     pass.setFilterLimitsNegative(false);
    //     pass.setFilterLimits(-1.0, maxSeedZLimit_);

    //     pcl::KdTreeFLANN<PointT> kdtree;
    //     CloudT::Ptr inCloudCluster_filtered(new CloudT);

    //     for (int idx = 0; idx < inCloudClusters.size(); ++idx) {
    //         CloudT::Ptr inCloudCluster = inCloudClusters[idx];
    //         CloudT::Ptr seedCluster(new CloudT);

    //         pass.setInputCloud(inCloudCluster);
    //         pass.filter(*inCloudCluster_filtered);

    //         if (!inCloudCluster_filtered->size()) {
    //             continue;
    //         }

    //         kdtree.setInputCloud(inCloudCluster_filtered);

    //         Eigen::Vector4f centroid;
    //         pcl::compute3DCentroid(*inCloudCluster_filtered, centroid);

    //         PointT searchPoint;
    //         searchPoint.x = centroid[0];
    //         searchPoint.y = centroid[1];
    //         searchPoint.z = centroid[2];

    //         int k = 1;      // Only need to find the single nearest point
    //         std::vector<int> pointIdxNKNSearch(k);
    //         std::vector<float> pointNKNSquaredDistance(k);
    //         kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance);

    //         PointT nearestPoint = inCloudCluster_filtered->points[pointIdxNKNSearch[0]];
    //         seedCluster->push_back(nearestPoint);
    //         seedClusters.push_back(seedCluster);

    //         idx_vec.push_back(idx);
    //     }
    // }

    // void Extraction::findGrowthDirection(std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec)
    // {
    //     pcl::PassThrough<PointT> pass;
    //     pass.setFilterFieldName("z");
    //     pass.setFilterLimitsNegative(false);

    //     pcl::KdTreeFLANN<PointT> kdtree;

    //     // cout << "Cluster #: " << seedClusters.size() << endl;

    //     for (int i = 0; i < idx_vec.size(); ++i) {
    //     // for (int i = 0; i < seedClusters.size(); ++i) {
    //         // Do something with each cloud in the vector
    //         CloudT::Ptr& seedCluster = seedClusters[i];
    //         CloudT::Ptr& CloudCluster = inCloudClusters[idx_vec[i]];

    //         seedCluster->header = CloudCluster->header;

    //         for(size_t k = 0; k < CloudCluster->points.size(); ++k){
    //             seedCluster->points.push_back(CloudCluster->points[k]);     
    //         }

    //         // Sort seedCluster points based on z values (ascending order)
    //         std::sort(seedCluster->points.begin(), seedCluster->points.end(), [](const PointT& a, const PointT& b) {
    //             return a.z < b.z;
    //         });
                        
    //         // CloudT::Ptr cloudWithinRadius(new CloudT);
    //         // CloudT::Ptr vecWithinRadius(new CloudT);
    //         // CloudT::Ptr vecWithinRadiusDiff(new CloudT);

    //         // std::vector<float> distances;

    //         // for (int j = 0; j < maxGrowthSearch_; ++j){
    //         //     CloudT::Ptr cloudCluster_candidate(new CloudT);
    
    //         //     if (seedCluster->points.size() <= j){
    //         //         break;
    //         //     }

    //         //     PointT lastSeed = seedCluster->points[j];

    //         //     PointT closestPoint;
    //         //     float minDistance = std::numeric_limits<float>::max();

    //         //     for(size_t k = 0; k < CloudCluster->points.size(); ++k)
    //         //     {
    //         //         PointT currentPoint = CloudCluster->points[k];

    //         //         float distance = pow(currentPoint.x - lastSeed.x, 2) +
    //         //                         pow(currentPoint.y - lastSeed.y, 2) +
    //         //                         pow(currentPoint.z - lastSeed.z, 2); // assuming these are 3D points

    //         //         if(distance < minDistance && currentPoint.z - lastSeed.z > 0)
    //         //         {
    //         //             minDistance = distance;
    //         //             closestPoint = currentPoint;
    //         //         }
    //         //     }



    //         //     float searchRadius = minDistance + offsetSearchRadius_;  // radius of the search sphere
    //         //     // float searchRadius = seedSearchRadius_;  // radius of the search sphere

    //         //     // pass.setFilterLimits(lastSeed.z+1e-4, 10.0);
    //         //     pass.setFilterLimits(lastSeed.z+1e-4, lastSeed.z+searchRadius);
    //         //     pass.setInputCloud(CloudCluster);
    //         //     pass.filter(*cloudCluster_candidate);

    //         //     if (!cloudCluster_candidate->size()) {
    //         //         break;
    //         //     }
                
    //         //     std::vector<int> pointIdxRadiusSearch;
    //         //     std::vector<float> pointRadiusSquaredDistance;
                
    //         //     kdtree.setInputCloud(cloudCluster_candidate);
    //         //     kdtree.radiusSearch(lastSeed, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                
    //         //     int nPoints = pointIdxRadiusSearch.size();

    //         //     if (!nPoints){;
    //         //         break;}
                
    //         //     cloudWithinRadius->resize(nPoints);
    //         //     vecWithinRadius->resize(nPoints);
    //         //     vecWithinRadiusDiff->resize(nPoints);
    //         //     distances.resize(nPoints);
               
    //         //     Eigen::Vector3f mean_vec = Eigen::Vector3f::Zero();

    //         //     for (size_t k = 0; k < nPoints; ++k) {
    //         //         const PointT& point = cloudCluster_candidate->points[pointIdxRadiusSearch[k]];
    //         //         distances[k] = pcl::geometry::distance(point, lastSeed);

    //         //         cloudWithinRadius->points[k].x = point.x;
    //         //         cloudWithinRadius->points[k].y = point.y;
    //         //         cloudWithinRadius->points[k].z = point.z;

    //         //         Eigen::Vector3f vec = (point.getVector3fMap() - lastSeed.getVector3fMap())/distances[k];
    //         //         vecWithinRadius->points[k].x = vec.x();
    //         //         vecWithinRadius->points[k].y = vec.y();
    //         //         vecWithinRadius->points[k].z = vec.z();

    //         //         // cout << vec.x() << " " << vec.y() << " " << vec.z() << endl;

    //         //         // mean_vec += vec;
    //         //         if (vec.z() > 0.7){
    //         //         // if (vec.z() > 0.9){

    //         //             // cout << "!!!" << vec.x() << " " << vec.y() << " " << vec.z() << endl;
    //         //             mean_vec += vec;
    //         //         }
    //         //         // else{
    //         //         //     cout << "???" << vec.x() << " " << vec.y() << " " << vec.z() << endl;
    //         //         // }
    //         //     }

    //         //     // cout << "============================" << endl;

    //         //     // // Copy the points within the radius to the new point cloud
    //         //     // for (size_t k = 0; k < nPoints; ++k) {  
    //         //     //     double distance = distances[k];
    //         //     //     const PointT& point = cloudCluster_candidate->points[pointIdxRadiusSearch[k]];
    //         //     //     cloudWithinRadius->points[k].x = point.x;
    //         //     //     cloudWithinRadius->points[k].y = point.y;
    //         //     //     cloudWithinRadius->points[k].z = point.z;

    //         //     //     Eigen::Vector3f vec = (point.getVector3fMap() - lastSeed.getVector3fMap())/distance;
    //         //     //     vecWithinRadius->points[k].x = vec.x();
    //         //     //     vecWithinRadius->points[k].y = vec.y();
    //         //     //     vecWithinRadius->points[k].z = vec.z();

    //         //     //     mean_vec += vec;
    //         //     // }

    //         //     float mean_vec_norm = mean_vec.norm();
    //         //     // cout << std::abs(mean_vec_norm) << endl;

    //         //     if (!std::abs(mean_vec_norm)){
    //         //         // cout << "NAN" << endl;
    //         //         continue;
    //         //     }
    //         //     else{
    //         //         mean_vec /= mean_vec_norm;

    //         //         // cout << mean_vec.norm() << " " << mean_vec.x() << " " << mean_vec.y() << " " << mean_vec.z() << endl;
    //         //         // cout << "====================================" << endl;
                    
    //         //         std::vector<float> diff_vec;

    //         //         for (size_t k = 0; k < nPoints; ++k) {
    //         //             vecWithinRadiusDiff->points[k].x =  vecWithinRadius->points[k].x - mean_vec[0];
    //         //             vecWithinRadiusDiff->points[k].y =  vecWithinRadius->points[k].y - mean_vec[1];
    //         //             vecWithinRadiusDiff->points[k].z =  vecWithinRadius->points[k].z - mean_vec[2];

    //         //             float norm = std::hypot(vecWithinRadiusDiff->points[k].x, vecWithinRadiusDiff->points[k].y);
    //         //             diff_vec.push_back(norm);
    //         //         }

    //         //         int n = 1;

    //         //         std::vector<float> sorted_vec(diff_vec);
    //         //         std::sort(sorted_vec.begin(), sorted_vec.end());
    //         //         for (size_t k = 0; k < nPoints; ++k) {
    //         //             // Find the nth minimum value and its index in the original vector
    //         //             double nth_min_value = sorted_vec[n-1];
    //         //             int nth_min_index = std::distance(diff_vec.begin(), std::find(diff_vec.begin(), diff_vec.end(), nth_min_value));
                        
    //         //             float vec_norm_xy = std::hypot(vecWithinRadius->points[nth_min_index].x, vecWithinRadius->points[nth_min_index].y);
    //         //             float tan_vec = vecWithinRadius->points[nth_min_index].z/vec_norm_xy;

    //         //             if(tan_vec < tan_vec_threshold_){                 
    //         //                 n++;
    //         //                 continue;
    //         //             }
    //         //             else{
    //         //                 seedCluster->points.push_back(cloudWithinRadius->points[nth_min_index]);
    //         //                 break;
    //         //             }
    //         //         }
    //         //     }

    //         //     // mean_vec /= mean_vec_norm;

    //         //     // cout << mean_vec.norm() << " " << mean_vec.x() << " " << mean_vec.y() << " " << mean_vec.z() << endl;
    //         //     // cout << "====================================" << endl;
                
    //         //     // std::vector<float> diff_vec;

    //         //     // for (size_t k = 0; k < nPoints; ++k) {
    //         //     //     vecWithinRadiusDiff->points[k].x =  vecWithinRadius->points[k].x - mean_vec[0];
    //         //     //     vecWithinRadiusDiff->points[k].y =  vecWithinRadius->points[k].y - mean_vec[1];
    //         //     //     vecWithinRadiusDiff->points[k].z =  vecWithinRadius->points[k].z - mean_vec[2];

    //         //     //     float norm = std::hypot(vecWithinRadiusDiff->points[k].x, vecWithinRadiusDiff->points[k].y);
    //         //     //     diff_vec.push_back(norm);
    //         //     // }

    //         //     // int n = 1;

    //         //     // std::vector<float> sorted_vec(diff_vec);
    //         //     // std::sort(sorted_vec.begin(), sorted_vec.end());
    //         //     // for (size_t k = 0; k < nPoints; ++k) {
    //         //     //     // Find the nth minimum value and its index in the original vector
    //         //     //     double nth_min_value = sorted_vec[n-1];
    //         //     //     int nth_min_index = std::distance(diff_vec.begin(), std::find(diff_vec.begin(), diff_vec.end(), nth_min_value));
                    
    //         //     //     float vec_norm_xy = std::hypot(vecWithinRadius->points[nth_min_index].x, vecWithinRadius->points[nth_min_index].y);
    //         //     //     float tan_vec = vecWithinRadius->points[nth_min_index].z/vec_norm_xy;

    //         //     //     if(tan_vec < tan_vec_threshold_){                 
    //         //     //         n++;
    //         //     //         continue;
    //         //     //     }
    //         //     //     else{
    //         //     //         seedCluster->points.push_back(cloudWithinRadius->points[nth_min_index]);
    //         //     //         break;
    //         //     //     }
    //         //     // }
    //         // }

    //         // cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
    //     }
    //     seedClusterFiltering(seedClusters);
    // }

    void Extraction::findGrowthDirection(std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec)
    {
        pcl::PassThrough<PointT> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimitsNegative(false);

        pcl::KdTreeFLANN<PointT> kdtree;

        // cout << "Cluster #: " << seedClusters.size() << endl;

        for (int i = 0; i < idx_vec.size(); ++i) {
        // for (int i = 0; i < seedClusters.size(); ++i) {
            // Do something with each cloud in the vector
            CloudT::Ptr& seedCluster = seedClusters[i];
            CloudT::Ptr& CloudCluster = inCloudClusters[idx_vec[i]];

            seedCluster->header = CloudCluster->header;
                        
            CloudT::Ptr cloudWithinRadius(new CloudT);
            CloudT::Ptr vecWithinRadius(new CloudT);
            CloudT::Ptr vecWithinRadiusDiff(new CloudT);

            std::vector<float> distances;

            for (int j = 0; j < maxGrowthSearch_; ++j){
                CloudT::Ptr cloudCluster_candidate(new CloudT);
    
                if (seedCluster->points.size() <= j){
                    break;
                }

                PointT lastSeed = seedCluster->points[j];

                PointT closestPoint;
                float minDistance = std::numeric_limits<float>::max();

                for(size_t k = 0; k < CloudCluster->points.size(); ++k)
                {
                    PointT currentPoint = CloudCluster->points[k];

                    float distance = pow(currentPoint.x - lastSeed.x, 2) +
                                    pow(currentPoint.y - lastSeed.y, 2) +
                                    pow(currentPoint.z - lastSeed.z, 2); // assuming these are 3D points

                    if(distance < minDistance && currentPoint.z - lastSeed.z > 0)
                    {
                        minDistance = distance;
                        closestPoint = currentPoint;
                    }
                }



                float searchRadius = minDistance + offsetSearchRadius_;  // radius of the search sphere
                // float searchRadius = seedSearchRadius_;  // radius of the search sphere

                // pass.setFilterLimits(lastSeed.z+1e-4, 10.0);
                pass.setFilterLimits(lastSeed.z+1e-4, lastSeed.z+searchRadius);
                pass.setInputCloud(CloudCluster);
                pass.filter(*cloudCluster_candidate);

                if (!cloudCluster_candidate->size()) {
                    break;
                }
                
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                
                kdtree.setInputCloud(cloudCluster_candidate);
                kdtree.radiusSearch(lastSeed, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                
                int nPoints = pointIdxRadiusSearch.size();

                if (!nPoints){;
                    break;}
                
                cloudWithinRadius->resize(nPoints);
                vecWithinRadius->resize(nPoints);
                vecWithinRadiusDiff->resize(nPoints);
                distances.resize(nPoints);
               
                Eigen::Vector3f mean_vec = Eigen::Vector3f::Zero();

                for (size_t k = 0; k < nPoints; ++k) {
                    const PointT& point = cloudCluster_candidate->points[pointIdxRadiusSearch[k]];
                    distances[k] = pcl::geometry::distance(point, lastSeed);

                    cloudWithinRadius->points[k].x = point.x;
                    cloudWithinRadius->points[k].y = point.y;
                    cloudWithinRadius->points[k].z = point.z;

                    Eigen::Vector3f vec = (point.getVector3fMap() - lastSeed.getVector3fMap())/distances[k];
                    vecWithinRadius->points[k].x = vec.x();
                    vecWithinRadius->points[k].y = vec.y();
                    vecWithinRadius->points[k].z = vec.z();

                    // cout << vec.x() << " " << vec.y() << " " << vec.z() << endl;

                    // mean_vec += vec;
                    if (vec.z() > 0.9){
                    // if (vec.z() > 0.9){

                        // cout << "!!!" << vec.x() << " " << vec.y() << " " << vec.z() << endl;
                        mean_vec += vec;
                    }
                    // else{
                    //     cout << "???" << vec.x() << " " << vec.y() << " " << vec.z() << endl;
                    // }
                }

                // cout << "============================" << endl;

                // // Copy the points within the radius to the new point cloud
                // for (size_t k = 0; k < nPoints; ++k) {  
                //     double distance = distances[k];
                //     const PointT& point = cloudCluster_candidate->points[pointIdxRadiusSearch[k]];
                //     cloudWithinRadius->points[k].x = point.x;
                //     cloudWithinRadius->points[k].y = point.y;
                //     cloudWithinRadius->points[k].z = point.z;

                //     Eigen::Vector3f vec = (point.getVector3fMap() - lastSeed.getVector3fMap())/distance;
                //     vecWithinRadius->points[k].x = vec.x();
                //     vecWithinRadius->points[k].y = vec.y();
                //     vecWithinRadius->points[k].z = vec.z();

                //     mean_vec += vec;
                // }

                float mean_vec_norm = mean_vec.norm();
                // cout << std::abs(mean_vec_norm) << endl;

                if (!std::abs(mean_vec_norm)){
                    // cout << "NAN" << endl;
                    continue;
                }
                else{
                    mean_vec /= mean_vec_norm;

                    // cout << mean_vec.norm() << " " << mean_vec.x() << " " << mean_vec.y() << " " << mean_vec.z() << endl;
                    // cout << "====================================" << endl;
                    
                    std::vector<float> diff_vec;

                    for (size_t k = 0; k < nPoints; ++k) {
                        vecWithinRadiusDiff->points[k].x =  vecWithinRadius->points[k].x - mean_vec[0];
                        vecWithinRadiusDiff->points[k].y =  vecWithinRadius->points[k].y - mean_vec[1];
                        vecWithinRadiusDiff->points[k].z =  vecWithinRadius->points[k].z - mean_vec[2];

                        float norm = std::hypot(vecWithinRadiusDiff->points[k].x, vecWithinRadiusDiff->points[k].y);
                        diff_vec.push_back(norm);
                    }

                    int n = 1;

                    std::vector<float> sorted_vec(diff_vec);
                    std::sort(sorted_vec.begin(), sorted_vec.end());
                    for (size_t k = 0; k < nPoints; ++k) {
                        // Find the nth minimum value and its index in the original vector
                        double nth_min_value = sorted_vec[n-1];
                        int nth_min_index = std::distance(diff_vec.begin(), std::find(diff_vec.begin(), diff_vec.end(), nth_min_value));
                        
                        float vec_norm_xy = std::hypot(vecWithinRadius->points[nth_min_index].x, vecWithinRadius->points[nth_min_index].y);
                        float tan_vec = vecWithinRadius->points[nth_min_index].z/vec_norm_xy;

                        if(tan_vec < tan_vec_threshold_){                 
                            n++;
                            continue;
                        }
                        else{
                            seedCluster->points.push_back(cloudWithinRadius->points[nth_min_index]);
                            break;
                        }
                    }
                }

                // mean_vec /= mean_vec_norm;

                // cout << mean_vec.norm() << " " << mean_vec.x() << " " << mean_vec.y() << " " << mean_vec.z() << endl;
                // cout << "====================================" << endl;
                
                // std::vector<float> diff_vec;

                // for (size_t k = 0; k < nPoints; ++k) {
                //     vecWithinRadiusDiff->points[k].x =  vecWithinRadius->points[k].x - mean_vec[0];
                //     vecWithinRadiusDiff->points[k].y =  vecWithinRadius->points[k].y - mean_vec[1];
                //     vecWithinRadiusDiff->points[k].z =  vecWithinRadius->points[k].z - mean_vec[2];

                //     float norm = std::hypot(vecWithinRadiusDiff->points[k].x, vecWithinRadiusDiff->points[k].y);
                //     diff_vec.push_back(norm);
                // }

                // int n = 1;

                // std::vector<float> sorted_vec(diff_vec);
                // std::sort(sorted_vec.begin(), sorted_vec.end());
                // for (size_t k = 0; k < nPoints; ++k) {
                //     // Find the nth minimum value and its index in the original vector
                //     double nth_min_value = sorted_vec[n-1];
                //     int nth_min_index = std::distance(diff_vec.begin(), std::find(diff_vec.begin(), diff_vec.end(), nth_min_value));
                    
                //     float vec_norm_xy = std::hypot(vecWithinRadius->points[nth_min_index].x, vecWithinRadius->points[nth_min_index].y);
                //     float tan_vec = vecWithinRadius->points[nth_min_index].z/vec_norm_xy;

                //     if(tan_vec < tan_vec_threshold_){                 
                //         n++;
                //         continue;
                //     }
                //     else{
                //         seedCluster->points.push_back(cloudWithinRadius->points[nth_min_index]);
                //         break;
                //     }
                // }
            }

            // Sort seedCluster points based on z values (ascending order)
            std::sort(seedCluster->points.begin(), seedCluster->points.end(), [](const PointT& a, const PointT& b) {
                return a.z < b.z;
            });
            
            // cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
        }

        
        seedClusterFiltering(seedClusters);
    }

    // void Extraction::findGrowthDirection(std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec)
    // {
    //     pcl::PassThrough<PointT> pass;
    //     pass.setFilterFieldName("z");
    //     pass.setFilterLimitsNegative(false);

    //     pcl::KdTreeFLANN<PointT> kdtree;

    //     // cout << "Cluster #: " << seedClusters.size() << endl;

    //     for (int i = 0; i < idx_vec.size(); ++i) {
    //     // for (int i = 0; i < seedClusters.size(); ++i) {
    //         // Do something with each cloud in the vector
    //         CloudT::Ptr& seedCluster = seedClusters[i];
    //         CloudT::Ptr& CloudCluster = inCloudClusters[idx_vec[i]];

    //         seedCluster->header = CloudCluster->header;
                        
    //         CloudT::Ptr cloudWithinRadius(new CloudT);
    //         CloudT::Ptr vecWithinRadius(new CloudT);
    //         CloudT::Ptr vecWithinRadiusDiff(new CloudT);

    //         std::vector<float> distances;

    //         for (int j = 0; j < maxGrowthSearch_; ++j){
    //             CloudT::Ptr cloudCluster_candidate(new CloudT);
    
    //             if (seedCluster->points.size() <= j){
    //                 break;
    //             }

    //             PointT lastSeed = seedCluster->points[j];
    //             float searchRadius = seedSearchRadius_;  // radius of the search sphere

    //             // pass.setFilterLimits(lastSeed.z+1e-4, 10.0);
    //             pass.setFilterLimits(lastSeed.z+1e-4, lastSeed.z+searchRadius);
    //             pass.setInputCloud(CloudCluster);
    //             pass.filter(*cloudCluster_candidate);

    //             if (!cloudCluster_candidate->size()) {
    //                 break;
    //             }
                
    //             std::vector<int> pointIdxRadiusSearch;
    //             std::vector<float> pointRadiusSquaredDistance;
                
    //             kdtree.setInputCloud(cloudCluster_candidate);
    //             kdtree.radiusSearch(lastSeed, searchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
                
    //             int nPoints = pointIdxRadiusSearch.size();

    //             if (!nPoints){;
    //                 break;}
                
    //             cloudWithinRadius->resize(nPoints);
    //             vecWithinRadius->resize(nPoints);
    //             vecWithinRadiusDiff->resize(nPoints);
    //             distances.resize(nPoints);
               
    //             Eigen::Vector3f mean_vec = Eigen::Vector3f::Zero();

    //             for (size_t k = 0; k < nPoints; ++k) {
    //                 const PointT& point = cloudCluster_candidate->points[pointIdxRadiusSearch[k]];
    //                 distances[k] = pcl::geometry::distance(point, lastSeed);

    //                 cloudWithinRadius->points[k].x = point.x;
    //                 cloudWithinRadius->points[k].y = point.y;
    //                 cloudWithinRadius->points[k].z = point.z;

    //                 Eigen::Vector3f vec = (point.getVector3fMap() - lastSeed.getVector3fMap())/distances[k];
    //                 vecWithinRadius->points[k].x = vec.x();
    //                 vecWithinRadius->points[k].y = vec.y();
    //                 vecWithinRadius->points[k].z = vec.z();

    //                 // cout << vec.x() << " " << vec.y() << " " << vec.z() << endl;

    //                 if (vec.z() > 0.9){

    //                     // cout << "!!!" << vec.x() << " " << vec.y() << " " << vec.z() << endl;
    //                     mean_vec += vec;
    //                 }
    //                 // else{
    //                 //     cout << "???" << vec.x() << " " << vec.y() << " " << vec.z() << endl;
    //                 // }
    //             }

    //             // cout << "============================" << endl;

    //             // // Copy the points within the radius to the new point cloud
    //             // for (size_t k = 0; k < nPoints; ++k) {  
    //             //     double distance = distances[k];
    //             //     const PointT& point = cloudCluster_candidate->points[pointIdxRadiusSearch[k]];
    //             //     cloudWithinRadius->points[k].x = point.x;
    //             //     cloudWithinRadius->points[k].y = point.y;
    //             //     cloudWithinRadius->points[k].z = point.z;

    //             //     Eigen::Vector3f vec = (point.getVector3fMap() - lastSeed.getVector3fMap())/distance;
    //             //     vecWithinRadius->points[k].x = vec.x();
    //             //     vecWithinRadius->points[k].y = vec.y();
    //             //     vecWithinRadius->points[k].z = vec.z();

    //             //     mean_vec += vec;
    //             // }

    //             float mean_vec_norm = mean_vec.norm();
    //             // cout << std::abs(mean_vec_norm) << endl;

    //             if (!std::abs(mean_vec_norm)){
    //                 // cout << "NAN" << endl;
    //                 continue;
    //             }
    //             else{
    //                 mean_vec /= mean_vec_norm;

    //                 // cout << mean_vec.norm() << " " << mean_vec.x() << " " << mean_vec.y() << " " << mean_vec.z() << endl;
    //                 // cout << "====================================" << endl;
                    
    //                 std::vector<float> diff_vec;

    //                 for (size_t k = 0; k < nPoints; ++k) {
    //                     vecWithinRadiusDiff->points[k].x =  vecWithinRadius->points[k].x - mean_vec[0];
    //                     vecWithinRadiusDiff->points[k].y =  vecWithinRadius->points[k].y - mean_vec[1];
    //                     vecWithinRadiusDiff->points[k].z =  vecWithinRadius->points[k].z - mean_vec[2];

    //                     float norm = std::hypot(vecWithinRadiusDiff->points[k].x, vecWithinRadiusDiff->points[k].y);
    //                     diff_vec.push_back(norm);
    //                 }

    //                 int n = 1;

    //                 std::vector<float> sorted_vec(diff_vec);
    //                 std::sort(sorted_vec.begin(), sorted_vec.end());
    //                 for (size_t k = 0; k < nPoints; ++k) {
    //                     // Find the nth minimum value and its index in the original vector
    //                     double nth_min_value = sorted_vec[n-1];
    //                     int nth_min_index = std::distance(diff_vec.begin(), std::find(diff_vec.begin(), diff_vec.end(), nth_min_value));
                        
    //                     float vec_norm_xy = std::hypot(vecWithinRadius->points[nth_min_index].x, vecWithinRadius->points[nth_min_index].y);
    //                     float tan_vec = vecWithinRadius->points[nth_min_index].z/vec_norm_xy;

    //                     if(tan_vec < tan_vec_threshold_){                 
    //                         n++;
    //                         continue;
    //                     }
    //                     else{
    //                         seedCluster->points.push_back(cloudWithinRadius->points[nth_min_index]);
    //                         break;
    //                     }
    //                 }
    //             }

    //             // mean_vec /= mean_vec_norm;

    //             // cout << mean_vec.norm() << " " << mean_vec.x() << " " << mean_vec.y() << " " << mean_vec.z() << endl;
    //             // cout << "====================================" << endl;
                
    //             // std::vector<float> diff_vec;

    //             // for (size_t k = 0; k < nPoints; ++k) {
    //             //     vecWithinRadiusDiff->points[k].x =  vecWithinRadius->points[k].x - mean_vec[0];
    //             //     vecWithinRadiusDiff->points[k].y =  vecWithinRadius->points[k].y - mean_vec[1];
    //             //     vecWithinRadiusDiff->points[k].z =  vecWithinRadius->points[k].z - mean_vec[2];

    //             //     float norm = std::hypot(vecWithinRadiusDiff->points[k].x, vecWithinRadiusDiff->points[k].y);
    //             //     diff_vec.push_back(norm);
    //             // }

    //             // int n = 1;

    //             // std::vector<float> sorted_vec(diff_vec);
    //             // std::sort(sorted_vec.begin(), sorted_vec.end());
    //             // for (size_t k = 0; k < nPoints; ++k) {
    //             //     // Find the nth minimum value and its index in the original vector
    //             //     double nth_min_value = sorted_vec[n-1];
    //             //     int nth_min_index = std::distance(diff_vec.begin(), std::find(diff_vec.begin(), diff_vec.end(), nth_min_value));
                    
    //             //     float vec_norm_xy = std::hypot(vecWithinRadius->points[nth_min_index].x, vecWithinRadius->points[nth_min_index].y);
    //             //     float tan_vec = vecWithinRadius->points[nth_min_index].z/vec_norm_xy;

    //             //     if(tan_vec < tan_vec_threshold_){                 
    //             //         n++;
    //             //         continue;
    //             //     }
    //             //     else{
    //             //         seedCluster->points.push_back(cloudWithinRadius->points[nth_min_index]);
    //             //         break;
    //             //     }
    //             // }
    //         }

    //         // cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
    //     }
    //     seedClusterFiltering(seedClusters);
    // }

    void Extraction::seedClusterFiltering(std::vector<CloudT::Ptr>& seedClusters)
    {
        // Indices of elements to remove
        int num_to_keep = 0;
        for (size_t i = 0; i < seedClusters.size(); ++i){
            int num_points = seedClusters[i]->points.size();

            if (num_points >= minSeedPts_){
                seedClusters[num_to_keep++] = seedClusters[i];
                // cout << i << " " << seedClusters[i]->points.back().z << endl;
                // if (seedClusters[i]->points.back().z > 0.5){
                //     seedClusters[num_to_keep++] = seedClusters[i];
                // }
            }
        }

        // cout << "Cluster #: " << num_to_keep << endl;
        seedClusters.resize(num_to_keep);
    }


    void Extraction::representativeLine(std::vector<CloudT::Ptr>& seedClusters, std::vector<StalkFeature::Ptr>& stalkFeatures)
    {
        for (const auto& seedCluster : seedClusters) {
            int num_points = seedCluster->points.size();

            // Compute the centroid of the point cloud
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*seedCluster, centroid);

            // Compute the eigenvalues and eigenvectors of the covariance matrix
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*seedCluster, centroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

            // Get the direction of the smallest eigenvalue, which is the line direction
            // Eigen::Vector3f line_direction = solver.eigenvectors().col(2);
            // if (line_direction(2) < 0) {
            //     line_direction *= -1;
            // }
                
            // Compute the point closest to the origin on the line
            Eigen::Vector3f line_origin = centroid.head<3>();
            Eigen::Vector3f line_root (seedCluster->points[0].x, seedCluster->points[0].y, seedCluster->points[0].z);
            Eigen::Vector3f line_top (seedCluster->points[num_points-1].x, seedCluster->points[num_points-1].y, seedCluster->points[num_points-1].z);
            
            Eigen::Vector3f line_direction = line_top - line_root;

            // Construct the StalkFeature object and push it back to the output vector
            StalkFeature::Ptr feature(new StalkFeature{seedCluster->header, *seedCluster, line_root, line_top, line_origin, line_direction.transpose()});
            stalkFeatures.push_back(feature);
        }
    }


    void Extraction::transformStalkCloud(tf2::Transform tf, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& tfm_inCloudClusters)
    {
        geometry_msgs::Transform transform;
        tf2::convert(tf, transform);
        tfm_inCloudClusters = inCloudClusters;

        for (int i = 0; i < inCloudClusters.size(); i++) {
            pcl_ros::transformPointCloud(*inCloudClusters[i], *tfm_inCloudClusters[i], transform);
            tfm_inCloudClusters[i]->header.frame_id = robot_frame_id_;
        }
    }


    void Extraction::dbscan(CloudT::Ptr inCloud, double eps, int minDbscanPts, std::vector<pcl::PointIndices>& cluster_indices)
    {       
        // Set up KD-tree
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud(inCloud);

        // Create the search pointer and cast the KD-tree object to it
        pcl::search::Search<PointT>::Ptr search = boost::dynamic_pointer_cast<pcl::search::Search<PointT>>(boost::make_shared<pcl::KdTreeFLANN<PointT>>(kdtree));

        // Set up clustering object
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(minDbscanPts);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(search);
        ec.setInputCloud(inCloud);

        // Perform clustering
        ec.extract(cluster_indices);
    }
}
