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
    //   tan_vec_threshold_(0.1),
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


    // For Range-view h_cloud
    //   seedSearchRadius_(0.12),
    //   maxSeedZLimit_(0.5),
      minSeedPts_(6),
      nIterations_(100),
      minInliers_(30),      // new-ACRE-long
    //   toleranceR_(0.03),
    //   min_z_addition_(0.10),
    //   max_z_addition_(0.16),
      offsetSearchRadius_(0.05),
      tan_vec_threshold_(0.3),
      ransacDistanceThreshold_(1.0)
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
    
    void Extraction::ransac(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud_inlier, CloudT::Ptr& outCloud_outlier, pcl::ModelCoefficients::Ptr& groundCoefficients)
    {
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
        seg.setInputCloud (inCloud);

        // seg.setDistanceThreshold (ransacDistanceThreshold_);
        // Set the distance threshold only once
        if (seg.getDistanceThreshold() != ransacDistanceThreshold_) {
            seg.setDistanceThreshold (ransacDistanceThreshold_);
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
                extract.setInputCloud(inCloud);
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
        outCloud_outlier = inCloud;
        groundCoefficients.reset(new pcl::ModelCoefficients);
        ROS_WARN("No valid segmentation found in %d iterations", nIterations_);
    }
   

    void Extraction::transformGroundPlane(const tf2::Transform tf, const CloudT::Ptr& groundCloud, pcl::ModelCoefficients::Ptr& groundCoefficients, PagslamInput &pagslamIn)
    {
        CloudT::Ptr tfm_groundCloud(new CloudT());
        pcl::ModelCoefficients::Ptr tfm_groundCoefficients (new pcl::ModelCoefficients);
        
        geometry_msgs::Transform transform;
        tf2::convert(tf, transform);

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

            PointT lowestPoint;
            lowestPoint.z = std::numeric_limits<float>::max();

            for (const auto& point : inCloudCluster->points) {
                if (point.z < lowestPoint.z) {
                    lowestPoint = point;
                }
            }
        
            seedCluster->push_back(lowestPoint);
            seedClusters.push_back(seedCluster);

            idx_vec.push_back(idx);
        }
    }


    void Extraction::findGrowthDirection(std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec)
    {
        pcl::PassThrough<PointT> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimitsNegative(false);

        pcl::KdTreeFLANN<PointT> kdtree;

        for (int i = 0; i < idx_vec.size(); ++i) {
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

                    if (vec.z() > 0.9){
                        mean_vec += vec;
                    }
                }

                float mean_vec_norm = mean_vec.norm();

                if (!std::abs(mean_vec_norm)){
                    continue;
                }
                else{
                    mean_vec /= mean_vec_norm;
                    
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
            }

            // Sort seedCluster points based on z values (ascending order)
            std::sort(seedCluster->points.begin(), seedCluster->points.end(), [](const PointT& a, const PointT& b) {
                return a.z < b.z;
            });
        }

        
        seedClusterFiltering(seedClusters);
    }


    void Extraction::seedClusterFiltering(std::vector<CloudT::Ptr>& seedClusters)
    {
        // Indices of elements to remove
        int num_to_keep = 0;
        for (size_t i = 0; i < seedClusters.size(); ++i){
            int num_points = seedClusters[i]->points.size();

            if (num_points >= minSeedPts_){
                seedClusters[num_to_keep++] = seedClusters[i];
            }
        }

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
