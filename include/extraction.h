#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>

#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/eigen.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <list>

#include <definitions.h>
#include <pagslam.h>
#include <omp.h>
#include <thread>


namespace ext
{
    class Extraction
    {
        public: 
            explicit Extraction(const int ransacMaxIterations, const std::string robot_frame_id, const double eps, const int minPts);
            
            Extraction(const Extraction &) = delete;
            Extraction operator=(const Extraction &) = delete;

            void ransac(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud_inlier, CloudT::Ptr& outCloud_outlier, pcl::ModelCoefficients::Ptr& groundCoefficients);
            void transformGroundPlane(const tf2::Transform tf, const CloudT::Ptr& groundCloud, pcl::ModelCoefficients::Ptr& groundCoefficients, PagslamInput &pagslamIn);
        
            bool stalkCloudExtraction(const CloudT::Ptr inCloud, CloudT::Ptr& outCloud);
            // bool stalkCloudClustersExtraction(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& outCloudClusters);
            bool stalkCloudClustersExtraction(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& outCloudClusters, pcl::ModelCoefficients::Ptr& groundCoefficients);
            void filterByZ(CloudT::Ptr inCloud, CloudT::Ptr& filteredCloud, float lowerLimit, float upperLimit);
            void printPointCloud(CloudT::Ptr cloud); 

            bool stalkSeedClustersExtraction(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters);
            void initialSeedPoint(CloudT::Ptr inCloud, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec);
            void findGrowthDirection(std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& seedClusters, std::vector<int>& idx_vec);
            void seedClusterFiltering(std::vector<CloudT::Ptr>& seedClusters);
            void representativeLine(std::vector<CloudT::Ptr>& seedClusters, std::vector<StalkFeature::Ptr>& stalkFeatures);

            void transformStalkCloud(tf2::Transform tf, std::vector<CloudT::Ptr>& inCloudClusters, std::vector<CloudT::Ptr>& tfm_inCloudClusters);
                        
            void dbscan(pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud, double eps, int minPts, std::vector<pcl::PointIndices>& cluster_indices);

            // void downsamplePointCloud(CloudT::Ptr cloud, float leafSize);
    
        private:
            int ransacMaxIterations_;       // Max iteration for ground feature extraction
            std::string robot_frame_id_;
            double eps_;                    // maximum distance between points in a cluster
            int minDbscanPts_;              // minimum number of points in a cluster
            float maxSeedZLimit_;
            int maxGrowthSearch_;
            float seedSearchRadius_;
            int minSeedPts_;

            int nIterations_;
            int minInliers_;
            float tan_vec_threshold_;
            
            float toleranceR_;
            float min_z_addition_;
            float max_z_addition_;
            float offsetSearchRadius_;
    };
}
