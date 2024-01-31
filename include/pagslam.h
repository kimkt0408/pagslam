#pragma once

#include <ros/ros.h>
#include <definitions.h>

// scene: the set of points to be aligned.
// model: teh reference to which the scene has to be aligned.
template <typename T>
struct FeatureMatch{
    T sceneFeature;
    T modelFeature;
    double dist;
};

/*
 * --------------- odom/map inputs and outputs ------------------
 */
struct PagslamInput
{
    PagslamInput()
    {
        groundFeature.cloud = CloudT::Ptr(new CloudT());
        groundFeature.coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
    };

    SE3 poseEstimate;
    Scalar distance;
    GroundFeature groundFeature;
    std::vector<StalkFeature::Ptr> stalkFeatures = {};
    std::vector<StalkFeature::Ptr> mapStalkFeatures = {};

    // // Copy constructor
    PagslamInput(const PagslamInput& other)
    {
        poseEstimate = other.poseEstimate;
        distance = other.distance;

        // Create new instances of CloudT and pcl::ModelCoefficients
        groundFeature.cloud = CloudT::Ptr(new CloudT());
        groundFeature.coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

        // Copy the data from in.groundFeature into the new instances
        *(groundFeature.cloud) = *(other.groundFeature.cloud);
        *(groundFeature.coefficients) = *(other.groundFeature.coefficients);

        // Copy the stalkFeatures vector
        stalkFeatures.clear();
        for (const auto& feature : other.stalkFeatures)
            stalkFeatures.push_back(std::make_shared<StalkFeature>(*feature));

        // Copy the mapStalkFeatures vector
        mapStalkFeatures.clear();
        for (const auto& feature : other.mapStalkFeatures)
            mapStalkFeatures.push_back(std::make_shared<StalkFeature>(*feature));
    }
};

struct PagslamOutput
{
    PagslamOutput(){};
    std::vector<int> matches;

    SE3 T_Map_Curr;
    SE3 T_Delta;
    GroundFeature ground;
    std::vector<StalkFeature::Ptr> stalks;
};

namespace pagslam
{
    class pagslam
    {
        public:
            // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
            explicit pagslam();

            // std::vector<GroundFeature> getPrevGroundModel();

            void projectFeatures(const SE3 &tf, GroundFeature &ground, std::vector<StalkFeature::Ptr> &stalks);
            void projectGround(const SE3 &tf, GroundFeature &ground);
            // void projectStalk(const SE3 &tf, StalkFeature::Ptr &stalk);
            void projectStalk(const SE3 &tf, StalkFeature::Ptr &stalk, bool firstStalk);
            void matchFeatures(const std::vector<StalkFeature::Ptr> &stalkFeatures, const std::vector<StalkFeature::Ptr> &mapStalkFeatures, std::vector<int> &matchIndices);

            bool TwoStepOptimizePose(const PagslamInput &in_proj, const bool stalkCheck, const std::vector<FeatureMatch<StalkFeature::Ptr>> &stalkMatches, SE3 &tf);
            void OptimizeXYYaw(const SE3& poseEstimate, const bool optimize, const std::vector<FeatureMatch<StalkFeature::Ptr>> &stalkMatches, double* out);
            void OptimizeZRollPitch(const SE3& poseEstimate, const GroundFeature &currFeature, std::vector<GroundFeature> &mapFeatures, double* out);
            void printCostCallback(const ceres::IterationSummary& summary);

            bool runPagslam(PagslamInput &in, PagslamOutput &out);
            
            std::vector<FeatureMatch<StalkFeature::Ptr>> matchStalks(const SE3 &tf, const std::vector<StalkFeature::Ptr> &currFeatures, std::vector<StalkFeature::Ptr> &mapFeatures, const Scalar distThresh);
            
            Scalar stalkDistance(Eigen::Vector3f v1, Eigen::Vector3f v2);
            Scalar groundDistance(GroundFeature g1, GroundFeature g2);

            template <typename T>
            void addFeatureMatches(const T &currFeature, const T &mapFeature, const double dist, std::vector<FeatureMatch<T>> &matches);
  
            std::string map_frame_id_;
            std::string robot_frame_id_;
            std::string h_lidar_frame_id_;
            std::string v_lidar_frame_id_;

        private:
            SE3 T_Map_Anchor_; // Pose of ANCHOR frame in the Map frame
            bool firstScan_;
            bool bool_groundMatch_;
            bool bool_dualLiDAR_;

            std::vector<GroundFeature> prevGroundFeatures_;
            Scalar stalkMatchThresh_;
            Scalar rangeGroundMatch_;
            Scalar AddNewStalkThreshDist_;
            int minStalkMatches_;
            int maxNumIterations_;
            Scalar huberLossThresh_;
    };
} // namespace pagslam

