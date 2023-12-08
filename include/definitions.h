#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/multi_array.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <map>
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>

#define PIDEF 3.14159265

using namespace std;
using namespace Eigen;
using namespace boost;

using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;
using Matrix3 = Matrix3d;
using Matrix4 = Matrix4d;
using MatrixX = MatrixXd;
using VectorX = VectorXd;
using Vector4 = Vector4d;
using Vector3 = Vector3d;
using Affine3 = Affine3d;
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using KDTree = pcl::KdTreeFLANN<PointT>;
using Quat = Quaterniond;
using Tran = Translation3d;

typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> VectorType;
typedef std::vector<SE3, Eigen::aligned_allocator<SE3>> SE3VectorType;typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> VectorType;

using Slash = VectorType;
using Scalar = double;


/*
 * --------------- Feature Structures ------------------
 */

struct StampedSE3
{
    StampedSE3(SE3 p, ros::Time s) : pose(p), stamp(s){};
    StampedSE3() : pose(SE3()), stamp(ros::Time::now()){};
    SE3 pose;
    ros::Time stamp;
};

struct GroundFeature{
    CloudT::Ptr cloud;
    pcl::ModelCoefficients::Ptr coefficients;
};

struct StalkFeature{
    using Ptr = std::shared_ptr<StalkFeature>;

    pcl::PCLHeader header;

    CloudT cloud;

    Eigen::Vector3f root;       // root (x,y,z) on the bottom of the stalk
    Eigen::Vector3f top;       // top of the stalk
    
    Eigen::Vector3f centroid;   // centroid (x,y,z) that lies on line
    Eigen::Vector3f direction;  // ray (direction)
};


// // scene points: the set of points to be aligned.
// // model points: the reference to which the scene has to be aligned.
// struct XYYawLineCost {
//     Vector3 point;
//     Vector3 root;
//     Vector3 direction;
//     double weight;

//     XYYawLineCost(Vector3 point, Vector3 root, Vector3 direction, double weight)
//         : point(point), root(root), direction(direction), weight(weight) {}

//     template <typename T>
//     bool operator()(const T* const params, T* residuals) const {

//         // n: normal of cylinder (x,y,z)
//         Eigen::Matrix<T, 2, 1> source_point{T(point[0]), T(point[1])};

//         Eigen::Matrix<T, 2, 1> model_root{T(root[0]), T(root[1])};
//         Eigen::Matrix<T, 2, 1> model_direction{T(direction[0]), T(direction[1])};

//         // last point
//         // transform point using current solution
//         Eigen::Matrix<T, 2, 1> last_point;
//         T r_last_curr[3] = {params[3], params[4], params[5]};
//         Eigen::Matrix<T, 2, 1> t_last_curr{params[0], params[1]};

//         ceres::AngleAxisRotatePoint(r_last_curr, source_point.data(), last_point.data());
//         last_point += t_last_curr;

//         // // projected point onto cylinder
//         // Eigen::Matrix<T, 2, 1> projected_point =
//         //     model_root + ((last_point - model_root).dot(model_direction) / (model_direction.dot(model_direction))) * model_direction;

//         // T Distance = (projected_point - last_point).norm();
//         T Distance = (model_root - last_point).norm();
//         residuals[0] = Distance * T(weight);
        
//         return true;
//     }
// };

// scene points: the set of points to be aligned.
// model points: the reference to which the scene has to be aligned.
// struct XYYawLineCost {
//     Vector3 point;
//     Vector3 root;
//     Vector3 direction;
//     double weight;

//     // int m = 0;
//     XYYawLineCost(Vector3 point, Vector3 root, Vector3 direction, double weight)
//         : point(point), root(root), direction(direction), weight(weight) {}

//     template <typename T>
//     bool operator()(const T* const params, T* residuals) const {

//         // n: normal of cylinder (x,y,z)
//         Eigen::Matrix<T, 3, 1> source_point{T(point[0]), T(point[1]), T(point[2])};

//         Eigen::Matrix<T, 3, 1> model_root{T(root[0]), T(root[1]), T(root[2])};
//         Eigen::Matrix<T, 3, 1> model_direction{T(direction[0]), T(direction[1]), T(direction[2])};

//         // last point
//         // transform point using current solution
//         Eigen::Matrix<T, 3, 1> last_point;
//         T r_last_curr[3] = {params[3], params[4], params[5]};
//         Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

//         // cout << "LAST POINT: " << last_point << endl;
//         ceres::AngleAxisRotatePoint(r_last_curr, source_point.data(), last_point.data());
//         last_point += t_last_curr;

//         // cout << "LAST POINT: " << static_cast<Eigen::Matrix<T, 3, 1>>(last_point) << endl;
//         // cout << "MODEL ROOT: " << *model_root.data() << " " << model_root << endl;

//         // projected point onto cylinder
//         Eigen::Matrix<T, 3, 1> projected_point =
//             model_root + ((last_point - model_root).dot(model_direction) / sqrt((model_direction.dot(model_direction)))) * model_direction;

//         // cout << model_root << "%\n" << last_point << "%\n" << projected_point << endl;
//         T Distance = (projected_point - last_point).norm();

//         // residuals[0] = Distance * T(weight);
//         residuals[0] = sqrt(Distance) * T(weight);
//         // std::cout << "!!!" << residuals[0] << std::endl;
//         // cout << m << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
        
        
//         return true;
//     }
// };

struct XYYawLineCost {
    Vector3 point;
    Vector3 root;
    Vector3 direction;
    CloudT modelCloud;
    double weight;

    // int m = 0;
    XYYawLineCost(Vector3 point, Vector3 root, Vector3 direction, CloudT modelCloud, double weight)
        : point(point), root(root), direction(direction), modelCloud(modelCloud), weight(weight) {}

    template <typename T>
    bool operator()(const T* const params, T* residuals) const {

        // n: normal of cylinder (x,y,z)
        Eigen::Matrix<T, 3, 1> source_point{T(point[0]), T(point[1]), T(point[2])};

        Eigen::Matrix<T, 3, 1> model_root{T(root[0]), T(root[1]), T(root[2])};
        Eigen::Matrix<T, 3, 1> model_direction{T(direction[0]), T(direction[1]), T(direction[2])};

        // last point
        // transform point using current solution
        Eigen::Matrix<T, 3, 1> last_point;
        T r_last_curr[3] = {params[3], params[4], params[5]};
        Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

        // cout << "LAST POINT: " << last_point << endl;
        ceres::AngleAxisRotatePoint(r_last_curr, source_point.data(), last_point.data());
        last_point += t_last_curr;

        // cout << "LAST POINT: " << static_cast<Eigen::Matrix<T, 3, 1>>(last_point) << endl;
        // cout << "MODEL ROOT: " << *model_root.data() << " " << model_root << endl;

        // // projected point onto cylinder
        // Eigen::Matrix<T, 3, 1> projected_point =
        //     model_root + ((last_point - model_root).dot(model_direction) / sqrt((model_direction.dot(model_direction)))) * model_direction;

        // // cout << model_root << "%\n" << last_point << "%\n" << projected_point << endl;
        // T Distance = (projected_point - last_point).norm();

        // // residuals[0] = Distance * T(weight);
        // residuals[0] = sqrt(Distance) * T(weight);

        // std::cout << "!!!" << residuals[0] << std::endl;
        // cout << m << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
        
        // Find the minimum distance between last_point and all the points in modelCloud.
        T minDistance = std::numeric_limits<T>::max(); // Initialize with a very large value.
        
        for (const auto& pointInCloud : modelCloud.points) {
            Eigen::Matrix<T, 3, 1> cloud_point{T(pointInCloud.x), T(pointInCloud.y), T(pointInCloud.z)};
            // Create a 2D vector (Eigen::Matrix) from the first two elements
            Eigen::Matrix<T, 2, 1> cloud_point_xy, last_point_xy;

            cloud_point_xy << cloud_point(0), cloud_point(1);
            last_point_xy << last_point(0), last_point(1);
            
            T distance = (cloud_point_xy - last_point_xy).norm();
            if (distance < minDistance) {
                minDistance = distance;
            }
            // T distance = (cloud_point[0:1] - last_point[0:1]).norm();
            // if (distance < minDistance) {
            //     minDistance = distance;
            // }
        }

        // Now minDistance holds the minimum distance from last_point to the modelCloud.
        residuals[0] = minDistance * T(weight); // Or any other function of minDistance as you need.

        return true;
    }
};

struct ZRollPitchGroundCost {
    Vector3 scene_point;
    Vector4 scene_coeff;
    Vector4 model_coeff;
    double weight;

    ZRollPitchGroundCost(Vector3 scene_point, Vector4 scene_coeff, Vector4 model_coeff, double weight)
        : scene_point(scene_point), scene_coeff(scene_coeff), model_coeff(model_coeff), weight(weight) {}
    
    template <typename T>
    bool operator()(const T* const params, T* residuals) const
    {
        Eigen::Matrix<T, 3, 1> cp{T(scene_point[0]), T(scene_point[1]),
                                  T(scene_point[2])};
        Eigen::Matrix<T, 4, 1> scene_p{T(scene_coeff[0]), T(scene_coeff[1]), T(scene_coeff[2]), T(scene_coeff[3])};
        Eigen::Matrix<T, 4, 1> ground_p{T(model_coeff[0]), T(model_coeff[1]), T(model_coeff[2]), T(model_coeff[3])};

        // ROTATION AS ANGLE AXIS
        Eigen::Matrix<T, 3, 1> last_point;
        T r_last_curr[3] = {params[3], params[4], params[5]};
        Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

        ceres::AngleAxisRotatePoint(r_last_curr, cp.data(), last_point.data());
        last_point += t_last_curr;
        
        // Eigen::Matrix<T, 3, 1> normal{scene_p[0], scene_p[1], scene_p[2]};
        // Compute distance from point to plane using normal vector
        // T numerator = last_point.dot(scene_p.segment(0, 3)) - ground_p.segment(0, 3).dot(scene_p.segment(0, 3));
        // T denominator = scene_p.segment(0, 3).norm();
        // T Distance = ceres::abs(numerator / denominator);

        T denominator = ground_p.segment(0, 3).norm();
        T numerator = ceres::abs(ground_p[0] * last_point[0] + ground_p[1] * last_point[1] +
                                 ground_p[2] * last_point[2] + ground_p[3]);
        T Distance = numerator / denominator;
        
        residuals[0] = Distance * T(weight);
        // std::cout << "!!!" << residuals[0] << std::endl;
        
        return true;
    }
};


// struct ZRollPitchGroundCost {
//     Vector3 scene_point;
//     Vector4 scene_coeff;
//     Vector4 model_coeff;
//     double weight;
//     double threshold;

//     ZRollPitchGroundCost(Vector3 scene_point, Vector4 scene_coeff, Vector4 model_coeff, double weight, double threshold)
//         : scene_point(scene_point), scene_coeff(scene_coeff), model_coeff(model_coeff), weight(weight), threshold(threshold) {}
    
//     template <typename T>
//     bool operator()(const T* const params, T* residuals) const
//     {
//         Eigen::Matrix<T, 3, 1> cp{T(scene_point[0]), T(scene_point[1]),
//                                   T(scene_point[2])};
//         Eigen::Matrix<T, 4, 1> scene_p{T(scene_coeff[0]), T(scene_coeff[1]), T(scene_coeff[2]), T(scene_coeff[3])};
//         Eigen::Matrix<T, 4, 1> ground_p{T(model_coeff[0]), T(model_coeff[1]), T(model_coeff[2]), T(model_coeff[3])};

//         // ROTATION AS ANGLE AXIS
//         Eigen::Matrix<T, 3, 1> last_point;
//         T r_last_curr[3] = {params[3], params[4], params[5]};
//         Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

//         ceres::AngleAxisRotatePoint(r_last_curr, cp.data(), last_point.data());
//         last_point += t_last_curr;

//         T denominator = ground_p.segment(0, 3).norm();
//         T numerator = ceres::abs(ground_p[0] * last_point[0] + ground_p[1] * last_point[1] +
//                                  ground_p[2] * last_point[2] + ground_p[3]);
//         T Distance = numerator / denominator;

//         // T numerator = last_point.dot(scene_p.segment(0, 3)) - ground_p.segment(0, 3).dot(scene_p.segment(0, 3));
//         // T denominator = scene_p.segment(0, 3).norm();
//         // T Distance = ceres::abs(numerator / denominator);

//         // T Distance = (last_point[0] - ground_p[0])*scene_coeff[0] + 
//         // (last_point[1] - ground_p[1])*scene_coeff[1] + 
//         // (last_point[2] - ground_p[2])*scene_coeff[2];
        
//         // Adjust the weight based on the distance from the ground plane
//         T adjusted_weight = T(weight);
//         if (Distance > T(threshold)) {adjusted_weight = T(1e3);} // 1e3
//         residuals[0] = Distance * adjusted_weight;

//         return true;
//     }
// };

// struct ZRollPitchGroundCost {
//     Vector3 scene_point;
//     Vector4 scene_coeff;
//     Vector4 model_coeff;
//     double weight;
//     double threshold;

//     ZRollPitchGroundCost(Vector3 scene_point, Vector4 scene_coeff, Vector4 model_coeff, double weight, double threshold)
//         : scene_point(scene_point), scene_coeff(scene_coeff), model_coeff(model_coeff), weight(weight), threshold(threshold) {}
    
//     template <typename T>
//     bool operator()(const T* const params, T* residuals) const
//     {
//         Eigen::Matrix<T, 3, 1> cp{T(scene_point[0]), T(scene_point[1]),
//                                   T(scene_point[2])};
//         Eigen::Matrix<T, 4, 1> scene_p{T(scene_coeff[0]), T(scene_coeff[1]), T(scene_coeff[2]), T(scene_coeff[3])};
//         Eigen::Matrix<T, 4, 1> ground_p{T(model_coeff[0]), T(model_coeff[1]), T(model_coeff[2]), T(model_coeff[3])};

//         // ROTATION AS ANGLE AXIS
//         Eigen::Matrix<T, 3, 1> last_point;
//         T r_last_curr[3] = {params[3], params[4], params[5]};
//         Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

//         ceres::AngleAxisRotatePoint(r_last_curr, cp.data(), last_point.data());
//         last_point += t_last_curr;

//         // Compute normal vector of scene_point using ICPN
//         Eigen::Matrix<T, 3, 1> normal{T(0), T(0), T(0)}; // Initialize normal to zero vector
//         std::vector<int> indices(1);
//         std::vector<float> distances(1);
//         pcl::PointXYZI pcl_point;
//         pcl_point.x = scene_point[0];
//         pcl_point.y = scene_point[1];
//         pcl_point.z = scene_point[2];
//         icp_normal_->nearestKSearch(pcl_point, 1, indices, distances);
//         if (indices.size() == 1)
//         {
//             normal[0] = icp_normal_->points[indices[0]].normal_x;
//             normal[1] = icp_normal_->points[indices[0]].normal_y;
//             normal[2] = icp_normal_->points[indices[0]].normal_z;
//         }

//         // Compute distance from point to plane using normal vector
//         T numerator = last_point.dot(normal) - ground_p.segment(0, 3).dot(normal);
//         T denominator = normal.norm();
//         T Distance = ceres::abs(numerator / denominator);

//         // Adjust the weight based on the distance from the ground plane
//         T adjusted_weight = T(weight);
//         // if (Distance > T(threshold)) {adjusted_weight = T(1e3);}
//         residuals[0] = Distance * adjusted_weight;

//         return true;
//     }
// };


