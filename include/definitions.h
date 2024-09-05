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

#include <visualization_msgs/MarkerArray.h>

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

        // Transform point using current solution
        Eigen::Matrix<T, 3, 1> last_point;
        T r_last_curr[3] = {params[3], params[4], params[5]};
        Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

        ceres::AngleAxisRotatePoint(r_last_curr, source_point.data(), last_point.data());
        last_point += t_last_curr;

        // // Accumulate the total distance between last_point and all points in modelCloud.
        // T totalDistance = T(0); // Initialize with zero.

        // // for (const auto& pointInCloud : modelCloud.points) {
        // Eigen::Matrix<T, 2, 1> cloud_point_2d{T(model_root[0]), T(model_root[1])};
        // Eigen::Matrix<T, 2, 1> last_point_2d{last_point.x(), last_point.y()};
        
        // // Calculate the 2D distance using only x and y components
        // T distance = (cloud_point_2d - last_point_2d).norm();
        // //     totalDistance += distance; // Sum all distances
        // // }

        // // Now totalDistance holds the sum of distances from last_point to all points in modelCloud.
        // residuals[0] = distance * T(weight); // Scale the total distance by the weight.

        // Accumulate the total distance between last_point and all points in modelCloud.
        T totalDistance = T(0); // Initialize with zero.

        for (const auto& pointInCloud : modelCloud.points) {
            Eigen::Matrix<T, 2, 1> cloud_point_2d{T(pointInCloud.x), T(pointInCloud.y)};
            Eigen::Matrix<T, 2, 1> last_point_2d{last_point.x(), last_point.y()};
            
            // Calculate the 2D distance using only x and y components
            T distance = (cloud_point_2d - last_point_2d).norm();
            totalDistance += distance; // Sum all distances
        }

        // Now totalDistance holds the sum of distances from last_point to all points in modelCloud.
        residuals[0] = totalDistance * T(weight); // Scale the total distance by the weight.


        return true;
    }


    // template <typename T>
    // bool operator()(const T* const params, T* residuals) const {

    //     // n: normal of cylinder (x,y,z)
    //     Eigen::Matrix<T, 3, 1> source_point{T(point[0]), T(point[1]), T(point[2])};

    //     Eigen::Matrix<T, 3, 1> model_root{T(root[0]), T(root[1]), T(root[2])};
    //     Eigen::Matrix<T, 3, 1> model_direction{T(direction[0]), T(direction[1]), T(direction[2])};

    //     // last point
    //     // transform point using current solution
    //     Eigen::Matrix<T, 3, 1> last_point;
    //     T r_last_curr[3] = {params[3], params[4], params[5]};
    //     Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]};

    //     // cout << "LAST POINT: " << last_point << endl;
    //     ceres::AngleAxisRotatePoint(r_last_curr, source_point.data(), last_point.data());
    //     last_point += t_last_curr;

    //     // std::cout << last_point << std::endl;

    //     // cout << "LAST POINT: " << static_cast<Eigen::Matrix<T, 3, 1>>(last_point) << endl;
    //     // cout << "MODEL ROOT: " << *model_root.data() << " " << model_root << endl;

    //     // // projected point onto cylinder
    //     // Eigen::Matrix<T, 3, 1> projected_point =
    //     //     model_root + ((last_point - model_root).dot(model_direction) / sqrt((model_direction.dot(model_direction)))) * model_direction;

    //     // // cout << model_root << "%\n" << last_point << "%\n" << projected_point << endl;
    //     // T Distance = (projected_point - last_point).norm();

    //     // // residuals[0] = Distance * T(weight);
    //     // residuals[0] = sqrt(Distance) * T(weight);

    //     // std::cout << "!!!" << residuals[0] << std::endl;
    //     // cout << m << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
        
    //     // Find the minimum distance between last_point and all the points in modelCloud.
    //     T minDistance = std::numeric_limits<T>::max(); // Initialize with a very large value.

    //     for (const auto& pointInCloud : modelCloud.points) {
    //         Eigen::Matrix<T, 3, 1> cloud_point{T(pointInCloud.x), T(pointInCloud.y), T(pointInCloud.z)};
    //         T distance = (cloud_point - last_point).norm();
    //         if (distance < minDistance) {
    //             minDistance = distance;
    //         }
    //     }

    //     // Now minDistance holds the minimum distance from last_point to the modelCloud.
    //     residuals[0] = minDistance * T(weight); // Or any other function of minDistance as you need.

    //     return true;
    // }
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

// struct YawRowCost {
//     Vector4 scene_coeff;  // Scene plane coefficients
//     Vector4 model_coeff;  // Model plane coefficients
//     double weight;        // Weight for the cost function

//     YawRowCost(Vector4 scene_coeff, Vector4 model_coeff, double weight)
//         : scene_coeff(scene_coeff), model_coeff(model_coeff), weight(weight) {}
    
//     template <typename T>
//     bool operator()(const T* const params, T* residuals) const
//     {
//         // Extract the rotation parameters (angle-axis representation)
//         T r_last_curr[3] = {params[3], params[4], params[5]}; // Rotation as angle axis
//         Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]}; // Translation (not used for normal alignment)

//         // Normal vector of the scene plane
//         Eigen::Matrix<T, 3, 1> scene_normal{T(scene_coeff[0]), T(scene_coeff[1]), T(scene_coeff[2])};
//         scene_normal.normalize(); // Normalize to ensure it's a unit vector

//         // Normal vector of the model plane
//         Eigen::Matrix<T, 3, 1> model_normal{T(model_coeff[0]), T(model_coeff[1]), T(model_coeff[2])};
//         model_normal.normalize(); // Normalize to ensure it's a unit vector

//         // Use a separate vector to store the rotated normal
//         Eigen::Matrix<T, 3, 1> rotated_scene_normal;

//         // Rotate the model normal using the rotation parameters
//         ceres::AngleAxisRotatePoint(r_last_curr, scene_normal.data(), rotated_scene_normal.data());

//         // Normalize the transformed normal
//         rotated_scene_normal.normalize();

//         // // Compute the alignment error between the normals using dot product
//         // T dot_product = scene_normal.dot(rotated_model_normal);

//         // // The residual is minimized when the dot product is 1 (normals are aligned)
//         // residuals[0] = T(weight) * (T(1.0) - dot_product); // Minimizing this residual aligns the normals

//         T rotated_scene_yaw = ceres::atan2(rotated_scene_normal[1], rotated_scene_normal[0]);
//         T model_yaw = ceres::atan2(model_normal[1], model_normal[0]);
        
//         // T angle_diff = rotated_scene_yaw - model_yaw;
//         // if (angle_diff > M_PI) {
//         //     angle_diff -= T(2.0 * M_PI);
//         // } else if (angle_diff < -M_PI) {
//         //     angle_diff += T(2.0 * M_PI);
//         // }
//         // residuals[0] = T(weight) * ceres::abs(angle_diff);

//         // Compute the angle difference in the range [-pi, pi]
//         T angle_diff = rotated_scene_yaw - model_yaw;

//         // Adjust the angle difference to be within the range [-pi, pi]
//         if (angle_diff > T(M_PI)) {
//             angle_diff -= T(2.0 * M_PI);
//         } else if (angle_diff < -T(M_PI)) {
//             angle_diff += T(2.0 * M_PI);
//         }

//         // Set the residual to the absolute angle difference
//         residuals[0] = T(weight) * ceres::abs(angle_diff);

//         return true;
//     }
// };

struct YawRowCost1 {
    Vector4 scene_coeff;  // Scene plane coefficients
    Vector4 model_coeff;  // Model plane coefficients
    double weight;        // Weight for the cost function

    YawRowCost1(Vector4 scene_coeff, Vector4 model_coeff, double weight)
        : scene_coeff(scene_coeff), model_coeff(model_coeff), weight(weight) {}
    
    template <typename T>
    bool operator()(const T* const params, T* residuals) const
    {
        // Extract the rotation parameters (angle-axis representation)
        T r_last_curr[3] = {params[3], params[4], params[5]}; // Rotation as angle axis
        Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]}; // Translation (not used for normal alignment)

        // Normal vector of the scene plane
        Eigen::Matrix<T, 3, 1> scene_normal{T(scene_coeff[0]), T(scene_coeff[1]), T(scene_coeff[2])};
        scene_normal.normalize(); // Normalize to ensure it's a unit vector

        // Normal vector of the model plane
        Eigen::Matrix<T, 3, 1> model_normal{T(model_coeff[0]), T(model_coeff[1]), T(model_coeff[2])};
        model_normal.normalize(); // Normalize to ensure it's a unit vector

        // Use a separate vector to store the rotated normal
        Eigen::Matrix<T, 3, 1> rotated_scene_normal;

        // Rotate the model normal using the rotation parameters
        ceres::AngleAxisRotatePoint(r_last_curr, scene_normal.data(), rotated_scene_normal.data());

        // Normalize the transformed normal
        rotated_scene_normal.normalize();

        // // Compute the alignment error between the normals using dot product
        // T dot_product = scene_normal.dot(rotated_model_normal);

        // // The residual is minimized when the dot product is 1 (normals are aligned)
        // residuals[0] = T(weight) * (T(1.0) - dot_product); // Minimizing this residual aligns the normals

        T rotated_scene_yaw = ceres::atan2(rotated_scene_normal[1], rotated_scene_normal[0]);
        T model_yaw = ceres::atan2(model_normal[1], model_normal[0]);
        
        cout << "Yaw Optimizing: scene / model: " << rotated_scene_yaw << " " << model_yaw << endl;
        // T angle_diff = rotated_scene_yaw - model_yaw;
        // if (angle_diff > M_PI) {
        //     angle_diff -= T(2.0 * M_PI);
        // } else if (angle_diff < -M_PI) {
        //     angle_diff += T(2.0 * M_PI);
        // }
        // residuals[0] = T(weight) * ceres::abs(angle_diff);

        // Compute the angle difference in the range [-pi, pi]
        T angle_diff = ceres::abs(rotated_scene_yaw - model_yaw);

        // Adjust the angle difference to be within the range [-pi, pi]
        if (angle_diff > T(2.0 * M_PI)) {
            angle_diff -= T(2.0 * M_PI);
        }

        // Set the residual to the absolute angle difference
        residuals[0] = T(weight) * ceres::abs(angle_diff);

        return true;
    }
};

struct YawRowCost2 {
    Vector4 scene_coeff;  // Scene plane coefficients
    Vector4 model_coeff;  // Model plane coefficients
    double rotation_angle; // Rotational angle between keyframes
    double weight;        // Weight for the cost function

    YawRowCost2(Vector4 scene_coeff, Vector4 model_coeff, double rotation_angle, double weight)
        : scene_coeff(scene_coeff), model_coeff(model_coeff), rotation_angle(rotation_angle), weight(weight) {}
    
    template <typename T>
    bool operator()(const T* const params, T* residuals) const
    {
        // Extract the rotation parameters (angle-axis representation)
        T r_last_curr[3] = {params[3], params[4], params[5]}; // Rotation as angle axis
        Eigen::Matrix<T, 3, 1> t_last_curr{params[0], params[1], params[2]}; // Translation (not used for normal alignment)

        // Normal vector of the scene plane
        Eigen::Matrix<T, 3, 1> scene_normal{T(scene_coeff[0]), T(scene_coeff[1]), T(scene_coeff[2])};
        scene_normal.normalize(); // Normalize to ensure it's a unit vector

        // Normal vector of the model plane
        Eigen::Matrix<T, 3, 1> model_normal{T(model_coeff[0]), T(model_coeff[1]), T(model_coeff[2])};
        model_normal.normalize(); // Normalize to ensure it's a unit vector

        // Use a separate vector to store the rotated normal
        Eigen::Matrix<T, 3, 1> rotated_scene_normal;

        // Rotate the model normal using the rotation parameters
        ceres::AngleAxisRotatePoint(r_last_curr, scene_normal.data(), rotated_scene_normal.data());

        // Normalize the transformed normal
        rotated_scene_normal.normalize();

        // // Compute the alignment error between the normals using dot product
        // T dot_product = scene_normal.dot(rotated_model_normal);

        // // The residual is minimized when the dot product is 1 (normals are aligned)
        // residuals[0] = T(weight) * (T(1.0) - dot_product); // Minimizing this residual aligns the normals

        T rotated_scene_yaw = ceres::atan2(rotated_scene_normal[1], rotated_scene_normal[0]);
        T model_yaw = ceres::atan2(model_normal[1], model_normal[0]);
        
        cout << "Yaw Optimizing: scene / model: " << rotated_scene_yaw << " " << model_yaw << endl;
        // T angle_diff = rotated_scene_yaw - model_yaw;
        // if (angle_diff > M_PI) {
        //     angle_diff -= T(2.0 * M_PI);
        // } else if (angle_diff < -M_PI) {
        //     angle_diff += T(2.0 * M_PI);
        // }
        // residuals[0] = T(weight) * ceres::abs(angle_diff);

        // Compute the angle difference in the range [-pi, pi]
        T angle_diff = ceres::abs((rotated_scene_yaw + rotation_angle) - model_yaw);

        // Adjust the angle difference to be within the range [-pi, pi]
        if (angle_diff > T(2.0 * M_PI)) {
            angle_diff -= T(2.0 * M_PI);
        }

        // Set the residual to the absolute angle difference
        residuals[0] = T(weight) * ceres::abs(angle_diff);

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


