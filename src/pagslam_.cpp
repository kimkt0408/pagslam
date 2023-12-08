#include <pagslam.h>

namespace pagslam
{
    pagslam::pagslam(){
        T_Map_Anchor_ = SE3();
        firstScan_ = true;
        bool_groundMatch_ = false;

        // (1) Simulation
        // stalkMatchThresh_ = 0.12;  // 0.12 // 0.3
        // minStalkMatches_ = 3; //2;
        // rangeGroundMatch_ = 4; // 4
        // AddNewStalkThreshDist_ = stalkMatchThresh_;
        // maxNumIterations_ = 20;

        // (2) ACRE
        // stalkMatchThresh_ = 0.08;  // 0.12 // 0.3
        // minStalkMatches_ = 3; //2;
        // rangeGroundMatch_ = 4; // 4
        // AddNewStalkThreshDist_ = 0.04;
        // maxNumIterations_ = 100;

        // (3) ACRE-3
        // stalkMatchThresh_ = 0.10;  // 0.12 // 0.3
        // minStalkMatches_ = 3; //2;
        // rangeGroundMatch_ = 4; // 4
        // AddNewStalkThreshDist_ = 0.08;
        // maxNumIterations_ = 100;

        // // (4) new ACRE-long
        // stalkMatchThresh_ = 0.06;  // 0.12 // 0.3
        // minStalkMatches_ = 2; //2;
        // rangeGroundMatch_ = 3; // 4
        // AddNewStalkThreshDist_ = 0.06;
        // maxNumIterations_ = 100;
        // huberLossThresh_ = 1; // 0.1

        // Range-view h_cloud
        stalkMatchThresh_ = 0.12;  // 0.12 // 0.3
        minStalkMatches_ = 3; //2;
        rangeGroundMatch_ = 10; // 4
        AddNewStalkThreshDist_ = stalkMatchThresh_*0.4;
        SkipStalkThreshDist_ = stalkMatchThresh_;
        maxNumIterations_ = 1000;
        huberLossThresh_ = 10; // 0.1
    }


    void pagslam::projectFeatures(const SE3 &tf, GroundFeature &ground, std::vector<StalkFeature::Ptr> &stalks){
        projectGround(tf, ground);
        bool firstStalk = false;
        for (auto &stalk : stalks){
            projectStalk(tf, stalk, firstStalk);
            if (firstStalk){
                firstStalk == false;
            }
        }
    }


    void pagslam::projectGround(const SE3 &tf, GroundFeature &ground){
        Matrix4 tfm = tf.matrix().inverse().transpose();

        auto& cloud_points = ground.cloud->points;
        auto& coeffs_values = ground.coefficients->values;

        for (auto& point : cloud_points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            Vector3 v(point.x, point.y, point.z);
            v = tf * v;
            point.x = v(0);
            point.y = v(1);
            point.z = v(2);
        }

        Vector4 coefficients(coeffs_values[0], coeffs_values[1], coeffs_values[2], coeffs_values[3]);
        VectorX transformedCoefficients = tfm * coefficients;

        coeffs_values[0] = transformedCoefficients(0);
        coeffs_values[1] = transformedCoefficients(1);
        coeffs_values[2] = transformedCoefficients(2);
        coeffs_values[3] = transformedCoefficients(3);
    }


    void pagslam::projectStalk(const SE3 &tf, StalkFeature::Ptr &stalk, bool firstStalk){
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
            //     cout << "After: " << point.x << " " << point.y << " " << point.z << endl;
            // }

        }
    }


    // std::vector<FeatureMatch<StalkFeature::Ptr>> pagslam::matchStalks(const SE3 &tf, const std::vector<StalkFeature::Ptr> &currFeatures, std::vector<StalkFeature::Ptr> &mapFeatures, const Scalar distThresh)
    // {
    //     std::vector<FeatureMatch<StalkFeature::Ptr>> matches;
    //     std::unordered_set<StalkFeature::Ptr> assignedFeatures; // Keep track of assigned map features
    //     // std::unordered_set<Scalar> assignedDistances; // Keep track of assigned map features
    //     std::unordered_map<StalkFeature::Ptr, Scalar> bestDistances;
    //     std::unordered_map<StalkFeature::Ptr, StalkFeature::Ptr> bestFeaturePairs;

    //     int n = 0;
    //     for (const auto &mf : mapFeatures){
    //         // bestDistances[mf] = 100;
    //         Scalar bestDist = 100;

    //         StalkFeature::Ptr bestCurrentFeature = nullptr;

    //         for (const auto &cf : currFeatures){
    //             StalkFeature::Ptr cf_proj = std::make_shared<StalkFeature>(*cf);
    //             projectStalk(tf, cf_proj);

    //             // find closest model in map
    //             // Scalar bestDist = 100;

    //             Scalar d = stalkDistance(cf_proj->root, mf->root);

    //             // if (d < bestDistances[mf]){
    //             //     bestCurrentFeature = cf;
    //             //     bestFeaturePairs[bestCurrentFeature] = mf;

    //             //     bestDistances[bestFeaturePairs[bestCurrentFeature]] = d;
    //             // }

    //             if (d < bestDist){
    //                 bestCurrentFeature = cf;
    //                 // bestFeaturePairs[bestCurrentFeature] = mf;

    //                 bestDist = d;

    //                 // cout << "DIST: " << bestDist << endl;
    //             }
    //         }

    //         // if (assignedFeatures.count(bestCurrentFeature) > 0) {

    //         //     cout << "CHANGED: " << bestDist  << " " << bestDistances[bestFeaturePairs[bestCurrentFeature]] << endl;
    //         //     if (bestDist < bestDistances[bestFeaturePairs[bestCurrentFeature]]){
    //         //         cout << "CHANGED MATCHES!" << bestDistances[bestFeaturePairs[bestCurrentFeature]] << " " << bestDist << endl;
    //         //         bestDistances.erase(bestFeaturePairs[bestCurrentFeature]);
    //         //         bestFeaturePairs[bestCurrentFeature] = mf;
    //         //         bestDistances[bestFeaturePairs[bestCurrentFeature]] = bestDist;

    //         //         assignedFeatures.insert(bestCurrentFeature);
    //         //     }
    //         //     else{
    //         //         cout << "SKIP!!" << endl;
    //         //         continue;
    //         //     }

    //         //     // continue;
    //         // }
    //         // else{
    //         if (bestDist < distThresh){
    //             if (assignedFeatures.count(bestCurrentFeature) > 0){
    //             // if (!isnan(bestFeaturePairs[bestCurrentFeature])){

    //                 cout << "CHANGED: " << bestDist  << " " << bestDistances[bestFeaturePairs[bestCurrentFeature]] << endl;

    //                 if (bestDist < bestDistances[bestFeaturePairs[bestCurrentFeature]]){
    //                     // cout << "CHANGED MATCHES!" << bestDistances[bestFeaturePairs[bestCurrentFeature]] << " " << bestDist << endl;
    //                     bestDistances.erase(bestFeaturePairs[bestCurrentFeature]);
    //                     bestFeaturePairs[bestCurrentFeature] = mf;
    //                     bestDistances[bestFeaturePairs[bestCurrentFeature]] = bestDist;

    //                     assignedFeatures.insert(bestCurrentFeature);
    //                 }
    //                 else{
    //                     // cout << "SKIP!!" << endl;
    //                     continue;
    //                 }
    //             }
    //             else{
    //                 bestFeaturePairs[bestCurrentFeature] = mf;
    //                 bestDistances[bestFeaturePairs[bestCurrentFeature]] = bestDist;



    //                 // bestDistances[mf] = bestDist;
    //                 addFeatureMatches(bestCurrentFeature, mf, bestDistances[mf], matches);
    //                 cout << n << " best distance: " << bestDistances[bestFeaturePairs[bestCurrentFeature]] << " " << bestDistances[mf]  << endl;
    //                 assignedFeatures.insert(bestCurrentFeature);
    //             }

    //         }
    //         // }

    //         n += 1;
    //     }



    //     // for (const auto &cf : currFeatures){
    //     //     StalkFeature::Ptr cf_proj = std::make_shared<StalkFeature>(*cf);
    //     //     projectStalk(tf, cf_proj);

    //     //     // find closest model in map
    //     //     Scalar bestDist = 100;
    //     //     StalkFeature::Ptr bestMapFeature = nullptr;

    //     //     for (const auto &mf : mapFeatures){
    //     //         // bestDistances[mf] = bestDist;
    //     //         // Skip if the map feature is already assigned
    //     //         // if (assignedFeatures.count(mf) > 0) {
    //     //         //     continue;
    //     //         // }
    //     //         Scalar d = stalkDistance(cf_proj->root, mf->root);
    //     //         // cout << "best distances: " << d << endl;
    //     //         if (d < bestDistances[mf]){
    //     //             bestDist = d;
    //     //             bestMapFeature = mf;
    //     //         }

    //     //         // if (d < bestDist){
    //     //         //     bestDist = d;
    //     //         //     bestMapFeature = mf;

    //     //         //     // cout << "Best distance1: " << bestDist << endl;
    //     //         // }
    //     //     }

    //     //     // if (bestMapFeature != nullptr) {
    //     //     //     cout << "!!!!!!!!" << bestDist << cf_proj->root << bestMapFeature->root << endl;
    //     //     //     addFeatureMatches(cf, bestMapFeature, bestDist, matches);

    //     //     //     // Mark the map feature as assigned
    //     //     //     assignedFeatures.insert(bestMapFeature);
    //     //     // }


    //     //     if ((bestDist < distThresh) && (bestDist < bestDistances[bestMapFeature])){
    //     //         // if (assignedFeatures.count(bestMapFeature) > 0){
    //     //         //     continue;
    //     //         // }

    //     //         bestDistances[bestMapFeature] = bestDist;
    //     //         cout << "Best distance: " << bestDistances[bestMapFeature] << endl;
    //     //         // cout << "!!!!!!!!" << bestDist << cf_proj->root << bestMapFeature->root << endl;
    //     //         addFeatureMatches(cf, bestMapFeature, bestDist, matches);

    //     //         // Mark the map feature as assigned
    //     //         // assignedFeatures.insert(bestMapFeature);
    //     //     }

    //     // }
    //     return matches;
    // }

    std::vector<FeatureMatch<StalkFeature::Ptr>> pagslam::matchStalks(const SE3 &tf,
    const std::vector<StalkFeature::Ptr> &currFeatures,
    std::vector<StalkFeature::Ptr> &mapFeatures, const Scalar distThresh)
    {
        std::vector<FeatureMatch<StalkFeature::Ptr>> matches;

        // This map will hold the best match (and its distance) for each current feature.
        std::unordered_map<StalkFeature::Ptr, std::pair<StalkFeature::Ptr, Scalar>> bestMatches;

        int n = 0;
        for (const auto &cf : currFeatures) {
            int m = 0;
            StalkFeature::Ptr cf_proj = std::make_shared<StalkFeature>(*cf);
            projectStalk(tf, cf_proj, false);

            Scalar bestDist = std::numeric_limits<Scalar>::max(); // Initialized to a very large value.
            StalkFeature::Ptr bestMapFeature = nullptr;

            for (const auto &mf : mapFeatures) {
                // Scalar d = stalkDistance(cf_proj->root, mf->root);
                Scalar d = stalkDistance(cf_proj->centroid, mf->centroid);

                if (d < bestDist) {
                    bestDist = d;
                    bestMapFeature = mf;
                    // cout << bestDist << endl;
                }
            }

            n = n + 1;

            // Check if this is a better match than a previous one for the current feature.
            if (bestDist < distThresh &&
                (bestMatches.find(cf) == bestMatches.end() || bestDist < bestMatches[cf].second))
            {
                // cout << n << " Best match: " << bestDist << endl << bestMapFeature->root << endl << cf_proj->root << endl;
                bestMatches[cf] = {bestMapFeature, bestDist};
            }
        }

        for (const auto &entry : bestMatches) {
            addFeatureMatches(entry.first, entry.second.first, entry.second.second, matches);
        }

        return matches;
    }


    // std::vector<FeatureMatch<StalkFeature::Ptr>> pagslam::matchStalks(const SE3 &tf, const std::vector<StalkFeature::Ptr> &currFeatures, std::vector<StalkFeature::Ptr> &mapFeatures, const Scalar distThresh)
    // {
    //     std::vector<FeatureMatch<StalkFeature::Ptr>> matches;
    //     std::unordered_set<StalkFeature::Ptr> assignedFeatures; // Keep track of assigned map features
    //     std::unordered_set<Scalar> assignedDistances; // Keep track of assigned map features

    //     for (const auto &cf : currFeatures){
    //         StalkFeature::Ptr cf_proj = std::make_shared<StalkFeature>(*cf);
    //         projectStalk(tf, cf_proj);

    //         // find closest model in map
    //         Scalar bestDist = 100;
    //         StalkFeature::Ptr bestMapFeature = nullptr;

    //         for (const auto &mf : mapFeatures){
    //             // Skip if the map feature is already assigned
    //             // if (assignedFeatures.count(mf) > 0) {
    //             //     continue;
    //             // }

    //             Scalar d = stalkDistance(cf_proj->root, mf->root);
    //             if (d < bestDist){
    //                 bestDist = d;
    //                 bestMapFeature = mf;

    //                 // cout << "Best distance1: " << bestDist << endl;
    //             }
    //         }

    //         // if (bestMapFeature != nullptr) {
    //         //     cout << "!!!!!!!!" << bestDist << cf_proj->root << bestMapFeature->root << endl;
    //         //     addFeatureMatches(cf, bestMapFeature, bestDist, matches);

    //         //     // Mark the map feature as assigned
    //         //     assignedFeatures.insert(bestMapFeature);
    //         // }


    //         if (bestDist < distThresh){
    //             // if (assignedFeatures.count(bestMapFeature) > 0){
    //             //     continue;
    //             // }

    //             cout << "Best distance2: " << bestDist << endl;
    //             // cout << "!!!!!!!!" << bestDist << cf_proj->root << bestMapFeature->root << endl;
    //             addFeatureMatches(cf, bestMapFeature, bestDist, matches);

    //             // Mark the map feature as assigned
    //             assignedFeatures.insert(bestMapFeature);
    //         }

    //     }
    //     return matches;
    // }

    Scalar pagslam::stalkDistance(Eigen::Vector3f v1, Eigen::Vector3f v2)
    {
        Scalar d = sqrt((v1(0)-v2(0))*(v1(0)-v2(0)) + (v1(1)-v2(1))*(v1(1)-v2(1)));

        return d;
    }

    Scalar pagslam::groundDistance(GroundFeature g1, GroundFeature g2)
    {
        float a1 = g1.coefficients->values[0];
        float b1 = g1.coefficients->values[1];
        float c1 = g1.coefficients->values[2];
        float d1 = g1.coefficients->values[3];

        float a2 = g2.coefficients->values[0];
        float b2 = g2.coefficients->values[1];
        float c2 = g2.coefficients->values[2];
        float d2 = g2.coefficients->values[3];

        float num = std::abs(a1 * b2 - a2 * b1 + b1 * c2 - b2 * c1 + c1 * d2 - c2 * d1);
        float den = std::sqrt((a1 * a1 + b1 * b1 + c1 * c1) * (a2 * a2 + b2 * b2 + c2 * c2));

        return num / den;
    }


    template <typename T>
    void pagslam::addFeatureMatches(const T &currFeature, const T &mapFeature, const double dist, std::vector<FeatureMatch<T>> &matches){
        FeatureMatch<T> match;

        match.sceneFeature = currFeature;
        match.modelFeature = mapFeature;
        match.dist = dist;

        matches.push_back(match);
    }


    bool pagslam::TwoStepOptimizePose(const PagslamInput &in_proj,
        const bool stalkCheck,
        const std::vector<FeatureMatch<StalkFeature::Ptr>> &stalkMatches,
        SE3 &tf)
    {
        double stalkOut[3];
        double groundOut[3];

        // (1) Stalk feature optimization
        OptimizeXYYaw(in_proj.poseEstimate, stalkCheck, stalkMatches, stalkOut);

        // (2) Ground feature optimization
        OptimizeZRollPitch(in_proj.poseEstimate, in_proj.groundFeature, prevGroundFeatures_, groundOut);

        // roll, pitch, yaw
        double q[4];
        double angleAxis[3] = {groundOut[1], groundOut[2], stalkOut[2]};
        ceres::AngleAxisToQuaternion(angleAxis, q);
        Quat quat(q[0], q[1], q[2], q[3]);

        tf.setQuaternion(quat);
        tf.translation()[0] = stalkOut[0];
        tf.translation()[1] = stalkOut[1];
        tf.translation()[2] = groundOut[0];

        return true;
    }


    // void pagslam::OptimizeXYYaw(const SE3& poseEstimate, const bool optimize, const std::vector<FeatureMatch<StalkFeature::Ptr>> &stalkMatches, double* out)
    // {
    //     auto t = poseEstimate.translation();
    //     auto q = poseEstimate.unit_quaternion();
    //     double quat[4] = {q.w(), q.x(), q.y(), q.z()};
    //     double rpy[3];

    //     ceres::QuaternionToAngleAxis(quat, rpy);
    //     double params[6] = {t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]};

    //     // cout << "-------------------------------------------------------------" << endl;
    //     // ROS_DEBUG_STREAM("XYYaw Before " << params[0] << " " << params[1] << " " << params[5]);
    //     cout << "XYYaw Before " << params[0] << " " << params[1] << " " << params[5] << endl;
    //     bool success = true;

    //     if(optimize){
    //         ceres::LossFunction *loss = NULL;
    //         // loss = new ceres::HuberLoss(0.1);
    //         loss = new ceres::HuberLoss(huberLossThresh_);
    //         ceres::Problem::Options problem_options;
    //         ceres::Problem problem(problem_options);

    //         problem.AddParameterBlock(params, 6);
    //         // setting z, roll and pitch as constant
    //         ceres::SubsetParameterization *subset_parameterization =
    //             new ceres::SubsetParameterization(6, {0, 1, 2, 3, 4});
    //         problem.SetParameterization(params, subset_parameterization);

    //         for (auto stalkMatch : stalkMatches){
    //             Vector3 root = stalkMatch.modelFeature->root.cast<double>();
    //             Vector3 direction = stalkMatch.modelFeature->direction.cast<double>();
    //             // double weight = stalkMatch.modelFeature->cloud.size();
    //             // double weight = 1.0/stalkMatch.sceneFeature->cloud.size();

    //             // cout << root << " " << direction << endl;

    //             // double weight = 1.0;
    //             int n = 0;
    //             double weight = 1e3/stalkMatch.sceneFeature->cloud.size();

    //             // cout << "!!!: " << stalkMatch.sceneFeature->cloud.size() << endl;
    //             for (auto point : stalkMatch.sceneFeature->cloud){
    //                 Vector3 pt (point.x, point.y, point.z);
    //                 ceres::CostFunction* cost =
    //                     new ceres::AutoDiffCostFunction<XYYawLineCost, 1, 6>(
    //                         new XYYawLineCost(pt, root, direction, weight));
    //                 problem.AddResidualBlock(cost, loss, params);
    //                 n = n + 1;
    //                 // cout << "&&&: " << problem.NumResidualBlocks() << endl;
    //             }
    //             // cout << "???: " << n << endl;
    //         }

    //         // cout << "=====================================" << endl;


    //         ceres::Solver::Options options;
    //         options.function_tolerance = 1e-6;  // Adjust this value as needed.
    //         options.parameter_tolerance = 1e-7;  // Adjust this value as needed.

    //         options.max_num_iterations = maxNumIterations_;

    //         options.linear_solver_type = ceres::DENSE_QR;
    //         // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    //         // options.logging_type = ceres::SILENT;

    //         // options.trust_region_strategy_type = ceres::DOGLEG;
    //         // options.preconditioner_type = ceres::JACOBI;

    //         // options.logging_type = ceres::PER_MINIMIZER_ITERATION;

    //         options.minimizer_progress_to_stdout = true;


    //         ceres::Solver::Summary summary;
    //         ceres::Solve(options, &problem, &summary);
    //         success = (summary.termination_type == 0);

    //         std::cout << summary.BriefReport() << "\n";

    //         // std::cout << summary.FullReport() << "\n" << endl;

    //     }


    //     if(optimize && success){
    //         out[0] = params[0];
    //         out[1] = params[1];
    //         out[2] = params[5];

    //         // ROS_DEBUG_STREAM("XYYaw: Optimized After " << out[0] << " " << out[1] << " " << out[2]);
    //         cout << "XYYaw: Optimized After " << out[0] << " " << out[1] << " " << out[2] << endl;
    //     }
    //     else{
    //         out[0] = t[0];
    //         out[1] = t[1];
    //         out[2] = rpy[2];

    //         // cout << "XYYaw: NOT Optimized After " << out[0] << " " << out[1] << " " << out[2] << endl;
    //     }
    // }

    void pagslam::OptimizeXYYaw(const SE3& poseEstimate, const bool optimize, const std::vector<FeatureMatch<StalkFeature::Ptr>> &stalkMatches, double* out)
    {
        auto t = poseEstimate.translation();
        auto q = poseEstimate.unit_quaternion();
        double quat[4] = {q.w(), q.x(), q.y(), q.z()};
        double rpy[3];

        ceres::QuaternionToAngleAxis(quat, rpy);
        double params[6] = {t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]};

        // cout << "-------------------------------------------------------------" << endl;
        ROS_DEBUG_STREAM("XYYaw Before " << params[0] << " " << params[1] << " " << params[5]);
        // cout << "XYYaw Before " << params[0] << " " << params[1] << " " << params[5] << endl;
        bool success = true;

        if(optimize){
            ceres::LossFunction *loss = NULL;
            // loss = new ceres::HuberLoss(0.1);
            loss = new ceres::HuberLoss(huberLossThresh_);
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);

            problem.AddParameterBlock(params, 6);
            // setting z, roll and pitch as constant
            ceres::SubsetParameterization *subset_parameterization =
                new ceres::SubsetParameterization(6, {2, 3, 4});
                // new ceres::SubsetParameterization(6, {1, 2, 3, 4});
            problem.SetParameterization(params, subset_parameterization);

            for (auto stalkMatch : stalkMatches){
                Vector3 root = stalkMatch.modelFeature->root.cast<double>();
                Vector3 direction = stalkMatch.modelFeature->direction.cast<double>();
                // double weight = stalkMatch.modelFeature->cloud.size();
                // double weight = 1.0/stalkMatch.sceneFeature->cloud.size();

                // cout << root << " " << direction << endl;

                // double weight = 1.0;
                int n = 0;
                double weight = 1e2/stalkMatch.sceneFeature->cloud.size();
                CloudT modelCloud = stalkMatch.modelFeature->cloud;

                // cout << "!!!: " << stalkMatch.sceneFeature->cloud.size() << endl;
                for (auto point : stalkMatch.sceneFeature->cloud){
                    Vector3 pt (point.x, point.y, point.z);
                    ceres::CostFunction* cost =
                        new ceres::AutoDiffCostFunction<XYYawLineCost, 1, 6>(
                            new XYYawLineCost(pt, root, direction, modelCloud, weight));
                            // new XYYawLineCost(pt, root, direction, weight));
                    problem.AddResidualBlock(cost, loss, params);
                    n = n + 1;
                    // cout << "&&&: " << problem.NumResidualBlocks() << endl;
                }
                // cout << "???: " << n << endl;
            }

            // cout << "=====================================" << endl;


            ceres::Solver::Options options;
            options.function_tolerance = 1e-6;  // Adjust this value as needed.
            options.parameter_tolerance = 1e-7;  // Adjust this value as needed.

            options.max_num_iterations = maxNumIterations_;

            // options.linear_solver_type = ceres::DENSE_QR;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            // options.logging_type = ceres::SILENT;

            // options.trust_region_strategy_type = ceres::DOGLEG;
            // options.preconditioner_type = ceres::JACOBI;

            // options.logging_type = ceres::PER_MINIMIZER_ITERATION;

            // options.minimizer_progress_to_stdout = true;


            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            success = (summary.termination_type == 0);

            std::cout << summary.BriefReport() << "\n";

            std::cout << summary.FullReport() << "\n" << endl;

        }


        if(optimize && success){
            out[0] = params[0];
            out[1] = params[1];
            out[2] = params[5];

            ROS_DEBUG_STREAM("XYYaw: Optimized After " << out[0] << " " << out[1] << " " << out[2]);
            // cout << "**************XYYaw: Optimized After " << out[0] << " " << out[1] << " " << out[2] << "**************" << endl;
        }
        else{
            out[0] = t[0];
            out[1] = t[1];
            out[2] = rpy[2];

            // cout << "XYYaw: NOT Optimized After " << out[0] << " " << out[1] << " " << out[2] << endl;
        }
    }

    void pagslam::OptimizeZRollPitch(const SE3& poseEstimate, const GroundFeature &currFeature, std::vector<GroundFeature> &mapFeatures, double* out){
        auto t = poseEstimate.translation();
        auto q = poseEstimate.unit_quaternion();
        double quat[4] = {q.w(), q.x(), q.y(), q.z()};
        double rpy[3];

        ceres::QuaternionToAngleAxis(quat, rpy);
        double params[6] = {t[0], t[1], t[2], rpy[0], rpy[1], rpy[2]};

        ROS_DEBUG_STREAM("ZRollPitch Before " << params[2] << " " << params[3] << " " << params[4]);
        // cout << "ZRollPitch Before " << params[2] << " " << params[3] << " " << params[4] << endl;;
        bool success = true;

        ceres::LossFunction *loss = NULL;
        loss = new ceres::HuberLoss(huberLossThresh_);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(params, 6);
        // setting z, roll and pitch as constant
        ceres::SubsetParameterization *subset_parameterization =
            new ceres::SubsetParameterization(6, {0, 1, 5});
        problem.SetParameterization(params, subset_parameterization);

        GroundFeature lastGroundFeature = mapFeatures.back();

        Vector4 scene_coeff (currFeature.coefficients->values[0], currFeature.coefficients->values[1], currFeature.coefficients->values[2], currFeature.coefficients->values[3]);
        Vector4 model_coeff (lastGroundFeature.coefficients->values[0], lastGroundFeature.coefficients->values[1], lastGroundFeature.coefficients->values[2], lastGroundFeature.coefficients->values[3]);

        double scene_model_diff = scene_coeff[0]*model_coeff[0] + scene_coeff[1]*model_coeff[1];

        // cout << scene_coeff << " " << model_coeff << " " << scene_model_diff << endl;

        // if (abs(scene_model_diff) > 1e-3){
        //     cout << "SLOPE: " << scene_model_diff << endl;
        //     success = 0;}
        // else{
        //     int i = 0;
        //     for (auto point : currFeature.cloud->points){
        //         const double distance = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        //         if (distance < rangeGroundMatch_){
        //             Vector3 scene_point (point.x, point.y, point.z);
        //             // double weight = 1/scene_point.norm();
        //             // double weight = 1;

        //             // ceres::CostFunction* cost =
        //             //         new ceres::AutoDiffCostFunction<ZRollPitchGroundCost, 1, 6>(
        //             //             new ZRollPitchGroundCost(scene_point, scene_coeff, model_coeff, weight));

        //             double weight = 1e-2; // 1
        //             double threshold = 0.3; // 0.2

        //             ceres::CostFunction* cost =
        //                 new ceres::AutoDiffCostFunction<ZRollPitchGroundCost, 1, 6>(
        //                     new ZRollPitchGroundCost(scene_point, scene_coeff, model_coeff, weight, threshold));

        //             problem.AddResidualBlock(cost, loss, params);
        //             i++;
        //         }
        //     }

        //     ceres::Solver::Options options;
        //     options.max_num_iterations = maxNumIterations_;

        //     options.linear_solver_type = ceres::DENSE_QR;
        //     // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        //     options.logging_type = ceres::SILENT;

        //     ceres::Solver::Summary summary;
        //     ceres::Solve(options, &problem, &summary);

        //     success = (summary.termination_type == 0);
        // }

        int i = 0;
        for (auto point : currFeature.cloud->points){
            // const double distance = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            // if (distance < rangeGroundMatch_){
                Vector3 scene_point (point.x, point.y, point.z);
                // double weight = 1/distance;
                double weight = 1;
                
                // double weight = scene_point.norm();
                // double weight = 1e-2; // 1e-2
                // double threshold = 0.2; // 0.2

                // double weight = 1; // 1e-2

                ceres::CostFunction* cost =
                        new ceres::AutoDiffCostFunction<ZRollPitchGroundCost, 1, 6>(
                            new ZRollPitchGroundCost(scene_point, scene_coeff, model_coeff, weight));
                // ceres::CostFunction* cost =
                //         new ceres::AutoDiffCostFunction<ZRollPitchGroundCost, 1, 6>(
                //             new ZRollPitchGroundCost(scene_point, scene_coeff, model_coeff, weight, threshold));

                problem.AddResidualBlock(cost, loss, params);
                i++;
            // }
        }


        std::vector<double*> parameter_blocks;
        problem.GetParameterBlocks(&parameter_blocks);

        double cost = 0.0;
        std::vector<double> residuals;
        // problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, nullptr, nullptr);

        // for (double* param_block : parameter_blocks) {
        //     // Assuming a 6-dimensional parameter block for this example.
        //     std::cout << "Parameter values: ";
        //     for (int i = 0; i < 6; i++) {
        //         std::cout << param_block[i] << " ";
        //     }
        //     std::cout << std::endl;
        // }


        ceres::Solver::Options options;
        options.function_tolerance = 1e-8;  // Adjust this value as needed.
        options.parameter_tolerance = 1e-10;  // Adjust this value as needed.
        options.max_num_iterations = maxNumIterations_;

        // options.linear_solver_type = ceres::DENSE_QR;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        // options.logging_type = ceres::SILENT;
        // options.logging_type = ceres::PER_MINIMIZER_ITERATION;

        ceres::Solver::Summary summary;
        // std::cout << summary.FullReport() << "\n";
        ceres::Solve(options, &problem, &summary);

        success = (summary.termination_type == 0);

        // std::cout << summary.FullReport() << "\n" << summary.termination_type << " " << success << endl;

        if(success){
            // out[0] = params[2];
            out[0] = 0;
            out[1] = params[3];
            out[2] = params[4];

            ROS_DEBUG_STREAM("ZRollPitch: Optimized After " << out[0] << " " << out[1] << " " << out[2]);
        }
        else{
            out[0] = t[2];
            out[1] = rpy[0];
            out[2] = rpy[1];

            ROS_DEBUG_STREAM("ZRollPitch: NOT Optimized After " << out[0] << " " << out[1] << " " << out[2]);
        }
    }


    void pagslam::matchFeatures(const std::vector<StalkFeature::Ptr> &stalkFeatures, const std::vector<StalkFeature::Ptr> &mapStalkFeatures, std::vector<int> &matchIndices)
    {
        size_t stalkCounter = 0;
        // size_t numMatch = 0;

        for (const auto &cf : stalkFeatures){
            // find closest model in map
            Scalar bestDist = 100;
            StalkFeature::Ptr bestStalk = mapStalkFeatures[0];

            size_t bestKey = 0;
            size_t key = 0;

            for (int j = 0; j < mapStalkFeatures.size(); ++j){
                StalkFeature::Ptr mf = mapStalkFeatures[j];
                // Scalar d = stalkDistance(cf->root, mf->root);
                Scalar d = stalkDistance(cf->centroid, mf->centroid);

                if (d < bestDist){
                    bestDist = d;
                    bestStalk = mf;
                    bestKey = key;
                }
                ++key;
            }



            // create feature matches according to model association
            if (bestDist < AddNewStalkThreshDist_){             // Case1: Matched with existing landmark
                cout << "MATCHED: " << stalkCounter << ": " << bestDist << endl;
                matchIndices[stalkCounter] = bestKey;
                // numMatch++;

            }
            else if (bestDist < SkipStalkThreshDist_){    
                matchIndices[stalkCounter] = -2;
                cout << "SKIP: " << stalkCounter << ": " << bestDist << endl;
            }
            else{                                               // Case2: Too far to register to the existing landmark
                cout << "NEW: " << stalkCounter << ": " << bestDist << endl;
            }
            stalkCounter++;
        }
        // ROS_DEBUG_STREAM("Num map stalk matches: " << numMatch);
    }


    bool pagslam::runPagslam(PagslamInput &in, PagslamOutput &out)
    {
        std::vector<int> matchIndices(in.stalkFeatures.size(), -1);  // each element: initialized to -1.
        bool success = true;

        PagslamInput in_proj(in);
        PagslamInput in_final(in);
        // PagslamInput in_before_opt(in);

        out.T_Map_Curr = in_proj.poseEstimate;

        // Matches will be all -1
        out.matches = matchIndices;
        out.stalks = in_proj.stalkFeatures;
        out.ground = in_proj.groundFeature;

        if (firstScan_){
            // cout << "======================================" << endl;
            // projectFeatures(in_proj.poseEstimate, in_proj.groundFeature, in_proj.stalkFeatures);
            // prevGroundFeatures_.push_back(in_proj.groundFeature);
            // cout << "======================================" << endl;

            // // First run, pose will be same as odom
            // out.T_Map_Curr = in_proj.poseEstimate;

            // // Matches will be all -1
            // out.matches = matchIndices;
            // out.stalks = in_proj.stalkFeatures;
            // out.ground = in_proj.groundFeature;

            if (in_proj.groundFeature.cloud->size() != 0 && in_proj.stalkFeatures.size() != 0){
                prevGroundFeatures_.push_back(in_proj.groundFeature);
                firstScan_ = false;
            }
        }
        else{
            // cout << in_proj.groundFeature.cloud->size() << " " << in_proj.stalkFeatures.size() << endl;
            // if (in_proj.groundFeature.cloud->size() == 0 || in_proj.stalkFeatures.size() == 0){
                // ROS_WARN("No ground or landmark models found");
            if (in_proj.stalkFeatures.size() == 0){
                ROS_WARN("Only ground model found");
            }

            if (in_proj.groundFeature.cloud->size() == 0 && in_proj.stalkFeatures.size() == 0){
                ROS_WARN("No ground and landmark models found");
                return false;
            }
            if (!in_proj.mapStalkFeatures.size()){
                ROS_WARN("Not the first scan but map is empty! Ignoring Message");

                // // First run, pose will be same as odom
                // out.T_Map_Curr = in_proj.poseEstimate;

                // // Matches will be all -1
                // out.matches = matchIndices;
                // out.stalks = in_proj.stalkFeatures;
                // out.ground = in_proj.groundFeature;
                // return true;
                return false;
            }
            ROS_WARN("Run P-AgSLAM");

            // matching models
            ROS_DEBUG_STREAM("============== BEFORE OPT =================");
            std::vector<FeatureMatch<StalkFeature::Ptr>> stalkMatches = matchStalks(in_proj.poseEstimate, in_proj.stalkFeatures, in_proj.mapStalkFeatures, stalkMatchThresh_);

            ROS_DEBUG_STREAM("Num Stalk feature matches: " << stalkMatches.size());

            // Optimize Pose
            // We do some checks to make sure we have enough information to perform pose estimation
            SE3 T_Delta = SE3();
            SE3 currPoseTf = in_proj.poseEstimate;

            bool stalkCheck = (in_proj.stalkFeatures.size() > 0) && (stalkMatches.size() >= minStalkMatches_);
            // bool stalkCheck = (in_proj.stalkFeatures.size() > 0);
            success = TwoStepOptimizePose(in_proj, stalkCheck, stalkMatches, currPoseTf);

            ROS_DEBUG_STREAM("\n---------- OPTIMIZATION INITIAL OUTPUT -----------------\n"
                       << in_proj.poseEstimate.matrix()
                       << "\n-----------------------------------------------------\n");

            ROS_DEBUG_STREAM("\n---------- OPTIMIZATION POSE OUTPUT -----------------\n"
                       << currPoseTf.matrix()
                       << "\n-----------------------------------------------------\n");

            ROS_DEBUG_STREAM("============== AFTER OPT =================");
            // projectFeatures(in_proj.poseEstimate, in_before_opt.groundFeature, in_before_opt.stalkFeatures);

            // int n = 0;
            // for (auto &stalk : in_before_opt.stalkFeatures){
            //     if (n == 0){
            //         projectStalk(in_proj.poseEstimate, stalk);
            //         n = n + 1;
            //     }
            // }
            // cout << "!!!!!!!!!!!!!!!!!!!!!!!" << endl;

            projectFeatures(currPoseTf, in_final.groundFeature, in_final.stalkFeatures);
            // cout << "???????????????????????" << endl;
            // for (auto &stalk : in_final.stalkFeatures){
            //     projectStalk(currPoseTf, stalk);
            // }

            matchFeatures(in_final.stalkFeatures, in.mapStalkFeatures, matchIndices);

            prevGroundFeatures_.push_back(in_final.groundFeature);

            out.matches = matchIndices;
            out.T_Map_Curr = currPoseTf;
            out.T_Delta = T_Delta;
            out.ground = in_final.groundFeature;
            out.stalks = in_final.stalkFeatures;
        }
        return success;
    }
}   // namespace pagslam
