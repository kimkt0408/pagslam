#include <mapManager.h>

MapManager::MapManager(const float searchRadius){
// MapManager::MapManager(){
    sqSearchRadius = searchRadius*searchRadius;
    searchThreshold = searchRadius;
    // mapStalkHits_ = 2; // SIM
    mapStalkHits_ = 2; // ACRE
    landmarks_.reset(new CloudT);
}


std::vector<StalkFeature::Ptr> MapManager::getMap() 
{
    std::vector<StalkFeature::Ptr> map;

    for(auto i = 0; i < stalkModels_.size(); ++i){
        if(stalkHits_[i] >= mapStalkHits_){
            map.push_back(stalkModels_[i]);
        }
    }
    return map;
}


void MapManager::getSubmap(const SE3& pose, std::vector<StalkFeature::Ptr>& submap)
{
    if(landmarks_->size() == 0) return;

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(landmarks_);
    std::vector<int> pointIdxKNNSearch;
    std::vector<float> pointKNNSquaredDistance;
    PointT searchPoint;

    // Search for nearby trees
    searchPoint.x = pose.translation()[0];
    searchPoint.y = pose.translation()[1];
    searchPoint.z = pose.translation()[2];

    // if(kdtree.nearestKSearch(searchPoint, 3, pointIdxKNNSearch, pointKNNSquaredDistance) > 0){
    if(kdtree.nearestKSearch(searchPoint, 30, pointIdxKNNSearch, pointKNNSquaredDistance) > 0){
        int idx_count = 0;
        auto map_size = stalkModels_.size();

        for(auto map_idx : pointIdxKNNSearch){
            // cout << "map: " << map_size << " " << map_idx << endl;

            if(map_size - map_idx < 1e4){
                matchesMap.insert(std::pair<int, int>(idx_count, map_idx));
                submap.push_back(stalkModels_[map_idx]);
                idx_count++;
            }
        }
        // for(size_t i = 0; i < pointIdxKNNSearch.size(); ++i){
        //     // Check if the squared distance is less than sqSearchRadius
        //     if(pointKNNSquaredDistance[i] <= searchThreshold) {
        //         int map_idx = pointIdxKNNSearch[i];

        //         if(map_size - map_idx < 1e4){
        //             matchesMap.insert(std::pair<int, int>(idx_count, map_idx));
        //             submap.push_back(stalkModels_[map_idx]);
        //             idx_count++;
        //         }
        //     }
        // }

    } 
    else {
        ROS_INFO("Not enough stalks around pose: Total: %ld", pointIdxKNNSearch.size());
    }
}


void MapManager::updateMap(std::vector<StalkFeature::Ptr>& stalks, const std::vector<int>& matches){
    
    landmarks_.reset(new CloudT);
    stalkModels_.clear();
    stalkHits_.clear();

    size_t i = 0;

    // cout << "MAP SIZE: " << stalks.size() << endl;
    // cout << "MAP SIZE: " << landmarks_.size() << endl;

    for(auto const& stalk : stalks){

        PointT pt;
        // pt.x = stalk->root(0); pt.y = stalk->root(1); pt.z = stalk->root(2);

        // landmarks_->push_back(pt);
        // stalkModels_.push_back(stalk);
        // stalkHits_.push_back(1);

        if(matches[i] == -1){        
            pt.x = stalk->root(0); pt.y = stalk->root(1); pt.z = stalk->root(2);
            landmarks_->push_back(pt);
            stalkModels_.push_back(stalk);
            stalkHits_.push_back(1);
        }
        else if (matches[i] != -2) {
            int matchIdx = matchesMap[matches[i]];
            stalkHits_[matchIdx] += 1;
            
            StalkFeature::Ptr stalk_tmp = stalkModels_[matchIdx];
            stalk_tmp->cloud.insert(stalk_tmp->cloud.end(), stalk->cloud.begin(), stalk->cloud.end());

            if (stalk_tmp->root(2) > stalk->root(2)){
                stalk_tmp->root = stalk->root;
            }

            if (stalk_tmp->top(2) < stalk->top(2)){
                stalk_tmp->top = stalk->top;
            }

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(stalk_tmp->cloud, centroid);

            // Compute the point closest to the origin on the line
            Eigen::Vector3f line_origin = centroid.head<3>();

            stalk_tmp->centroid = line_origin;

            // Compute the eigenvalues and eigenvectors of the covariance matrix
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(stalk_tmp->cloud, centroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

        
            // Get the direction of the smallest eigenvalue, which is the line direction
            Eigen::Vector3f line_direction = solver.eigenvectors().col(2);
            if (line_direction(2) < 0) {
                line_direction *= -1;
            }
                
            stalk_tmp->direction = line_direction;

            float cloud_ratio = static_cast<float>(stalk->cloud.size()) / (stalk->cloud.size() + stalk_tmp->cloud.size());
            // float cloud_ratio = stalk->cloud.size() % (stalk->cloud.size() + stalk_tmp->cloud.size());

            pt.x = cloud_ratio * stalk->root(0) + (1-cloud_ratio) * stalk_tmp->root(0); 
            pt.y = cloud_ratio * stalk->root(1) + (1-cloud_ratio) * stalk_tmp->root(1); 
            pt.z = cloud_ratio * stalk->root(2) + (1-cloud_ratio) * stalk_tmp->root(2); 
            
            // cout << "!!!!: " << stalk->root(0) << " " << stalk_tmp->root(0) << " " << pt.x << endl
            // << stalk->root(1) << " " << stalk_tmp->root(1) << " " << pt.y <<  endl
            // << stalk->root(2) << " " << stalk_tmp->root(2) << " " << pt.z << endl;

            // cout << "=====================================" << endl;

            landmarks_->points[matchIdx] = pt;
            stalkModels_[matchIdx] = stalk_tmp; 
        }
        i++;
    }
    matchesMap.clear();
}

// void MapManager::updateMap(std::vector<StalkFeature::Ptr>& stalks, const std::vector<int>& matches){
//     size_t i = 0;
//     for(auto const& stalk : stalks){

//         PointT pt;

//         if(matches[i] == -1){        
//             pt.x = stalk->root(0); pt.y = stalk->root(1); pt.z = stalk->root(2);
//             landmarks_->push_back(pt);
//             stalkModels_.push_back(stalk);
//             stalkHits_.push_back(1);
//         } 
//         else {
//             int matchIdx = matchesMap[matches[i]];
//             stalkHits_[matchIdx] += 1;
            
//             StalkFeature::Ptr stalk_tmp = stalkModels_[matchIdx];
//             stalk_tmp->cloud.insert(stalk_tmp->cloud.end(), stalk->cloud.begin(), stalk->cloud.end());

//             // if (stalk_tmp->root(2) > stalk->root(2)){
//             //     stalk_tmp->root = stalk->root;
//             // }

//             if (stalk_tmp->top(2) < stalk->top(2)){
//                 stalk_tmp->top = stalk->top;
//             }

//             Eigen::Vector4f centroid;
//             pcl::compute3DCentroid(stalk_tmp->cloud, centroid);

//             // Compute the point closest to the origin on the line
//             Eigen::Vector3f line_origin = centroid.head<3>();

//             stalk_tmp->centroid = line_origin;

//             // Compute the eigenvalues and eigenvectors of the covariance matrix
//             Eigen::Matrix3f covariance;
//             pcl::computeCovarianceMatrixNormalized(stalk_tmp->cloud, centroid, covariance);
//             Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

        
//             // Get the direction of the smallest eigenvalue, which is the line direction
//             Eigen::Vector3f line_direction = solver.eigenvectors().col(2);
//             if (line_direction(2) < 0) {
//                 line_direction *= -1;
//             }
                
//             stalk_tmp->direction = line_direction;

//             float cloud_ratio = static_cast<float>(stalk->cloud.size()) / (stalk->cloud.size() + stalk_tmp->cloud.size());
//             // float cloud_ratio = stalk->cloud.size() % (stalk->cloud.size() + stalk_tmp->cloud.size());

//             pt.x = cloud_ratio * stalk->root(0) + (1-cloud_ratio) * stalk_tmp->root(0); 
//             pt.y = cloud_ratio * stalk->root(1) + (1-cloud_ratio) * stalk_tmp->root(1); 
//             pt.z = cloud_ratio * stalk->root(2) + (1-cloud_ratio) * stalk_tmp->root(2); 
            
//             // cout << "!!!!: " << stalk->root(0) << " " << stalk_tmp->root(0) << " " << pt.x << endl
//             // << stalk->root(1) << " " << stalk_tmp->root(1) << " " << pt.y <<  endl
//             // << stalk->root(2) << " " << stalk_tmp->root(2) << " " << pt.z << endl;

//             // cout << "=====================================" << endl;

//             landmarks_->points[matchIdx] = pt;
//             stalkModels_[matchIdx] = stalk_tmp; 
//         }
//         i++;
//     }
//     matchesMap.clear();
// }

    
