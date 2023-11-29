#include <definitions.h>

class MapManager {
    public:
        explicit MapManager(const float searchRadius = 50);
        std::vector<StalkFeature::Ptr> getMap();
        void getSubmap(const SE3& pose, std::vector<StalkFeature::Ptr>& submap);
        void updateMap(std::vector<StalkFeature::Ptr>& stalks, const std::vector<int>& matches);
    
    private:
        float sqSearchRadius;
        float searchThreshold;
        int mapStalkHits_;

        CloudT::Ptr landmarks_;
        std::vector<StalkFeature::Ptr> stalkModels_;
        std::map<int, int> matchesMap;
        std::vector<size_t> stalkHits_;
};
