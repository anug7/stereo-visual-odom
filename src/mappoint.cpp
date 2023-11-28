
#include "base_slam/mappoint.hpp"
#include "base_slam/feature.hpp"


namespace base_slam {

    MapPoint::Ptr MapPoint::createMapPoint()
    {
        static unsigned long factory_id_ = 0;
        MapPoint::Ptr mp(new MapPoint());
        mp->id_ = factory_id_++;
        return mp;
    }

    MapPoint::MapPoint(long id, const Vec3 pos)
             :id_(id), pos_(pos){}

    void MapPoint::addObservations(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> ul(acc_lock_);
        obsvtns_.push_back(feature);
        obsvtns_count_++;
    }

    std::list<std::weak_ptr<Feature>> MapPoint::getObservations()
    {
        std::unique_lock<std::mutex> ul(acc_lock_);
        return this->obsvtns_;
    }

    void MapPoint::removeObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> ul(acc_lock_);
        for(auto iter = obsvtns_.begin(); iter != obsvtns_.end(); iter++)
        {
            if(iter->lock() == feature)
            {
                obsvtns_.erase(iter);
                feature->mappoint_ptr_.reset();
                obsvtns_count_--;
                break;
            }
        }
    }

}
