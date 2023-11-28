
#ifndef _MAPPOINT_HPP_
#define _MAPPOINT_HPP_

#include "common.hpp"

namespace base_slam {

    struct Frame;
    struct Feature;

    struct MapPoint {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<MapPoint> Ptr;

            unsigned long id_ = 0;
            bool is_outlier_ = false;
            std::mutex acc_lock_;
            Vec3 pos_ = Vec3::Zero();
            int obsvtns_count_ = 0;
            std::list<std::weak_ptr<Feature>> obsvtns_;

            MapPoint(){}

            MapPoint(long id, const Vec3 pos);

            void setPos(const Vec3 &pos)
            {
                std::unique_lock<std::mutex> ul(acc_lock_);
                pos_ = pos;
            }

            Vec3 getPos()
            {
                std::unique_lock<std::mutex> ul(acc_lock_);
                return pos_;
            }

            void addObservations(std::shared_ptr<Feature> feature);
            void removeObservation(std::shared_ptr<Feature> feature);

            std::list<std::weak_ptr<Feature>> getObservations();

            static MapPoint::Ptr createMapPoint();
    };

}

#endif
