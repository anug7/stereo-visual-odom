#ifndef _FEATURE_HPP_
#define _FEATURE_HPP_

#include "common.hpp"

namespace base_slam {
    struct Frame;
    struct MapPoint;

    struct Feature {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Feature> Ptr;

            std::weak_ptr<Frame> frame_ptr_; //access to containing frame
            cv::KeyPoint pos_; //2d coordinate in the image
            std::weak_ptr<MapPoint> mappoint_ptr_; //access to 3d point
            bool is_outlier_ = false;
            bool is_on_limg_ = true; //flag to check if it's a left image

            Feature(){}

            Feature (std::shared_ptr<Frame> frame, const cv::KeyPoint kp)
                    :frame_ptr_(frame), pos_(kp){}
    };
}


#endif
