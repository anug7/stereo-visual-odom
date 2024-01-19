#ifndef _FRAME_HPP_
#define _FRAME_HPP_

#include "common.hpp"
#include <Eigen/src/Core/util/Memory.h>
#include <mutex>

namespace base_slam {
    struct MapPoint;
    struct Feature;

    struct Frame {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Frame> Ptr;

            unsigned long id_ = 0;
            unsigned long keyframe_id_ = 0;
            bool is_keyframe = false;
            double timestamp = 0.;
            std::string fname = "";
            SE3 cam_pose_;
            std::mutex acc_lock_; //pose will be updated from backend as well as frontend
            cv::Mat limg_, rimg_;
            bool is_depth_img_;
            //Ref to list of features in the frame
            //Left is the base
            std::vector<std::shared_ptr<Feature>> left_features_, right_features_;

            Frame(){}

            Frame(long id, double ts, const SE3 &pose, const cv::Mat &left, const cv::Mat &right_or_depth, bool depth_img=false);

            SE3 getPose()
            {
                std::unique_lock<std::mutex> lock(acc_lock_);
                return cam_pose_;
            }

            void setPose(const SE3 pose)
            {
                std::unique_lock<std::mutex> lock(acc_lock_);
                cam_pose_ = pose;
            }

            void setKeyFrame();
            static std::shared_ptr<Frame> createFrame();
    };
}

#endif
