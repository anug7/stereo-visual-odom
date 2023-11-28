
#ifndef _FRONTEND_HPP_
#define _FRONTEND_HPP_

#include <opencv2/features2d.hpp>

#include "base_slam/common.hpp"
#include "base_slam/config.hpp"
#include "base_slam/feature.hpp"
#include "base_slam/frame.hpp"
#include "base_slam/map.hpp"
#include "base_slam/backend.hpp"
#include "base_slam/camera.hpp"
#include "base_slam/viewer.hpp"
#include "base_slam/g2o_types.hpp"
#include "base_slam/algorithm.hpp"


namespace base_slam {

    enum class FrontEndStatus {
        INIT,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };


    class FrontEnd {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<FrontEnd> Ptr;

            FrontEnd();

            bool addFrame(Frame::Ptr frame);

            void setMap(Map::Ptr map);

            void setBackend(Backend::Ptr backend);

            void setViewer(Viewer::Ptr viewer);

            FrontEndStatus getStatus() const;

            void setCamera(Camera::Ptr left, Camera::Ptr right);

        private:
            bool track();

            bool reset();

            int trackLastFrame();

            //Estimates pose of current frame
            int estimateCurrentPose();

            //consider current frame as keyframe and insert it to backend
            bool insertKeyFrame();

            //Try initializing of frontend with stereo images saved in current_frame_
            bool stereoInit();

            //Detect features in right image of current frame
            int detectFeatures();

            //Find corresponsing features in right image
            int findFeaturesInRight();

            //Initialize map with single image
            bool buildInitMap();

            //Triangulate 2D points in current frame
            int triangulateNewPoints();

            //set features in keyfram as new observation of map points
            void setObservationsForKeyPoints();

            FrontEndStatus status_ = FrontEndStatus::INIT;

            Frame::Ptr current_frame_ = nullptr;
            Frame::Ptr last_frame_ = nullptr;
            Camera::Ptr left_cam_ = nullptr;
            Camera::Ptr right_cam_ = nullptr;

            Map::Ptr map_ = nullptr;
            Backend::Ptr backend_ = nullptr;
            Viewer::Ptr viewer_ = nullptr;

            SE3 relative_motion_;
            int tracking_inliers_ = 0;

            int nfeats_ = 200;
            int nfeats_init_ = 100;
            int nfeats_tracking_ = 50;
            int nfeats_tracking_bad_ = 20;
            int nfeats_for_keyframe_ = 80;

            cv::Ptr<cv::GFTTDetector> gftt_;
    };

}

#endif
