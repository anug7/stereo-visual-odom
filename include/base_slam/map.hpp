#ifndef _MAP_HPP_
#define _MAP_HPP_

#include "base_slam/common.hpp"
#include "base_slam/frame.hpp"
#include "base_slam/mappoint.hpp"


namespace base_slam {

    class Map{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            typedef std::shared_ptr<Map> Ptr;
            typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandMarkType;
            typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFrameType;

            Map(){}

            void insertKeyFrame(Frame::Ptr frame);
            void insertMapPoint(MapPoint::Ptr mpoint);

            LandMarkType getAllPoints();
            LandMarkType getActivePoints();
            KeyFrameType getAllKeyFrames();
            KeyFrameType getActiveKeyFrames();

            void cleanMap();
        private:
            void removeOldKeyFrame();

            std::mutex acc_lock_;
            LandMarkType landmarks_;        //list of all landmarks observed so far
            LandMarkType active_landmarks_;  //list of active landmarks
            KeyFrameType keyframes_;        //all keyframes detected
            KeyFrameType active_keyframes_;  //list of active keyframe used for backend

            Frame::Ptr current_frame_ = nullptr;

            int nactive_keyframe = 7; //Number of active keyframe used for backend
    };

}

#endif
