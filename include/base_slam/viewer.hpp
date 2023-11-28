#ifndef _VIEWER_HPP_
#define _VIEWER_HPP_

#include <thread>
#include <pangolin/pangolin.h>

#include "base_slam/common.hpp"
#include "base_slam/frame.hpp"
#include "base_slam/map.hpp"
#include "base_slam/feature.hpp"

namespace base_slam {

    class Viewer{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Viewer> Ptr;

            Viewer();

            void setMap(Map::Ptr map_);

            void close();

            void addCurrentFrame(Frame::Ptr current_frame);

            void updateMap();

        private:

            void threadLoop();

            void drawFrame(Frame::Ptr frame, const float *clr);

            void drawMapPoints();

            void followCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

            cv::Mat plotFrameImage();

            Frame::Ptr current_frame_ = nullptr;
            Map::Ptr map_ = nullptr;

            std::thread viewer_thread_;
            bool is_viewer_running_ = true;

            std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
            std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
            bool map_updated_ = false;

            std::mutex acc_lock_;
    };

}


#endif
