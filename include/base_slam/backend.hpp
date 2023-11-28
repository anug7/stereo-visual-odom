#ifndef _BACKEND_HPP_
#define _BACKEND_HPP_

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>


#include "base_slam/common.hpp"
#include "base_slam/frame.hpp"
#include "base_slam/feature.hpp"
#include "base_slam/camera.hpp"
#include "base_slam/map.hpp"
#include "base_slam/algorithm.hpp"
#include "base_slam/g2o_types.hpp"


namespace base_slam {

    class Backend {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Backend> Ptr;

            Backend();

            void setCameras(Camera::Ptr lcam, Camera::Ptr rcam);

            void setMap(Map::Ptr _map);

            void updateMap();

            void stop();
        private:
            void backendLoop();

            void optimize(Map::KeyFrameType &keyframes, Map::LandMarkType &lmarks);

            Map::Ptr map_;
            std::thread backend_thread_;
            std::mutex acc_lock_;

            std::condition_variable map_update_;
            std::atomic<bool> backend_running_;

            Camera::Ptr lcam_ = nullptr, rcam_ = nullptr;
    };
}

#endif
