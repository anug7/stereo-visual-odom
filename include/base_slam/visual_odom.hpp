#ifndef _VISUAL_ODOM_HPP_
#define _VISUAL_ODOM_HPP_

#include <iostream>
#include <chrono>

#include "base_slam/common.hpp"
#include "base_slam/dataset.hpp"
#include "base_slam/viewer.hpp"
#include "base_slam/frontend.hpp"
#include "base_slam/backend.hpp"
#include "base_slam/config.hpp"


namespace base_slam
{
    class VisualOdom
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<VisualOdom> Ptr;

            VisualOdom(std::string &config_path);
            ~VisualOdom();
            bool init();

            void run();

            bool step();

            FrontEndStatus getFrontEndStatus() const;

        private:
            bool inited = false;
            std::string cfg_path_;

            FrontEnd::Ptr frontend_ = nullptr;
            Backend::Ptr backend_ = nullptr;
            Map::Ptr map_ = nullptr;
            Viewer::Ptr viewer_ = nullptr;

            Dataset *dataset_ = nullptr;
    };
}


#endif
