#ifndef _DATASET_HPP_
#define _DATASET_HPP_

#include <fstream>
#include <boost/format.hpp>

#include "base_slam/common.hpp"
#include "base_slam/camera.hpp"
#include "base_slam/frame.hpp"


namespace base_slam {

    class Dataset {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Dataset> Ptr;
            Dataset(const std::string &dset_path){dset_path_ = dset_path;};

            virtual bool init() = 0;

            virtual Frame::Ptr nextFrame() = 0;

            virtual Camera::Ptr getCamera(int cam_id) const = 0;

            virtual ~Dataset(){}
        protected:
            std::string dset_path_;
            int current_image_idx_;

            std::vector<Camera::Ptr> cameras_;
    };

}
#endif
