#ifndef _KITTI_DATASET_HPP_
#define _KITTI_DATASET_HPP_

#include <fstream>
#include <boost/format.hpp>

#include "base_slam/dataset.hpp"
#include "base_slam/common.hpp"
#include "base_slam/camera.hpp"
#include "base_slam/frame.hpp"


namespace base_slam {

    class KittiDataset: public Dataset {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<KittiDataset> Ptr;
            KittiDataset(const std::string &dset_path): Dataset(dset_path) {};

            virtual bool init() override;

            virtual Frame::Ptr nextFrame() override ;

            virtual Camera::Ptr getCamera(int cam_id) const override;

        protected:
            std::string dset_path_;
            int current_image_idx_;

            std::vector<Camera::Ptr> cameras_;
    };

}


#endif
