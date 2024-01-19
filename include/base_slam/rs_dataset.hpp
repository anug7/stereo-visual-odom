#ifndef _RS_DATASET_HPP_
#define _RS_DATASET_HPP_

#include "base_slam/dataset.hpp"

namespace base_slam {

  class RsDataset: public Dataset {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<RsDataset> Ptr;
            RsDataset(std::string ddir): Dataset(ddir){}
            virtual bool init() override;
            virtual Frame::Ptr nextFrame() override;
            virtual Camera::Ptr getCamera(int cam_id=0) const override;
    };
}

#endif
