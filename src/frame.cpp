
#include "base_slam/frame.hpp"

namespace base_slam {

    Frame::Frame(long id, double ts, const SE3 &pose, const cv::Mat &left, const cv::Mat &right_or_depth, bool depth_img)
          :id_(id), timestamp(ts), cam_pose_(pose), limg_(left), rimg_(right_or_depth)
    {
        this->is_depth_img_ = depth_img;
    }

    void Frame::setKeyFrame()
    {
        static unsigned long factory_kframe_ids_ = 0;
        this->is_keyframe = true;
        this->keyframe_id_ = factory_kframe_ids_++;
    }

    Frame::Ptr Frame::createFrame()
    {
        static unsigned long factory_ids_ = 0;
        Frame::Ptr frame(new Frame());
        frame->id_ = factory_ids_++;
        return frame;
    }
}

