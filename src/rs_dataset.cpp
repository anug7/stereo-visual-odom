
#include "base_slam/rs_dataset.hpp"


namespace base_slam {

    bool RsDataset::init()
    {
        Mat33 K;
        K << 391, 0, 326,
             0, 391, 242,
             0, 0, 1;
        Vec3 t;
        t << 0, 0, 0;

        t = K.inverse() * t;
        Camera::Ptr new_cam(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3(SO3(), t)));
        cameras_.emplace_back(new_cam);
        std::cout << "Camera: 0" << " extrinsics: " << t.transpose() << std::endl;
        current_image_idx_ = 1;
        return true;
    }

    Frame::Ptr RsDataset::nextFrame()
    {
        boost::format fmt1("%s/clr/%05d.png"), fmt2("%s/depth/%05d.png");
        cv::Mat limg, rimg;
        std::string fname = (fmt1 % dset_path_ % current_image_idx_).str(),
                    fname2 = (fmt2 % dset_path_ % current_image_idx_).str();
        limg = cv::imread(fname, cv::IMREAD_GRAYSCALE);
        rimg = cv::imread(fname2, -1);
        if(limg.data == nullptr || rimg.data == nullptr)
        {
            std::cout << "image not found: " << current_image_idx_ << std::endl;
            return nullptr;
        }
        auto nframe = Frame::createFrame();
        nframe->limg_ = limg;
        nframe->rimg_ = rimg;
        nframe->fname = fname;
        nframe->is_depth_img_ = true;
        ++current_image_idx_;
        return nframe;
    }

    Camera::Ptr RsDataset::getCamera(int cam_id) const
    {
        return cameras_.at(cam_id);
    }
}
