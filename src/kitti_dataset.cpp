
#include "base_slam/kitti_dataset.hpp"


namespace base_slam {

    bool KittiDataset::init()
    {
        std::ifstream fin(dset_path_ + "/calib.txt");
        if(!fin)
        {
            std::cout << "File not found: " << dset_path_ + "/calib.txt" << std::endl;
            return false;
        }
        for(int i = 0; i < 4; ++i)
        {
            char camera_names[3];
            for(int k = 0; k < 3; ++k) fin >> camera_names[k];
            double pdata[12];
            for(int k = 0; k < 12; ++k) fin >> pdata[k];
            Mat33 K;
            K << pdata[0], pdata[1], pdata[2],
                 pdata[4], pdata[5], pdata[6],
                 pdata[8], pdata[9], pdata[10];
            Vec3 t;
            t << pdata[3], pdata[7], pdata[11];

            t = K.inverse() * t;
            K = K * 0.5;

            Camera::Ptr new_cam(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3(SO3(), t)));
            cameras_.emplace_back(new_cam);
            std::cout << "Camera: " << i << " extrinsics: " << t.transpose() << std::endl;
        }
        fin.close();
        current_image_idx_ = 0;
        return true;
    }

    Frame::Ptr KittiDataset::nextFrame()
    {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat limg, rimg;
        std::string fname = (fmt % dset_path_ % 0 % current_image_idx_).str();
        limg = cv::imread(fname, cv::IMREAD_GRAYSCALE);
        rimg = cv::imread((fmt % dset_path_ % 1 % current_image_idx_).str(), cv::IMREAD_GRAYSCALE);
        if(limg.data == nullptr || rimg.data == nullptr)
        {
            std::cout << "image not found: " << current_image_idx_ << std::endl;
            return nullptr;
        }

        cv::Mat limg_rszd, rimg_rszd;
        cv::resize(limg, limg_rszd, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(rimg, rimg_rszd, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        auto nframe = Frame::createFrame();
        nframe->limg_ = limg_rszd;
        nframe->rimg_ = rimg_rszd;
        nframe->fname = fname;
        ++current_image_idx_;
        return nframe;
    }

    Camera::Ptr KittiDataset::getCamera(int cam_id) const
    {
        return cameras_.at(cam_id);
    }
}
