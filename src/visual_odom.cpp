
#include "base_slam/visual_odom.hpp"
#include <chrono>

namespace base_slam
{
    VisualOdom::VisualOdom(std::string &cfg_path)
    {
        cfg_path_ = cfg_path;
    }

    bool VisualOdom::init()
    {
        if(!Config::setParameterFile(cfg_path_))
        {
            return false;
        }

        dataset_ = Dataset::Ptr(new Dataset(Config::getConfigValue<std::string>("dataset_dir")));
        assert(dataset_->init() == true);

        frontend_ = FrontEnd::Ptr(new FrontEnd);
        backend_  = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->setBackend(backend_);
        frontend_->setMap(map_);
        frontend_->setViewer(viewer_);
        frontend_->setCamera(dataset_->getCamera(0), dataset_->getCamera(1));

        backend_->setMap(map_);
        backend_->setCameras(dataset_->getCamera(0), dataset_->getCamera(1));

        viewer_->setMap(map_);
        return true;
    }


    void VisualOdom::run()
    {
        int i = 100;
        while(1)
        {
            //std::cout << "VO in action" << std::endl;
            if(step() == false)
            {
                std::cout << "Stopping VO" << std::endl;
                break;
            }
        }
        backend_->stop();
        if(viewer_) viewer_->close();

        std::cout << "VO stopped" << std::endl;
    }

    bool VisualOdom::step()
    {
        Frame::Ptr nframe = dataset_->nextFrame();
        if(nframe == nullptr) return false;
        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->addFrame(nframe);
        auto t2 = std::chrono::steady_clock::now();
        auto tt = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        //std::cout << "VO frame time: " << tt.count() << std::endl;
        return true;
    }
}
