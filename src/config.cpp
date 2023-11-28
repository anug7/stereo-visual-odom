#include "base_slam/config.hpp"


namespace base_slam {

    std::shared_ptr<Config> Config::config_ = nullptr;

    bool Config::setParameterFile(const std::string &fname)
    {
        if(config_ == nullptr)
        {
            config_ = std::shared_ptr<Config>(new Config);
        }

        config_->fileStorage_ = cv::FileStorage(fname.c_str(), cv::FileStorage::READ);
        if(!config_->fileStorage_.isOpened())
        {
            config_->fileStorage_.release();
            return false;
        }
        return true;
    }

    Config::~Config()
    {
        if(fileStorage_.isOpened())
        {
            fileStorage_.release();
        }
    }
}
