
#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#include "base_slam/common.hpp"


namespace base_slam {
    class Config{
        private:
            static std::shared_ptr<Config> config_;
            cv::FileStorage fileStorage_;
            Config(){};

        public:
            ~Config();

            static bool setParameterFile(const std::string &fname);

            template <typename T>
            static T getConfigValue(const std::string &key)
            {
                T val = static_cast<T>(Config::config_->fileStorage_[key]);
                return val;
            }
    };
}

#endif
