
#include "base_slam/map.hpp"
#include "base_slam/feature.hpp"


namespace base_slam {

    Map::KeyFrameType Map::getAllKeyFrames()
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        return keyframes_;
    }

    Map::KeyFrameType Map::getActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        return active_keyframes_;
    }

    Map::LandMarkType Map::getAllPoints()
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        return landmarks_;
    }

    Map::LandMarkType Map::getActivePoints()
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        return active_landmarks_;
    }

    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;
        //if not added, add the frame
        if(keyframes_.find(frame->keyframe_id_) == keyframes_.end())
        {
            keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        }else {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        if(active_keyframes_.size() > nactive_keyframe)
        {
            removeOldKeyFrame();
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr mpoint)
    {
        if(landmarks_.find(mpoint->id_) == landmarks_.end())
        {
            landmarks_.insert(std::make_pair(mpoint->id_, mpoint));
            active_landmarks_.insert(std::make_pair(mpoint->id_, mpoint));
        } else {
            landmarks_[mpoint->id_] = mpoint;
            active_landmarks_[mpoint->id_] = mpoint;
        }
    }

    void Map::removeOldKeyFrame()
    {
        if(current_frame_ == nullptr) return;

        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto twc = current_frame_->getPose().inverse();
        for(auto& kf: active_keyframes_)
        {
            if(kf.second == current_frame_) continue;
            auto dis = (kf.second->getPose() * twc).log().norm();
            if(dis > max_dis)
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if(dis < min_dis)
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2;
        Frame::Ptr frame_to_remove = nullptr;
        if(min_dis < min_dis_th)
        {
            frame_to_remove = keyframes_.at(min_kf_id);
        }else{
            frame_to_remove = keyframes_.at(max_kf_id);
        }

        std::cout << "Keyframe removed: " << frame_to_remove->keyframe_id_ << std::endl;
        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        for(auto feat: frame_to_remove->left_features_)
        {
            auto mp = feat->mappoint_ptr_.lock();
            if(mp)
            {
                mp->removeObservation(feat);
            }
        }
        for(auto feat: frame_to_remove->right_features_)
        {
            if(feat == nullptr) continue;
            auto mp = feat->mappoint_ptr_.lock();
            if(mp)
            {
                mp->removeObservation(feat);
            }
        }

        cleanMap();
    }

    void Map::cleanMap()
    {
        int lmarks_removed = 0;
        for(auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();)
        {
            if (iter->second->obsvtns_count_ == 0)
            {
                iter = active_landmarks_.erase(iter);
                lmarks_removed++;
            } else {
                ++iter;
            }
        }
        std::cout << "Removed active landmarks: " << lmarks_removed << std::endl;
    }
}
