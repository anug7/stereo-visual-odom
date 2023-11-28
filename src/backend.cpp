#include "base_slam/backend.hpp"
#include <robust_kernel_impl.h>


namespace base_slam {

    Backend::Backend()
    {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::backendLoop, this));
        std::cout << "Backend created" << std::endl;
    }

    void Backend::updateMap()
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        map_update_.notify_one();
    }

    void Backend::stop()
    {
        backend_running_.store(false);
        map_update_.notify_one();
        if(backend_thread_.joinable()) backend_thread_.join();
    }

    void Backend::backendLoop()
    {
        while(backend_running_.load())
        {
            std::unique_lock<std::mutex> lc(acc_lock_);
            map_update_.wait(lc);

            Map::KeyFrameType  active_keyframes = map_->getActiveKeyFrames();
            Map::LandMarkType active_lmarks = map_->getActivePoints();

            optimize(active_keyframes, active_lmarks);
        }
    }

    void Backend::optimize(Map::KeyFrameType &kfs, Map::LandMarkType &lmarks)
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
            );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        std::map<unsigned long, VertexPose*> vertices;
        unsigned long max_kf_id = 0;

        for(auto &kf_pair: kfs)
        {
            auto kf = kf_pair.second;
            VertexPose *vpose = new VertexPose();
            vpose->setId(kf->keyframe_id_);
            vpose->setEstimate(kf->getPose());
            optimizer.addVertex(vpose);

            if(kf->keyframe_id_ > max_kf_id) max_kf_id = kf->keyframe_id_;

            vertices.emplace(kf->keyframe_id_, vpose);
        }

        std::map<unsigned long, VertexXYZ*> vlmarks;
        Mat33 K = lcam_->K();
        SE3 lext = lcam_->getPose();
        SE3 rext = rcam_->getPose();

        //Edges
        int index = 1;
        //Robust kernel threshold. From literature
        double chi2_th = 5.991;
        std::map<EdgeProjection*, Feature::Ptr> edges_to_features;

        for(auto &mp_pair: lmarks)
        {
            if(mp_pair.second->is_outlier_) continue;
            unsigned long lmark_id = mp_pair.second->id_;
            auto obsvtns = mp_pair.second->getObservations();
            for(auto &obs: obsvtns)
            {
                if(obs.lock() == nullptr) continue;
                auto feat = obs.lock();
                if(feat->is_outlier_ || feat->frame_ptr_.lock() == nullptr) continue;

                auto frame = feat->frame_ptr_.lock();
                EdgeProjection *edge = nullptr;
                if(feat->is_on_limg_)
                {
                    edge = new EdgeProjection(K, lext);
                }else
                {
                    edge = new EdgeProjection(K, rext);
                }

                if(vlmarks.find(lmark_id) == vlmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(mp_pair.second->getPos());
                    v->setId(lmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vlmarks.emplace(lmark_id, v);
                    optimizer.addVertex(v);
                }
                //Create edge between Camera and the landmark
                if(vertices.find(frame->keyframe_id_) != vertices.end() &&
                   vlmarks.find(lmark_id) != vlmarks.end())
                {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_)); //Cam pose;
                    edge->setVertex(1, vlmarks.at(lmark_id)); // Landmark location
                    edge->setMeasurement(toVec2(feat->pos_.pt));
                    edge->setInformation(Mat22::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edges_to_features.emplace(edge, feat);
                    optimizer.addEdge(edge);
                    index++;
                }else
                {
                    //not valid pair found
                    delete edge;
                }
            }
        }

        optimizer.initializeOptimization();
        //optimizer.setVerbose(true);
        optimizer.optimize(10);
        int cinliers = 0, coutliers = 0;
        int iters = 0;

        while(iters < 5)
        {
            cinliers = coutliers = 0;
            for(auto &ef: edges_to_features)
            {
               if(ef.first->chi2() > chi2_th)
               {
                   ++coutliers;
               }else
               {
                   ++cinliers;
               }
            }
            double inlier_ratio = cinliers / double(cinliers + coutliers);
            if(inlier_ratio > 0.5) break;
            else
            {
                chi2_th *= 2;
                ++iters;
            }
        }

        for(auto &ef: edges_to_features)
        {
            if(ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                ef.second->mappoint_ptr_.lock()->removeObservation(ef.second);
            }else
            {
                ef.second->is_outlier_ = false;
            }
        }
        std::cout << "Inlier/Outlier ratio in optimization: " << cinliers << "/" << coutliers << std::endl;

        //Update camera pose after optimizations
        for(auto &v: vertices)
        {
            kfs.at(v.first)->setPose(v.second->estimate());
        }

        //Update landmarks after optimizations
        for(auto &vlmark: vlmarks)
        {
            lmarks.at(vlmark.first)->setPos(vlmark.second->estimate());
        }
    }

    void Backend::setCameras(Camera::Ptr lcam, Camera::Ptr rcam)
    {
        lcam_ = lcam;
        rcam_ = rcam;
    }

    void Backend::setMap(Map::Ptr map_)
    {
        this->map_ = map_;
    }

}
