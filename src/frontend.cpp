
#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>

#include "base_slam/algorithm.hpp"
#include "base_slam/frontend.hpp"
#include "base_slam/g2o_types.hpp"



namespace base_slam {

    FrontEnd::FrontEnd()
    {
        int nfeats = Config::getConfigValue<int>("num_features");
        //gftt_ = cv::GFTTDetector::create(1000, 0.01, 20);
        gftt_ = cv::GFTTDetector::create();
        orb_ = cv::ORB::create(nfeats);
        sift_ = cv::SIFT::create(nfeats);
        nfeats_init_ = Config::getConfigValue<int>("num_features_init");
        nfeats_ = nfeats;
    }

    bool FrontEnd::addFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;

        switch(status_)
        {
            case FrontEndStatus::INIT: {
                if(current_frame_->is_depth_img_) stereoInitRGBD();
                else stereoInit();
                break;
            }
            case FrontEndStatus::TRACKING_GOOD:
            case FrontEndStatus::TRACKING_BAD:
                track();
                break;
            case FrontEndStatus::LOST:
                reset();
                break;
        }
        last_frame_ = current_frame_;
        return true;
    }

    bool FrontEnd::track()
    {
        if(last_frame_)
        {
            //update current frame's pose from the last frame's pose
            current_frame_->setPose(relative_motion_ * last_frame_->getPose());
        }
        int ntrack_last = trackLastFrame();
        tracking_inliers_ = estimateCurrentPose();

        if(tracking_inliers_ > nfeats_tracking_)
        {
            status_ = FrontEndStatus::TRACKING_GOOD;
        }else if (tracking_inliers_ > nfeats_tracking_bad_)
        {
            status_ = FrontEndStatus::TRACKING_BAD;
        }else {
            status_ = FrontEndStatus::LOST;
        }

        insertKeyFrame();
        relative_motion_ = current_frame_->getPose() * last_frame_->getPose().inverse();
        std::cout << "Rel motion: \n" << relative_motion_.matrix() << std::endl;
        if(viewer_) {viewer_->addCurrentFrame(current_frame_);}
        return true;
    }

    int FrontEnd::trackLastFrame()
    {
        std::vector<cv::Point2f> kps_last, kps_current;
        for(auto &kp: last_frame_->left_features_)
        {
            if(kp->mappoint_ptr_.lock())
            {
                auto mp = kp->mappoint_ptr_.lock();
                auto px = left_cam_->world2Pixel(mp->pos_, current_frame_->getPose());
                kps_last.emplace_back(kp->pos_.pt);
                kps_current.emplace_back(cv::Point2f(px[0], px[1]));
            }else{
                kps_last.emplace_back(kp->pos_.pt);
                kps_current.emplace_back(kp->pos_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(last_frame_->limg_, current_frame_->limg_, kps_last, kps_current,
                                 status, error, cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        int num_good_pts = 0;
        for(size_t i = 0; i < status.size(); ++i)
        {
            if(status[i])
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->mappoint_ptr_ = last_frame_->left_features_[i]->mappoint_ptr_;
                current_frame_->left_features_.emplace_back(feat);
                num_good_pts++;
            }
        }
        std::cout << "Good points: " << num_good_pts << " in the last image" << std::endl;
        return num_good_pts;
    }

    int FrontEnd::estimateCurrentPose()
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                      std::make_unique<BlockSolverType>(
                        std::make_unique<LinearSolverType>()
                      )
              );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        //vertex
        VertexPose *vpose = new VertexPose();
        vpose->setId(0);
        vpose->setEstimate(current_frame_->getPose());
        optimizer.addVertex(vpose);

        int index = 1;
        std::vector<EdgeProjectionPoseOnly*> edges;
        std::vector<Feature::Ptr> feats;
        Mat33 K = left_cam_->K();
        for(size_t i = 0; i < current_frame_->left_features_.size(); ++i)
        {
            auto mp = current_frame_->left_features_[i]->mappoint_ptr_.lock();
            if(mp)
            {
                feats.emplace_back(current_frame_->left_features_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vpose);
                edge->setMeasurement(toVec2(current_frame_->left_features_[i]->pos_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                optimizer.addEdge(edge);
                edges.emplace_back(edge);
                ++index;
            }
        }

        const double chi2_th = 5.991;
        int noutliers = 0;
        for(int iter = 0; iter < 4; ++iter)
        {
            vpose->setEstimate(current_frame_->getPose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            noutliers = 0;

            for(size_t i = 0; i < edges.size(); i++)
            {
                auto e = edges[i];
                if(feats[i]->is_outlier_)
                {
                    e->computeError();
                }
                if(e->chi2() < chi2_th)
                {
                    feats[i]->is_outlier_ = false;
                    e->setLevel(0);
                }else
                {
                    feats[i]->is_outlier_ = true;
                    e->setLevel(1);
                    ++noutliers;
                }
                if(iter == 2) e->setRobustKernel(nullptr);
            }
        }

        std::cout << "Outlier/inliers in pose estimating: " << noutliers << ", " << feats.size() - noutliers << std::endl;
        current_frame_->setPose(vpose->estimate());
        //std::cout << "Current pose: \n" << current_frame_->getPose().matrix() << std::endl;

        for(auto feat: feats)
        {
            if(feat->is_outlier_)
            {
                //Reset 3d point.
                feat->mappoint_ptr_.reset();
                //Try to use it in the future
                feat->is_outlier_ = false;
            }
        }
        return feats.size() - noutliers;
    }

    bool FrontEnd::insertKeyFrame()
    {
        if(tracking_inliers_ >= nfeats_for_keyframe_)
        {
            //tracked feats are good enough.
            std::cout << "Inliers decreased" << std::endl;
            return false;
        }

        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);

        std::cout << "New keyframe added: " << current_frame_->id_ << ", Kframe id: " << current_frame_->keyframe_id_ << std::endl;

        setObservationsForKeyPoints();
        detectFeatures();
        if(!current_frame_->is_depth_img_){
            findFeaturesInRight();
            triangulateNewPoints();
        }else {
            auto pts = findZfromDepthImage();
            std::cout << "D points: " << pts << std::endl;
        }
        backend_->updateMap();
        if(viewer_) viewer_->updateMap();

        return true;
    }

    void FrontEnd::setObservationsForKeyPoints()
    {
        for(auto &feat: current_frame_->left_features_)
        {
            auto mp = feat->mappoint_ptr_.lock();
            if(mp) mp->addObservations(feat);
        }
    }

    int FrontEnd::detectFeatures()
    {
        cv::Mat mask(current_frame_->limg_.size(), CV_8UC1, 255);
        cv::rectangle(mask, cv::Point2f(0, 0), cv::Point2f(60, 480), 0, cv::FILLED);
        cv::rectangle(mask, cv::Point2f(0, 0), cv::Point2f(640, 90), 0, cv::FILLED);
        cv::rectangle(mask, cv::Point2f(380, 350), cv::Point2f(430, 480), 0, cv::FILLED);
        // Don't detect feature in already existing regions
        for(auto &feat: current_frame_->left_features_)
        {
            cv::rectangle(mask, feat->pos_.pt - cv::Point2f(10, 10),
                          feat->pos_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
        }
        std::vector<cv::KeyPoint> kps;
        gftt_->detect(current_frame_->limg_, kps, mask);
        //orb_->detect(current_frame_->limg_, kps, mask);
        //sift_->detect(current_frame_->limg_, kps, mask);
        int nfeats = 0;
        for(auto &kp: kps)
        {
            current_frame_->left_features_.emplace_back(
                Feature::Ptr(new Feature(current_frame_, kp))
                );
            ++nfeats;
        }
        std::cout << "Feats detected: " << nfeats << std::endl;
        return nfeats;
    }

    int FrontEnd::findFeaturesInRight()
    {
        std::vector<cv::Point2f> lkps, rkps;
        for(auto &kp: current_frame_->left_features_)
        {
            lkps.emplace_back(kp->pos_.pt);
            auto mp = kp->mappoint_ptr_.lock();
            if(mp)
            {
                auto px = right_cam_->world2Pixel(mp->pos_, current_frame_->getPose());
                rkps.emplace_back(cv::Point2f(px[0], px[1]));
            }else
            {
                rkps.emplace_back(kp->pos_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(current_frame_->limg_, current_frame_->rimg_, lkps, rkps,
                                 status, error, cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        int ngood_pts = 0;
        for(size_t i = 0; i < status.size(); i++)
        {
            if(status[i])
            {
                cv::KeyPoint kp(rkps[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_limg_ = false;
                current_frame_->right_features_.emplace_back(feat);
                ngood_pts++;
            }else
            {
                current_frame_->right_features_.emplace_back(nullptr);
            }
        }
        std::cout << "Found KPs in right image: " << ngood_pts <<  std::endl;
        return ngood_pts;
    }

    int FrontEnd::findZfromDepthImage()
    {
        SE3 pose_twc = current_frame_->getPose().inverse();
        int npts = 0;
        for(size_t i = 0; i < current_frame_->left_features_.size(); ++i)
        {
            if(current_frame_->left_features_[i]->mappoint_ptr_.expired())
            {
                auto &kp = current_frame_->left_features_[i]->pos_.pt;
                Vec3 points = left_cam_->pixel2Cam(Vec2(kp.x, kp.y));
                if(current_frame_->rimg_.cols > 0)
                {
                    //Find depth from depth image
                    points[2] = ((float)current_frame_->rimg_.at<uint16_t>(kp.y, kp.x)) * 0.001;
                    if(points[2] > 0)
                    {
                        auto nmap_point = MapPoint::createMapPoint();
                        auto pworld = pose_twc * points;
                        nmap_point->setPos(pworld);
                        nmap_point->addObservations(current_frame_->left_features_[i]);
                        current_frame_->left_features_[i]->mappoint_ptr_ = nmap_point;
                        map_->insertMapPoint(nmap_point);
                        ++npts;
                    }
                }
            }
        }
        return npts;
    }

    int FrontEnd::triangulateNewPoints()
    {
        std::vector<SE3> poses{left_cam_->getPose(), right_cam_->getPose()};
        SE3 pose_twc = current_frame_->getPose().inverse();
        int npts = 0;
        for(size_t i = 0; i < current_frame_->left_features_.size(); ++i)
        {
            //Triangulate point only if Mappoint is not there
            if(current_frame_->left_features_[i]->mappoint_ptr_.expired() &&
               current_frame_->right_features_[i] != nullptr)
            {
                std::vector<Vec3> points{
                  left_cam_->pixel2Cam(
                      Vec2(current_frame_->left_features_[i]->pos_.pt.x,
                           current_frame_->left_features_[i]->pos_.pt.y)),
                  right_cam_->pixel2Cam(
                      Vec2(current_frame_->right_features_[i]->pos_.pt.x,
                           current_frame_->right_features_[i]->pos_.pt.y)
                      )
                    };
                Vec3 pworld = Vec3::Zero();

                if(triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    auto nmap_point = MapPoint::createMapPoint();
                    pworld = pose_twc * pworld;
                    nmap_point->setPos(pworld);
                    nmap_point->addObservations(current_frame_->left_features_[i]);
                    nmap_point->addObservations(current_frame_->right_features_[i]);
                    current_frame_->left_features_[i]->mappoint_ptr_ = nmap_point;
                    current_frame_->right_features_[i]->mappoint_ptr_ = nmap_point;
                    map_->insertMapPoint(nmap_point);
                    ++npts;
                }
            }
        }
        std::cout << "New Landmarks: " << npts << std::endl;
        return npts;
    }

    bool FrontEnd::buildInitMap()
    {
        std::vector<SE3> poses{left_cam_->getPose(), right_cam_->getPose()};
        size_t clandmarks = 0;

        for(size_t i = 0; i < current_frame_->left_features_.size(); ++i)
        {
            if(current_frame_->right_features_[i] == nullptr) continue;
            std::vector<Vec3> points{
              left_cam_->pixel2Cam(
                  Vec2(current_frame_->left_features_[i]->pos_.pt.x,
                       current_frame_->left_features_[i]->pos_.pt.y)
                  ),
              right_cam_->pixel2Cam(
                  Vec2(current_frame_->right_features_[i]->pos_.pt.x,
                       current_frame_->right_features_[i]->pos_.pt.y)
                  )
            };
            Vec3 pworld = Vec3::Zero();
            bool tstat =  triangulation(poses, points, pworld);
            if(tstat && pworld[2] > 0)
            {
                auto npoint = MapPoint::createMapPoint();
                npoint->setPos(pworld);
                npoint->addObservations(current_frame_->left_features_[i]);
                npoint->addObservations(current_frame_->right_features_[i]);
                current_frame_->left_features_[i]->mappoint_ptr_ = npoint;
                current_frame_->right_features_[i]->mappoint_ptr_  = npoint;
                ++clandmarks;
                map_->insertMapPoint(npoint);
            }
        }
        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);
        backend_->updateMap();
        auto op = left_cam_->pixel2World(toVec2(cv::Point2f(current_frame_->limg_.cols / 2, current_frame_->limg_.rows/2)), current_frame_->getPose());
        std::cout << "Optical center: " << op.matrix() << std::endl;
        std::cout << "Initmap created with: " << clandmarks << " landmarks" << std::endl;
        return true;
    }

    bool FrontEnd::buildInitMapRGBD()
    {
        SE3 pose_twc = current_frame_->getPose().inverse();
        int npts = 0;
        for(size_t i = 0; i < current_frame_->left_features_.size(); ++i)
        {
            if(current_frame_->left_features_[i]->mappoint_ptr_.expired())
            {
                auto &kp = current_frame_->left_features_[i]->pos_.pt;
                Vec3 points = left_cam_->pixel2Cam(Vec2(kp.x, kp.y));
                if(current_frame_->rimg_.cols > 0)
                {
                    //Find depth from depth image
                    cv::Rect rect(kp.x - 3, kp.y - 3, 6, 6);
                    auto op = current_frame_->rimg_(rect);
                    double minval = 0, maxval = 0;
                    cv::Point l1, l2;
                    cv::minMaxLoc(op, &minval, &maxval, &l1, &l2);
                    if(maxval > 0)
                    {
                        points[2] = maxval * 0.001;
                        auto nmap_point = MapPoint::createMapPoint();
                        auto pworld = pose_twc * points;
                        nmap_point->setPos(pworld);
                        nmap_point->addObservations(current_frame_->left_features_[i]);
                        current_frame_->left_features_[i]->mappoint_ptr_ = nmap_point;
                        map_->insertMapPoint(nmap_point);
                        ++npts;
                    }
                }
            }
        }
        current_frame_->setKeyFrame();
        map_->insertKeyFrame(current_frame_);
        backend_->updateMap();

        auto op = left_cam_->pixel2World(toVec2(cv::Point2f(current_frame_->limg_.cols / 2, current_frame_->limg_.rows/2)), current_frame_->getPose());
        std::cout << "Optical center: " << op.matrix() << std::endl;
        std::cout << "Initmap created with: " << npts << " landmarks" << std::endl;
        return npts > 0;
    }

    bool FrontEnd::reset()
    {
        //std::cout << "to be implemented" << std::endl;
        return true;
    }

    void FrontEnd::setMap(Map::Ptr map_)
    {
        this->map_ = map_;
    }

    void FrontEnd::setViewer(Viewer::Ptr viewer)
    {
        viewer_ = viewer;
    }

    void FrontEnd::setCamera(Camera::Ptr lcam, Camera::Ptr rcam)
    {
        left_cam_ = lcam;
        right_cam_ = rcam;
    }

    bool FrontEnd::stereoInit()
    {
        detectFeatures();
        int nfeat_right = findFeaturesInRight();
        if(nfeat_right < nfeats_init_)
        {
            return false;
        }
        bool map_success = buildInitMap();
        if(map_success)
        {
            status_ = FrontEndStatus::TRACKING_GOOD;
            if(viewer_)
            {
                viewer_->addCurrentFrame(current_frame_);
                viewer_->updateMap();
            }
            std::cout << "Stereo init successfully" << std::endl;
            return true;
        }
        return false;
    }

    bool FrontEnd::stereoInitRGBD()
    {
          detectFeatures();
          int npts = findZfromDepthImage();
          if(npts < nfeats_init_)
          {
              return false;
          }
          bool map_success = buildInitMapRGBD();
          if(map_success)
          {
              status_ = FrontEndStatus::TRACKING_GOOD;
              if(viewer_)
              {
                  viewer_->addCurrentFrame(current_frame_);
                  viewer_->updateMap();
              }
              std::cout << "Stereo init successfully RGBD" << std::endl;
              return true;
          }
          return false;
    }

    void FrontEnd::setBackend(Backend::Ptr backend)
    {
        backend_ = backend;
    }

}
