
#include <opencv2/imgproc.hpp>
#include <unistd.h>

#include "base_slam/viewer.hpp"
#include <pangolin/display/display.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/handler/handler.h>


namespace base_slam
{

    Viewer::Viewer()
    {
        viewer_thread_ = std::thread(std::bind(&Viewer::threadLoop, this));
    }

    void Viewer::close()
    {
          is_viewer_running_ = false;
          viewer_thread_.join();
    }

    void Viewer::addCurrentFrame(Frame::Ptr current_frame)
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        current_frame_ = current_frame;
    }

    void Viewer::updateMap()
    {
        std::unique_lock<std::mutex> lc(acc_lock_);
        assert(map_ != nullptr);
        active_keyframes_ = map_->getActiveKeyFrames();
        active_landmarks_ = map_->getActivePoints();
        map_updated_ = true;
    }

    void Viewer::threadLoop()
    {
        pangolin::CreateWindowAndBind("base_slam", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0)
            );

        pangolin::View &vis_disp = pangolin::CreateDisplay();
        vis_disp.SetBounds(0, 1, 0, 1, -1024.0/768.0);
        vis_disp.SetHandler(new pangolin::Handler3D(vis_camera));

        const float blue[3] = {0, 0, 1},
                    green[3] = {0, 1, 0};

        while(!pangolin::ShouldQuit() && is_viewer_running_)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_disp.Activate(vis_camera);

            std::unique_lock<std::mutex> lc(acc_lock_);
            if(current_frame_)
            {
                drawFrame(current_frame_, green);
                followCurrentFrame(vis_camera);

                cv::Mat img = plotFrameImage();
                cv::imshow("image", img);
                cv::waitKey(1);
            }
            if(map_)
            {
               drawMapPoints();
            }
            pangolin::FinishFrame();
            usleep(5000);
        }
    }

    cv::Mat Viewer::plotFrameImage()
    {
        cv::Mat img_out;
        cv::cvtColor(current_frame_->limg_, img_out, cv::COLOR_GRAY2BGR);
        for(size_t i = 0; i < current_frame_->left_features_.size(); ++i)
        {
            if (current_frame_->left_features_[i]->mappoint_ptr_.lock())
            {
                auto feat = current_frame_->left_features_[i];
                cv::circle(img_out, feat->pos_.pt, 2, cv::Scalar(0, 250, 0), 2);
            }
        }
        return img_out;
    }

    void Viewer::followCurrentFrame(pangolin::OpenGlRenderState& vcam)
    {
        SE3 Twc = current_frame_->getPose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vcam.Follow(m, true);
    }

    void Viewer::drawFrame(Frame::Ptr frame, const float *clr)
    {
        SE3 Twc = frame->getPose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if (clr == nullptr)
        {
            glColor3f(1, 0, 0);
        } else
        {
            glColor3f(clr[0], clr[1], clr[2]);
        }

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void Viewer::drawMapPoints()
    {
       const float red[3] = {1., 0., 0.};
       for(auto &kf: active_keyframes_)
       {
           drawFrame(kf.second, red);
       }

       glPointSize(2);
       glBegin(GL_POINTS);
       for(auto &lmark: active_landmarks_)
       {
           auto pos = lmark.second->getPos();
           glColor3f(0., 0., 1.0);
           glVertex3d(pos[0], pos[1], pos[2]);
       }
       glEnd();
    }

    void Viewer::setMap(Map::Ptr map_)
    {
        this->map_ = map_;
    }

}
