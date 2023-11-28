#ifndef _CAMERA_HPP_
#define _CAMERA_HPP_

#include "base_slam/common.hpp"


namespace base_slam {

    class Camera {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Camera> Ptr;

            double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0;

            SE3 pose_;
            SE3 pose_inv_;

            Camera();

            Camera(double fx, double fy, double cx, double cy, double baseline,
                   const SE3 &pose);

            SE3 getPose() const;

            Mat33 K() const;

            Vec3 world2Cam(const Vec3 &p_wrld, const SE3 &Tcw);

            Vec3 cam2World(const Vec3 &p_cam, const SE3 &Tcw);

            Vec2 cam2Pixel(const Vec3 &p_cam);

            Vec3 pixel2Cam(const Vec2 &p_pxl, double depth = 1);

            Vec3 pixel2World(const Vec2 &p_pxl, const SE3 &Tcw, double depth = 1);

            Vec2 world2Pixel(const Vec3 &p_wrld, const SE3 &Tcw);
    };

}


#endif
