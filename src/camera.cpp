#include "base_slam/camera.hpp"


namespace base_slam {

    Camera::Camera(){}

    Camera::Camera(double fx, double fy, double cx, double cy, double baseline,
                   const SE3 &pose)
                   :fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline)
    {
        pose_ = pose;
        pose_inv_ = pose.inverse();
    }

    SE3 Camera::getPose() const {return pose_;}

    Mat33 Camera::K() const
    {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    Vec3 Camera::world2Cam(const Vec3 &p_wrld, const SE3 &Tcw)
    {
        return pose_ * Tcw * p_wrld;
    }

    Vec3 Camera::cam2World(const Vec3 &p_cam, const SE3 &Tcw)
    {
        return Tcw.inverse() * pose_inv_ * p_cam;
    }

    Vec3 Camera::pixel2Cam(const Vec2 &p_pxl, double depth)
    {
        auto x = (p_pxl(0, 0) - cx_) * depth / fx_,
             y = (p_pxl(1, 0) - cy_) * depth / fy_;
        return Vec3(x, y, depth);
    }

    Vec2 Camera::cam2Pixel(const Vec3 &p_cam)
    {
        return Vec2(fx_ * p_cam(0, 0) / p_cam(2, 0) + cx_,
                    fy_ * p_cam(1, 0) / p_cam(2, 0) + cy_);
    }

    Vec2 Camera::world2Pixel(const Vec3 &p_wrld, const SE3 &Tcw)
    {
        return cam2Pixel(world2Cam(p_wrld, Tcw));
    }

    Vec3 Camera::pixel2World(const Vec2 &p_pxl, const SE3 &Tcw, double depth)
    {
        return cam2World(pixel2Cam(p_pxl, depth), Tcw);
    }
}
