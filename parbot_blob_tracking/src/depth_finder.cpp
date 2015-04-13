#include <ros/ros.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <vector>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

sensor_msgs::ImageConstPtr& depth_msg;
image_geometry::PinholeCameraModel& cam_model;

void cam_callback(const sensor_msgs::ImageConstPtr& new_depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cam_model.fromCameraInfo(info_msg);
    depth_msg = new_depth_msg;
}

void blobs_callback(const cmvision::Blobs& blobs)
{
    point new_point;
    new_point = find_3d_point(depth_msg, blobs.blobs[0].x, blobs.blobs[0].y, new_point);
}

struct 3_d_point
{
    double x,y,z;
};

point find_3d_point(const sensor_msgs::ImageConstPtr& depth_msg, int target_x, int target_y, 3_d_point point)
{
        // Use correct principal point from calibration
        float center_x = cam_model.cx();
        float center_y = cam_model.cy();

        int scan_height = depth_msg->height;

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) );
        float constant_x = unit_scaling / cam_model.fx();
        float constant_y = unit_scaling / cam_model.fy();

        const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
        int row_step = depth_msg->step / sizeof(T);

        int offset = (int)(cam_model.cy()-scan_height/2);
        depth_row += offset*row_step; // Offset to center of image

        depth_row = row_step * target_y;

        double x = (double)(u - center_x) * depth * constant_x;
        double y = -(double)(v - center_y) * depth * constant_y;
        double z = depth_row[target_x];

        point.x = x;
        point.y = y;
        point.z = z;
        return point;
}