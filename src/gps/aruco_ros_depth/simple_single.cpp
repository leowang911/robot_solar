/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <map> // 添加 map 头文件
#include <eigen3/Eigen/Dense>// 添加 Eigen 头文件

// 平面拟合函数
bool fitPlane(const std::vector<cv::Point3f>& points, cv::Vec4f& plane_params)
{
    if (points.size() < 3)
    {
        ROS_WARN("Not enough points to fit a plane");
        return false;
    }

    Eigen::MatrixXf A(points.size(), 3);
    Eigen::VectorXf b(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = 1.0;
        b(i) = points[i].z;
    }

    // Solve for plane parameters using least squares: Ax = b
    Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

    // 平面方程 ax + by + cz + d = 0
    plane_params[0] = x[0]; // a
    plane_params[1] = x[1]; // b
    plane_params[2] = -1.0; // c
    plane_params[3] = x[2]; // d

    return true;
}

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  ros::Publisher position_pub;
  ros::Publisher marker_pub; // rviz visualization marker
  ros::Publisher pixel_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;


  std::string marker_frame_prefix; // 添加 marker_frame_prefix 成员变量
  std::map<int, ros::Publisher> marker_pose_publishers; // 用于存储每个 markerID 的 Publisher


  double marker_size;
  // int marker_id;
  std::vector<int> marker_ids;
  std::string marker_ids_str;


  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Subscriber depth_sub;
  cv::Mat depthImage;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
  ArucoSimple() :
      cam_info_received(false), nh("~"), it(nh)
  {
    nh.param<std::string>("marker_frame_prefix", marker_frame_prefix, "aruco_marker_frame"); // 初始化 marker_frame_prefix

    
    if (nh.hasParam("corner_refinement"))
      ROS_WARN(
          "Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params.thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    depth_sub = it.subscribe("/camera/depth/image_raw", 1, &ArucoSimple::depth_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);

    nh.param<double>("marker_size", marker_size, 0.05);
    // nh.param<int>("marker_id", marker_id, 300);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    
    
    
    nh.param<std::string>("marker_ids", marker_ids_str, ""); // 获取markerIds参数

    // 解析逗号分隔的markerIds
    std::stringstream ss(marker_ids_str);
    std::string id;
    while (std::getline(ss, id, ','))
    {
      marker_ids.push_back(std::stoi(id));
    }
    ROS_INFO("Tracking marker IDs: ");
    for (const auto& mid : marker_ids)
    {
      ROS_INFO(" - %d", mid);
    }
    // 为每个 markerID 创建固定的话题
    for (const auto& marker_id : marker_ids)
    {
      // std::string topic_name = "pose_marker_" + std::to_string(marker_id);
      // std::string topic_name = "camera/aruco_" + std::to_string(marker_id) + "/pixel";
      std::string topic_name = "/camera/aruco_" + std::to_string(marker_id)+ "/pose";
      
      marker_pose_publishers[marker_id] = nh.advertise<geometry_msgs::PoseStamped>(topic_name, 10);
      ROS_INFO("Created topic for marker ID %d: %s", marker_id, topic_name.c_str());
    }

    // ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if (reference_frame.empty())
      reference_frame = camera_frame;

    ROS_INFO("ArUco node started with marker size of %f m and marker id to track: %d", marker_size, marker_ids);
    ROS_INFO("ArUco node will publish pose to TF with %s as parent and %s as child.", reference_frame.c_str(),
             marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }

  bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
  {
    std::string errMsg;

    if (!_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01),
                                      &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform(refFrame, childFrame, ros::Time(0), // get latest available
                                    transform);
      }
      catch (const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }

  float getAverageDepth(const cv::Mat& depthImage, int pixel_x, int pixel_y, int window_size = 3)
  {
    int half_window = window_size / 2;
    float sum_depth = 0.0;
    int valid_count = 0;

    for (int dx = -half_window; dx <= half_window; ++dx)
    {
      for (int dy = -half_window; dy <= half_window; ++dy)
      {
        int nx = pixel_x + dx;
        int ny = pixel_y + dy;

        if (nx >= 0 && nx < depthImage.cols && ny >= 0 && ny < depthImage.rows)
        {
          float depth = depthImage.at<float>(ny, nx);
          if (std::isfinite(depth) && depth > 0.15) // 添加深度值过滤条件
          {
            sum_depth += depth;
            valid_count++;
          }
        }
      }
    }

    if (valid_count > 0)
    {
      return sum_depth / valid_count;
    }
    else
    {
      return std::numeric_limits<float>::quiet_NaN(); // 无效值
    }
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    

    if (!depthImage.empty())
    {
      for (std::size_t i = 0; i < markers.size(); ++i)
      {
        // 获取标记中心的像素坐标
        int pixel_x = static_cast<int>(markers[i].getCenter().x);
        int pixel_y = static_cast<int>(markers[i].getCenter().y);
        float x = std::numeric_limits<float>::quiet_NaN(); // 初始化为无效值
        float y = std::numeric_limits<float>::quiet_NaN();
        float z = std::numeric_limits<float>::quiet_NaN();

        // 检查像素坐标是否在深度图像范围内
        if (pixel_x >= 0 && pixel_x < depthImage.cols && pixel_y >= 0 && pixel_y < depthImage.rows)
        {
          // 从深度图像中获取深度值
          float depth = getAverageDepth(depthImage, pixel_x, pixel_y, 10); // 使用 10x10 窗口计算平均深度

          if (std::isfinite(depth) && depth > 0.15) // 添加深度值过滤条件
          {
            // 使用彩色相机的内参和深度值计算 3D 坐标
            float fx = camParam.CameraMatrix.at<float>(0, 0);
            float fy = camParam.CameraMatrix.at<float>(1, 1);
            float cx = camParam.CameraMatrix.at<float>(0, 2);
            float cy = camParam.CameraMatrix.at<float>(1, 2);

            x = (pixel_x - cx) * depth / fx;
            y = (pixel_y - cy) * depth / fy;
            z = depth;

            ROS_INFO("Marker ID: %d, position: x=%f, y=%f, z=%f", markers[i].id, x, y, z);

            // 使用独立的 x, y, z 值更新 poseMsg
            geometry_msgs::PoseStamped poseMsg;
            poseMsg.pose.position.x = x;
            poseMsg.pose.position.y = y;
            poseMsg.pose.position.z = z;
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = ros::Time::now();

            // 提取点云并拟合平面
            std::vector<cv::Point3f> points;
            int window_size = 20; // 使用 20*20 窗口
            int half_window = window_size / 2;

            for (int dx = -half_window; dx <= half_window; ++dx)
            {
              for (int dy = -half_window; dy <= half_window; ++dy)
              {
                int nx = pixel_x + dx;
                int ny = pixel_y + dy;

                if (nx >= 0 && nx < depthImage.cols && ny >= 0 && ny < depthImage.rows)
                {
                  float depth = depthImage.at<float>(ny, nx);
                  if (std::isfinite(depth) && depth > 0.15)
                  {
                    float px = (nx - cx) * depth / fx;
                    float py = (ny - cy) * depth / fy;
                    float pz = depth;
                    points.emplace_back(px, py, pz);
                  }
                }
              }
            }

            // 检查点云数量是否足够
            if (points.size() > 10)
            {
              // 拟合平面
              cv::Vec4f plane_params; // 平面方程 ax + by + cz + d = 0
              fitPlane(points, plane_params);

              // 提取法向量
              cv::Vec3f normal(plane_params[0], plane_params[1], plane_params[2]);

              // 确保法向量方向一致（例如指向相机）
              if (normal[2] > 0)
              {
                normal = -normal;
              }

              // 计算四元数
              tf::Vector3 z_axis(normal[0], normal[1], normal[2]);
              tf::Vector3 y_axis(0, -1, 0); // 假设 Y 轴向下
              tf::Vector3 x_axis = y_axis.cross(z_axis).normalized();
              y_axis = z_axis.cross(x_axis).normalized();

              tf::Matrix3x3 rotation_matrix(
                  x_axis.x(), y_axis.x(), z_axis.x(),
                  x_axis.y(), y_axis.y(), z_axis.y(),
                  x_axis.z(), y_axis.z(), z_axis.z()
              );

              tf::Quaternion quaternion;
              rotation_matrix.getRotation(quaternion);

              // 更新 poseMsg 的朝向
              poseMsg.pose.orientation.x = quaternion.x();
              poseMsg.pose.orientation.y = quaternion.y();
              poseMsg.pose.orientation.z = quaternion.z();
              poseMsg.pose.orientation.w = quaternion.w();
            }
            else
            {
              ROS_WARN("Not enough points to fit a plane for Marker ID: %d", markers[i].id);
              poseMsg.pose.orientation.x = 0.0;
              poseMsg.pose.orientation.y = 0.0;
              poseMsg.pose.orientation.z = 0.0;
              poseMsg.pose.orientation.w = 1.0; // 默认朝向
            }

            // 发布到对应的 markerID 话题
            if (marker_pose_publishers.find(markers[i].id) != marker_pose_publishers.end())
            {
              marker_pose_publishers[markers[i].id].publish(poseMsg);
            }
            else
            {
              ROS_WARN("No publisher found for Marker ID: %d", markers[i].id);
            }
          }
          else
          {
            ROS_WARN("Invalid or too small depth value at marker center for Marker ID: %d", markers[i].id);
          }
        }
        else
        {
          ROS_WARN("Marker center is out of depth image bounds");
        }
      }
    }
    else
    {
      ROS_WARN("Depth image is empty");
    }

    if ((image_pub.getNumSubscribers() == 0) && (debug_pub.getNumSubscribers() == 0)
        && (pose_pub.getNumSubscribers() == 0) && (transform_pub.getNumSubscribers() == 0)
        && (position_pub.getNumSubscribers() == 0) && (marker_pub.getNumSubscribers() == 0)
        && (pixel_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for ArUco markers");
      return;
    }

    static tf::TransformBroadcaster br;
   
    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // detection results will go into "markers"
        markers.clear();
        mDetector.detect(inImage, markers, camParam, marker_size, false);

        for (std::size_t i = 0; i < markers.size(); ++i)
        {
          // 检查当前标记是否在配置的 marker_ids 中
          if (std::find(marker_ids.begin(), marker_ids.end(), markers[i].id) == marker_ids.end())
          {
            continue; // 跳过未配置的 markerID
          }

          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame != camera_frame)
          {
            getTransform(reference_frame, camera_frame, cameraToReference);
          }

          transform = static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft)
                      * transform;

          // geometry_msgs::PoseStamped poseMsg1;
          // tf::poseTFToMsg(transform, poseMsg1.pose);
          // poseMsg1.header.frame_id = reference_frame;
          // poseMsg1.header.stamp = curr_stamp;

              

          // 从 map 中获取对应的 Publisher 并发布 poseMsg
          // if (marker_pose_publishers.find(markers[i].id) != marker_pose_publishers.end())
          // {
          //   marker_pose_publishers[markers[i].id].publish(poseMsg1);
          // }
          // else
          // {
          //   ROS_WARN("No publisher found for marker ID %d", markers[i].id);
          // }
            
        }
        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size != -1)
        {
          for (std::size_t i = 0; i < markers.size(); ++i)
          {
            markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if (image_pub.getNumSubscribers() > 0)
        {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (debug_pub.getNumSubscribers() > 0)
        {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }
  void depth_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      // 将深度图像从 16UC1 转换为 32FC1，并将单位从毫米转换为米
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_ptr->image.convertTo(depthImage, CV_32F, 0.001); // 转换为米（1毫米 = 0.001米）
      ROS_DEBUG("Received depth image");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }


  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CameraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(tf::Vector3(-msg.P[3] / msg.P[0], -msg.P[7] / msg.P[5], 0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple_test");

  ArucoSimple node;

  ros::spin();
}
