
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "TransformWithCovariance.hpp"


namespace rviz_doodles
{

//=============
// RvizDoodlesNode class
//=============

  class RvizDoodlesNode : public rclcpp::Node
  {
//    Map map_;
//    Localizer localizer_;
//
//    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
//    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;
//    rclcpp::Subscription<flock_vlam_msgs::msg::Map>::SharedPtr map_sub_;
//
//    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_pub_;
//    rclcpp::Publisher<flock_vlam_msgs::msg::Observations>::SharedPtr observations_pub_;
//    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_;
//
//    bool have_camera_info_{false};
//    sensor_msgs::msg::CameraInfo cameraInfo_;
//    cv::Mat camera_matrix_;
//    cv::Mat dist_coeffs_;
//
//    cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//    cv::Ptr<cv::aruco::DetectorParameters> detectorParameters_ = cv::aruco::DetectorParameters::create();

    const int timer_inverval_milliseconds_ = 100;

    const int draw_basic_marker_interval_milliseconds_ = 2000;
    const int draw_basic_marker_skip_count_ = draw_basic_marker_interval_milliseconds_ / timer_inverval_milliseconds_;
    const int draw_axes_interval_milliseconds_ = 500;
    const int draw_axes_skip_count_ = draw_axes_interval_milliseconds_ / timer_inverval_milliseconds_;

    uint32_t basic_marker_shape_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::TimerBase::SharedPtr timer_sub_;
    int skip_count_ = 0;

  public:

    explicit RvizDoodlesNode()
      : Node("rviz_doodles_node")
    {
//      // ROS subscriptions
//      auto cameraInfo_cb = std::bind(&RvizDoodlesNode::camera_info_callback, this, std::placeholders::_1);
//      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", cameraInfo_cb);
//
//      auto image_raw_sub_cb = std::bind(&RvizDoodlesNode::image_callback, this, std::placeholders::_1);
//      image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>("image_raw", image_raw_sub_cb);
//
//      auto map_sub_cb = std::bind(&RvizDoodlesNode::map_callback, this, std::placeholders::_1);
//      map_sub_ = create_subscription<flock_vlam_msgs::msg::Map>("/flock_map", map_sub_cb);
//
//
//      // ROS publishers
//      camera_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("camera_pose", 1);
//      observations_pub_ = create_publisher<flock_vlam_msgs::msg::Observations>("/flock_observations", 1);
//      image_marked_pub_ = create_publisher<sensor_msgs::msg::Image>("image_marked", 1);
//
//      detectorParameters_->doCornerRefinement = true;

      basic_marker_shape_ = visualization_msgs::msg::Marker::CUBE;

      marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("marker", 10);
      markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

      auto timer_pub_cb = std::bind(&RvizDoodlesNode::timer_callback, this);
      timer_sub_ = create_wall_timer(std::chrono::milliseconds(timer_inverval_milliseconds_), timer_pub_cb);

      RCLCPP_INFO(get_logger(), "rviz_doodles_node ready");
    }

  private:

    void draw_axes(const std::string &frame_id, const std::string &ns, int id, const TransformWithCovariance &t)
    {
      // Adjust the id space to allow for three axis arrows
      id *= 3;

      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = frame_id;
      marker.header.stamp = this->now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = ns + ".axes";
      marker.id = id;

      // Set the marker type.
      marker.type = visualization_msgs::msg::Marker::ARROW;

      auto c = t.transform().getOrigin();
      auto q = t.transform().getRotation();

      marker.pose.position.x = c.x();
      marker.pose.position.y = c.y();
      marker.pose.position.z = c.z();

      auto qx = tf2::Quaternion(0., 0., 0., 1.);
      auto qx1 = q * qx;
      marker.pose.orientation.x = qx1.x();
      marker.pose.orientation.y = qx1.y();
      marker.pose.orientation.z = qx1.z();
      marker.pose.orientation.w = qx1.w();

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      markers.markers.push_back(marker);


      marker.id = id + 1;

      auto qy = tf2::Quaternion(0., 0., TF2SIMDSQRT12, TF2SIMDSQRT12);
      auto qy1 = q * qy;
      marker.pose.orientation.x = qy1.x();
      marker.pose.orientation.y = qy1.y();
      marker.pose.orientation.z = qy1.z();
      marker.pose.orientation.w = qy1.w();

      // Set the color
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;

      markers.markers.push_back(marker);


      marker.id = id + 2;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = -0.707;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.707;
      auto qz = tf2::Quaternion(0., -TF2SIMDSQRT12, 0., TF2SIMDSQRT12);
      auto qz1 = q * qz;
      marker.pose.orientation.x = qz1.x();
      marker.pose.orientation.y = qz1.y();
      marker.pose.orientation.z = qz1.z();
      marker.pose.orientation.w = qz1.w();

      // Set the color
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;

      markers.markers.push_back(marker);


      markers_pub_->publish(markers);
    }

    int draw_basic_axes(const std::string ns)
    {
      TransformWithCovariance t_map_map(
        TransformWithCovariance::mu_type {0., 0., 0., 0., 0., 0.},
        TransformWithCovariance::cov_type {});

      draw_axes("map", ns, 0, t_map_map);

      TransformWithCovariance t_map_drone(
        TransformWithCovariance::mu_type {0.5, 0.5, 0.5, 0., 0., TF2SIMD_PI / 8},
        TransformWithCovariance::cov_type {});

      draw_axes("map", ns, 1, t_map_drone);

      return draw_axes_skip_count_;
    }

    int draw_basic_marker(const std::string ns)
    {
      visualization_msgs::msg::Marker marker;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = ns + ".basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = basic_marker_shape_;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

//      marker.lifetime = 1000;

      // Publish the marker
      marker_pub_->publish(marker);

      // Cycle between different shapes
      switch (basic_marker_shape_)
      {
        case visualization_msgs::msg::Marker::CUBE:
          basic_marker_shape_ = visualization_msgs::msg::Marker::SPHERE;
          break;
        case visualization_msgs::msg::Marker::SPHERE:
          basic_marker_shape_ = visualization_msgs::msg::Marker::ARROW;
          break;
        case visualization_msgs::msg::Marker::ARROW:
          basic_marker_shape_ = visualization_msgs::msg::Marker::CYLINDER;
          break;
        case visualization_msgs::msg::Marker::CYLINDER:
          basic_marker_shape_ = visualization_msgs::msg::Marker::CUBE;
          break;
      }

      return draw_basic_marker_skip_count_;
    }

    void timer_callback(void)
    {
      auto ns = "rviz_doodle";
      if (skip_count_-- <= 0) {
//        skip_count_ = draw_basic_marker(ns);
        skip_count_ = draw_basic_axes(ns);
      }
    }

#if 0
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
      if (!have_camera_info_) {
        // Save the info message because we pass it along with the observations.
        cameraInfo_ = *msg;
        tf2_util::load_camera_info(*msg, camera_matrix_, dist_coeffs_);

        RCLCPP_INFO(get_logger(), "have camera info");
        have_camera_info_ = true;
      }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      if (!have_camera_info_) {
        return;
      }

      // Convert ROS to OpenCV
      cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(msg);

      process_image(color, msg->header);
    }

    void map_callback(const flock_vlam_msgs::msg::Map::SharedPtr msg)
    {
      map_.load_from_msg(msg);
    }

    void process_image(cv_bridge::CvImagePtr color, std_msgs::msg::Header &header_msg)
    {
      // Color to gray for detection
      cv::Mat gray;
      cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

      // Detect markers
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detectorParameters_);

      RCLCPP_DEBUG(get_logger(), "process_image: Found %d markers", ids.size());

      // Stop if no markers were detected
      if (ids.size() == 0) {
        return;
      }

      // Calculate the pose of this camera in the map frame.
      Observations observations(ids, corners);
      auto camera_pose_f_map = localizer_.average_camera_pose_f_map(observations, camera_matrix_, dist_coeffs_);

      if (camera_pose_f_map.is_valid()) {
        // Publish the camera pose in the map frame
        auto camera_pose_f_map_msg = camera_pose_f_map.to_pose_with_covariance_stamped_msg(header_msg);

        // for now just publish a pose message not a pose
        geometry_msgs::msg::PoseWithCovarianceStamped cam_pose_f_map;
        cam_pose_f_map.pose.pose = camera_pose_f_map_msg.pose.pose;
        cam_pose_f_map.header = header_msg;
        cam_pose_f_map.header.frame_id = "map";
        cam_pose_f_map.pose.covariance[0] = 6e-3;
        cam_pose_f_map.pose.covariance[7] = 6e-3;
        cam_pose_f_map.pose.covariance[14] = 6e-3;
        cam_pose_f_map.pose.covariance[21] = 2e-3;
        cam_pose_f_map.pose.covariance[28] = 2e-3;
        cam_pose_f_map.pose.covariance[35] = 2e-3;
        camera_pose_pub_->publish(cam_pose_f_map);
      }

      // Publish the observations only if multiple markers exist in the image
      if (ids.size() > 1) {
        auto observations_msg = observations.to_msg(header_msg, cameraInfo_);
        observations_pub_->publish(observations_msg);
      }

      // Publish an annotated image
      if (count_subscribers(image_marked_pub_->get_topic_name()) > 0) {

        // Compute marker poses in two ways to verify that the math is working.
        // Compute marker poses using OpenCV methods. The estimatePoseSingleMarkers() method
        // returns the pose of the marker in the camera frame - t_camera_marker.
//        std::vector<cv::Vec3d> rvecs, tvecs;
//        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        // Compute marker poses using vlam info. Note this can only be done if
        // a camera pose in map frame is determined and we have a marker pose in
        // the map frame. The calculation is to take the marker location in the map
        // frame t_map_marker and transform (pre-multiply) it by t_map_camera.inverse()
        // to get t_camera_marker.
        std::vector<cv::Vec3d> rvecs_map, tvecs_map;
        localizer_.markers_pose_f_camera(camera_pose_f_map, ids, rvecs_map, tvecs_map);

        // Draw poses
//        for (int i = 0; i < rvecs.size(); i++) {
//          cv::aruco::drawAxis(color->image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.1);
//        }
        for (int i = 0; i < rvecs_map.size(); i++) {
          cv::aruco::drawAxis(color->image, camera_matrix_, dist_coeffs_, rvecs_map[i], tvecs_map[i], 0.1);
        }

        // Publish result
        image_marked_pub_->publish(color->toImageMsg());
      }
    };
#endif
  };
}

//=============
// main()
//=============

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rviz_doodles::RvizDoodlesNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
