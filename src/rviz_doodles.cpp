
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

#include <map>

namespace rviz_doodles
{

//=============
// Context struct
//=============

  struct Context
  {
    rclcpp::Node &node_;

    const static int timer_inverval_milliseconds_ = 100;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    Context(rclcpp::Node &node)
      : node_(node)
    {
      marker_pub_ = node_.create_publisher<visualization_msgs::msg::Marker>("marker", 10);
      markers_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
    }
  };

//=============
// TransformHolder struct
//=============

  struct TransformHolder
  {
    const std::string frame_id_;
    const std::string parent_frame_id_;
    TransformWithCovariance::mu_type t_parent_self_;

    TransformHolder(const std::string &frame_id, const std::string &parent_frame_id,
                    TransformWithCovariance::mu_type &t_parent_self)
      : frame_id_(frame_id), parent_frame_id_(parent_frame_id),
        t_parent_self_(t_parent_self)
    {
    }
  };

//=============
// Emitter interface
//=============

  class IEmitter
  {
    virtual void do_emit(void) = 0;

  protected:
    std::shared_ptr<Context> cxt_;
    const std::string ns_;

    std::map<std::string, std::shared_ptr<TransformHolder>> transforms_;
    std::vector<visualization_msgs::msg::Marker> markers_;

    IEmitter(std::shared_ptr<Context> &cxt, const std::string &ns)
      : ns_(ns)
    {
      cxt_ = cxt;
    }

    ~IEmitter()
    {}

  public:
    void emit(void)
    {
      do_emit();
    }

    void add_transform(const std::string &frame_id, const std::string &parent_frame_id,
                       TransformWithCovariance::mu_type t_parent_self)
    {
      auto th = std::make_shared<TransformHolder>(frame_id, parent_frame_id, t_parent_self);
      transforms_[frame_id] = th;
    }

    void add_marker(const std::string &frame_id, const std::string &ns, const visualization_msgs::msg::Marker &marker)
    {
      visualization_msgs::msg::Marker m(marker);

      m.header.frame_id = frame_id;

      // Build up a ns for this marker by concatenating frame_ids. Note that this requires that
      // the transforms be added before the markers are added.
      std::string my_ns = ns;
      std::string my_frame_id = frame_id;
      while (my_frame_id.length()) {
        my_ns.append(".");
        my_ns.append(my_frame_id);
        my_frame_id = transforms_.count(my_frame_id) ? transforms_[my_frame_id]->parent_frame_id_ : "";
      }
      m.ns = my_ns;

      markers_.push_back(m);
    }

    void add_axes(const std::string &frame_id, const std::string &ns)
    {
      visualization_msgs::msg::Marker marker;

      // Set the frame ID and timestamp. The frame_id will be set when this
      //  marker is added.
      marker.header.stamp = cxt_->node_.now();

      // Set the namespace and id for this marker. The ns will be set when this
      // marker is added
      marker.id = 0;

      // Set the marker type.
      marker.type = visualization_msgs::msg::Marker::ARROW;

      marker.pose.position.x = 0.;
      marker.pose.position.y = 0.;
      marker.pose.position.z = 0.;

      marker.pose.orientation.x = 0.;
      marker.pose.orientation.y = 0.;
      marker.pose.orientation.z = 0.;
      marker.pose.orientation.w = 1.;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 1.0;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      add_marker(frame_id, ns, marker);


      marker.id = 1;

      marker.pose.orientation.x = 0.;
      marker.pose.orientation.y = 0.;
      marker.pose.orientation.z = TF2SIMDSQRT12;
      marker.pose.orientation.w = TF2SIMDSQRT12;

      // Set the color
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;

      add_marker(frame_id, ns, marker);


      marker.id = 2;

      marker.pose.orientation.x = 0.;
      marker.pose.orientation.y = -TF2SIMDSQRT12;
      marker.pose.orientation.z = 0.;
      marker.pose.orientation.w = TF2SIMDSQRT12;

      // Set the color
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;

      add_marker(frame_id, ns, marker);
    }
  };

//=============
// MapFrameEmitter
//=============

  class MapFrameEmitter : public IEmitter
  {
    void do_emit(void)
    {
      visualization_msgs::msg::MarkerArray markers;

      // Loop over all the markers, adjust their transformation, and add them to the markers msg.
      for (auto m : markers_) {
        // m is a copy of the item in the vector so we can modify it.
        // If the frame_id of this marker is a valid transformation, then transform the
        // marker
        if (transforms_.count(m.header.frame_id)) {
          auto holder = transforms_[m.header.frame_id];
          TransformWithCovariance t(holder->t_parent_self_, TransformWithCovariance::cov_type());

          // Walk up the transform chain if there are more transforms
          while (transforms_.count(holder->parent_frame_id_)) {
            holder = transforms_[holder->parent_frame_id_];
            TransformWithCovariance t1(holder->t_parent_self_, TransformWithCovariance::cov_type());
            auto tf2_transform = t1.transform() * t.transform();
            t = TransformWithCovariance(tf2_transform, TransformWithCovariance::cov_type());
          }

          // Do the transformation
          auto c = t.transform().getOrigin();
          auto q = t.transform().getRotation();

          m.pose.position.x += c.x();
          m.pose.position.y += c.y();
          m.pose.position.z += c.z();

          auto qm = tf2::Quaternion(
            m.pose.orientation.x,
            m.pose.orientation.y,
            m.pose.orientation.z,
            m.pose.orientation.w);
          auto qm1 = q * qm;
          m.pose.orientation.x = qm1.x();
          m.pose.orientation.y = qm1.y();
          m.pose.orientation.z = qm1.z();
          m.pose.orientation.w = qm1.w();

          // Set the frame_id of this marker to be the parent of the last transformation
          m.header.frame_id = holder->parent_frame_id_;
        }

        // Add m to the markers message
        markers.markers.push_back(m);
      }

      // And publish these markers
      cxt_->markers_pub_->publish(markers);
    }

  public:
    MapFrameEmitter(std::shared_ptr<Context> &cxt, const std::string &ns)
      : IEmitter(cxt, ns)
    {}

  };

//=============
// Animation interface
//=============

  class IAnimation
  {
  protected:
    std::shared_ptr<Context> cxt_;
    std::shared_ptr<IEmitter> emt_;

  public:
    IAnimation(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt)
    {
      cxt_ = cxt;
      emt_ = emt;
    }

    int step(void)
    {
      return do_step();
    }

  private:
    virtual int do_step(void) = 0;
  };

//=============
// BasicAxes class
//=============

  class BasicAxes : public IAnimation
  {
    const int draw_axes_interval_milliseconds_ = 500;
    const int draw_axes_skip_count_ = draw_axes_interval_milliseconds_ / Context::timer_inverval_milliseconds_;

  public:
    BasicAxes(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt)
      : IAnimation(cxt, emt)
    {
      emt_->add_transform("basic", "map",
                          TransformWithCovariance::mu_type{0.5, 0.5, 0.1, 0., 0., TF2SIMD_PI / 8});
      emt_->add_axes("map", "axes");
      emt_->add_axes("basic", "axes");
    }

  private:
    virtual int do_step(void)
    {
      emt_->emit();
      return draw_axes_skip_count_;
    }
  };

//=============
// RvizDoodlesNode class
//=============

  class RvizDoodlesNode : public rclcpp::Node
  {
    const int timer_inverval_milliseconds_ = 100;

    const int draw_basic_marker_interval_milliseconds_ = 2000;
    const int draw_basic_marker_skip_count_ = draw_basic_marker_interval_milliseconds_ / timer_inverval_milliseconds_;

    uint32_t basic_marker_shape_;
    rclcpp::TimerBase::SharedPtr timer_sub_;
    int skip_count_ = 0;

    std::shared_ptr<Context> cxt_;
    std::shared_ptr<IEmitter> emt_;
    std::shared_ptr<IAnimation> ani_;

  public:

    explicit RvizDoodlesNode()
      : Node("rviz_doodles_node")
    {
      basic_marker_shape_ = visualization_msgs::msg::Marker::CUBE;

      auto timer_pub_cb = std::bind(&RvizDoodlesNode::timer_callback, this);
      timer_sub_ = create_wall_timer(std::chrono::milliseconds(timer_inverval_milliseconds_), timer_pub_cb);

      RCLCPP_INFO(get_logger(), "rviz_doodles_node ready");
    }

  private:

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
      cxt_->marker_pub_->publish(marker);

      // Cycle between different shapes
      switch (basic_marker_shape_) {
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
      if (!cxt_) {
        cxt_ = std::make_shared<Context>(*this);
      }
      if (!emt_) {
        emt_ = std::make_shared<MapFrameEmitter>(cxt_, "rviz_doodle");
      }
      if (!ani_) {
        ani_ = std::make_shared<BasicAxes>(cxt_, emt_);
      }
      if (skip_count_-- <= 0) {
        skip_count_ = ani_->step();
      }
    }
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
