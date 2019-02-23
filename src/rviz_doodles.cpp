
#include "rclcpp/clock.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


#include "TransformWithCovariance.hpp"

#include <unordered_map>

namespace rviz_doodles
{

//=============
// Context struct
//=============

  struct Context
  {
    rclcpp::Node &node_;

    const static int timer_interval_milliseconds_ = 100;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_ =
      node_.create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_message_pub_ =
      node_.create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

    explicit Context(rclcpp::Node &node)
      : node_(node)
    {}
  };

//=============
// TransformHolder struct
//=============

  struct TransformHolder
  {
    const std::string frame_id_;
    const std::string parent_frame_id_;
    TransformWithCovariance::mu_type t_parent_self_;

    TransformHolder(std::string frame_id, std::string parent_frame_id,
                    const TransformWithCovariance::mu_type &t_parent_self)
      : frame_id_(std::move(frame_id)), parent_frame_id_(std::move(parent_frame_id)),
        t_parent_self_(t_parent_self)
    {}
  };

//=============
// Emitter interface
//=============

  class IEmitter
  {
    // function overridden by the derived classes.
    virtual void do_emit() = 0;

  protected:
    std::shared_ptr<Context> cxt_;

    std::unordered_map<std::string, TransformHolder> transforms_;
    std::vector<visualization_msgs::msg::Marker> markers_;
    std::unordered_map<std::string, TransformHolder> temp_transforms_;
    std::vector<visualization_msgs::msg::Marker> temp_markers_;

    bool ns_updated_ = false;

    IEmitter(std::shared_ptr<Context> cxt)
      : cxt_(std::move(cxt))
    {}

    ~IEmitter()
    {
      auto stamp = cxt_->node_.now();

      // Delete any markers we have created. The transforms will time out after a while.
      visualization_msgs::msg::MarkerArray markers;

      // Loop over all the markers, mark them for deletion, and add them to the markers msg.
      // Can use &m as a reference and modify the original marker because it will never be
      // used again.
      for (auto &m : markers_) {
        m.header.stamp = stamp;
        m.type = visualization_msgs::msg::Marker::DELETE;

        // Add m to the markers message
        markers.markers.emplace_back(m);
      }

      cxt_->markers_pub_->publish(markers);
    }

  public:
    // The public non-virtual API function that delegates to private overridden virtual functions.
    void emit()
    {
      do_emit();
    }

    // Move transforms and markers to the temp_xxx containers where they will
    // be modified during execution of the animation.
    void reset()
    {
      if (!ns_updated_) {
        // Walk through the markers updating their namespaces with their
        // transform's name.
        for (auto &m : markers_) {
          // Build up a ns for this marker by concatenating frame_ids.
          std::string _ns = m.ns;
          std::string _frame_id = m.header.frame_id;
          while (_frame_id.length()) {
            _ns.append(".");
            _ns.append(_frame_id);
            auto pp = transforms_.find(_frame_id);
            _frame_id = pp != transforms_.end() ? pp->second.parent_frame_id_ : "";
          }
          m.ns = _ns;
        }
        ns_updated_ = true;
      }

      // Clear out the working lists.
      temp_transforms_.clear();
      temp_markers_.clear();

      // Move the transforms to the working list.
      for (auto &t : transforms_) {
        temp_transforms_.emplace(t.first, t.second);
      }

      // Move the markers to the working list.
      for (auto &m : markers_) {
        temp_markers_.emplace_back(m);
      }
    }

    TransformHolder *get_temp_transform_holder(const std::string &frame_id)
    {
      auto p = temp_transforms_.find(frame_id);
      if (p == temp_transforms_.end()) {
        return nullptr;
      }
      return &p->second;
    }

    visualization_msgs::msg::Marker *get_marker(int idx)
    {
      if (idx >= markers_.size()) {
        return nullptr;
      }
      return &markers_[idx];
    }

    void add_transform(const std::string &frame_id, const std::string &parent_frame_id,
                       const TransformWithCovariance::mu_type &t_parent_self)
    {
      transforms_.emplace(frame_id, TransformHolder(frame_id, parent_frame_id, t_parent_self));
    }

    void add_marker(std::string frame_id, std::string ns, visualization_msgs::msg::Marker marker)
    {
      marker.header.frame_id = std::move(frame_id);
      marker.ns = std::move(ns);
      markers_.emplace_back(std::move(marker));
    }

    void add_axes(const std::string &frame_id, const std::string &ns)
    {
      visualization_msgs::msg::Marker marker;

      // Set the frame ID and timestamp. The frame_id and the stamp will be set when this
      // marker is added.

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
    void do_emit() override
    {
      auto stamp = cxt_->node_.now();

      visualization_msgs::msg::MarkerArray markers;

      // Loop over all the markers, adjust their transformation, and add them to the markers msg.
      for (auto m : temp_markers_) {
        // m is a copy of the item in the vector so we can modify it.
        // If the frame_id of this marker is a valid transformation, then transform the
        // marker
        auto frame_id = m.header.frame_id;
        auto pp = temp_transforms_.find(m.header.frame_id);
        if (pp != temp_transforms_.end()) {
          auto &holder = pp->second;
          frame_id = holder.parent_frame_id_;
          TransformWithCovariance t(holder.t_parent_self_);

          // Walk up the transform chain if there are more transforms
          pp = temp_transforms_.find(holder.parent_frame_id_);
          while (pp != temp_transforms_.end()) {
            auto &th = pp->second;
            frame_id = th.parent_frame_id_;
            TransformWithCovariance t1(th.t_parent_self_);
            auto tf2_transform = t1.transform() * t.transform();
            t = TransformWithCovariance(tf2_transform);
            pp = temp_transforms_.find(th.parent_frame_id_);
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
          m.header.frame_id = frame_id;
        }

        m.header.stamp = stamp;

        // Add m to the markers message
        markers.markers.emplace_back(m);
      }

      // And publish these markers
      cxt_->markers_pub_->publish(markers);
    }

  public:
    MapFrameEmitter(std::shared_ptr<Context> &cxt)
      : IEmitter(cxt)
    {}
  };


//=============
// Tf2Emitter
//=============

  class Tf2Emitter : public IEmitter
  {
    void do_emit() override
    {
      auto stamp = cxt_->node_.now();

      // First publish transforms
      tf2_msgs::msg::TFMessage tf_message;

      // Walk through each TransformHolder and create a
      // TransformStamped message and add it to the tf_message.
      for (auto const &t_pair : temp_transforms_) {
        auto &th = t_pair.second;
        auto mu = th.t_parent_self_;

        tf2::Quaternion q;
        q.setRPY(mu[3], mu[4], mu[5]);
        auto tf2_transform = tf2::Transform(q, tf2::Vector3(mu[0], mu[1], mu[2]));

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = th.parent_frame_id_;
        msg.child_frame_id = th.frame_id_;
        msg.transform = tf2::toMsg(tf2_transform);

        tf_message.transforms.emplace_back(msg);
      }

      cxt_->tf_message_pub_->publish(tf_message);

      // Then publish markers
      visualization_msgs::msg::MarkerArray markers;

      // Loop over all the markers, and add them to the markers msg.
      for (auto &m : temp_markers_) {
        m.header.stamp = stamp;

        // Add m to the markers message
        markers.markers.emplace_back(m);
      }

      cxt_->markers_pub_->publish(markers);
    }

  public:
    Tf2Emitter(std::shared_ptr<Context> &cxt)
      : IEmitter(cxt)
    {}
  };

//=============
// AnimationPlan, AnimationPhase structs
//=============

  struct AnimationPlan
  {
    const std::string frame_id_;
    TransformWithCovariance::mu_type mu_delta_;

    AnimationPlan(int step_count, std::string frame_id, const TransformWithCovariance::mu_type &mu_final)
      : frame_id_(std::move(frame_id))
    {
      for (int i = 0; i < mu_final.size(); i += 1) {
        mu_delta_[i] = mu_final[i] / step_count;
      }
    }
  };

  struct AnimationPhase
  {
    int step_count_;
    std::vector<AnimationPlan> mu_deltas_;

    AnimationPhase(int step_count, const std::string &frame_id, const TransformWithCovariance::mu_type &mu_final)
      : step_count_(step_count), mu_deltas_{AnimationPlan{step_count, frame_id, mu_final}}
    {}
  };

//=============
// Animation interface
//=============

  class IAnimation
  {
    std::vector<AnimationPhase> phases_;
    int current_phase_ = -1;
    int current_step_ = 0;
    int skip_count_;

    // Method to override for custom behavior at each step.
    virtual void do_step()
    {}

  protected:
    std::shared_ptr<Context> cxt_;
    std::shared_ptr<IEmitter> emt_;

  public:
    using mu_type = TransformWithCovariance::mu_type;

    IAnimation(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt, int skip_count)
      : cxt_(cxt), emt_(emt), skip_count_(skip_count)
    {}

    void add_phase_duration_final(const std::string &frame_id, double duration_secs, const mu_type &mu_final)
    {
      // calculate the step_count
      int step_count = static_cast<int>(std::ceil(duration_secs * 1000. / Context::timer_interval_milliseconds_));
      step_count = std::max(step_count, 1);

      // Add the phase to the list.
      phases_.emplace_back(AnimationPhase{step_count, frame_id, mu_final});
    }

    int step()
    {
      if (current_phase_ == -1) {
        emt_->reset();
        current_phase_ = 0;
        current_step_ = 0;
      }

      emt_->emit();

      if (current_phase_ >= phases_.size()) {
        current_phase_ = -1;
      } else {

        // The current phase
        auto &phase = phases_[current_phase_];
        current_step_ += 1;

        // Update the transforms specified in this phase.
        for (auto &d : phase.mu_deltas_) {
          auto pp = emt_->get_temp_transform_holder(d.frame_id_);
          if (pp) {
            for (int i = 0; i < d.mu_delta_.size(); i += 1) {
              pp->t_parent_self_[i] += d.mu_delta_[i];

              // Undo angle wrap around.
              if (i > 2 && i < 6) {
                if (pp->t_parent_self_[i] > TF2SIMD_PI) {
                  pp->t_parent_self_[i] -= TF2SIMD_2_PI;
                } else if (pp->t_parent_self_[i] < -TF2SIMD_PI) {
                  pp->t_parent_self_[i] += TF2SIMD_2_PI;
                }
              }
            }
          }
        }

        // Test for the end of this phase
        if (current_step_ >= phase.step_count_) {
          current_phase_ += 1;
          current_step_ = 0;
        }
      }

      do_step();

      return skip_count_;
    }
  };

//=============
// BasicAxes class
//=============

  class BasicAxes : public IAnimation
  {
    constexpr static int basic_axes_interval_milliseconds_ = 500;
    constexpr static int basic_axes_skip_count_ =
      basic_axes_interval_milliseconds_ / Context::timer_interval_milliseconds_;

  public:
    BasicAxes(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt)
      : IAnimation(cxt, emt, basic_axes_skip_count_)
    {
      emt_->add_transform("basic", "map", mu_type{1., 1., 0.5, 0., 0., TF2SIMD_PI / 8});
      emt_->add_axes("map", "axes");
      emt_->add_axes("basic", "axes");
    }
  };

//=============
// BasicMarkers class
//=============

  class BasicMarkers : public IAnimation
  {
    constexpr static int basic_markers_interval_milliseconds_ = 1000;
    constexpr static int basic_markers_skip_count_ =
      basic_markers_interval_milliseconds_ / Context::timer_interval_milliseconds_;

  public:
    BasicMarkers(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt)
      : IAnimation(cxt, emt, basic_markers_skip_count_)
    {
      // Add a marker. Do this first so it gets index 0.
      visualization_msgs::msg::Marker marker;

      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::CUBE;
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

      // Add the marker to the emit list
      emt_->add_marker("basic", "basic_marker", marker);

      // Add a transform and axes
      emt_->add_transform("basic", "map", mu_type{1., 1., 0.5, 0., 0., TF2SIMD_PI / 8});
      emt_->add_axes("map", "axes");
    }

  private:
    virtual void do_step()
    {
      // Cycle between different shapes
      auto m = emt_->get_marker(0);
      switch (m->type) {
        case visualization_msgs::msg::Marker::CUBE:
          m->type = visualization_msgs::msg::Marker::SPHERE;
          break;
        case visualization_msgs::msg::Marker::SPHERE:
          m->type = visualization_msgs::msg::Marker::ARROW;
          break;
        case visualization_msgs::msg::Marker::ARROW:
          m->type = visualization_msgs::msg::Marker::CYLINDER;
          break;
        default:
        case visualization_msgs::msg::Marker::CYLINDER:
          m->type = visualization_msgs::msg::Marker::CUBE;
          break;
      }
    }
  };

//=============
// RotatingAxes class
//=============

  class RotatingAxes : public IAnimation
  {
  public:
    RotatingAxes(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt)
      : IAnimation(cxt, emt, 0)
    {
      emt_->add_transform("basic", "map", mu_type{1., 1., 0.5, 0., 0., TF2SIMD_PI / 8});
      emt_->add_axes("map", "axes");
      emt_->add_axes("basic", "axes");

      add_phase_duration_final("basic", 2.0, mu_type{0., 0., 0., TF2SIMD_PI, 0., 0.});
      add_phase_duration_final("basic", 2.0, mu_type{0., 0., 0., TF2SIMD_PI, 0., 0.});
    }
  };

//=============
// DroneModel class
//=============

  class DroneModel : public IAnimation
  {

  public:
    DroneModel(std::shared_ptr<Context> &cxt, std::shared_ptr<IEmitter> &emt)
      : IAnimation(cxt, emt, 0)
    {
      emt_->add_transform("marker", "map", mu_type{});
      emt_->add_transform("drone", "map", mu_type{});
      emt_->add_transform("camera", "drone", mu_type{});

      emt_->add_axes("map", "axes");
      emt_->add_axes("marker", "axes");
      emt_->add_axes("drone", "axes");
      emt_->add_axes("camera", "axes");

      double drone_target_distance = 3.5;
      add_phase_duration_final("marker", 2.0, mu_type{0., 0., 2., TF2SIMD_PI / 2, 0., -TF2SIMD_PI / 2});
      add_phase_duration_final("drone", 2.0, mu_type{-drone_target_distance, 0., 2., 0., 0., 0.});
      add_phase_duration_final("camera", 2.0, mu_type{1., 0., 0., -TF2SIMD_PI / 2, 0., -TF2SIMD_PI / 2});

      double y_move_distance = 2.0;
      double yaw_delta = std::atan2(y_move_distance, drone_target_distance);
      add_phase_duration_final("drone", 2.0, mu_type{0., y_move_distance, 0., 0., 0., -yaw_delta});
      add_phase_duration_final("drone", 2.0, mu_type{0., -y_move_distance, 0., 0., 0., yaw_delta});
      add_phase_duration_final("drone", 2.0, mu_type{0., -y_move_distance, 0., 0., 0., yaw_delta});
      add_phase_duration_final("drone", 2.0, mu_type{0., y_move_distance, 0., 0., 0., -yaw_delta});

      add_phase_duration_final("marker", 2.0, mu_type{});
    }
  };

//=============
// RvizDoodlesNode class
//=============

  class RvizDoodlesNode : public rclcpp::Node
  {
    int skip_count_ = 0;

    rclcpp::TimerBase::SharedPtr timer_sub_;

    std::shared_ptr<Context> cxt_;
    std::shared_ptr<IEmitter> emt_;
    std::shared_ptr<IAnimation> ani_;

  public:
    RvizDoodlesNode()
      : Node("rviz_doodles_node")
    {
      auto result = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

      cxt_ = std::make_shared<Context>(*this);

//     emt_ = std::make_shared<MapFrameEmitter>(cxt_);
     emt_ = std::make_shared<Tf2Emitter>(cxt_);

//     ani_ = std::make_shared<BasicAxes>(cxt_, emt_);
//     ani_ = std::make_shared<BasicMarkers>(cxt_, emt_);
//     ani_ = std::make_shared<RotatingAxes>(cxt_, emt_);
     ani_ = std::make_shared<DroneModel>(cxt_, emt_);


      auto ms = Context::timer_interval_milliseconds_;
      auto timer_sub_lambda = [this]() -> void
      {
        if (this->skip_count_-- <= 0) {
          this->skip_count_ = this->ani_->step();
        }
      };
      timer_sub_ = create_wall_timer(std::chrono::milliseconds(ms), timer_sub_lambda);

      RCLCPP_INFO(get_logger(), "rviz_doodles_node ready");
    }

    ~RvizDoodlesNode()
    {
      RCLCPP_INFO(get_logger(), "rviz_doodles_node shutting down");

      timer_sub_->cancel();
    }
  };
}

//=============
// main()
//=============

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);
  rclcpp::install_signal_handlers();

  // Spin until rclcpp::ok() returns false.
  rclcpp::spin(std::make_shared<rviz_doodles::RvizDoodlesNode>());

  // Shut down ROS
  rclcpp::uninstall_signal_handlers();
  rclcpp::shutdown();

  return 0;
}
