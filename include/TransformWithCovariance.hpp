//
// Created by peter on 2/18/19.
//

#ifndef _TRANSFORMWITHCOVARIANCE_HPP
#define _TRANSFORMWITHCOVARIANCE_HPP

#include <array>

#include "tf2/LinearMath/Transform.h"

class TransformWithCovariance
{
public:
  using mu_type = std::array<double, 6>;
  using cov_type = std::array<double, 36>;

private:
  bool is_valid_{false};
  tf2::Transform transform_;
  cov_type cov_;

  static tf2::Transform transformFromMu(const mu_type &mu)
  {
    tf2::Quaternion q;
    q.setRPY(mu[3], mu[4], mu[5]);
    return tf2::Transform(q, tf2::Vector3(mu[0], mu[1], mu[2]));
  }

public:
  TransformWithCovariance(const tf2::Transform &transform, const cov_type &cov)
    : is_valid_(true), transform_(transform), cov_(cov)
  {}

  TransformWithCovariance(const tf2::Transform &transform)
    : is_valid_(true), transform_(transform)
  {}

  TransformWithCovariance(const mu_type &mu, const cov_type &cov)
    : is_valid_(true), transform_(transformFromMu(mu)), cov_(cov)
  {}

  TransformWithCovariance(const mu_type &mu)
    : is_valid_(true), transform_(transformFromMu(mu))
  {}

  TransformWithCovariance(const tf2::Quaternion &q)
    : is_valid_(true), transform_(q), cov_()
  {}

  TransformWithCovariance(const tf2::Quaternion &q, tf2::Vector3 &c)
    : is_valid_(true), transform_(q, c), cov_()
  {}

  auto is_valid() const
  { return is_valid_; }

  auto transform() const
  { return transform_; }

  auto cov() const
  { return cov_; }

  mu_type mu() const
  {
    double roll, pitch, yaw;
    transform_.getBasis().getRPY(roll, pitch, yaw);
    auto c = transform_.getOrigin();
    return mu_type{c[0], c[1], c[2], roll, pitch, yaw};
  }

  void update_simple_average(TransformWithCovariance &newVal, int previous_update_count);
};


#endif //_TRANSFORMWITHCOVARIANCE_HPP
