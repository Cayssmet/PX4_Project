#include "px4_offboard/frame_transforms.hpp"

#include <algorithm>

namespace px4_offboard
{

namespace
{

constexpr double kHalfPi = kPi / 2.0;

/// 把标量夹到 [-1, 1]，避免 asin/acos 因浮点误差返回 NaN。
double clamp_unit(const double value)
{
  return std::max(-1.0, std::min(1.0, value));
}

}  // namespace

double normalize_angle(double angle)
{
  // remainder 把结果映射到 [-pi, pi]，再把 -pi 折到 +pi，得到 (-pi, pi]。
  const double wrapped = std::remainder(angle, 2.0 * kPi);
  if (wrapped <= -kPi) {
    return wrapped + 2.0 * kPi;
  }
  return wrapped;
}

Quaternion quaternion_from_euler(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);

  Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

Quaternion quaternion_normalized(const Quaternion & q)
{
  const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  if (!(norm_sq > 0.0) || !std::isfinite(norm_sq)) {
    return Quaternion{};
  }
  const double inv = 1.0 / std::sqrt(norm_sq);
  return Quaternion{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

Quaternion quaternion_multiply(const Quaternion & a, const Quaternion & b)
{
  Quaternion r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

Quaternion quaternion_conjugate(const Quaternion & q)
{
  return Quaternion{q.w, -q.x, -q.y, -q.z};
}

Vector3 quaternion_rotate(const Quaternion & q_in, const Vector3 & v)
{
  const Quaternion q = quaternion_normalized(q_in);

  // v' = v + 2w(u × v) + 2(u × (u × v)),其中 u 是四元数的矢量部分。
  const Vector3 u{q.x, q.y, q.z};
  const Vector3 uv{
    u.y * v.z - u.z * v.y,
    u.z * v.x - u.x * v.z,
    u.x * v.y - u.y * v.x};
  const Vector3 uuv{
    u.y * uv.z - u.z * uv.y,
    u.z * uv.x - u.x * uv.z,
    u.x * uv.y - u.y * uv.x};

  return Vector3{
    v.x + 2.0 * (q.w * uv.x + uuv.x),
    v.y + 2.0 * (q.w * uv.y + uuv.y),
    v.z + 2.0 * (q.w * uv.z + uuv.z)};
}

EulerAngles quaternion_to_euler(const Quaternion & q_in)
{
  const Quaternion q = quaternion_normalized(q_in);
  const double w = q.w;
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;

  EulerAngles e;
  e.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  e.pitch = std::asin(clamp_unit(2.0 * (w * y - z * x)));
  e.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return e;
}

double quaternion_yaw(const Quaternion & q_in)
{
  const Quaternion q = quaternion_normalized(q_in);
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

const Quaternion & ned_enu_quaternion()
{
  // quaternion_from_euler(M_PI, 0.0, M_PI_2)
  static const Quaternion q{0.0, 0.70710678118654752440, 0.70710678118654752440, 0.0};
  return q;
}

const Quaternion & aircraft_baselink_quaternion()
{
  // quaternion_from_euler(M_PI, 0.0, 0.0)
  static const Quaternion q{0.0, 1.0, 0.0, 0.0};
  return q;
}

Vector3 map_position_to_px4(const Vector3 & p_enu)
{
  return Vector3{p_enu.y, p_enu.x, -p_enu.z};
}

Vector3 map_velocity_to_px4(const Vector3 & v_enu)
{
  return map_position_to_px4(v_enu);
}

Quaternion map_attitude_to_px4(const Quaternion & q_enu_flu)
{
  return quaternion_multiply(
    quaternion_multiply(ned_enu_quaternion(), quaternion_normalized(q_enu_flu)),
    aircraft_baselink_quaternion());
}

Vector3 body_vector_flu_to_frd(const Vector3 & v_flu)
{
  return Vector3{v_flu.x, -v_flu.y, -v_flu.z};
}

double map_yaw_to_px4(double yaw_map)
{
  return normalize_angle(kHalfPi - yaw_map);
}

double map_yaw_rate_to_px4(double yaw_rate_map)
{
  return -yaw_rate_map;
}

Vector3 px4_position_to_map(const Vector3 & p_ned)
{
  return map_position_to_px4(p_ned);
}

Vector3 px4_velocity_to_map(const Vector3 & v_ned)
{
  return map_position_to_px4(v_ned);
}

Quaternion px4_attitude_to_map(const Quaternion & q_ned_frd)
{
  return quaternion_multiply(
    quaternion_multiply(
      quaternion_conjugate(ned_enu_quaternion()),
      quaternion_normalized(q_ned_frd)),
    quaternion_conjugate(aircraft_baselink_quaternion()));
}

Vector3 frd_vector_to_flu(const Vector3 & v_frd)
{
  return body_vector_flu_to_frd(v_frd);
}

double px4_yaw_to_map(double yaw_px4)
{
  return normalize_angle(kHalfPi - yaw_px4);
}

double px4_yaw_rate_to_map(double yaw_rate_px4)
{
  return -yaw_rate_px4;
}

}  // namespace px4_offboard
