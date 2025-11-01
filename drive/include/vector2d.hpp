#pragma once

#include <cmath>
#include <numbers>

namespace sjsu::drive {

/**
 * @brief A simple Vector 2D library.
 *
 * @note all angles are in radians, since math is easier in radians.
 *
 */
struct vector2d
{
  float x, y;

  constexpr vector2d(float p_x = 0, float p_y = 0)
    : x(p_x)
    , y(p_y)
  {
  }

  constexpr vector2d operator+(vector2d const& b)
  {
    return vector2d(x + b.x, y + b.y);
  }
  constexpr vector2d operator-(vector2d const& b)
  {
    return vector2d(x - b.x, y - b.y);
  }
  constexpr vector2d operator*(float s)
  {
    return vector2d(x * s, y * s);
  }
  constexpr vector2d operator/(float s)
  {
    return vector2d(x / s, y / s);
  }
  /**
   * @brief Compute the dot product
   *
   * @param a Vector a
   * @param b Vector b
   * @return Dot product of `a` and `b`
   */
  static constexpr float dot(vector2d const& a, vector2d const& b)
  {
    return a.x * b.x + a.y * b.y;
  }

  /**
   * @brief Returns a vector rotated 90 degrees counter clockwise when looking
   * from above (x points towards the right, y points upwards)
   *
   * @return Returns a vector rotated 90 degrees counter clockwise when looking
   * from above
   */
  static constexpr vector2d rotate_90_ccw(vector2d const& a)
  {
    return vector2d(-a.y, a.x);
  }

  /**
   * @brief Returns a vector rotated 90 degrees clockwise when looking from
   * above (x points towards the right, y points upwards)
   *
   * @return Returns a vector rotated 90 degrees clockwise when looking from
   * above
   */
  static constexpr vector2d rotate_90_cw(vector2d const& a)
  {
    return vector2d(a.y, -a.x);
  }

  /**
   * @brief Returns the signed angle in radians between a vector and the upwards
   * vector (positive y-axis). Returns a negative angle for vectors to the left
   * of the y-axis. Returns a positive angle for vectors to the right of the
   * y-axis.
   *
   * @param a Vector to find angle of
   * @return Returns the signed angle in radians between a vector and the
   * upwards vector (positive y-axis).
   */
  static constexpr float bearing_angle(vector2d const& a)
  {
    return atan2(a.x, a.y);
  }

  /**
   * @brief Returns the angle clockwise in radians between a vector and the
   * upwards vector (positive y-axis). Returns a positive angle between 0 and
   * 2pi.
   *
   * @param a Vector to find angle of
   * @return Returns the angle clockwise in radians between a vector and the
   * upwards vector (positive y-axis).
   */
  static constexpr float bearing_angle_2pi(vector2d const& a)
  {
    float angle = bearing_angle(a);
    if (angle < 0) {
      return angle + std::numbers::pi * 2;
    }
    return angle;
  }

  /**
   * @brief Returns the signed angle in radians between a vector and the right
   * vector (positive x-axis). Returns a negative angle for vectors to the below
   * of the x-axis. Returns a positive angle for vectors to the above of the
   * x-axis.
   *
   * @param a Vector to find angle of
   * @return Returns the signed angle in radians between a vector and the right
   * vector (positive x-axis).
   */
  static constexpr float polar_angle(vector2d const& a)
  {
    return atan2(a.y, a.x);
  }

  /**
   * @brief Returns the counter-clockwise angle in radians between a vector and
   * the right vector (positive x-axis). Returns a positive angle between 0 and
   * 2pi.
   *
   * @param a Vector to find angle of
   * @return Returns the counter-clockwise angle in radians between a vector and
   * the right vector (positive x-axis)
   */
  static constexpr float polar_angle_2pi(vector2d const& a)
  {
    float angle = polar_angle(a);
    if (angle < 0) {
      return angle + std::numbers::pi * 2;
    }
    return angle;
  }

  /**
   * @brief Return a vector a distance `r` from the origin and has an clockwise
   * angle `bearing` from the positive y-axis
   *
   * @param r Distance of vector from origin
   * @param bearing Clockwise angle from positive y-axis
   * @return Return a vector a distance `r` from the origin and has an clockwise
   * angle `bearing` from the positive y-axis
   */
  static constexpr vector2d from_bearing(float r, float bearing)
  {
    return vector2d(r * sinf(bearing), r * cosf(bearing));
  }
  /**
   * @brief Return a vector a distance `r` from the origin and has an
   * counter-clockwise angle `theta` from the positive x-axis
   *
   * @param r Distance of vector from origin
   * @param theta Counter-Clockwise angle from positive x-axis
   * @return Return a vector a distance `r` from the origin and has an
   * counter-clockwise angle `theta` from the positive x-axis
   */
  static constexpr vector2d from_polar(float r, float theta)
  {
    return vector2d(r * cosf(theta), r * sinf(theta));
  }

  /**
   * @brief Euclidean length of a vector
   *
   * @param a Vector to calculate length of
   * @return Euclidean length of a vector
   */
  static constexpr float length(vector2d const& a)
  {
    return sqrt(vector2d::length_squared(a));
  }
  /**
   * @brief Euclidean length of a vector, squared
   *
   * @param a Vector to calculate squared length of
   * @return Euclidean length of a vector, squared
   */
  static constexpr float length_squared(vector2d const& a)
  {
    return vector2d::dot(a, a);
  }

  /**
   * @brief Angle between two vectors, in radians
   *
   * @param a Vector a
   * @param b Vector b
   * @return Angle between two vectors, in radians
   */
  static constexpr float angle_between(vector2d const& a, vector2d const& b)
  {
    return acos(vector2d::cos_of_angle_between(a, b));
  }

  /**
   * @brief Cosine of the angle between two vectors.
   *
   * @param a Vector a
   * @param b Vector b
   * @return Cosine of the angle between two vectors.
   */
  static constexpr float cos_of_angle_between(vector2d const& a,
                                              vector2d const& b)
  {
    return vector2d::dot(a, b) / (vector2d::length(a) * vector2d::length(b));
  }
  /**
   * @brief Non-Standard "Cross Product" of 2 2d vectors.
   * Used to determine whether the angle from `a` to `b` is clockwise or
   * counter-clockwise
   *
   * @param a Vector a
   * @param b Vector b
   * @return Returns the z component of the resulting 3d cross product with
   * their "z" components set to 0
   */
  static constexpr float cross_2d(vector2d const& a, vector2d const& b)
  {
    return (a.x * b.y) - (a.y * b.x);
  }

  /**
   * @brief Determines if a vector is clockwise or counter-clockwise relative to
   * another vector.
   *
   * @param a Vector a
   * @param b Vector b
   * @return Returns true if the angle from `a` to `b` is clockwise or false if
   * counter-clockwise
   */
  static constexpr bool is_clockwise(vector2d const& a, vector2d const& b)
  {
    return cross_2d(a, b) < 0;
  }
};
}  // namespace sjsu::drive

// sjsu::drive::vector2d&& operator+(const sjsu::drive::vector2d& a, const
// sjsu::drive::vector2d& b); sjsu::drive::vector2d&& operator-(const
// sjsu::drive::vector2d& a, const sjsu::drive::vector2d& b);
// sjsu::drive::vector2d&& operator*(const sjsu::drive::vector2d& a, float s);
constexpr sjsu::drive::vector2d operator*(float s,
                                          sjsu::drive::vector2d const& a)
{
  return sjsu::drive::vector2d(a.x * s, a.y * s);
}