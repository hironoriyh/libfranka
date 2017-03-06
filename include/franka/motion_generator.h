#pragma once

#include <memory>

/**
 * @file motion_generator.h
 * Contains the Motion Generator classes.
 */

namespace franka {

class Robot;

/**
 * Motion generator base class.
 */
class MotionGenerator {
 public:
  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Motion generator to move
   */
  MotionGenerator(MotionGenerator&& motion_generator) noexcept;

  /**
   * Creates a new motion generator.
   * @param[in] robot Robot instance
   */
  explicit MotionGenerator(Robot& robot) noexcept;

  /**
   * Destructs the motion generator.
   */
  virtual ~MotionGenerator() noexcept;

  MotionGenerator(const MotionGenerator&) = delete;

 protected:
  /**
   * Robot instance.
   */
  Robot& robot;
};

/**
 * Allows to stream Cartesian pose commands to the franka robot
 */
class CartesianPoseMotionGenerator : public MotionGenerator {
 public:
  /**
   * Creates a new motion generator.
   * @param[in] robot Robot instance
   */
  explicit CartesianPoseMotionGenerator(Robot& robot);

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Motion generator to move
   */
  CartesianPoseMotionGenerator(
      CartesianPoseMotionGenerator&& motion_generator) noexcept;

  /**
   * Destructs the motion generator.
   */
  ~CartesianPoseMotionGenerator() noexcept override;

  /**
   * Tries to set a cartesian motion command as a homogeneous transformation.
   *
   * @param[in] desired_pose Homogeneous transformation
   * \f${}_O \mathbf{T}_{EE,d}\f$, column major, that transforms from the
   * end-effector frame \f$EE\f$ to base frame \f$O\f$
   * @throw MotionGeneratorException when non-valid pose was passed
   */
  void setDesiredPose(const std::array<double, 16>& desired_pose) noexcept;

  /**
   * Checks a cartesian pose command (a homogeneous transformation) for
   * validity.
   *
   * @param[in] transform Homogeneous transformation to be checked,
   * passed as column major array
   * @return True if transformation has ortho-normal rotation matrix,
   * the last row is [0 0 0 1] and the array defines a column major matrix
   */
  static bool checkHomogeneousTransformation(std::array<double, 16> transform);
};

/**
 * Allows to stream Cartesian velocity commands to the franka robot
 */
class CartesianVelocityMotionGenerator : public MotionGenerator {
 public:
  /**
   * Creates a new motion generator.
   * @param[in] robot Robot instance
   */
  explicit CartesianVelocityMotionGenerator(Robot& robot);

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Motion generator to move
   */
  CartesianVelocityMotionGenerator(
      CartesianVelocityMotionGenerator&& motion_generator) noexcept;

  /**
   * Destructs the motion generator.
   */
  ~CartesianVelocityMotionGenerator() noexcept override;

  /**
   * Sets a desired Cartesian velocity command.
   *
   * @param[in] desired_velocity Desired Cartesian velocity w.r.t. O-frame
   * {dx in [m/s], dx in [m/s], dz in [m/s],
   * omegax in [rad/s], omegay in [rad/s], omegaz in [rad/s]}
   */
  void setDesiredVelocity(
      const std::array<double, 6>& desired_velocity) noexcept;
};

/**
 * Allows to stream joint pose commands to the franka robot
 */
class JointPoseMotionGenerator : public MotionGenerator {
 public:
  /**
   * Creates a new motion generator.
   * @param[in] robot Robot instance
   */
  explicit JointPoseMotionGenerator(Robot& robot);

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Motion generator to move
   */
  JointPoseMotionGenerator(
      JointPoseMotionGenerator&& motion_generator) noexcept;

  /**
   * Destructs the motion generator.
   */
  ~JointPoseMotionGenerator() noexcept override;

  /**
   * Sets a desired joint pose command.
   *
   * @param[in] desired_pose Desired joint angles in [rad]
   */
  void setDesiredPose(const std::array<double, 7>& desired_pose) noexcept;
};

/**
 * Allows to stream joint velocity commands to the franka robot
 */
class JointVelocityMotionGenerator : public MotionGenerator {
 public:
  /**
   * Creates a new motion generator.
   * @param[in] robot Robot instance
   */
  explicit JointVelocityMotionGenerator(Robot& robot);

  /**
   * Moves a motion generator.
   *
   * @param[in] motion_generator Motion generator to move
   */
  JointVelocityMotionGenerator(
      JointVelocityMotionGenerator&& motion_generator) noexcept;

  /**
   * Destructs the motion generator.
   */
  ~JointVelocityMotionGenerator() noexcept override;

  /**
   * Sets a desired joint velocity command.
   *
   * @param[in] desired_velocity Desired joint velocities in [rad/s]
   */
  void setDesiredVelocity(
      const std::array<double, 7>& desired_velocity) noexcept;
};

}  // namespace franka