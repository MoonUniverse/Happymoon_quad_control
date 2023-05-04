#include "happymoon_control.h"

namespace happymoon_control
{
  HappyMoonControl::HappyMoonControl(rclcpp::NodeOptions options)
      : Node("happymoon_quad_control", options),
        // subscribe to odometry data
        odometry_sub_(create_subscription<nav_msgs::msg::Odometry>(
            "visual_slam/tracking/odometry", rclcpp::QoS(10),
            std::bind(&HappyMoonControl::ReadOdomData, this, std::placeholders::_1))),
        happymoon_config{
            declare_parameter<double>("kpxy", 10.0),
            declare_parameter<double>("kdxy", 4.0),
            declare_parameter<double>("kpz", 15.0),
            declare_parameter<double>("kdz", 6.0),
            declare_parameter<double>("krp", 12.0),
            declare_parameter<double>("kyaw", 5.0),

            declare_parameter<double>("refVelXYKp", 0.5),
            declare_parameter<double>("refVelZKp", 0.5),
            declare_parameter<double>("refVelHeadingKp", 0.5),
            declare_parameter<double>("refVelRateheadingKp", 0.5),

            declare_parameter<double>("ref_vxy_error_max", 0.5),
            declare_parameter<double>("ref_vz_error_max", 0.5),
            declare_parameter<double>("pxy_error_max", 0.6),
            declare_parameter<double>("vxy_error_max", 1.0),
            declare_parameter<double>("pz_error_max", 0.3),
            declare_parameter<double>("vz_error_max", 0.75),
            declare_parameter<double>("yaw_error_max", 0.7),

            declare_parameter<double>("k_drag_x", 0.0),
            declare_parameter<double>("k_drag_y", 0.0),
            declare_parameter<double>("k_drag_z", 0.0),

            declare_parameter<double>("k_thrust_horz", 1.0)}
  {
    RCLCPP_INFO(this->get_logger(), "HappyMoonControl Node has been initialized.");
  }

  HappyMoonControl::~HappyMoonControl()
  {
    RCLCPP_INFO(this->get_logger(), "HappyMoonControl Node has been destroyed.");
  }

  void HappyMoonControl::ReadOdomData(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg == nullptr)
    {
      RCLCPP_ERROR(this->get_logger(), "Odometry data is null.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Odometry data received.");
    QuadStateEstimateData happymoon_state_estimate;
    QuadStateReferenceData happymoon_state_reference;
    happymoon_state_estimate = QuadStateEstimate(*msg);
    happymoon_state_reference = QuadReferenceState(
        happymoon_reference, happymoon_state_estimate, happymoon_config);
    ControlRun(happymoon_state_estimate, happymoon_state_reference,
               happymoon_config);
  }

  QuadStateEstimateData HappyMoonControl::QuadStateEstimate(
      const nav_msgs::msg::Odometry &state_estimate_msg)
  {
    QuadStateEstimateData happymoon_state_estimate;
    happymoon_state_estimate.position =
        geometryToEigen_.geometryToEigen(state_estimate_msg.pose.pose.position);
    happymoon_state_estimate.velocity =
        geometryToEigen_.geometryToEigen(state_estimate_msg.twist.twist.linear);
    happymoon_state_estimate.orientation = geometryToEigen_.geometryToEigen(
        state_estimate_msg.pose.pose.orientation);
    happymoon_state_estimate.roll_pitch_yaw =
        mathcommon_.quaternionToEulerAnglesZYX(
            happymoon_state_estimate.orientation);
    happymoon_state_estimate.bodyrates =
        geometryToEigen_.geometryToEigen(state_estimate_msg.twist.twist.angular);

    return happymoon_state_estimate;
  }

  QuadStateReferenceData
  HappyMoonControl::QuadReferenceState(HappymoonReference ref_msg,
                                       QuadStateEstimateData est_msg,
                                       const PositionControllerParams &config)
  {
    QuadStateReferenceData happymoon_reference_state;
    happymoon_reference_state.position.x() = ref_msg.position.x();
    happymoon_reference_state.position.y() = ref_msg.position.y();
    happymoon_reference_state.position.z() = ref_msg.position.z();
    happymoon_reference_state.heading = ref_msg.heading;
    happymoon_reference_state.velocity.x() =
        happymoon_config.refVelXYKp *
        (ref_msg.position.x() - est_msg.position.x());
    mathcommon_.limit(&happymoon_reference_state.velocity.x(),
                      -config.ref_vxy_error_max, config.ref_vxy_error_max);

    happymoon_reference_state.velocity.y() =
        happymoon_config.refVelXYKp *
        (ref_msg.position.y() - est_msg.position.y());
    mathcommon_.limit(&happymoon_reference_state.velocity.y(),
                      -config.ref_vxy_error_max, config.ref_vxy_error_max);

    happymoon_reference_state.velocity.z() =
        happymoon_config.refVelZKp *
        (ref_msg.position.z() - est_msg.position.z());
    mathcommon_.limit(&happymoon_reference_state.velocity.z(),
                      -config.ref_vz_error_max, config.ref_vz_error_max);

    return happymoon_reference_state;
  }

  void HappyMoonControl::ControlRun(const QuadStateEstimateData &state_estimate,
                                    const QuadStateReferenceData &state_reference,
                                    const PositionControllerParams &config)
  {
    ControlCommand command;
    // Compute desired control commands
    const Eigen::Vector3d pid_error_accelerations =
        computePIDErrorAcc(state_estimate, state_reference, config);
    // std::cout << "pid_error_accelerations: " << std::endl <<
    // pid_error_accelerations << std::endl;
    const Eigen::Vector3d desired_acceleration =
        pid_error_accelerations - kGravity_;
    command.collective_thrust = computeDesiredCollectiveMassNormalizedThrust(
        state_estimate.orientation, desired_acceleration);
    const Eigen::Quaterniond desired_attitude =
        computeDesiredAttitude(desired_acceleration, state_reference.heading,
                               state_estimate.orientation);

    const Eigen::Vector3d desired_r_p_y =
        mathcommon_.quaternionToEulerAnglesZYX(desired_attitude);
    geometry_msgs::msg::Vector3 desired_r_p_y_rate;
    desired_r_p_y_rate.x = 0.5 * desired_r_p_y.x();
    desired_r_p_y_rate.y = 0.5 * desired_r_p_y.y();
    desired_r_p_y_rate.z = 0.2 * desired_r_p_y.z();

    //     mavros_msgs::AttitudeTarget expect_px;

    //     if (current_state.armed && current_state.mode == "OFFBOARD")
    //     {
    //       expect_px.header.frame_id = "base_link";
    //       expect_px.header.stamp = ros::Time::now();
    //       expect_px.type_mask = 7;
    //       expect_px.orientation = geometryToEigen_.eigenToGeometry(desired_attitude);
    //       expect_px.body_rate = desired_r_p_y_rate;
    // #define simulation
    // #ifdef simulation
    //       expect_px.thrust = command.collective_thrust / config.k_thrust_horz;
    //       ROS_ERROR("expect_px.thrust  :%f", expect_px.thrust);
    // #endif

    // // [0.13018744 0.12771589]
    // // #define realquad
    // #ifdef realquad
    //       expect_px.thrust =
    //           config.k_thrust_horz *
    //           (0.13018744 * command.collective_thrust / 7.1 + 0.12771589);
    //       ROS_ERROR("expect_px.thrust  :%f", expect_px.thrust);
    // #endif
    //     }
    //     else
    //     {
    //       expect_px.header.frame_id = "base_link";
    //       expect_px.header.stamp = ros::Time::now();
    //       expect_px.orientation = geometryToEigen_.eigenToGeometry(desired_attitude);
    //       expect_px.body_rate = desired_r_p_y_rate;
    //       expect_px.thrust = 0;
    //     }
    //     ctrlExpectAngleThrustPX4.publish(expect_px);
  }

  Eigen::Vector3d HappyMoonControl::computePIDErrorAcc(
      const QuadStateEstimateData &state_estimate,
      const QuadStateReferenceData &reference_state,
      const PositionControllerParams &config)
  {
    // Compute the desired accelerations due to control errors in world frame
    // with a PID controller
    Eigen::Vector3d acc_error;

    // x acceleration
    double x_pos_error =
        reference_state.position.x() - state_estimate.position.x();
    mathcommon_.limit(&x_pos_error, -config.pxy_error_max, config.pxy_error_max);

    double x_vel_error =
        reference_state.velocity.x() - state_estimate.velocity.x();
    mathcommon_.limit(&x_vel_error, -config.vxy_error_max, config.vxy_error_max);

    acc_error.x() = config.kpxy * x_pos_error + config.kdxy * x_vel_error;

    // y acceleration
    double y_pos_error =
        reference_state.position.y() - state_estimate.position.y();
    mathcommon_.limit(&y_pos_error, -config.pxy_error_max, config.pxy_error_max);

    double y_vel_error =
        reference_state.velocity.y() - state_estimate.velocity.y();
    mathcommon_.limit(&y_vel_error, -config.vxy_error_max, config.vxy_error_max);

    acc_error.y() = config.kpxy * y_pos_error + config.kdxy * y_vel_error;

    // z acceleration
    double z_pos_error =
        reference_state.position.z() - state_estimate.position.z();
    mathcommon_.limit(&z_pos_error, -config.pz_error_max, config.pz_error_max);

    double z_vel_error =
        reference_state.velocity.z() - state_estimate.velocity.z();
    mathcommon_.limit(&z_vel_error, -config.vz_error_max, config.vz_error_max);

    acc_error.z() = config.kpz * z_pos_error + config.kdz * z_vel_error;

    return acc_error;
  }

  double HappyMoonControl::computeDesiredCollectiveMassNormalizedThrust(
      const Eigen::Quaterniond &attitude_estimate,
      const Eigen::Vector3d &desired_acc)
  {
    const Eigen::Vector3d body_z_axis =
        attitude_estimate * Eigen::Vector3d::UnitZ();

    double normalized_thrust = desired_acc.dot(body_z_axis);
    if (normalized_thrust < kMinNormalizedCollectiveThrust_)
    {
      normalized_thrust = kMinNormalizedCollectiveThrust_;
    }
    if (normalized_thrust > kMaxNormalizedCollectiveThrust_)
    {
      normalized_thrust = kMaxNormalizedCollectiveThrust_;
    }
    return normalized_thrust;
  }

  Eigen::Quaterniond HappyMoonControl::computeDesiredAttitude(
      const Eigen::Vector3d &desired_acceleration, const double reference_heading,
      const Eigen::Quaterniond &attitude_estimate)
  {
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
        Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));

    // Compute desired orientation
    const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

    Eigen::Vector3d z_B;
    if (almostZero(desired_acceleration.norm()))
    {
      // In case of free fall we keep the thrust direction to be the estimated one
      // This only works assuming that we are in this condition for a very short
      // time (otherwise attitude drifts)
      z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
    }
    else
    {
      z_B = desired_acceleration.normalized();
    }

    const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
    const Eigen::Vector3d x_B =
        computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);

    const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();

    // From the computed desired body axes we can now compose a desired attitude
    const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());

    const Eigen::Quaterniond desired_attitude(R_W_B);

    return desired_attitude;
  }

  Eigen::Vector3d HappyMoonControl::computeRobustBodyXAxis(
      const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
      const Eigen::Vector3d &y_C, const Eigen::Quaterniond &attitude_estimate)
  {
    Eigen::Vector3d x_B = x_B_prototype;

    if (almostZero(x_B.norm()))
    {
      // if cross(y_C, z_B) == 0, they are collinear =>
      // every x_B lies automatically in the x_C - z_C plane

      // Project estimated body x-axis into the x_C - z_C plane
      const Eigen::Vector3d x_B_estimated =
          attitude_estimate * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d x_B_projected =
          x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
      if (almostZero(x_B_projected.norm()))
      {
        // Not too much intelligent stuff we can do in this case but it should
        // basically never occur
        x_B = x_C;
      }
      else
      {
        x_B = x_B_projected.normalized();
      }
    }
    else
    {
      x_B.normalize();
    }

    // if the quad is upside down, x_B will point in the "opposite" direction
    // of x_C => flip x_B (unfortunately also not the solution for our problems)
    //  if (x_B.dot(x_C) < 0.0)
    //  {
    //    x_B = -x_B;
    //  }

    return x_B;
  }


  bool HappyMoonControl::almostZero(const double value)
  {
    return fabs(value) < kAlmostZeroValueThreshold_;
  }

  bool HappyMoonControl::almostZeroThrust(const double thrust_value)
  {
    return fabs(thrust_value) < kAlmostZeroThrustThreshold_;
  }

}