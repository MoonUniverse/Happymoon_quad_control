#ifndef HAPPYMOON_CONTROL_H
#define HAPPYMOON_CONTROL_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <eigen3/Eigen/Dense>
#include "geometry_eigen_conversions.h"
#include "math_common.h"

using namespace std;

struct QuadStateEstimateData
{
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d roll_pitch_yaw;
  Eigen::Vector3d bodyrates; // Body rates are represented in body coordinates
};

struct QuadStateReferenceData
{
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d roll_pitch_yaw;
  double heading;
  double heading_rate;
  double heading_acceleration;
};

struct HappymoonReference
{
  Eigen::Vector3d position;
  double heading;
  double heading_rate;
};

struct ControlCommand
{
  rclcpp::Time timestamp;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d bodyangle;
  Eigen::Vector3d bodyrates;             // Body rates in body frame
  Eigen::Vector3d angular_accelerations; // Angular accelerations in body frame
  double collective_thrust;
};

struct PositionControllerParams
{
  double kpxy; // [1/s^2]
  double kdxy; // [1/s]

  double kpz; // [1/s^2]
  double kdz; // [1/s]

  double krp;  // [1/s]
  double kyaw; // [1/s]

  double refVelXYKp;
  double refVelZKp;
  double refVelHeadingKp;
  double refVelRateheadingKp;

  double ref_vxy_error_max; //[m/s]
  double ref_vz_error_max;  //[m/s]
  double pxy_error_max;     // [m]
  double vxy_error_max;     // [m/s]
  double pz_error_max;      // [m]
  double vz_error_max;      // [m/s]
  double yaw_error_max;     // [rad]

  // Whether or not to compensate for aerodynamic effects
  double k_drag_x; // x-direction rotor drag coefficient
  double k_drag_y; // y-direction rotor drag coefficient
  double k_drag_z; // z-direction rotor drag coefficient
  // thrust correction coefficient due to body horizontal velocity
  double k_thrust_horz;
};

namespace happymoon_control
{
  class HappyMoonControl : public rclcpp::Node
  {
  public:
    explicit HappyMoonControl(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
    virtual ~HappyMoonControl();

  private:
    const rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    void ReadOdomData(const nav_msgs::msg::Odometry::SharedPtr msg);

    // params
    HappymoonReference happymoon_reference;
    PositionControllerParams happymoon_config;
    ControlCommand happymoon_control_command;

    // state
    QuadStateEstimateData QuadStateEstimate(const nav_msgs::msg::Odometry &state_estimate_msg);
    QuadStateReferenceData QuadReferenceState(HappymoonReference ref_msg,
                                              QuadStateEstimateData est_msg,
                                              const PositionControllerParams &config);
    // control
    void ControlRun(const QuadStateEstimateData &state_estimate,
                    const QuadStateReferenceData &state_reference,
                    const PositionControllerParams &config);

    Eigen::Vector3d
    computePIDErrorAcc(const QuadStateEstimateData &state_estimate,
                       const QuadStateReferenceData &reference_state,
                       const PositionControllerParams &config);
    double computeDesiredCollectiveMassNormalizedThrust(
        const Eigen::Quaterniond &attitude_estimate,
        const Eigen::Vector3d &desired_acc);

    Eigen::Vector3d
    computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype,
                           const Eigen::Vector3d &x_C, const Eigen::Vector3d &y_C,
                           const Eigen::Quaterniond &attitude_estimate);

    Eigen::Quaterniond
    computeDesiredAttitude(const Eigen::Vector3d &desired_acceleration,
                           const double reference_heading,
                           const Eigen::Quaterniond &attitude_estimate);

    bool almostZero(const double value);
    bool almostZeroThrust(const double thrust_value);

    GeometryEigenConversions geometryToEigen_;
    MathCommon mathcommon_;

    // Constants
    const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
    static constexpr double kMinNormalizedCollectiveThrust_ = 9.81;
    static constexpr double kMaxNormalizedCollectiveThrust_ = 15.0;
    static constexpr double kAlmostZeroValueThreshold_ = 0.001;
    static constexpr double kAlmostZeroThrustThreshold_ = 0.01;
  };
}

#endif
