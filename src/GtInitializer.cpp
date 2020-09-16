//
// Created by yubaidi at 2020-09-03
// An initializer using parameters passed in from server parameters
//

#include "Initializer/GtInitializer.h"

using namespace std;
using namespace Eigen;

namespace larvio {

bool StaticInitializer::tryGtInit(const std::vector<ImuData>& imu_msg_buffer,
    MonoCameraMeasurementPtr img_msg) {
 
  // Since we are grabbing the parameters from configuration, 
  bInit = true;

  return true;
}


void StaticInitializer::initializeGravityAndBias(const double& time_bound,
    const std::vector<ImuData>& imu_msg_buffer) {
  // Initialize gravity and gyro bias.
  Vector3d sum_angular_vel = Vector3d::Zero();
  Vector3d sum_linear_acc = Vector3d::Zero();

  int usefulImuSize = 0;
  double last_imu_time;
  for (const auto& imu_msg : imu_msg_buffer) {
    double imu_time = imu_msg.timeStampToSec;
    if (imu_time < lower_time_bound) continue;
    if (imu_time > time_bound) break;

    sum_angular_vel += Tg*(imu_msg.angular_velocity-As*Ma*imu_msg.linear_acceleration);
    sum_linear_acc += Ma*imu_msg.linear_acceleration;

    usefulImuSize++;

    last_imu_time = imu_time;
  }

  // Compute gyro bias.
  gyro_bias = sum_angular_vel / usefulImuSize;

  // This is the gravity in the IMU frame.
  Vector3d gravity_imu =
    sum_linear_acc / usefulImuSize;

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  double gravity_norm = gravity_imu.norm();
  Vector3d gravity_world(0.0, 0.0, -gravity_norm);

  // Set rotation
  Quaterniond q0_w_i = Quaterniond::FromTwoVectors(
    gravity_imu, -gravity_world);	
  orientation = q0_w_i.coeffs();  

  // Set other state and timestamp
  state_time = last_imu_time;
  position = Vector3d(0.0, 0.0, 0.0);
  velocity = Vector3d(0.0, 0.0, 0.0);
  acc_bias = Vector3d(0.0, 0.0, 0.0);

  printf("Inclinometer-initializer completed by using %d imu data !!!\n\n",usefulImuSize);

  return;
}


void StaticInitializer::assignInitialState(std::vector<ImuData>& imu_msg_buffer,
        Eigen::Vector3d& m_gyro_old, Eigen::Vector3d& m_acc_old, IMUState& imu_state) {
  if (!bInit) {
    printf("Cannot assign initial state before initialization !!!\n");
    return;
  }

  // Remove used imu data
  int usefulImuSize = 0;
  for (const auto& imu_msg : imu_msg_buffer) {
    double imu_time = imu_msg.timeStampToSec;
    if (imu_time > state_time) break;
    usefulImuSize++;
  }
  if (usefulImuSize>=imu_msg_buffer.size())
    usefulImuSize--;

  // Initialize last m_gyro and last m_acc
  const auto& imu_msg = imu_msg_buffer[usefulImuSize];
  m_gyro_old = imu_msg.angular_velocity;
  m_acc_old = imu_msg.linear_acceleration;

  // Earse used imu data
  imu_msg_buffer.erase(imu_msg_buffer.begin(),
      imu_msg_buffer.begin()+usefulImuSize);

  // Set initial state
  imu_state.time = state_time;
  imu_state.gyro_bias = gyro_bias;
  imu_state.acc_bias = acc_bias;
  imu_state.orientation = orientation;
  imu_state.position = position;
  imu_state.velocity = velocity;

  return;
  }
}