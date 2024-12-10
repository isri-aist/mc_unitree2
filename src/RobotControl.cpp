
#include <mc_rtc/logging.h>
#include "RobotControl.h"

namespace mc_unitree
{

/**
 * @brief Interface constructor and destructor
 * DDS connection with robot using specified parameters.
 * Control level is set to LOW-level.
 *
 * @param config_param Configuration file parameters
 */
RobotControl::RobotControl(mc_rbdyn::Robot * robot, const RobotConfigParameter & config_param)
  : robot_(robot)
{
  stateIn_.qIn_.resize(robot_->refJointOrder().size());
  stateIn_.dqIn_.resize(robot_->refJointOrder().size());
  stateIn_.tauIn_.resize(robot_->refJointOrder().size());
  stateIn_.footForceIn_.resize(robot_->forceSensors().size());
  
  /* Initialize */
  ChannelFactory::Instance()->Init(0, config_param.network_);
  
  low_cmd.head()[0] = 0xFE;
  low_cmd.head()[1] = 0xEF;
  low_cmd.level_flag() = 0xFF;
  low_cmd.gpio() = 0;
  
  for (const auto jname : robot_->refJointOrder())
  {
    auto jointId = robot_->jointIndexByName(jname);
    low_cmd.motor_cmd()[jointId].mode() = (0x01);   // motor switch to servo (PMSM) mode
    low_cmd.motor_cmd()[jointId].q() = (PosStopF);
    low_cmd.motor_cmd()[jointId].kp() = (0);
    low_cmd.motor_cmd()[jointId].dq() = (VelStopF);
    low_cmd.motor_cmd()[jointId].kd() = (0);
    low_cmd.motor_cmd()[jointId].tau() = (0);
  }
  
  /*create publisher*/
  lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
  lowcmd_publisher->InitChannel();
  
  /*create subscriber*/
  lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
  lowstate_subscriber->InitChannel(std::bind(&RobotControl::LowStateMessageHandler, this, std::placeholders::_1), 1);
}

/**
 * @brief Interface constructor and destructor
 * Running simulation only. No connection to real robot.
 *
 * @param config_param Configuration file parameters
 * @param host "simulation" only
 */
RobotControl::RobotControl(mc_rbdyn::Robot * robot, const RobotConfigParameter & config_param, const std::string & host)
  : robot_(robot)
{
  stateIn_.qIn_.resize(robot_->refJointOrder().size());
  stateIn_.dqIn_.resize(robot_->refJointOrder().size());
  stateIn_.tauIn_.resize(robot_->refJointOrder().size());
}
  
void RobotControl::LowStateMessageHandler(const void* message)
{
  low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

uint32_t RobotControl::crc32_core(uint32_t* ptr, uint32_t len)
{
  unsigned int xbit = 0;
  unsigned int data = 0;
  unsigned int CRC32 = 0xFFFFFFFF;
  const unsigned int dwPolynomial = 0x04c11db7;
  
  for (unsigned int i = 0; i < len; i++)
  {
    xbit = 1 << 31;
    data = ptr[i];
    for (unsigned int bits = 0; bits < 32; bits++)
    {
      if (CRC32 & 0x80000000)
      {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      }
      else
      {
        CRC32 <<= 1;
      }
      
      if (data & xbit)
        CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }
  
  return CRC32;
}

  
/**
 * @brief Stores the data received by "recvData ()" in the specified argument
 * 
 * @param state Current sensor values information
 */
void RobotControl::getState(RobotSensorInfo & state)
{
  /*  Set current sensor values */
  for (const auto jname : robot_->refJointOrder())
  {
    auto jointId = robot_->jointIndexByName(jname);
    auto & motor_state = low_state.motor_state()[jointId];
    state.qIn_[jointId] = (double)motor_state.q();
    state.dqIn_[jointId] = (double)motor_state.dq();
    state.tauIn_[jointId] = (double)motor_state.tau_est();
  }
  for(int i = 0; i < state.footForceIn_.size(); i++)
  {
    state.footForceIn_[i] = (double)low_state.foot_force()[i];
  }
  
  /* Set body(imu) sensor values */
  for(int i = 0; i < 3; i++)
  {
    auto & imu_state = low_state.imu_state();
    state.accIn_(i) = (double)imu_state.accelerometer()[i];
    state.rpyIn_(i) = (double)imu_state.rpy()[i];
    state.rateIn_(i) = (double)imu_state.gyroscope()[i];
  }
}
  
/**
 * @brief Set start state values for simulation
 * 
 * @param stance Value defined by RobotModule
 * @param state Current sensor values information
 */
void RobotControl::setStartState(const std::map<std::string, std::vector<double>> & stance,
                                   RobotSensorInfo & state)
{
  /* Start stance */
  for (const auto jname : robot_->refJointOrder())
  {
    auto jointId = robot_->jointIndexByName(jname);
    state.qIn_[jointId] = 0.0;
    state.dqIn_[jointId] = 0.0;
    state.tauIn_[jointId] = 0.0;
  }

  /* Set position(Angle) values */
  try
  {
    for (auto joint : stance)
    {
      if (robot_->hasJoint(joint.first))
        state.qIn_[robot_->jointIndexByName(joint.first)] = joint.second[0];
    }
  }
  catch(const std::exception& e)
  {
    mc_rtc::log::error("[mc_unitree] Failed to get the value defined by RobotModule: {}", e.what());
    return;
  }

  /* Set foot force sensor values */
  for(int i = 0; i < state.footForceIn_.size(); i++)
  {
    state.footForceIn_[i] = 0;
  }
  
  /* Set body(imu) sensor values */
  state.accIn_.setZero();
  state.rpyIn_.setZero();
  state.rateIn_.setZero();
  
  /* copy to private member state */
  stateIn_ = state;
};

/**
 * @brief Loop back the value of "data" to "state"
 * 
 * @param data Command data for sending to Go2 robot
 * @param state Current sensor values information
 */
void RobotControl::loopbackState(const RobotCommandData & data, RobotSensorInfo & state)
{
  /*  Set current sensor values */
  for (const auto jname : robot_->refJointOrder())
  {
    auto jointId = robot_->jointIndexByName(jname);
    state.qIn_[jointId] = data.qOut_[jointId];
    state.dqIn_[jointId] = data.dqOut_[jointId];
    state.tauIn_[jointId] = data.tauOut_[jointId];

    if (state.qIn_[jointId] > robot_->qu()[jointId][0])
      state.qIn_[jointId] = robot_->qu()[jointId][0];
    else if (state.qIn_[jointId] < robot_->ql()[jointId][0])
      state.qIn_[jointId] = robot_->ql()[jointId][0];
  }
}

/**
 * @brief Set send command data to Go2 robot
 * 
 * @param cm ControlMode (position, velocity or torque)
 * @param data Command data for sending to Go2 robot
 * @param position_only true: Set to 0 except for position
 * @return true Success
 * @return false Could not send
 */
bool RobotControl::setSendCmd(const ControlMode cm, const RobotCommandData & data, bool position_only)
{
  switch(cm)
  {
  case mc_unitree::ControlMode::Position:
    for (const auto jname : robot_->refJointOrder())
    {
      auto jointId = robot_->jointIndexByName(jname);
      auto & motor_cmd = low_cmd.motor_cmd()[jointId];
      motor_cmd.q() = (float)data.qOut_[jointId];
      motor_cmd.kp() = 8.0;
      motor_cmd.dq() = (float)data.dqOut_[jointId];
      motor_cmd.kd() = 1.0;
      motor_cmd.tau() = 0;
    }
    break;
  case mc_unitree::ControlMode::Velocity:
    for (const auto jname : robot_->refJointOrder())
    {
      auto jointId = robot_->jointIndexByName(jname);
      auto & motor_cmd = low_cmd.motor_cmd()[jointId];
      motor_cmd.q() = mc_unitree::PosStopF;
      motor_cmd.kp() = 0;
      motor_cmd.dq() = (float)data.dqOut_[jointId];
      motor_cmd.kd() = 4.0;
      motor_cmd.tau() = 0;
    }
    break;
  case mc_unitree::ControlMode::Torque:
    for (const auto jname : robot_->refJointOrder())
    {
      auto jointId = robot_->jointIndexByName(jname);
      auto & motor_cmd = low_cmd.motor_cmd()[jointId];
      motor_cmd.q() = mc_unitree::PosStopF;
      motor_cmd.kp() = 0;
      motor_cmd.dq() = mc_unitree::VelStopF;
      motor_cmd.kd() = 0;
      motor_cmd.tau() = (float)data.tauOut_[jointId];
    }
    break;
  }
  if(position_only)
  {
    for (const auto jname : robot_->refJointOrder())
    {
      auto jointId = robot_->jointIndexByName(jname);
      auto & motor_cmd = low_cmd.motor_cmd()[jointId];
      motor_cmd.q() = (float)data.qOut_[jointId];
      motor_cmd.kp() = 0;
      motor_cmd.dq() = 0;
      motor_cmd.kd() = 0;
      motor_cmd.tau() = 0;
    }
  }
  low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
  
#if 0
  /* Ensuring safety */
  motiontime++;
  if(motiontime > 10)
  {
    if(cm == mc_unitree::ControlMode::Position)
    {
      safe_->PositionLimit(low_cmd);
    }
    safe_->PowerProtect(low_cmd, state_, 1);
  }
#endif
  
#if 0
  //printf("[send] qOut: %lf, dqOut:%lf, tauOut: %lf, q: %lf, dq: %lf, tau: %lf\n",
  //data.qOut_[2], data.dqOut_[2], data.tauOut_[2],
  //state_.motorState[2].q,
  //state_.motorState[2].dq,
  //state_.motorState[2].tauEst);
#endif

  try
  {
    /* Send command */
    lowcmd_publisher->Write(low_cmd);
  }
  catch(const std::exception& e)
  {
    mc_rtc::log::error("[mc_unitree] Could not send data from Go2 robot due to library error: {}", e.what());
    return false;
  }
  
  return true;
}

} // namespace mc_unitree
