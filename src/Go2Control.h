#ifndef _GO2_CONTROL_H_
#define _GO2_CONTROL_H_

#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <vector>
#include <map>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

#include <mc_control/mc_controller.h>
#include "ControlMode.h"

//using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

namespace mc_unitree
{
  constexpr double PosStopF = (2.146E+9f);
  constexpr double VelStopF = (16000.0f);
  
  const std::string ROBOT_NAME = "go2";
  const std::string CONFIGURATION_FILE = "/usr/local/etc/mc_unitree/mc_rtc.yaml";
  
/**
 * @brief Configuration file parameters for mc_unitree
 */
struct Go2ConfigParameter
{
  Go2ConfigParameter()
  : network_(""), mode_(ControlMode::Position)
  {}
  /* Communication information with a real robot */
  /* Connection network */
  std::string network_;
  /* ControlMode : Position/Velocity/Torque (Velocity and Torque are not supported)*/
  ControlMode mode_ = ControlMode::Position;
};

/**
 * @brief Current sensor values information of Go2 robot
 */
struct Go2SensorInfo
{
  /* Position(Angle) values */
  std::vector<double> qIn_;
  /* Velocity values */
  std::vector<double> dqIn_;
  /* Torque values */
  std::vector<double> tauIn_;
  /* Orientation sensor */
  Eigen::Vector3d rpyIn_;
  /* Accelerometer */
  Eigen::Vector3d accIn_;
  /* Angular velocity */
  Eigen::Vector3d rateIn_;
  /* Foot force sensors */
  std::vector<double> footForceIn_;
};

/**
 * @brief Command data for sending to Go2 robot
 */
struct Go2CommandData
{
  /* Position(Angle) values */
  std::vector<double> qOut_;
  /* Velocity values */
  std::vector<double> dqOut_;
  /* Torque values */
  std::vector<double> tauOut_;
};

/**
 * @brief mc_rtc control interface for Go2 robot
 */
class Go2Control
{
protected:
  mc_rbdyn::Robot* robot_ = nullptr;
  
private:
  /* Communication information with a real robot */
  unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
  unitree_go::msg::dds_::LowState_ low_state{};  // default init
  
  /*publisher*/
  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
  /*subscriber*/
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
  
  /* Control loop status */
  bool running_ = true;
  /* Current sensor values information */
  Go2SensorInfo stateIn_;
  
  int Tpi = 0;
  int motiontime = 0;
    
  void LowStateMessageHandler(const void* messages);
  
  uint32_t crc32_core(uint32_t* ptr, uint32_t len);
  
public:
  /**
   * @brief Interface constructor and destructor
   * DDS connection with robot using specified parameters.
   * Control level is set to LOW-level.
   *
   * @param config_param Configuration file parameters
   */
  Go2Control(mc_rbdyn::Robot * robot, const Go2ConfigParameter & config_param);

  /**
   * @brief Interface constructor and destructor
   * Running simulation only. No connection to real robot.
   *
   * @param config_param Configuration file parameters
   * @param host "simulation" only
   */
  Go2Control(mc_rbdyn::Robot * robot, const Go2ConfigParameter & config_param, const std::string & host);

  ~Go2Control()
  {
  };

  /**
   * @brief Receive data from Go2 robot
   */
  void recvData();

  /**
   * @brief Send data to Go2 robot
   */
  void sendData();

  /**
   * @brief Stores the data received by "recvData ()" in the specified argument
   * 
   * @param state Current sensor values information
   */
  void getState(Go2SensorInfo & state);
  
  /**
   * @brief Set the start state values for simulation
   * 
   * @param stance Value defined by RobotModule
   * @param state Current sensor values information
   */
  void setStartState(const std::map<std::string, std::vector<double>> & stance, Go2SensorInfo & state);

  /**
   * @brief Loop back the value of "data" to "state"
   * 
   * @param data Command data for sending to Go2 robot
   * @param state Current sensor values information
   */
  void loopbackState(const Go2CommandData & data, Go2SensorInfo & state);

  /**
   * @brief Set send command data to Go2 robot
   * 
   * @param cm ControlMode (position, velocity or torque)
   * @param data Command data for sending to Go2 robot
   * @param kp0 true: Set to 0 except for position
   * @return true Success
   * @return false Could not send
   */
  bool setSendCmd(const ControlMode cm,const Go2CommandData & data, bool position_only);
};

} // namespace mc_unitree
#endif
