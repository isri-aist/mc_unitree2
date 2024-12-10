#pragma once

#include <mc_control/mc_global_controller.h>
#include <condition_variable>
#include <thread>

#include "RobotControl.h"

namespace mc_unitree
{
/**
 * @brief mc_rtc control interface for unitree robots
 */
struct MCControlUnitree2
{
  /**
   * @brief Interface constructor and destructor
   */
  MCControlUnitree2(mc_control::MCGlobalController & controller, const std::string & host);

  virtual ~MCControlUnitree2();

  /**
   * @brief Return a reference to the global mc_rtc controller
   */
  mc_control::MCGlobalController & controller()
  {
    return globalController_;
  }
  
  void robotControlCallback(void);
  
private:
  
  void robotControl(ControlMode cm);

  void addLogEntryRobotInfo();

  RobotConfigParameter config_param_;
  
  /*LowCmd write thread*/
  ThreadPtr lowCmdWriteThreadPtr;
  
  /*! Global mc_rtc controller */
  mc_control::MCGlobalController & globalController_;

  /*! Connection host */
  std::string host_;
  
  std::chrono::system_clock::time_point now_;
  
  RobotControl * robot_;
  RobotSensorInfo state_;
  RobotCommandData cmdData_;
  mc_rtc::Logger logger_;
  double delay_;
  bool robotControl_ready_;
  bool controller_init_once_;
};

} // namespace mc_unitree
