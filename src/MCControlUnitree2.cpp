// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>

#include <mc_rbdyn/rpy_utils.h>

// mc_unitree
#include "MCControlUnitree2.h"

#if defined(ENABLE_RT_PREEMPT)
#include <pthread.h>
#include "rtapi.h"
#endif

namespace mc_unitree
{

MCControlUnitree2::MCControlUnitree2(mc_control::MCGlobalController & controller, const std::string & host)
  : globalController_(controller), host_(host),  robot_(nullptr),
    logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc_unitree-" + controller.robot().name()),
    delay_(0.0), robotControl_ready_(false), controller_init_once_(false)
{
  // Get parameter values from the configuration file
  auto unitreeConfig = controller.configuration().config("Unitree");
  
  auto robot_name = controller.robot().name();
  if(!unitreeConfig.has(robot_name))
  {
    mc_rtc::log::error(
      "[mc_unitree] A name that matches the controller robot name is not defined in the configuration file");
    return;
  }
  
  for (size_t i = 0; i < controller.robot().refJointOrder().size(); i++)
  {
    cmdData_.qOut_[i] = 0.0;
    cmdData_.dqOut_[i] = 0.0;
    cmdData_.tauOut_[i] = 0.0;
  }
  
  /* Get communication information */
  mc_rtc::Configuration config_robot = unitreeConfig(robot_name);
  if(!config_robot.has("network-interface"))
  {
    mc_rtc::log::warning("[mc_unitree]'network-interface' config entry missing");
  }
  else
  {
    config_robot("network-interface", config_param_.network_);
  }
  
  auto & robot = controller.controller().robots().robot(robot_name);
  /* Connect to robot (real or simulation) */
  if(host != "simulation")
  {
    /* Try to connect via UDP to the robot */
    mc_rtc::log::info("[mc_unitree] Connecting to {} robot on {}",
                      robot_name, config_param_.network_);
    
    robot_ = new RobotControl(&robot, config_param_);
  }
  else
  {
    mc_rtc::log::info("[mc_unitree] Running simulation only. No connection to real robot");
    
    /* Set start state values */
    robot_ = new RobotControl(&robot, config_param_, host);
    robot_->setStartState(robot.stance(), state_);
  }
  
  /* Setup log entries */
  logger_.start(controller.current_controller(), controller.timestep());
  addLogEntryRobotInfo();
  logger_.addLogEntry("delay", [this]() { return delay_; });
  
  /* Run QP (every timestep ms) and send result joint commands to the robot */
  controller.running = true;
  now_ = std::chrono::high_resolution_clock::now();
  if(host_ != "simulation")
  {
    try
    {
      /*loop publishing thread*/
#if defined(ENABLE_RT_PREEMPT)
      pthread_create(&lowCmdWriteThread, NULL,
                     [](void* arg) -> void* {
                       auto* ctrl = static_cast<mc_unitree::MCControlUnitree2*>(arg);
                       ctrl->robotControlCallback();
                       return nullptr;
                     }, NULL);
#else      
      lowCmdWriteThreadPtr = CreateRecurrentThreadEx("robotControl", UT_CPU_ID_NONE, (uint64_t)lround(1.0/controller.timestep()), &MCControlUnitree2::robotControlCallback, this);
      //lowCmdWriteThreadPtr = CreateRecurrentThreadEx("robotControl", UT_CPU_ID_NONE, 2000, &robotControl_callback, this);
#endif
      while(controller.running)
      {
        sleep(1);
      }
    }
    catch(const std::exception& e)
    {
      mc_rtc::log::error("[mc_unitree] Could not send data from Go2 robot due to library error: {}", e.what());
    }
  }
  else
  {
    while(controller.running)
    {
      robotControlCallback();
    }
  }
  
  mc_rtc::log::info("[mc_unitree] interface initialized");
}
  
  
/* Destructor */
MCControlUnitree2::~MCControlUnitree2()
{
#if defined(ENABLE_RT_PREEMPT)
  pthread_join(lowCmdWriteThread, NULL);
#endif
  
  delete robot_;
}

void MCControlUnitree2::robotControlCallback(void)
{
#if defined(ENABLE_RT_PREEMPT)
  if (set_sched_prio(RT_PRIO_MAX-2, TASK_PERIOD) == -1)
  {
    perror("set_sched_prio failed on MCControlUnitree2.\n");
    return;
  } /* in art_process */
#endif
  
  delay_ = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - now_).count();
  now_ = std::chrono::high_resolution_clock::now();
  this->robotControl(config_param_.mode_);
  
  if(robotControl_ready_ && !controller_init_once_)
  {
    controller_init_once_ = true;
    /* Set up interface GUI tab */
    globalController_.init(state_.qIn_);
    globalController_.running = true;
    globalController_.controller().gui()->addElement(
        {"Unitree"}, mc_rtc::gui::Button("Stop controller", [&]() { this->globalController_.running = false; }));
  }
}

void MCControlUnitree2::robotControl(ControlMode cm)
{
  auto start_t_ = std::chrono::high_resolution_clock::now();
  auto robot_name = globalController_.robot().name();
  static size_t robotControl_call_count = 0;

  if(host_ != "simulation")
  {
    /* Store the data received by "recvData()" in "state" */
    robot_->getState(state_);
    robotControl_call_count++;
    if(robotControl_call_count > 30) robotControl_ready_ = true;
  }

  /* Send sensor readings to mc_rtc controller */
  globalController_.setEncoderValues(robot_name, state_.qIn_);
  globalController_.setEncoderVelocities(robot_name, state_.dqIn_);
  globalController_.setJointTorques(robot_name, state_.tauIn_);
  globalController_.setSensorLinearAcceleration(robot_name, state_.accIn_);
  globalController_.setSensorOrientation(robot_name, Eigen::Quaterniond(mc_rbdyn::rpyToMat(state_.rpyIn_)));
  globalController_.setSensorAngularVelocity(robot_name, state_.rateIn_);

  bool run_ret = true;
  if(host_ == "simulation" || robotControl_ready_)
  {
    run_ret = globalController_.run();
  }

  if(run_ret)
  {
    /* Update control value from the data in a robot */
    auto & robot = globalController_.controller().robots().robot(robot_name);
    for (const auto jname : robot.refJointOrder())
    {
      auto jointId = robot.jointIndexByName(jname);
      switch(cm)
      {
        case mc_unitree::ControlMode::Position:
          cmdData_.qOut_[jointId] = robot.mbc().q[jointId][0];
          cmdData_.dqOut_[jointId] = robot.mbc().alpha[jointId][0];
          break;
        case mc_unitree::ControlMode::Velocity:
          cmdData_.dqOut_[jointId] = robot.mbc().alpha[jointId][0];
          break;
        case mc_unitree::ControlMode::Torque:
          cmdData_.tauOut_[jointId] = robot.mbc().jointTorque[jointId][0];
          break;
      }
    }

    if(host_ != "simulation")
    {
      /* Send command data */
      robot_->setSendCmd(cm, cmdData_, !robotControl_ready_);
    }
    else
    {
      /* Loop back the value of "cmdData_" to "state_" */
      robot_->loopbackState(cmdData_, state_);
      robotControl_ready_ = true;
    }
  }

  /* Wait until next controller run */
  if(host_ == "simulation")
  {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_t_).count();
    if(elapsed > globalController_.timestep() * 1000)
    {
      mc_rtc::log::warning(
        "[mc_unitree] Loop time {} exeeded timestep of {} ms", elapsed, globalController_.timestep() * 1000);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(
          static_cast<unsigned int>((globalController_.timestep() * 1000 - elapsed)) * 1000));
    }
  }

  /* Print controller data to the log */
  logger_.log();
}


void MCControlUnitree2::addLogEntryRobotInfo()
{
  /* Sensors */
  /* Position(Angle) values */
  logger_.addLogEntry("measured_joint_position", [this]() -> const std::vector<double> & { return state_.qIn_; });
  /* Velocity values */
  logger_.addLogEntry("measured_joint_velocity", [this]() -> const std::vector<double> & { return state_.dqIn_; });
  /* Torque values */
  logger_.addLogEntry("measured_joint_torque", [this]() -> const std::vector<double> & { return state_.tauIn_; });
  /* Orientation sensor */
  logger_.addLogEntry("measured_imu_rpy", [this]() -> const Eigen::Vector3d & { return state_.rpyIn_; });
  /* Accelerometer */
  logger_.addLogEntry("measured_imu_accel", [this]() -> const Eigen::Vector3d & { return state_.accIn_; });
  /* Angular velocity */
  logger_.addLogEntry("measured_imu_rate", [this]() -> const Eigen::Vector3d & { return state_.rateIn_; });
  /* Foot force sensors */
  if (!state_.footForceIn_.empty())
  {
    logger_.addLogEntry("measured_foot_force", [this]() -> const std::vector<double> & { return state_.footForceIn_; });
  }
  
  /* Command data to send to the robot */
  /* Position(Angle) values */
  logger_.addLogEntry("sent_joint_position", [this]() -> const std::vector<double> & { return cmdData_.qOut_; });
  /* Velocity values */
  logger_.addLogEntry("sent_joint_velocity", [this]() -> const std::vector<double> & { return cmdData_.dqOut_; });
  /* Torque values */
  logger_.addLogEntry("sent_joint_torque", [this]() -> const std::vector<double> & { return cmdData_.tauOut_; });
}

} // namespace mc_unitree
