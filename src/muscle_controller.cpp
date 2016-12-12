#include <arl_controllers/muscle_controller.h>
#include <pluginlib/class_list_macros.h>

namespace muscle_controllers {

  MuscleController::MuscleController()
    : loop_count_(0) {}

  MuscleController::~MuscleController() {
    sub_command_.shutdown();
  }

  bool MuscleController::init(arl_interfaces::MuscleInterface *robot, ros::NodeHandle &nh) {

    std::string muscle_name;
    if (!nh.getParam("name", muscle_name))
    {
      ROS_ERROR("No muscle name given (namespace: %s)", nh.getNamespace().c_str());
      return false;
    }

    if (!pid_controller_.init(ros::NodeHandle(nh, "pid"))){
      return false;
    }

    robot_ = robot;
    muscle_ = robot_->getHandle(muscle_name);

    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<arl_hw_msgs::Muscle>(nh, "state", 1));
    sub_command_ = nh.subscribe<std_msgs::Float64>("command", 1, &MuscleController::setCommandCB, this);

    return true;
  }

  void MuscleController::starting(const ros::Time &time) {

    command_struct_.desired_pressure_ = muscle_.getDesiredPressure();

    command_.initRT(command_struct_);

  }

  void MuscleController::update(const ros::Time &time, const ros::Duration &period) {
    ROS_INFO("Muscle Controller Update");
    command_struct_ = *(command_.readFromRT());

    // publish state
    if (loop_count_ % 10 == 0)
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.current_pressure = muscle_.getCurrentPressure();
        controller_state_publisher_->msg_.desired_pressure = command_struct_.desired_pressure_;
        controller_state_publisher_->msg_.tension = muscle_.getTension();


        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;


    muscle_.setActivation(command_struct_.desired_pressure_);

  }

  void MuscleController::setCommandCB(const std_msgs::Float64ConstPtr &msg) {
    command_struct_.desired_pressure_ = msg->data;

    command_.writeFromNonRT(command_struct_);
  }

}

PLUGINLIB_EXPORT_CLASS(muscle_controllers::MuscleController, controller_interface::ControllerBase)
