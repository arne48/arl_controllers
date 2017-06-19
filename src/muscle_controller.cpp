#include <arl_controllers/muscle_controller.h>
#include <pluginlib/class_list_macros.h>

namespace muscle_controllers {

  MuscleController::MuscleController()
    : loop_count_(0), control_mode_(arl_hw_msgs::Muscle::CONTROL_MODE_BY_ACTIVATION) {
    kal_x_ = 40000;
    kal_P_ = 100;
    kal_R_ = 100;
    kal_Q_ = 0.2;
    kal_u_ = 0.0;
  }

  MuscleController::~MuscleController() {
    sub_act_command_.shutdown();
    sub_press_command_.shutdown();
  }

  bool MuscleController::init(arl_interfaces::MuscleInterface *robot, ros::NodeHandle &nh) {
    std::string muscle_name;
    if (!nh.getParam("name", muscle_name)) {
      ROS_ERROR("No muscle name given (namespace: %s)", nh.getNamespace().c_str());
      return false;
    }

    if (!pid_controller_.init(ros::NodeHandle(nh, "pid"))) {
      ROS_ERROR("No pid parameters found (namespace: %s)", nh.getNamespace().c_str());
      return false;
    }

    robot_ = robot;
    muscle_ = robot_->getHandle(muscle_name);

    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<arl_hw_msgs::Muscle>(nh, "state", 1));
    sub_act_command_ = nh.subscribe<std_msgs::Float64>("activation_command", 1, &MuscleController::setActCommandCB, this);
    sub_press_command_ = nh.subscribe<std_msgs::Float64>("pressure_command", 1, &MuscleController::setPressCommandCB, this);

    return true;
  }

  void MuscleController::starting(const ros::Time &time) {
    command_struct_.desired_pressure_ = muscle_.getDesiredPressure();
    command_struct_.activation_ = muscle_.getActivation();
    command_struct_.mode_ = arl_hw_msgs::Muscle::CONTROL_MODE_BY_ACTIVATION;
    command_.initRT(command_struct_);
    pid_controller_.reset();
  }

  void MuscleController::update(const ros::Time &time, const ros::Duration &period) {
    //ROS_DEBUG("Muscle Controller Update");
    command_struct_ = *(command_.readFromRT());

    control_mode_ = command_struct_.mode_;
    double activation = command_struct_.activation_;
    double desired_pressure = command_struct_.desired_pressure_;

    //If muscle is controlled by activation just this value is set no need to use the pid controller
    if(control_mode_ == arl_hw_msgs::Muscle::CONTROL_MODE_BY_ACTIVATION){
      muscle_.setActivation(activation);
      muscle_.setDesiredPressure(desired_pressure);

    } else if(control_mode_ == arl_hw_msgs::Muscle::CONTROL_MODE_BY_PRESSURE) {
      muscle_.setDesiredPressure(desired_pressure);      
      double error = muscle_.getDesiredPressure() - muscle_.getCurrentPressure();
      double pid_value = pid_controller_.computeCommand(error, period);
      // ~ 100 x value range
      pid_value /= 800000;
      muscle_.setActivation(pid_value);
    }

    // publish state
    if (loop_count_ % 5 == 0) {
      if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
        controller_state_publisher_->msg_.name = muscle_.getName();
        controller_state_publisher_->msg_.current_pressure = muscle_.getCurrentPressure();
        controller_state_publisher_->msg_.desired_pressure = muscle_.getDesiredPressure();
        controller_state_publisher_->msg_.tension = muscle_.getTension();
        controller_state_publisher_->msg_.activation = muscle_.getActivation();
        controller_state_publisher_->msg_.control_mode = control_mode_;

        //kalman filter for volatile tension values
        kal_x_ = (muscle_.getTension() * kal_P_ + kal_x_ * kal_R_) / (kal_P_ + kal_R_);
        kal_P_ = 1.0 / (1.0 / kal_P_ + 1.0 / kal_R_);
        kal_x_ += kal_u_;
        kal_P_ += kal_Q_;

        controller_state_publisher_->msg_.tension_filtered = kal_x_;

        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;

  }

  void MuscleController::setActCommandCB(const std_msgs::Float64ConstPtr &msg) {
    command_struct_.activation_ = msg->data;
    //Maybe not needed to reset value to 0.0
    command_struct_.desired_pressure_ = 0.0;
    command_struct_.mode_ = arl_hw_msgs::Muscle::CONTROL_MODE_BY_ACTIVATION;
    command_.writeFromNonRT(command_struct_);
  }

  void MuscleController::setPressCommandCB(const std_msgs::Float64ConstPtr &msg) {
    command_struct_.desired_pressure_ = msg->data;
    //Maybe not needed to reset value to 0.0
    command_struct_.mode_ = arl_hw_msgs::Muscle::CONTROL_MODE_BY_PRESSURE;
    command_.writeFromNonRT(command_struct_);
  }

}

PLUGINLIB_EXPORT_CLASS(muscle_controllers::MuscleController, controller_interface::ControllerBase)
