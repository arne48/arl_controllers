#include <arl_controllers/muscle_controller.h>
#include <pluginlib/class_list_macros.h>

namespace muscle_controllers {

  MuscleController::MuscleController()
    : loop_count_(0) {}

  MuscleController::~MuscleController() {
    sub_command_.shutdown();
  }

  bool MuscleController::init(arl_interfaces::MuscleInterface *robot, ros::NodeHandle &nh) {
    robot_ = robot;
    return true;
  }

  void MuscleController::update(const ros::Time &time, const ros::Duration &period) {
    ROS_INFO("Muscle Controller Update");

    /* //EXAMPLE
    double val = robot_->getHandle(robot_->getNames()[0]).getDesiredPressure();
    robot_->getHandle(robot_->getNames()[0]).setDesiredPressure(val+0.01);
     */
  }

  void MuscleController::setCommandCB(const std_msgs::Float64ConstPtr &msg) {
  }

}

PLUGINLIB_EXPORT_CLASS(muscle_controllers::MuscleController, controller_interface::ControllerBase)
