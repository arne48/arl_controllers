#ifndef MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H
#define MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H


#include <ros/node_handle.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <arl_interfaces/muscle_interface.h>

namespace muscle_controllers {

  class MuscleController : public controller_interface::Controller<arl_interfaces::MuscleInterface> {
  public:

    struct Commands {
      double position_; // Last commanded position
      double velocity_; // Last commanded velocity
      bool has_velocity_; // false if no velocity command has been specified
    };

    MuscleController();

    ~MuscleController();

    bool init(arl_interfaces::MuscleInterface *robot, ros::NodeHandle &n);

    void starting(const ros::Time &time);

    void update(const ros::Time &time, const ros::Duration &period);

    arl_interfaces::MuscleHandle muscles_;
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

    arl_interfaces::MuscleInterface *robot_;

  private:
    int loop_count_;

    //boost::scoped_ptr <realtime_tools::RealtimePublisher<arl_msgs::MuscleCommand>> controller_state_publisher_;

    ros::Subscriber sub_command_;

    void setCommandCB(const std_msgs::Float64ConstPtr &msg);
  };

} // namespace

#endif //MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H

