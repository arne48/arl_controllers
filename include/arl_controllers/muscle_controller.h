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

  /**
   * Controller for interacting with pneumatic artificial muscles
   */
  class MuscleController : public controller_interface::Controller<arl_interfaces::MuscleInterface> {
  public:

    /**
     * Command structure for realtime safe buffer
     */
    struct Commands {
      double position_; /**<  Last commanded position */
      double velocity_; /**<  Last commanded velocity */
      bool has_velocity_; /**<  false if no velocity command has been specified */
    };

    /**
     * Default Constructor
     * @return
     */
    MuscleController();

    /**
     * Destructor
     */
    ~MuscleController();

    /**
     * Initialize muscle controller
     * @param robot Pointer to robot's hardware handle
     * @param nh Reference to ROS node handle
     * @return returns rather the initilization was successful or not
     */
    bool init(arl_interfaces::MuscleInterface *robot, ros::NodeHandle &nh);

    /**
     * Function which will be periodically called to update controllers interaction with robot's hardware
     * @param time
     * @param period
     */
    void update(const ros::Time &time, const ros::Duration &period);

    arl_interfaces::MuscleHandle muscles_;  /**< MuscleHandles of all muscles controlled by this controller */
    realtime_tools::RealtimeBuffer<Commands> command_;  /**< Realtime safe buffer */
    Commands command_struct_;  /**< pre allocated memory for usage in realtime buffer */

    arl_interfaces::MuscleInterface *robot_;  /**< Handle to robot's muscle interface */

  private:
    int loop_count_;
    //boost::scoped_ptr <realtime_tools::RealtimePublisher<arl_msgs::MuscleCommand>> controller_state_publisher_;
    ros::Subscriber sub_command_;  /**< Subscription to MuscleCommands */

    /**
     * Callback for MuscleCommands issued to controlled muscles
     * @param msg MuscleCommand for interaction with muscles
     */
    void setCommandCB(const std_msgs::Float64ConstPtr &msg);
  };

}

#endif //MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H

