#ifndef MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H
#define MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H


#include <ros/node_handle.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <arl_interfaces/muscle_interface.h>
#include <arl_hw_msgs/Muscle.h>

#define SAMPLE_SIZE 20

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
      double activation_; /**<  Normalized activation of muscle */
      double desired_pressure_; /**<  Desired pressure of muscle */
      uint8_t mode_; /**<  Current mode of muscle control */
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


    void starting(const ros::Time &time);

    /**
     * Function which will be periodically called to update controllers interaction with robot's hardware
     * @param time
     * @param period
     */
    void update(const ros::Time &time, const ros::Duration &period);

    arl_interfaces::MuscleHandle muscle_;  /**< MuscleHandles of all muscles controlled by this controller */
    realtime_tools::RealtimeBuffer<Commands> command_;  /**< Realtime safe buffer */
    Commands command_struct_;  /**< pre allocated memory for usage in realtime buffer */

    arl_interfaces::MuscleInterface *robot_;  /**< Handle to robot's muscle interface */

  private:
    uint8_t control_mode_;
    int loop_count_;
    double tension_buffer_[SAMPLE_SIZE];
    control_toolbox::Pid pid_controller_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher<arl_hw_msgs::Muscle> > controller_state_publisher_;
    ros::Subscriber sub_act_command_;  /**< Subscription to MuscleCommands by activation */
    ros::Subscriber sub_press_command_;  /**< Subscription to MuscleCommands by pressure */


    /**
     * Callback for MuscleCommands issued to controlled muscles by activation
     * @param msg MuscleCommand for interaction with muscles
     */
    void setActCommandCB(const std_msgs::Float64ConstPtr &msg);

    /**
     * Callback for MuscleCommands issued to controlled muscles by pressure
     * @param msg MuscleCommand for interaction with muscles
     */
    void setPressCommandCB(const std_msgs::Float64ConstPtr &msg);
  };

}

#endif //MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H

