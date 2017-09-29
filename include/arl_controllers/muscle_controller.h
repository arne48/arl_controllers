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

namespace muscle_controllers {

  /**
   * Controller for interacting with pneumatic artificial muscles
   */
  class MuscleController : public controller_interface::Controller<arl_interfaces::MuscleInterface> {
  public:

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
    arl_interfaces::MuscleInterface *robot_;  /**< Handle to robot's muscle interface */

  private:
    uint8_t control_mode_;
    int loop_count_;
    control_toolbox::Pid pid_controller_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher<arl_hw_msgs::Muscle> > controller_state_publisher_;

    double kal_x_;
    double kal_P_;
    double kal_R_;
    double kal_Q_;
    double kal_u_;

  };

}

#endif //MUSCLE_CONTROLLERS_MUSCLE_CONTROLLER_H

