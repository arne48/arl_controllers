#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <arl_interfaces/muscle_interface.h>
#include <arl_controllers/muscle_controller.h>
#include <arl_hw_msgs/Muscle.h>

class MuscleControllerTest : public ::testing::Test {
public:
  MuscleControllerTest()
    : nh_(ros::NodeHandle()),
      des1(11.0), cur1(22.0), ten1(33.0), act1(44.0),
      des2(0.0), cur2(0.0), ten2(0.0), act2(0.0),
      name1("muscle_0"), name2("muscle_1") {

    // Setup the joint state interface
    arl_interfaces::MuscleHandle m_handle_1(name1, &des1, &cur1, &ten1, &act1);
    m_iface_.registerHandle(m_handle_1);

    arl_interfaces::MuscleHandle m_handle_2(name2, &des2, &cur2, &ten2, &act2);
    m_iface_.registerHandle(m_handle_2);

    // Initialize ROS interfaces
    pub_ = nh_.advertise<std_msgs::Float64>("/good_muscle_controller/command", 1);
    sub_ = nh_.subscribe<arl_hw_msgs::Muscle>("/good_muscle_controller/state",1,&MuscleControllerTest::muscleStateCb, this);
  }

protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  arl_interfaces::MuscleInterface m_iface_;

  double des1, cur1, ten1, act1;
  double des2, cur2, ten2, act2;
  std::string name1, name2;

  // Received joint state messages counter
  int rec_msgs_;

  // Last received joint state message
  arl_hw_msgs::Muscle last_msg_;

  void muscleStateCb(const arl_hw_msgs::MuscleConstPtr &msg) {
    last_msg_ = *msg;
    ++rec_msgs_;
  }
};

TEST_F(MuscleControllerTest, initControllerBad) {
  ros::NodeHandle bad0_("/bad_muscle_controller0");
  muscle_controllers::MuscleController mc;
  EXPECT_FALSE(mc.init(&m_iface_, bad0_));

  ros::NodeHandle bad1_("/bad_muscle_controller1");
  EXPECT_FALSE(mc.init(&m_iface_, bad1_));

}

TEST_F(MuscleControllerTest, initControllerGood) {
  ros::NodeHandle pnh_("/good_muscle_controller");
  muscle_controllers::MuscleController mc;
  EXPECT_TRUE(mc.init(&m_iface_, pnh_));

}

TEST_F(MuscleControllerTest, muscleControllerAccess) {
  ros::NodeHandle pnh_("/good_muscle_controller");
  muscle_controllers::MuscleController mc;
  EXPECT_TRUE(mc.init(&m_iface_, pnh_));

  EXPECT_DOUBLE_EQ(11.0, mc.muscle_.getDesiredPressure());
  EXPECT_DOUBLE_EQ(22.0, mc.muscle_.getCurrentPressure());
  EXPECT_DOUBLE_EQ(33.0, mc.muscle_.getTension());
  EXPECT_DOUBLE_EQ(44.0, mc.muscle_.getActivation());

  mc.muscle_.setDesiredPressure(0.0);
  mc.muscle_.setCurrentPressure(0.0);
  mc.muscle_.setTension(0.0);
  mc.muscle_.setActivation(0.0);

  EXPECT_DOUBLE_EQ(0.0, mc.muscle_.getDesiredPressure());
  EXPECT_DOUBLE_EQ(0.0, mc.muscle_.getCurrentPressure());
  EXPECT_DOUBLE_EQ(0.0, mc.muscle_.getTension());
  EXPECT_DOUBLE_EQ(0.0, mc.muscle_.getActivation());

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "muscle_controller_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
