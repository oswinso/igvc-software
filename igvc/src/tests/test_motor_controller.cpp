//
// Created by Andrew Tuttle
//

#include <gtest/gtest.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <igvc/SerialPort.h>
#include <std_msgs/Bool.h>

class TestMotorController : public testing::Test {
	public:
		TestMotorController()
			: handle()
			, mock_motors_pub(handle.advertise<igvc_msgs::velocity_pair>("/motors", 1))
			, mock_robot_enabled_pub(handle.advertise<std_msgs::Bool>("/robot_enabled", 1))
			, encoders_sub(handle.subscribe("/encoders", 1, &TestMotorController::encodersCallback, this))
			{
			}
		void encodersCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
		{
		}

	protected:
		virtual void SetUp()
		{
			while (!IsNodeReady())
			{
				ros::spinOnce();
			}
		}

		virtual void TearDown()
		{
		}

		bool IsNodeReady()
		{
			return (mock_motors_pub.getNumSubscribers() > 0) && (mock_robot_enabled_pub.getNumSubscribers() >0)
				&&  (encoders_sub.getNumSubscribers() > 0)
		}

		ros::NodeHandle handle;
		ros::Publisher mock_motors_pub;
		ros::Publisher mock_robot_enabled_pub;
		ros::Subscriber encoders_sub;
};

TEST_F()
{
	
}