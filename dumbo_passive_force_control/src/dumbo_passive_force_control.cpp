/*
 *  dumbo_passive_force_control.cpp
 *
 *
 *  Created on: Nov 25, 2013
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2013, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <controller_manager_msgs/SwitchController.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <kdl_conversions/kdl_msg.h>
#include <std_srvs/Empty.h>

class PassiveForceControlNode
{
public:
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_CommandVel_;
	ros::Publisher topicPub_CommandTwist_;

	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointState_;
    ros::Subscriber topicSub_ft_gravity_compensated_;

    // service clients for switching Dumbo's controllers through the controller manager
    ros::ServiceClient switch_controllers_client_;

	ros::Time last_publish_time;


	PassiveForceControlNode()
	{
		n_ = ros::NodeHandle("~");
		m_kdl_wrapper_initialized = false;
		m_received_ft = false;
		m_received_js = false;
		m_initialized = false;

		getROSParameters();
		if(isInitialized())
		{
            topicPub_CommandVel_ = n_.advertise<std_msgs::Float64MultiArray>("command_vel", 1);
            topicSub_JointState_ = n_.subscribe("/joint_states", 1, &PassiveForceControlNode::topicCallback_joint_states, this);
            topicSub_ft_gravity_compensated_ = n_.subscribe("ft_gravity_compensated", 1, &PassiveForceControlNode::topicCallback_ft_gravity_compensated, this);
            switch_controllers_client_ = n_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

            // stop joint trajectory controller and start joint velocity controller
            controller_manager_msgs::SwitchController switch_controller_srv;
            switch_controller_srv.request.strictness = switch_controller_srv.request.STRICT;
            switch_controller_srv.request.stop_controllers.resize(1);
            switch_controller_srv.request.stop_controllers[0] = m_arm_select+"_arm_joint_trajectory_controller";
            switch_controller_srv.request.start_controllers.resize(1);
            switch_controller_srv.request.start_controllers[0] = m_arm_select+"_arm_joint_velocity_controller";

            switch_controllers_client_.call(switch_controller_srv);

            if(!switch_controller_srv.response.ok)
            {
                ROS_ERROR("Error stopping joint trajectory controller and starting joint velocity controller");
            }

		}


	}

	~PassiveForceControlNode()
	{
        restoreControllers();
    }

    void restoreControllers()
    {
        // restart joint trajectory controller and stop joint velocity controller
        controller_manager_msgs::SwitchController switch_controller_srv;
        switch_controller_srv.request.strictness = switch_controller_srv.request.STRICT;
        switch_controller_srv.request.stop_controllers.resize(1);
        switch_controller_srv.request.stop_controllers[0] = m_arm_select+"_arm_joint_velocity_controller";
        switch_controller_srv.request.start_controllers.resize(1);
        switch_controller_srv.request.start_controllers[0] = m_arm_select+"_arm_joint_trajectory_controller";

        switch_controllers_client_.call(switch_controller_srv);

        if(!switch_controller_srv.response.ok)
        {
            ROS_ERROR("Error stopping joint velocity controller and starting joint trajectory controller");
        }

        n_.shutdown();
    }


	bool isInitialized()
	{
		return m_initialized;
	}

	void getROSParameters()
	{
		/// Get joint names
		XmlRpc::XmlRpcValue JointNamesXmlRpc;
		std::vector<std::string> JointNames;
		if (n_.hasParam("joint_names"))
		{
			n_.getParam("joint_names", JointNamesXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
			return;
		}


		/// Resize and assign of values to the JointNames
		JointNames.resize(JointNamesXmlRpc.size());
		for (int i = 0; i < JointNamesXmlRpc.size(); i++)
		{
			JointNames[i] = (std::string)JointNamesXmlRpc[i];
		}

		m_joint_names = JointNames;
		m_DOF = JointNames.size();
		m_joint_pos.resize(m_DOF);

		/// Get arm selection parameter (left or right arm)
		XmlRpc::XmlRpcValue ArmSelectXmlRpc;
		std::string ArmSelect;
		if (n_.hasParam("arm_select"))
		{
			n_.getParam("arm_select", ArmSelectXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter arm_select not set, shutting down node...");
			n_.shutdown();
			return;
		}

		ArmSelect = (std::string)(ArmSelectXmlRpc);
		if(!setArmSelect(ArmSelect))
		{
			ROS_ERROR("Error setting ArmSelect, shutting down node");
			n_.shutdown();
			return;
		}


		// get the controller gains

		if (n_.hasParam("force_gain"))
		{
			n_.getParam("force_gain", m_force_gain);
		}

		else
		{
			ROS_ERROR("Parameter v_gain not set, shutting down node...");
			n_.shutdown();
			return;
		}


		if (n_.hasParam("torque_gain"))
		{
			n_.getParam("torque_gain", m_torque_gain);
		}

		else
		{
			ROS_ERROR("Parameter w_gain not set, shutting down node...");
			n_.shutdown();
			return;
		}


		// get the velocity limits

		if (n_.hasParam("v_limit"))
		{
			n_.getParam("v_limit", m_v_limit);
		}

		else
		{
			ROS_ERROR("Parameter v_limit not set, shutting down node...");
			n_.shutdown();
			return;
		}

		if (n_.hasParam("w_limit"))
		{
			n_.getParam("w_limit", m_w_limit);
		}

		else
		{
			ROS_ERROR("Parameter w_limit not set, shutting down node...");
			n_.shutdown();
			return;
		}


		if(m_force_gain<0.0 || m_torque_gain<0.0 || m_v_limit<=0.0 || m_w_limit<=0.0)
		{
			ROS_ERROR("Invalid gains/limits (<= 0.0), shutting down node...");
			n_.shutdown();
			return;
		}

		if(!m_kdl_wrapper_initialized)
		{

			if(m_dumbo_kdl_wrapper.init("arm_base_link", m_arm_select+std::string("_arm_7_link")) )
			{
                m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(100.0);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper");
				return;
			}

			if(m_dumbo_kdl_wrapper_ft.init("arm_base_link", m_arm_select+std::string("_arm_ft_sensor")) )
			{
                m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(100.0);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper with FT sensor as tip link");
				return;
			}
		}

		m_initialized = true;

	}

	bool setArmSelect(std::string ArmSelect)
	{
		if(ArmSelect!="left" && ArmSelect!="right")
		{
			ROS_ERROR("Invalid arm_select parameter, shutting down node...");
			n_.shutdown();
			return false;
		}
		m_arm_select = ArmSelect;
		return true;
	}

    void topicCallback_ft_gravity_compensated(const geometry_msgs::WrenchStampedPtr &msg)
	{
        m_ft_gravity_compensated = *msg;
		m_received_ft = true;
	}

    void topicCallback_joint_states(const sensor_msgs::JointStatePtr &msg)
	{
		ROS_DEBUG("Received joint states");
		std::vector<double> JointPos(m_DOF,0.0);

		// search for joints in joint state msg
        for(unsigned int joint_counter=0; joint_counter<m_DOF; joint_counter++)
        {
            for(unsigned int i=0; i<msg->name.size(); i++)
            {
                if(msg->name[i]==m_joint_names[joint_counter])
                {
                    JointPos[joint_counter] = msg->position[i];
                    break;
                }
            }
        }

		m_joint_pos = JointPos;
		m_joint_pos_stamp = msg->header.stamp;
		m_received_js = true;

	}

	bool CalculateControlSignal(std::vector<double> &joint_vel)
	{
		std::vector<double> vel_screw(6,0);
		bool ret;

		if(!m_kdl_wrapper_initialized)
		{
			if(m_dumbo_kdl_wrapper.init("arm_base_link", m_arm_select+std::string("_arm_7_link")) )
			{
                m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(100.0);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper");
				return false;
			}

			if(m_dumbo_kdl_wrapper_ft.init("arm_base_link", m_arm_select+std::string("_arm_ft_sensor")) )
			{
                m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(100.0);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper with FT sensor as tip link");
				return false;
			}
		}

		// first transform the FT measurement to the arm base link
		KDL::JntArray q_in(m_DOF);
		for(unsigned int i=0; i<m_DOF; i++) q_in(i) = m_joint_pos[i];

		KDL::Frame F_ft;
		m_dumbo_kdl_wrapper_ft.fk_solver_pos->JntToCart(q_in, F_ft);

		KDL::Wrench wrench_ft_frame;
        tf::wrenchMsgToKDL(m_ft_gravity_compensated.wrench, wrench_ft_frame);

		KDL::Wrench wrench_base_frame;
		wrench_base_frame = F_ft.M*wrench_ft_frame;

		for(unsigned int i=0; i<3; i++) vel_screw[i] = m_force_gain * wrench_base_frame.force(i);
		for(unsigned int i=0; i<3; i++) vel_screw[i+3] = m_torque_gain * wrench_base_frame.torque(i);

		//saturate velocity screws
		double v_scale = (sqrt(pow(vel_screw[0], 2.0) + pow(vel_screw[1], 2.0) + pow(vel_screw[2], 2.0))/m_v_limit);

		if(v_scale>1.0)
		{
			for(int i=0; i<3; i++)
			{
				vel_screw[i] /= v_scale;
			}
		}

		double w_scale = sqrt(pow(vel_screw[3], 2.0) + pow(vel_screw[4], 2.0) + pow(vel_screw[5], 2.0))/(m_w_limit);

		if(w_scale>1.0)
		{
			for(int i=0; i<3; i++)
			{
				vel_screw[i+3] /= w_scale;
			}
		}



		KDL::JntArray q_dot_out;
		KDL::Twist v_in;
		joint_vel.resize(7);

		for(unsigned int i=0; i<6; i++) v_in(i) = vel_screw[i];

		ret = m_dumbo_kdl_wrapper.ik_solver_vel->CartToJnt(q_in, v_in, q_dot_out);
		for(unsigned int i=0; i<7; i++) joint_vel[i] = q_dot_out(i);

//		joint_vel = std::vector<double>(7, 0.0);
		joint_vel[6] = m_torque_gain*8*wrench_ft_frame.torque(2);

		return true;
	}


	void publishVel()
	{
		ROS_DEBUG("Publish vel");
		std::vector<double> joint_vel;
		if(m_received_ft&&m_received_js)
		{
			ros::Time now = ros::Time::now();
            if(((now - m_joint_pos_stamp).toSec()<0.2) && ((now-m_ft_gravity_compensated.header.stamp).toSec()<0.2))
			{
				if(CalculateControlSignal(joint_vel))
				{
                    std_msgs::Float64MultiArray joint_vel_msg;
                    joint_vel_msg.data.resize(m_DOF);

					for(unsigned int i=0; i<m_DOF; i++)
					{
                        joint_vel_msg.data[i] = joint_vel.at(i);
					}
					topicPub_CommandVel_.publish(joint_vel_msg);
					ROS_DEBUG("Velocity command");
					last_publish_time = ros::Time::now();
				}
			}
			else
			{
				ROS_ERROR("Joint state or F/T measurement too old");
				return;
			}

		}

		else if(!m_received_ft)
		{
		  static ros::Time t = ros::Time::now();
		  if((ros::Time::now()-t).toSec()>2.0)
		    {
			ROS_ERROR("Haven't received FT measurements");
			t = ros::Time::now();
		    }
		}

		else if(!m_received_js)
		{
		  static ros::Time t = ros::Time::now();
		  if((ros::Time::now()-t).toSec()>2.0)
		    {
			ROS_ERROR("Haven't received joint states");
			t = ros::Time::now();
		    }
		}

	}


private:
	bool m_kdl_wrapper_initialized;

	double m_force_gain;
	double m_torque_gain;
	double m_v_limit;
	double m_w_limit;


	unsigned int m_DOF;
	bool m_initialized;

	std::string m_arm_select;

	std::vector<std::string> m_joint_names;
    geometry_msgs::WrenchStamped m_ft_gravity_compensated;
	std::vector<double> m_joint_pos;
	ros::Time m_joint_pos_stamp;

	KDLWrapper m_dumbo_kdl_wrapper;
	KDLWrapper m_dumbo_kdl_wrapper_ft;

	bool m_received_js;
	bool m_received_ft;
};

PassiveForceControlNode *controller;

void shutdownHandler(int sig)
{
    controller->restoreControllers();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "force_reactive_controller");

	PassiveForceControlNode Controller;
    controller = &Controller;

    signal(SIGINT, shutdownHandler);

	double frequency;
	if (Controller.n_.hasParam("loop_rate"))
	{
		Controller.n_.getParam("loop_rate", frequency);
		//frequency of driver has to be much higher then controller frequency
		frequency *= 1;
	}

	else
	{
		ROS_ERROR("Parameter loop_rate not available, shutting down node...");
		Controller.n_.shutdown();
		return 0;
	}
	ROS_DEBUG("Loop rate: %f", frequency);

	ros::Rate loop_rate(frequency);

	double timeout_;
	if(Controller.n_.hasParam("timeout"))
	{
		Controller.n_.getParam("timeout", timeout_);

	}

	else
	{
		ROS_ERROR("Parameter timeout not available");
		return 0;
	}

	if(timeout_<=0.0)
	{
		ROS_ERROR("Incorrect timeout parameter");
		return 0;
	}
	ros::Duration timeout;
	timeout.fromSec(timeout_);
	ROS_INFO("Timeout: %f sec", timeout.toSec());


	if(!Controller.isInitialized())
	{
		ROS_ERROR("Incorrect controller initialization, shutting down node.");
		return 0;
	}


	ros::Time begin = ros::Time::now();

	// make the safety monitor sleep (stop checking f/t limits)
	std_srvs::Empty srv;
	ros::service::call("/dumbo_safety_monitor/sleep", srv);

    while(ros::ok())
	{
		if((ros::Time::now()-begin).toSec()>=timeout.toSec())
		{
            ROS_INFO("Timed out, shutting down node...");
			return 0;
		}

		Controller.publishVel();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
