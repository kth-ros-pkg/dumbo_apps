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
#include <brics_actuator/JointVelocities.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <fir_filter/filt.h>


class PassiveForceControlNode
{
public:
	ros::NodeHandle n_;

	/// declaration of topics to publish
	ros::Publisher topicPub_CommandVel_;
	ros::Publisher topicPub_CommandTwist_;

	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointState_;
	ros::Subscriber topicSub_FT_compensated_;

	ros::Time last_publish_time;

	tf::TransformListener *tf_listener_;


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
			topicPub_CommandVel_ = n_.advertise<brics_actuator::JointVelocities>("command_vel", 1);
			topicSub_JointState_ = n_.subscribe("state", 1, &PassiveForceControlNode::topicCallback_joint_states, this);
			topicSub_FT_compensated_ = n_.subscribe("ft_compensated", 1, &PassiveForceControlNode::topicCallback_ft_compensated, this);
		}

		tf_listener_ = new tf::TransformListener();

	}

	~PassiveForceControlNode()
	{
		delete tf_listener_;
		for(unsigned int i=0; i<m_DOF; i++)
			delete m_filters[i];
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
				m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(0.3);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper");
				return;
			}
		}

		// get the FIR low pass filter cutoff frequency

		if (n_.hasParam("cutoff_freq"))
		{
			n_.getParam("cutoff_freq", m_cutoff_freq);
		}

		else
		{
			ROS_ERROR("Parameter cutoff_freq not set, shutting down node...");
			n_.shutdown();
			return;
		}

		// initialize the FIR filters
		m_filters.resize(m_DOF);

		for(unsigned int i = 0; i<m_DOF; i++)
			m_filters[i] = new Filter(LPF, 4, 150, m_cutoff_freq);

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

	void topicCallback_ft_compensated(const geometry_msgs::WrenchStampedPtr &msg)
	{
		m_ft_compensated = *msg;
		m_received_ft = true;
	}

	void topicCallback_joint_states(const control_msgs::JointTrajectoryControllerStatePtr &msg)
	{
		ROS_DEBUG("Received joint states");
		std::vector<double> JointPos(m_DOF,0.0);

		// search for joints in joint state msg
		if(msg->actual.positions.size()==m_DOF)
		{
			for(unsigned int i=0; i<m_DOF; i++)
			{
				if(msg->joint_names[i]!=m_joint_names[i])
				{
					ROS_ERROR("Error in received joint name");
					return;
				}

				else
				{
					 JointPos[i] = msg->actual.positions[i];
				}
			}

		}

		m_joint_pos = JointPos;
		m_joint_pos_stamp = msg->header.stamp;
		m_received_js = true;

	}

	bool calculateControlSignal(std::vector<double> &joint_vel)
	{
		std::vector<double> vel_screw(6,0);
		bool ret;

		if(!m_kdl_wrapper_initialized)
		{
			if(m_dumbo_kdl_wrapper.init("arm_base_link", m_arm_select+std::string("_arm_7_link")) )
			{
				m_dumbo_kdl_wrapper.ik_solver_vel->setLambda(0.3);
				m_kdl_wrapper_initialized = true;
			}

			else
			{
				ROS_ERROR("Error initializing Dumbo KDL wrapper");
				return false;
			}
		}

		// first transform the FT measurement to the arm base link
		geometry_msgs::Vector3Stamped f;
		f.header = m_ft_compensated.header;
		f.header.stamp = ros::Time();
		f.vector = m_ft_compensated.wrench.force;
		geometry_msgs::Vector3Stamped f_base;

		try
		{
			tf_listener_->transformVector("arm_base_link", f, f_base);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("Error transforming F/T measurement to the arm base link");
			ROS_ERROR("%s.", ex.what());
			return false;
		}

		geometry_msgs::Vector3Stamped t;
		t.header = m_ft_compensated.header;
		t.header.stamp = ros::Time();
		t.vector = m_ft_compensated.wrench.torque;
		geometry_msgs::Vector3Stamped t_base;

		try
		{
			tf_listener_->transformVector("arm_base_link", t, t_base);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("Error transforming F/T measurement to the arm base link");
			ROS_ERROR("%s.", ex.what());
			return false;
		}


		// put the transformed wrench back in the member variable
		m_ft_compensated.header.frame_id = f_base.header.frame_id;
		m_ft_compensated.wrench.force = f_base.vector;
		m_ft_compensated.wrench.torque = t_base.vector;


		vel_screw[0] = m_force_gain * m_ft_compensated.wrench.force.x;
		vel_screw[1] = m_force_gain * m_ft_compensated.wrench.force.y;
		vel_screw[2] = m_force_gain * m_ft_compensated.wrench.force.z;

		vel_screw[3] = m_torque_gain * m_ft_compensated.wrench.torque.x;
		vel_screw[4] = m_torque_gain * m_ft_compensated.wrench.torque.y;
		vel_screw[5] = m_torque_gain * m_ft_compensated.wrench.torque.z;

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


//		ret =  m_DumboKDL_->getInvVel(m_JointPos, vel_screw, joint_vel_, m_bound_max);
		KDL::JntArray q_in(m_DOF);
		KDL::JntArray q_dot_out;
		KDL::Twist v_in;
		joint_vel.resize(7);

		for(unsigned int i=0; i<m_DOF; i++) q_in(i) = m_joint_pos[i];
		for(unsigned int i=0; i<6; i++) v_in(i) = vel_screw[i];

		ret = m_dumbo_kdl_wrapper.ik_solver_vel->CartToJnt(q_in, v_in, q_dot_out);
		for(unsigned int i=0; i<m_DOF; i++) joint_vel[i] = q_dot_out(i);

		// filter the joint velocities
		for(unsigned int i=0; i<m_DOF; i++)
			joint_vel[i] = m_filters[i]->do_sample(joint_vel[i]);

		return true;
	}


	void publishVel()
	{
		ROS_DEBUG("Publish vel");
		std::vector<double> joint_vel;
		if(m_received_ft&&m_received_js)
		{
			ros::Time now = ros::Time::now();
			if(((now - m_joint_pos_stamp).toSec()<0.2) && ((now-m_ft_compensated.header.stamp).toSec()<0.2))
			{
				if(calculateControlSignal(joint_vel))
				{

					brics_actuator::JointVelocities joint_vel_msg;
					joint_vel_msg.velocities.resize(m_DOF);

					for(unsigned int i=0; i<m_DOF; i++)
					{
						joint_vel_msg.velocities[i].unit = "rad";
						joint_vel_msg.velocities[i].joint_uri = m_joint_names[i].c_str();
						joint_vel_msg.velocities[i].value = joint_vel.at(i);
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
	geometry_msgs::WrenchStamped m_ft_compensated;
	std::vector<double> m_joint_pos;
	ros::Time m_joint_pos_stamp;
	KDLWrapper m_dumbo_kdl_wrapper;

	// FIR low pass filters for joint velocities
	double m_cutoff_freq;
	std::vector<Filter*> m_filters;

	bool m_received_js;
	bool m_received_ft;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "force_reactive_controller");

	PassiveForceControlNode Controller;

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

	while(Controller.n_.ok())
	{
		if((ros::Time::now()-begin).toSec()>=timeout.toSec())
		{
			ROS_INFO("Timed out, shutting down node...");
			Controller.n_.shutdown();
			return 0;
		}

		Controller.publishVel();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
