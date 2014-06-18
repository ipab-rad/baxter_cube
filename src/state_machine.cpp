#include "ros/ros.h"

#include "std_msgs/Bool.h"

#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "geometry_msgs/Pose.h"

#include "baxter_core_msgs/AssemblyState.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/EndEffectorCommand.h"
#include "baxter_core_msgs/AnalogIOState.h"
#include "baxter_core_msgs/DigitalIOState.h"
#include "baxter_core_msgs/ITBState.h"
#include "baxter_core_msgs/EndEffectorState.h"
#include "baxter_core_msgs/EndpointState.h"

#include "baxter_core_msgs/SolvePositionIK.h"

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/joint_state_group.h"
#include "moveit/robot_state/conversions.h"

#include "image_transport/image_transport.h"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/highgui/highgui.hpp"

#include "tf/transform_listener.h"

#include "Eigen/Geometry"

class TopicManager
{

	private:
	
		ros::Subscriber robot_state_sub;
		ros::Subscriber robot_joint_states_sub;
		ros::Subscriber left_range_state_sub;
		ros::Subscriber left_gripper_state_sub;
		ros::Subscriber left_endpoint_state_sub;
		ros::Subscriber right_endpoint_state_sub;
		ros::Subscriber left_object_transform_sub;
		ros::Subscriber right_object_transform_sub;
		ros::Subscriber right_shoulder_button_sub;
		ros::Subscriber left_shoulder_button_sub;
		ros::Subscriber right_itb_button_sub;
		
		ros::Publisher robot_enable_pub;
		ros::Publisher left_joint_cmd_pub;
		ros::Publisher right_joint_cmd_pub;
		ros::Publisher left_gripper_cmd_pub;
		ros::Publisher display_pub;
		ros::Publisher reset_tracker_pub;
		
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;
		
		ros::ServiceClient l_IKSolverClient;
		ros::ServiceClient r_IKSolverClient;
		
		robot_model::RobotModelPtr kinematic_model;
		robot_state::RobotStatePtr kinematic_state;
		robot_state::JointStateGroup* left_joint_state_group;
		
		baxter_core_msgs::AssemblyState robot_state;
		baxter_core_msgs::JointCommand left_joint_cmd;
		baxter_core_msgs::AnalogIOState left_range_state;
		baxter_core_msgs::EndEffectorState left_gripper_state;
		baxter_core_msgs::EndpointState left_endpoint_state;
		baxter_core_msgs::EndpointState right_endpoint_state;
		baxter_core_msgs::EndEffectorCommand left_gripper_cmd;
		
		std_msgs::Bool robot_enable;
		std_msgs::Bool reset_tracker;
		
		sensor_msgs::JointState robot_joint_states;
		//sensor_msgs::Image image;
		
		cv_bridge::CvImagePtr cv_ptr;
		
		geometry_msgs::Pose left_object_transform;
		geometry_msgs::Pose right_object_transform;
		
		tf::TransformListener tl;
		
		geometry_msgs::TransformStamped world2leftcamera_msg;
		geometry_msgs::TransformStamped left2rightcamera_msg;
		
		int right_shoulder_button;
		int left_shoulder_button;
		
		bool right_itb_button;
		
		bool robot_state_valid;
		bool robot_joint_states_valid;
		bool left_range_state_valid;
		bool left_gripper_state_valid;
		bool left_endpoint_state_valid;
		bool right_endpoint_state_valid;
		bool left_object_transform_valid;
		bool right_object_transform_valid;
		bool image_valid;
		
		bool init_demo;
		bool init_demo_initial;
		bool init_demo_final;
		bool velocity_control;
		
		bool dance_finished;
	
		
	public:
	
		TopicManager( ros::NodeHandle n ) : it( n )
		{
		
			/* Subscribed topics */
			robot_state_sub = n.subscribe( "robot/state", 1, &TopicManager::robotStateCb, this );
			robot_joint_states_sub = n.subscribe( "robot/joint_states", 1, &TopicManager::robotJointStatesCb, this );
			left_range_state_sub = n.subscribe( "robot/analog_io/left_hand_range/state", 1, &TopicManager::leftRangeStateCb, this );
			left_gripper_state_sub = n.subscribe( "robot/end_effector/left_gripper/state", 1, &TopicManager::leftGripperStateCb, this );
			left_endpoint_state_sub = n.subscribe( "/robot/limb/left/endpoint_state", 1, &TopicManager::leftEndpointStateCb, this);
			right_endpoint_state_sub = n.subscribe( "/robot/limb/right/endpoint_state", 1, &TopicManager::rightEndpointStateCb, this);
			left_object_transform_sub = n.subscribe( "left_object_transform", 1, &TopicManager::leftObjectTransformCb, this );
			right_object_transform_sub = n.subscribe( "right_object_transform", 1, &TopicManager::rightObjectTransformCb, this );
			right_shoulder_button_sub = n.subscribe( "robot/digital_io/right_shoulder_button/state", 1, &TopicManager::rightShoulderButtonCb, this );
			left_shoulder_button_sub = n.subscribe( "robot/digital_io/left_shoulder_button/state", 1, &TopicManager::leftShoulderButtonCb, this );
			right_itb_button_sub = n.subscribe( "robot/itb/torso_right_itb/state", 1, &TopicManager::rightItbButtonCb, this );
			
			image_sub = it.subscribe( "cameras/right_hand_camera/image", 1, &TopicManager::imageCb, this );
			
			/* Publishing topics */
			robot_enable_pub = n.advertise< std_msgs::Bool >( "robot/set_super_enable", 1, this );
			left_joint_cmd_pub = n.advertise< baxter_core_msgs::JointCommand >( "robot/limb/left/joint_command", 1, this );
			right_joint_cmd_pub = n.advertise< baxter_core_msgs::JointCommand >( "robot/limb/right/joint_command", 1, this );
			left_gripper_cmd_pub = n.advertise< baxter_core_msgs::EndEffectorCommand >( "robot/end_effector/left_gripper/command", 1, this );
			display_pub = n.advertise < sensor_msgs::Image > ( "robot/xdisplay", 1, this );
			reset_tracker_pub = n.advertise < std_msgs::Bool > ( "reset_tracker", 1, this );
			
			/* Service for the IK Solver */
			l_IKSolverClient = n.serviceClient< baxter_core_msgs::SolvePositionIK >( "/ExternalTools/left/PositionKinematicsNode/IKService" );
			r_IKSolverClient = n.serviceClient< baxter_core_msgs::SolvePositionIK >( "/ExternalTools/right/PositionKinematicsNode/IKService" );
			
			/* Load the robot model */
  		robot_model_loader::RobotModelLoader robot_model_loader( "robot_description" );

  		/* Get a shared pointer to the model */
  		kinematic_model = robot_model_loader.getModel();

		  /* WORKING WITH THE KINEMATIC STATE */
		  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
		  kinematic_state = robot_state::RobotStatePtr( new robot_state::RobotState ( kinematic_model ) );
		  
		  /* Initialization of the flags for the subscribers */
		  robot_state_valid = false;
		  robot_joint_states_valid = false;
		  left_range_state_valid = false;
		  left_gripper_state_valid = false;
		  left_endpoint_state_valid = false;
		  right_endpoint_state_valid = false;
		  left_object_transform_valid = false;
		  right_object_transform_valid = false;
		  
		  image_valid = false;
		  
		  velocity_control = false;
		  
		  init_demo_initial = false;
		  init_demo_final = false;
		  init_demo = false;
		
		}
		
		~TopicManager() {}
		
		void robotStateCb( const baxter_core_msgs::AssemblyStatePtr &msg )
		{
		
			robot_state = *msg;
			
			robot_state_valid = true;
		
		}
		
		void robotJointStatesCb( const sensor_msgs::JointStatePtr &msg )
		{
		
			robot_joint_states = *msg;
			
			robot_joint_states_valid = true;
		
		}
		
		void leftRangeStateCb( const baxter_core_msgs::AnalogIOStatePtr &msg )
		{
		
			left_range_state = *msg;
			
			left_range_state_valid = true;
		
		}
		
		void leftGripperStateCb( const baxter_core_msgs::EndEffectorStatePtr &msg )
		{
		
			left_gripper_state = *msg;
			
			left_gripper_state_valid = true;
		
		}
		
		void leftEndpointStateCb( const baxter_core_msgs::EndpointStatePtr &msg )
		{
		
			left_endpoint_state = *msg;
			
			left_endpoint_state_valid = true;
		
		}
		
		void rightEndpointStateCb( const baxter_core_msgs::EndpointStatePtr &msg )
		{
		
			right_endpoint_state = *msg;
			
			right_endpoint_state_valid = true;
		
		}
		
		void leftObjectTransformCb( const geometry_msgs::PosePtr &msg )
		{
		
			left_object_transform = *msg;
			
			left_object_transform_valid = true;
		
		}
		
		void rightObjectTransformCb( const geometry_msgs::PosePtr &msg )
		{
		
			right_object_transform = *msg;
			
			right_object_transform_valid = true;
		
		}
		
		void rightShoulderButtonCb( const baxter_core_msgs::DigitalIOStatePtr &msg )
		{
		
			right_shoulder_button = msg->state;
			
			if ( right_shoulder_button == 0 )
				releaseObject();
		
		}
		
		void leftShoulderButtonCb( const baxter_core_msgs::DigitalIOStatePtr &msg )
		{
		
			left_shoulder_button = msg->state;
			
			if ( left_shoulder_button == 0 && init_demo_initial == init_demo_final )
			{
				
				init_demo_final = !init_demo_final;
				
				if ( !init_demo )
					init_demo = true;
				else
					init_demo = false;
				
			}
			
			if ( left_shoulder_button == 1 && init_demo_initial == !init_demo_final )
			{
			
				init_demo_final = !init_demo_final;
			
			}
		
		}
		
		void rightItbButtonCb( const baxter_core_msgs::ITBStatePtr &msg )
		{
		
			right_itb_button = msg->buttons[0];
		
		}
		
		void imageCb( const sensor_msgs::ImageConstPtr &msg )
		{
		
			try
    	{
    
      	cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
      	
      	image_valid = true;
    
    	}
    	catch ( cv_bridge::Exception& e )
    	{
    
      	ROS_ERROR( "cv_bridge exception: %s", e.what () );
      	return;
    
    	}
    
		}
		
		bool demoInit()
		{
		
			return init_demo;
		
		}
		
		bool velocityControl()
		{
		
			return velocity_control;
		
		}
		
		bool dataValid()
		{
		
			return ( robot_state_valid &&
							 robot_joint_states_valid &&
							 left_range_state_valid &&
							 left_gripper_state_valid &&
							 left_endpoint_state_valid &&
							 right_endpoint_state_valid &&
							 image_valid );
		
		}
		
		bool initialized()
		{
		
			return ( robot_state.enabled && left_gripper_state.calibrated ) ? true : false;
		
		}
		
		void initialization()
		{
		
			enableRobot();
			
			left_gripper_cmd.command = "calibrate";
			left_gripper_cmd.id = 65538;
				
			left_gripper_cmd_pub.publish( left_gripper_cmd );
		
		}
		
		void computeIK( const geometry_msgs::Pose &pose, const std::string arm = "left" )
		{
		
			baxter_core_msgs::SolvePositionIK srv;
	
			geometry_msgs::PoseStamped target_pose;

			target_pose.pose = pose;

			target_pose.header.seq = 0;
			target_pose.header.stamp = ros::Time::now();
			target_pose.header.frame_id = "/world";

			srv.request.pose_stamp.push_back( target_pose );
			
			if ( arm == "left" ? l_IKSolverClient.call( srv ) : r_IKSolverClient.call( srv ) )
			{
				
				ROS_INFO( "Kinematics Valid: %d", srv.response.isValid[0] );
				
				if ( srv.response.isValid[0] )
				{

					std::vector< double > pos_cmd( srv.response.joints[0].position );
					std::vector< std::string > name_cmd( srv.response.joints[0].name );
					baxter_core_msgs::JointCommand cmd;

					cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
					cmd.command = pos_cmd;
					cmd.names = name_cmd;

					arm == "left" ? left_joint_cmd_pub.publish( cmd ) : right_joint_cmd_pub.publish( cmd );

				}

			}
			else
			{

				ROS_ERROR("Failed to call service");

			}
		
		}
		
		bool goingToWaitingPosition()
		{
		
			geometry_msgs::Pose left_target_pose;
			
			left_target_pose.position.x = 0.427753147796;
			left_target_pose.position.y = 0.378422680993;
			left_target_pose.position.z = 0.422769849615;
			
			left_target_pose.orientation.x = 0.380941309953;
			left_target_pose.orientation.y = 0.818982770786;
			left_target_pose.orientation.z = -0.202192138417;
			left_target_pose.orientation.w = 0.378509284812;
			
			geometry_msgs::Pose right_target_pose;
			
			right_target_pose.position.x = 0.427753147796;
			right_target_pose.position.y = -0.378422680993;
			right_target_pose.position.z = 0.422769849615;
			
			right_target_pose.orientation.x = -0.380941309953;
			right_target_pose.orientation.y = 0.818982770786;
			right_target_pose.orientation.z = 0.202192138417;
			right_target_pose.orientation.w = 0.378509284812;
			
			double left_error = sqrt( pow( left_endpoint_state.pose.position.x - left_target_pose.position.x, 2 ) +
										 			 			pow( left_endpoint_state.pose.position.y - left_target_pose.position.y, 2 ) +
										 			 			pow( left_endpoint_state.pose.position.z - left_target_pose.position.z, 2 ) +
										 			 			pow( left_endpoint_state.pose.orientation.x - left_target_pose.orientation.x, 2 ) +
										 			 			pow( left_endpoint_state.pose.orientation.y - left_target_pose.orientation.y, 2 ) +
										 			 			pow( left_endpoint_state.pose.orientation.z - left_target_pose.orientation.z, 2 ) +
										 			 			pow( left_endpoint_state.pose.orientation.w - left_target_pose.orientation.w, 2 ) );
			
			double right_error = sqrt( pow( right_endpoint_state.pose.position.x - right_target_pose.position.x, 2 ) +
										 			 			 pow( right_endpoint_state.pose.position.y - right_target_pose.position.y, 2 ) +
										 			 			 pow( right_endpoint_state.pose.position.z - right_target_pose.position.z, 2 ) +
										 			 			 pow( right_endpoint_state.pose.orientation.x - right_target_pose.orientation.x, 2 ) +
										 			 			 pow( right_endpoint_state.pose.orientation.y - right_target_pose.orientation.y, 2 ) +
										 			 			 pow( right_endpoint_state.pose.orientation.z - right_target_pose.orientation.z, 2 ) +
										 			 			 pow( right_endpoint_state.pose.orientation.w - right_target_pose.orientation.w, 2 ) );
			
			if ( left_error >= 0.1 )
				computeIK( left_target_pose, "left" );
			
			if ( right_error >= 0.1 )
				computeIK( right_target_pose, "right" );
			
			if ( !objectReleased() )
				releaseObject();
			
			if ( left_error < 0.1 && right_error < 0.1 && objectReleased() )
				return true;
			else
				return false;
			
		}
		
		//bool objectReacheable()
		bool objectVisible()
		{
		
			if ( right_object_transform_valid )
			{
			
				double tx = right_object_transform.position.x;
				double ty = right_object_transform.position.y;
				double tz = right_object_transform.position.z;
				double qx = right_object_transform.orientation.x;
				double qy = right_object_transform.orientation.y;
				double qz = right_object_transform.orientation.z;
				double qw = right_object_transform.orientation.w;
				
				if ( tx == -1 && ty == -1 && tz == -1 && qx == -1 && qy == -1 && qz == -1 && qw == -1 )
				{
				
					return false;
				
				}
				else
					return true;
			
			}
			else
				return false;
		
		}
		
		bool reachingForObject()
		{
		
			tf::StampedTransform world2leftcamera;
			tf::StampedTransform left2rightcamera;
			
			//ROS_INFO_STREAM( "Object Transform: " << right_object_transform );
			
			//ROS_INFO( "Object Transform Valid: %d", objectVisible() );
			
			try
			{
	
				if ( objectVisible() )
				{
					
					tl.waitForTransform( "/base", "/left_hand_camera", ros::Time( 0 ), ros::Duration( 1.0 ) );
					tl.lookupTransform( "/base", "/left_hand_camera", ros::Time( 0 ), world2leftcamera );
					
					tl.waitForTransform( "/left_hand_camera", "/right_hand_camera", ros::Time( 0 ), ros::Duration( 1.0 ) );
					tl.lookupTransform( "/left_hand_camera", "/right_hand_camera", ros::Time( 0 ), left2rightcamera );
				
					tf::transformStampedTFToMsg( world2leftcamera, world2leftcamera_msg );
					tf::transformStampedTFToMsg( left2rightcamera, left2rightcamera_msg );
					
					/* Object frame in the right hand camera reference frame */
					Eigen::Vector3f right_object_trans( right_object_transform.position.x,
																						 	right_object_transform.position.y,
																	 					 	right_object_transform.position.z );
					
					Eigen::Quaternion< float > right_object_rot( right_object_transform.orientation.w,
																								 			 right_object_transform.orientation.x,
																								 			 right_object_transform.orientation.y,
																								 			 right_object_transform.orientation.z );
					
					/* Conversion of the object frame to the left hand camera reference frame */
					Eigen::Vector3f left2right_camera_frame_trans( left2rightcamera_msg.transform.translation.x,
																												 left2rightcamera_msg.transform.translation.y,
																												 left2rightcamera_msg.transform.translation.z );
					
					Eigen::Quaternion< float > left2right_camera_frame_rot( left2rightcamera_msg.transform.rotation.w,
																											 						left2rightcamera_msg.transform.rotation.x,
																											 						left2rightcamera_msg.transform.rotation.y,
																											 						left2rightcamera_msg.transform.rotation.z );
			
					Eigen::Vector3f left_object_trans = left2right_camera_frame_rot._transformVector( right_object_trans );
					
					left_object_trans = left_object_trans + left2right_camera_frame_trans;
					
					Eigen::Quaternion< float > left_object_rot = left2right_camera_frame_rot*right_object_rot;
					
					/* Computation of the error in the pose */
					Eigen::Matrix3f left_object_rot_mat = left_object_rot.toRotationMatrix();
					Eigen::Vector3f left_object_angles = left_object_rot_mat.eulerAngles( 0, 1, 2 );
					
					//ROS_INFO_STREAM( "Object Angles: " << left_object_angles );
					
					/* Added to test the task jacobian */
					
					/*Eigen::MatrixXd task_jacob( 6, 6 );
					
					task_jacob << -cos( left_object_angles[1] )*cos( left_object_angles[2] ), sin( left_object_angles[2] ), -sin( left_object_angles[1] ), 0, left_object_trans[0]*sin( left_object_angles[1] )*cos( left_object_angles[2] ) - left_object_trans[2]*sin( left_object_angles[1] ), left_object_trans[0]*cos( left_object_angles[1] )*sin( left_object_angles[2] ) + left_object_trans[1]*cos( left_object_angles[2] ),
												-sin( left_object_angles[2] ), -cos( left_object_angles[0] )*cos( left_object_angles[2] ), sin( left_object_angles[0] ), left_object_trans[1]*sin( left_object_angles[0] )*cos( left_object_angles[2] ) + left_object_trans[2]*cos( left_object_angles[0] ), 0, -left_object_trans[0]*cos( left_object_angles[2] ) + left_object_trans[1]*cos( left_object_angles[0] )*sin( left_object_angles[2] ),
												sin( left_object_angles[1] ), -sin( left_object_angles[0] ), -cos( left_object_angles[0] )*cos( left_object_angles[1] ), -left_object_trans[1]*cos( left_object_angles[0] ) + left_object_trans[2]*sin( left_object_angles[0] )*cos( left_object_angles[1] ), left_object_trans[0]*cos( left_object_angles[1] ) + left_object_trans[2]*cos( left_object_angles[0] )*sin( left_object_angles[1] ), 0,
												0, 0, 0, -cos( left_object_angles[1] )*cos( left_object_angles[2] ), left_object_angles[0]*sin( left_object_angles[1] )*cos( left_object_angles[2] ) - sin( left_object_angles[2] ) + left_object_angles[2]*cos( left_object_angles[1] ), left_object_angles[0]*cos( left_object_angles[1] )*sin( left_object_angles[2] ) - left_object_angles[1]*cos( left_object_angles[2] ) + sin( left_object_angles[1] ),
												0, 0, 0, -sin( left_object_angles[2] ) + left_object_angles[1]*sin( left_object_angles[0] )*cos( left_object_angles[2] ) - left_object_angles[2]*cos( left_object_angles[0] ), -cos( left_object_angles[0] )*cos( left_object_angles[2] ), -left_object_angles[0]*cos( left_object_angles[2] ) + left_object_angles[1]*cos( left_object_angles[0] )*sin( left_object_angles[2] ) - sin( left_object_angles[0] ),
												0, 0, 0, sin( left_object_angles[1] ) - left_object_angles[1]*cos( left_object_angles[0] ) + left_object_angles[2]*sin( left_object_angles[0] )*cos( left_object_angles[1] ), left_object_angles[0]*cos( left_object_angles[1] ) - sin( left_object_angles[0] ) + left_object_angles[2]*cos( left_object_angles[0] )*sin( left_object_angles[1] ), - cos( left_object_angles[0] )*cos( left_object_angles[1] );
						
					Eigen::MatrixXd task_jacob_inv = task_jacob.inverse();
					
					Eigen::VectorXd pose_error( 6, 1 );
					
					// Desired Rotation
					Eigen::Quaternion< float > desired_rot( 1, 0, 0, 0 );
					Eigen::Matrix3f desired_rot_mat = desired_rot.toRotationMatrix();
					Eigen::Vector3f desired_angles = desired_rot_mat.eulerAngles( 0, 1, 2 );
					
					pose_error[0] = left_object_trans[0];
					pose_error[1] = left_object_trans[1];
					pose_error[2] = left_object_trans[2];
					pose_error[3] = left_object_angles[0] - desired_angles[0];
					pose_error[4] = left_object_angles[1] - desired_angles[1];
					pose_error[5] = left_object_angles[2] - desired_angles[2];
					
					Eigen::VectorXd cam_vel( 6, 1 );
					
					cam_vel = task_jacob_inv*pose_error;*/
					
					/* */
					
					Eigen::VectorXd pose_error( 6, 1 );
					
					pose_error[0] = left_object_trans[0];
					pose_error[1] = left_object_trans[1] + 0.02;
					pose_error[2] = left_object_trans[2];
					pose_error[3] = left_object_angles[0];
					pose_error[4] = left_object_angles[1];
					pose_error[5] = left_object_angles[2];
					
					Eigen::VectorXd cam_vel( 6, 1 );
					
					cam_vel[0] = 3*pose_error[0];
					cam_vel[1] = 3*pose_error[1];
					cam_vel[2] = pose_error[2];
					cam_vel[3] = pose_error[3];
					cam_vel[4] = pose_error[4];
					cam_vel[5] = pose_error[5];
					
					//ROS_INFO_STREAM( "Camera Velocity: " << cam_vel );
					
					computeJointVelocities( cam_vel );
					
					/* Conversion of the object frame to the world reference frame */
					Eigen::Vector3f camera_frame_trans( world2leftcamera_msg.transform.translation.x,
																							world2leftcamera_msg.transform.translation.y,
																							world2leftcamera_msg.transform.translation.z );
					
					Eigen::Quaternion< float > camera_frame_rot( world2leftcamera_msg.transform.rotation.w,
																											 world2leftcamera_msg.transform.rotation.x,
																											 world2leftcamera_msg.transform.rotation.y,
																											 world2leftcamera_msg.transform.rotation.z );
			
					Eigen::Vector3f object_trans = camera_frame_rot._transformVector( left_object_trans );
					
					object_trans = object_trans + camera_frame_trans;
					
					Eigen::Quaternion< float > object_rot = camera_frame_rot*left_object_rot;
					
					geometry_msgs::Pose target_pose;
					
					target_pose.position.x = object_trans[0];
					target_pose.position.y = object_trans[1];
					target_pose.position.z = object_trans[2];
					
					target_pose.orientation.x = object_rot.x();
					target_pose.orientation.y = object_rot.y();
					target_pose.orientation.z = object_rot.z();
					target_pose.orientation.w = object_rot.w();
					
					double error = sqrt( pow( left_endpoint_state.pose.position.x - target_pose.position.x, 2 ) +
															 pow( left_endpoint_state.pose.position.y - target_pose.position.y, 2 ) +
															 pow( left_endpoint_state.pose.position.z - target_pose.position.z, 2 ) );
			
					//ROS_INFO_STREAM( " Target Position: " << target_pose );
					
					//ROS_INFO_STREAM( "Error: " << error );
					
					if ( error < 0.15 )
					{
						
						changeControlMode();
						
						return true;
						
					}
					else
						return false;
			
				}
				else
				{
				
					if ( velocity_control )
						changeControlMode();
					
					return false;
				
				}
			
			}
			catch( tf::TransformException e )
			{
	
				ROS_ERROR( "%s", e.what() );
				
				if ( velocity_control )
						changeControlMode();
				
				return false;
		
			}

		}
		
		void computeJointVelocities( const Eigen::VectorXd &twist )
		{
		
			/* Set all joints in this state to their current values */
			kinematic_state->setStateValues ( robot_joint_states );
			
			/* Get the configuration for the joints in the left arm */
		  left_joint_state_group = kinematic_state->getJointStateGroup ( "left_arm" );
		  
		  /* Get the names of the joints in the left_arm */
		  std::vector< std::string > left_joint_names = left_joint_state_group->getJointModelGroup ()->getJointModelNames ();
		  
		  /* Get the joint values*/
			std::vector<double> joint_values;
			left_joint_state_group->getVariableValues(joint_values);
		  
		  /* Compute the joint velocities */
		  Eigen::VectorXd qdot;
		  left_joint_state_group->computeJointVelocity( qdot, twist, "left_hand_camera" );
			
		  if ( qdot.maxCoeff() > 0.3 )
  			qdot = 0.3*qdot/qdot.maxCoeff();
		  
		  if ( qdot.minCoeff() < -0.3 )
  			qdot = -0.3*qdot/qdot.minCoeff();
		  
		  std::vector< double > vel_cmd;
			std::vector< std::string > name_cmd;
		  
		  for ( int i = 0; i < 7; i++ )
		  {
		  
		  	vel_cmd.push_back( qdot[i] );
		  	name_cmd.push_back( left_joint_names[i] );
		  	//ROS_INFO_STREAM( "Velocity: " << qdot[i] );
		  	//ROS_INFO_STREAM( "Name: " << left_joint_names[i] );
		  	//ROS_INFO_STREAM( "Joint Values: " << joint_values[i] );
			
			}
			
			baxter_core_msgs::JointCommand cmd;

			cmd.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
			cmd.command = vel_cmd;
			cmd.names = name_cmd;

			left_joint_cmd_pub.publish( cmd );
			
			velocity_control = true;
		
		}
		
		void changeControlMode()
		{
		
			/* Set all joints in this state to their current values */
			kinematic_state->setStateValues ( robot_joint_states );
			
			/* Get the configuration for the joints in the left arm */
		  left_joint_state_group = kinematic_state->getJointStateGroup ( "left_arm" );
		  
		  /* Get the names of the joints in the left_arm */
		  std::vector< std::string > left_joint_names = left_joint_state_group->getJointModelGroup ()->getJointModelNames ();
			
			/* Get the joint values*/
			std::vector<double> joint_values;
			left_joint_state_group->getVariableValues(joint_values);
			
			std::vector< double > pos_cmd;
			std::vector< std::string > name_cmd;
			
			for ( int i = 0; i < 7; i++ )
		  {
		  
		  	pos_cmd.push_back( joint_values[i] );
		  	name_cmd.push_back( left_joint_names[i] );
			
			}
			
			baxter_core_msgs::JointCommand cmd;
			
			cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
			cmd.command = pos_cmd;
			cmd.names = name_cmd;

			left_joint_cmd_pub.publish( cmd );
			
			velocity_control = false;
		
		}
		
		bool objectGrasped()
		{
		
			return left_gripper_state.gripping;
		
		}
		
		bool objectMissed()
		{
		
			return ( left_gripper_state.missed == 1 );
		
		}
		
		void graspObject()
		{
		
			if ( left_range_state.value < 100 )
			{
			
				left_gripper_cmd.command = "grip";
				left_gripper_cmd.id = 65538;
				
				left_gripper_cmd_pub.publish( left_gripper_cmd );
			
			}
		
		}
		
		bool goingToSafePosition()
		{
		
			geometry_msgs::Pose target_pose;
			
			target_pose.position.x = 0.5;
			target_pose.position.y = 0;
			target_pose.position.z = 0.5;
			
			Eigen::Vector3f axis( 0, 1, 0 );
			
			axis.normalize();
			
			double angle = 3.14;
			
			Eigen::Quaternion< float > quaternion( Eigen::AngleAxis< float > ( angle, axis ) );
			
			target_pose.orientation.x = quaternion.x();
			target_pose.orientation.y = quaternion.y();
			target_pose.orientation.z = quaternion.z();
			target_pose.orientation.w = quaternion.w();
			
			computeIK( target_pose );
			
			double error = sqrt( pow( left_endpoint_state.pose.position.x - target_pose.position.x, 2 ) +
										 pow( left_endpoint_state.pose.position.y - target_pose.position.y, 2 ) +
										 pow( left_endpoint_state.pose.position.z - target_pose.position.z, 2 ) +
										 pow( left_endpoint_state.pose.orientation.x - target_pose.orientation.x, 2 ) +
										 pow( left_endpoint_state.pose.orientation.y - target_pose.orientation.y, 2 ) +
										 pow( left_endpoint_state.pose.orientation.z - target_pose.orientation.z, 2 ) +
										 pow( left_endpoint_state.pose.orientation.w - target_pose.orientation.w, 2 ) );
			
			if ( error < 0.1 )
				return true;
			else
				return false;
			
		}
		
		bool placingObject( int &counter, int &cube )
		{
		
			geometry_msgs::Pose target_pose;
			
			if ( counter == 0 )
			{
				
				if ( cube == 0 )
				{
				
					target_pose.position.x = -0.1;
					target_pose.position.y = 0.9;
					target_pose.position.z = 0.2;
					
				}
				
				if ( cube == 1 )
				{
				
					target_pose.position.x = 0.0;
					target_pose.position.y = 1.0;
					target_pose.position.z = 0.2;
					
				}
				
				if ( cube == 2 )
				{
				
					target_pose.position.x = 0.15;
					target_pose.position.y = 0.9;
					target_pose.position.z = 0.2;
					
				}
				
				if ( cube == 3 )
				{
				
					target_pose.position.x = 0.0;
					target_pose.position.y = 0.8;
					target_pose.position.z = 0.2;
					
				}
			
			}
			else if ( counter == 1 )
			{
				
				if ( cube == 0 )
				{
				
					target_pose.position.x = -0.1;
					target_pose.position.y = 0.9;
					target_pose.position.z = 0.0;
					
				}
				
				if ( cube == 1 )
				{
				
					target_pose.position.x = 0.0;
					target_pose.position.y = 1.0;
					target_pose.position.z = 0.0;
					
				}
				
				if ( cube == 2 )
				{
				
					target_pose.position.x = 0.15;
					target_pose.position.y = 0.9;
					target_pose.position.z = 0.0;
					
				}
				
				if ( cube == 3 )
				{
				
					target_pose.position.x = 0.0;
					target_pose.position.y = 0.8;
					target_pose.position.z = 0.0;
					
				}
			
			}
			
			Eigen::Vector3f axis( 0, 1, 0 );
			
			axis.normalize();
			
			double angle = 3.14;
			
			Eigen::Quaternion< float > quaternion( Eigen::AngleAxis< float > ( angle, axis ) );
			
			target_pose.orientation.x = quaternion.x();
			target_pose.orientation.y = quaternion.y();
			target_pose.orientation.z = quaternion.z();
			target_pose.orientation.w = quaternion.w();
			
			computeIK( target_pose );
			
			double error = sqrt( pow( left_endpoint_state.pose.position.x - target_pose.position.x, 2 ) +
										 pow( left_endpoint_state.pose.position.y - target_pose.position.y, 2 ) +
										 pow( left_endpoint_state.pose.position.z - target_pose.position.z, 2 ) +
										 pow( left_endpoint_state.pose.orientation.x - target_pose.orientation.x, 2 ) +
										 pow( left_endpoint_state.pose.orientation.y - target_pose.orientation.y, 2 ) +
										 pow( left_endpoint_state.pose.orientation.z - target_pose.orientation.z, 2 ) +
										 pow( left_endpoint_state.pose.orientation.w - target_pose.orientation.w, 2 ) );
			
			if ( error < 0.1 && counter == 0 )
			{
			
				counter = 1;
				return false;
			
			}
			else if ( error < 0.1 && counter == 1 )
				return true;
			else
				return false;
		
		}
		
		bool objectReleased()
		{
		
			return !( left_gripper_state.gripping || left_gripper_state.missed );
		
		}
		
		void releaseObject()
		{
		
			left_gripper_cmd.command = "release";
			left_gripper_cmd.id = 65538;
				
			left_gripper_cmd_pub.publish( left_gripper_cmd );
		
		}
		
		bool leavingObject( int &cube )
		{
		
			geometry_msgs::Pose target_pose;
			
			if ( cube == 0 )
				{
				
					target_pose.position.x = -0.12;
					target_pose.position.y = 0.9;
					target_pose.position.z = 0.4;
					
				}
				
				if ( cube == 1 )
				{
				
					target_pose.position.x = 0.0;
					target_pose.position.y = 1.02;
					target_pose.position.z = 0.4;
					
				}
				
				if ( cube == 2 )
				{
				
					target_pose.position.x = 0.12;
					target_pose.position.y = 0.9;
					target_pose.position.z = 0.4;
					
				}
				
				if ( cube == 3 )
				{
				
					target_pose.position.x = 0.0;
					target_pose.position.y = 0.78;
					target_pose.position.z = 0.4;
					
				}
			
			Eigen::Vector3f axis( 0, 1, 0 );
			
			axis.normalize();
			
			double angle = 3.14;
			
			Eigen::Quaternion< float > quaternion( Eigen::AngleAxis< float > ( angle, axis ) );
			
			target_pose.orientation.x = quaternion.x();
			target_pose.orientation.y = quaternion.y();
			target_pose.orientation.z = quaternion.z();
			target_pose.orientation.w = quaternion.w();
			
			computeIK( target_pose );
			
			double error = sqrt( pow( left_endpoint_state.pose.position.x - target_pose.position.x, 2 ) +
										 pow( left_endpoint_state.pose.position.y - target_pose.position.y, 2 ) +
										 pow( left_endpoint_state.pose.position.z - target_pose.position.z, 2 ) +
										 pow( left_endpoint_state.pose.orientation.x - target_pose.orientation.x, 2 ) +
										 pow( left_endpoint_state.pose.orientation.y - target_pose.orientation.y, 2 ) +
										 pow( left_endpoint_state.pose.orientation.z - target_pose.orientation.z, 2 ) +
										 pow( left_endpoint_state.pose.orientation.w - target_pose.orientation.w, 2 ) );
			
			if ( error < 0.1 )
			{
			
				if ( cube < 3 )
					cube += 1;
				else
					cube = 0;
					
				return true;
				
			}
			else
				return false;
		
		}
		
		bool robotEnabled()
		{
		
			return robot_state.enabled;
		
		}
		
		void enableRobot ()
		{
		
			robot_enable.data = true;
			
			robot_enable_pub.publish( robot_enable );
		
		}
		
		void screenImage( const std::string state = "" )
		{
			
			cv_bridge::CvImage screen_image;
			
			if ( state == "success" )
			{
			
				screen_image.encoding = sensor_msgs::image_encodings::BGR8;
				//screen_image.image = cv::imread("src/baxter_demo_museum/include/baxter_demo_museum/face.jpg");
				screen_image.image = cv::imread("face.jpg");
				display_pub.publish( screen_image.toImageMsg() );
			
			}
			else if ( state == "failed" )
			{
			
				screen_image.encoding = sensor_msgs::image_encodings::BGR8;
				//screen_image.image = cv::imread("src/baxter_demo_museum/include/baxter_demo_museum/facef.jpg");
				screen_image.image = cv::imread("facef.jpg");
				display_pub.publish( screen_image.toImageMsg() );
			
			}
			else
			{
			
				//screen_image.image = cv::Mat::zeros( 600, 1024, CV_8U );
				/*screen_image.image = cv::Mat::zeros(  , CV_8U );
				cv_ptr->image.copyTo( screen_image.image( cv::Rect( 192, 100, 832, 500) ) );*/
				
				//display_pub.publish( screen_image.toImageMsg() );
				display_pub.publish( cv_ptr->toImageMsg() );
				
			}
		
		}
		
		bool startDance()
		{
		
			return right_itb_button;
		
		}
		
		bool danceFinished()
		{
		
			return dance_finished;
		
		}
		
		void dance( int &counter )
		{
		
			if ( counter == 0 )
			{
			
				
			
			}
		
		}
		
		void tracker_reset( bool b)
		{
			
			reset_tracker.data = b;
			
			reset_tracker_pub.publish( reset_tracker );
		
		}
		
};

int main(int argc, char **argv)
{

	ros::init( argc, argv, "state_machine");
	ros::NodeHandle nh;
	
	ros::Rate r ( 100 );
	
	enum state_machine
	{
	
		initialization,
		initial_state,
		searching_object,
		reaching_object,
		grasping_object,
		going_away,
		placing_object,
		waiting,
		releasing_object,
		leaving_object,
		dancing
	
	};
	
	state_machine sm = initialization;
	state_machine sm_prev;
	
	TopicManager tm ( nh );
	
	ros::Time start;
	ros::Duration d;
	
	int counter = 0;
	int cube = 0;
	
	while ( ros::ok() )
	{
	
		if ( tm.dataValid() )
			break;
			
		//ROS_INFO( "In first loop" );
		
		ros::spinOnce();
		r.sleep();
	
	}
	
	while ( ros::ok() )
	{
	
		switch ( sm )
		{
		
			case initialization:
			
				if ( tm.initialized() )
					sm = initial_state;
				else
					tm.initialization();
				
				break;
			
			case initial_state:
			
				if ( tm.goingToWaitingPosition() && tm.demoInit() )
				{
				
					sm_prev = sm;
					sm = searching_object;
					
				}
				
				break;
			
			case searching_object:
			
				if ( tm.objectVisible() )
				{
				
					ros::Duration( 1.0 ).sleep();
					
					if ( tm.objectVisible() )
					{
					
						sm_prev = sm;
						sm = reaching_object;
						start = ros::Time::now();
					
					}
					
				}
				
				break;
			
			case reaching_object:
			
				d = ros::Time::now() - start;
				
				if ( d.toSec() < 10 )
				{
				
					if ( tm.reachingForObject() )
					{
						
						sm_prev = sm;
						sm = grasping_object;
						start = ros::Time::now();
						
					}
					
				}
				else
				{
					
					sm_prev = sm;
					sm = initial_state;
					tm.changeControlMode();
					
				}
				
				break;
			
			case grasping_object:
				
				d = ros::Time::now() - start;
				
				if ( d.toSec() < 2 )
				{
				
					if ( tm.objectGrasped() )
					{
				
						sm_prev = sm;
						sm = going_away;
					
					}
					else if ( tm.objectMissed() )
					{
				
						sm_prev = sm;
						sm = initial_state;
				
					}
					else
						tm.graspObject();
				
				}
				else
				{
					
					sm_prev = sm;
					sm = initial_state;
					
				}
				
				break;
			
			case going_away:
			
				if ( !tm.objectGrasped() )
				{
				
					sm_prev = sm;
					sm = initial_state;
				
				}
				else if ( tm.goingToSafePosition() )
				{
					
					sm_prev = sm;
					sm = placing_object;
				
				}
				
				break;
			
			case placing_object:
			
				if ( !tm.objectGrasped() )
				{
				
					sm_prev = sm;
					sm = initial_state;
				
				}
				else if ( tm.placingObject( counter, cube ) )
				{
				
					counter = 0;
					sm_prev = sm;
					sm = waiting;
					
				}
				
				break;
			
			case waiting:
			
				ros::Duration( 1.0 ).sleep();
				sm_prev = sm;
				sm = releasing_object;
				
				break;
			
			case releasing_object:
			
				if ( tm.objectReleased() )
				{
					
					sm_prev = sm;
					sm = leaving_object;
					
				}
				else
					tm.releaseObject();
				
				break;
			
			case leaving_object:
				
				if ( tm.leavingObject( cube ) )
				{
				
					sm_prev = sm;
					sm = initial_state;
					
				}
				
				break;
			
			case dancing:
			
				if ( !tm.danceFinished() )
					tm.dance( counter );
				else
				{
					
					counter = 0;
					sm_prev = sm;
					sm = initial_state;
					
				}
				
				break;
		
		}
		
		if ( !tm.demoInit() )
		{
		
			if ( tm.velocityControl() )
				tm.changeControlMode();
			
			sm = initial_state;
			
			if ( tm.startDance() )
				sm = dancing;
		
		}
		
		if ( tm.objectGrasped() )
			tm.screenImage( "success" );
		else if ( tm.objectMissed() )
			tm.screenImage( "failed" );
		else if ( tm.objectReleased() && ( sm_prev == grasping_object || sm_prev == going_away ) )
			tm.screenImage( "failed" );
		else
			tm.screenImage();
		
		if ( sm == searching_object || sm == reaching_object || sm == grasping_object )
			tm.tracker_reset( false );
		else
			tm.tracker_reset( true );
		
		//ROS_INFO( "Initiate: %d", tm.demoInit() );
		ROS_INFO( "State: %d", sm );
		//ROS_INFO( "Counter: %d", counter );
		ROS_INFO( "Cube: %d", cube );
		
		ros::spinOnce();
		r.sleep();
	
	}
	
	return 0;
	
}
