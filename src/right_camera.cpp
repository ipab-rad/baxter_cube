#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "baxter_core_msgs/DigitalIOState.h"

#include "geometry_msgs/Pose.h"

#include "std_msgs/Bool.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "cv_bridge/cv_bridge.h"

#include "image_transport/image_transport.h"

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMbEdgeTracker.h>

bool init_demo;
bool init_demo_initial;
bool init_demo_final;

bool reset_tracker;

int left_shoulder_button;

class ImageProcessing
{

	private:
	
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;
		
		cv::Mat yellow, blue, green;
		
		std::vector< std::vector< cv::Point > > y_contours, b_contours, g_contours;
		
		int y_min;
		int y_max;
		int b_min;
		int b_max;
		int g_min;
		int g_max;
		//int s_min;
		//int s_max;
		
		bool ybg_roi_found;
		bool yb_roi_found;
		bool yg_roi_found;
		bool bg_roi_found;
		
		bool image_valid;
		
		void filterImage ()
		{
		
			cv::Moments y_moments, b_moments, g_moments;
			
			int dist_thresh = 60; // Amount of pixels that can separate the centroids of two regions
			
			cv::findContours ( yellow, y_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
			cv::findContours ( blue, b_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
			cv::findContours ( green, g_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
			
			if ( !y_contours.empty() )
    	{
    		
    		double y_area;
    		double y_aux = cv::contourArea( y_contours[0] );
    
    		while ( y_contours.size() > 1 )
    		{
    			
    			y_area = cv::contourArea( y_contours[1] );
    			
    			if ( y_area < y_aux )
    			{
    				
    				y_contours.erase( y_contours.begin() + 1 );
    			
    			}
    			else
    			{
    			
    				y_aux = y_area;
    				y_contours.erase( y_contours.begin() );
    			
    			}
    		}
    	
    		y_area = y_aux;
    		
    	}
    	
    	if ( !b_contours.empty() )
    	{
    		
    		double b_area;
    		double b_aux = cv::contourArea( b_contours[0] );
    
    		while ( b_contours.size() > 1 )
    		{
    			
    			b_area = cv::contourArea( b_contours[1] );
    			
    			if ( b_area < b_aux )
    			{
    				
    				b_contours.erase( b_contours.begin() + 1 );
    			
    			}
    			else
    			{
    			
    				b_aux = b_area;
    				b_contours.erase( b_contours.begin() );
    			
    			}
    		}
    	
    		b_area = b_aux;
    		
    	}
    	
    	if ( !g_contours.empty() )
    	{
    		
    		double g_area;
    		double g_aux = cv::contourArea( g_contours[0] );
    
    		while ( g_contours.size() > 1 )
    		{
    			
    			g_area = cv::contourArea( g_contours[1] );
    			
    			if ( g_area < g_aux )
    			{
    				
    				g_contours.erase( g_contours.begin() + 1 );
    			
    			}
    			else
    			{
    			
    				g_aux = g_area;
    				g_contours.erase( g_contours.begin() );
    			
    			}
    		}
    	
    		g_area = g_aux;
    		
    	}
    	
    	if ( !y_contours.empty() && !b_contours.empty() && !g_contours.empty() )
			{
			
				y_moments = cv::moments( y_contours[0], true );
				y_centre.x = y_moments.m10/y_moments.m00;
				y_centre.y = y_moments.m01/y_moments.m00;
				
				b_moments = cv::moments( b_contours[0], true );
				b_centre.x = b_moments.m10/b_moments.m00;
				b_centre.y = b_moments.m01/b_moments.m00;
				
				g_moments = cv::moments( g_contours[0], true );
				g_centre.x = g_moments.m10/g_moments.m00;
				g_centre.y = g_moments.m01/g_moments.m00;
				
				double yb_dist = sqrt( ( pow( y_centre.x - b_centre.x, 2 ) ) + ( pow( y_centre.y - b_centre.y, 2 ) ) );
				double yg_dist = sqrt( ( pow( y_centre.x - g_centre.x, 2 ) ) + ( pow( y_centre.y - g_centre.y, 2 ) ) );
				double bg_dist = sqrt( ( pow( b_centre.x - g_centre.x, 2 ) ) + ( pow( b_centre.y - g_centre.y, 2 ) ) );
				
				//ROS_INFO_STREAM( "yb_dist = " << yb_dist << ", yg_dist = " << yg_dist << ", bg_dist = " << bg_dist );
				
				if ( ( yb_dist < dist_thresh ) && ( yg_dist < dist_thresh ) && ( bg_dist < dist_thresh ) )
				{
				
					ybg_roi_found = true;
					yb_roi_found = false;
					yg_roi_found = false;
					bg_roi_found = false;
				
				}
				else
				{
				
					ybg_roi_found = false;
					yb_roi_found = false;
					yg_roi_found = false;
					bg_roi_found = false;
				
				}
			
			}
			else if ( !y_contours.empty() && !b_contours.empty() )
  	  {
  	  	
  	  	y_moments = cv::moments( y_contours[0], true );
				y_centre.x = y_moments.m10/y_moments.m00;
				y_centre.y = y_moments.m01/y_moments.m00;
				
				b_moments = cv::moments( b_contours[0], true );
				b_centre.x = b_moments.m10/b_moments.m00;
				b_centre.y = b_moments.m01/b_moments.m00;
				
				double yb_dist = sqrt( ( pow( y_centre.x - b_centre.x, 2 ) ) + ( pow( y_centre.y - b_centre.y, 2 ) ) );
				
				if ( yb_dist < dist_thresh )
				{
				
					ybg_roi_found = false;
					yb_roi_found = true;
					yg_roi_found = false;
					bg_roi_found = false;
				
				}
				else
				{
				
					ybg_roi_found = false;
					yb_roi_found = false;
					yg_roi_found = false;
					bg_roi_found = false;
				
				}
				
  	  }
  	  else if ( !y_contours.empty() && !g_contours.empty() )
  	  {
  	  		
 	  		y_moments = cv::moments( y_contours[0], true );
				y_centre.x = y_moments.m10/y_moments.m00;
				y_centre.y = y_moments.m01/y_moments.m00;
			
				g_moments = cv::moments( g_contours[0], true );
				g_centre.x = g_moments.m10/g_moments.m00;
				g_centre.y = g_moments.m01/g_moments.m00;
					
				double yg_dist = sqrt( ( pow( y_centre.x - g_centre.x, 2 ) ) + ( pow( y_centre.y - g_centre.y, 2 ) ) );
			
				if ( yg_dist < dist_thresh )
				{
				
					ybg_roi_found = false;
					yb_roi_found = false;
					yg_roi_found = true;
					bg_roi_found = false;
				
				}
				else
				{
				
					ybg_roi_found = false;
					yb_roi_found = false;
					yg_roi_found = false;
					bg_roi_found = false;
				
				}
				
			}
 	  	else if ( !b_contours.empty() && !g_contours.empty() )
 	  	{
  	  			
  			b_moments = cv::moments( b_contours[0], true );
				b_centre.x = b_moments.m10/b_moments.m00;
				b_centre.y = b_moments.m01/b_moments.m00;
				
				g_moments = cv::moments( g_contours[0], true );
				g_centre.x = g_moments.m10/g_moments.m00;
				g_centre.y = g_moments.m01/g_moments.m00;
						
				double bg_dist = sqrt( ( pow( b_centre.x - g_centre.x, 2 ) ) + ( pow( b_centre.y - g_centre.y, 2 ) ) );
			
				if ( bg_dist < dist_thresh )
				{
				
					ybg_roi_found = false;
					yb_roi_found = false;
					yg_roi_found = false;
					bg_roi_found = true;
				
				}
				else
				{
				
					ybg_roi_found = false;
					yb_roi_found = false;
					yg_roi_found = false;
					bg_roi_found = false;
				
				}
  	  	
 	  	}		
 			else
 			{
  	  			
 				ybg_roi_found = false;
				yb_roi_found = false;
				yg_roi_found = false;
				bg_roi_found = false;
  	  			
 			}
	  	
	  	if ( ybg_roi_found )
	  	{
	  	
	  		cv::cvtColor( yellow, yellow, CV_GRAY2BGR );
	  		cv::cvtColor( blue, blue, CV_GRAY2BGR );
	  		cv::cvtColor( green, green, CV_GRAY2BGR );
	  		
	  		cv::drawContours( yellow, y_contours, 0, cv::Scalar(0, 255, 0), -1 );
	  		cv::drawContours( blue, b_contours, 0, cv::Scalar(0, 255, 0), -1 );
  			cv::drawContours( green, g_contours, 0, cv::Scalar(0, 255, 0), -1 );
	  	
	  	}
  		else if ( yb_roi_found )
  		{
	  	
  			cv::cvtColor( yellow, yellow, CV_GRAY2BGR );
  			cv::cvtColor( blue, blue, CV_GRAY2BGR );
  			
  			cv::drawContours( yellow, y_contours, 0, cv::Scalar(0, 255, 0), -1 );
  			cv::drawContours( blue, b_contours, 0, cv::Scalar(0, 255, 0), -1 );
  		
	  	}
 			else if ( yg_roi_found )
 			{
	  			
 				cv::cvtColor( yellow, yellow, CV_GRAY2BGR );
 				cv::cvtColor( green, green, CV_GRAY2BGR );
 				
 				cv::drawContours( yellow, y_contours, 0, cv::Scalar(0, 255, 0), -1 );
  			cv::drawContours( green, g_contours, 0, cv::Scalar(0, 255, 0), -1 );
	  		
			}
			else if ( bg_roi_found )
			{
	  				
				cv::cvtColor( blue, blue, CV_GRAY2BGR );
	  		cv::cvtColor( green, green, CV_GRAY2BGR );
	  		
	  		cv::drawContours( blue, b_contours, 0, cv::Scalar(0, 255, 0), -1 );
  			cv::drawContours( green, g_contours, 0, cv::Scalar(0, 255, 0), -1 );
	  		
			}
			
			//ROS_INFO_STREAM( "ROI ybg: " << ybg_roi_found << ", ROI yb: " << yb_roi_found << ", ROI yg: " << yg_roi_found << ", ROI bg: " << bg_roi_found );
	  		
		}
		
		bool pointsValid()
		{
		
			bool y_valid = ( y_centre.x >= 0 && y_centre.x <= 640 && y_centre.y >= 0 && y_centre.y <= 400 );
			bool b_valid = ( b_centre.x >= 0 && b_centre.x <= 640 && b_centre.y >= 0 && b_centre.y <= 400 );
			bool g_valid = ( g_centre.x >= 0 && g_centre.x <= 640 && g_centre.y >= 0 && g_centre.y <= 400 );
			
			return ( y_valid && b_valid && g_valid );
		
		}
	
	public:
	
		cv_bridge::CvImagePtr cv_ptr;
		
		cv::Point y_centre, b_centre, g_centre;
		
		bool initialized;
	
		ImageProcessing ( const ros::NodeHandle n ) : it ( n )
		{
		
			image_sub = it.subscribe("cameras/right_hand_camera/image", 1, &ImageProcessing::imageCb, this);
			
			y_min = 15;
			y_max = 35;
			b_min = 95;
			b_max = 130;
			g_min = 35;
			g_max = 95;
			
			//s_min = 0;//90;
			//s_max = 220;//210;
			
			cv::namedWindow( "Yellow" );
			//cv::moveWindow( "Yellow", 960, 0 );
			cv::createTrackbar( "Min threshold", "Yellow",  &y_min, 360);
			cv::createTrackbar( "Max threshold", "Yellow",  &y_max, 360);
			
			cv::namedWindow( "Blue" );
			//cv::moveWindow( "Yellow", 0, 540 );
			cv::createTrackbar( "Min threshold", "Blue",  &b_min, 360);
			cv::createTrackbar( "Max threshold", "Blue",  &b_max, 360);
			
			cv::namedWindow( "Green" );
			//cv::moveWindow( "Yellow", 960, 540 );
			cv::createTrackbar( "Min threshold", "Green",  &g_min, 360);
			cv::createTrackbar( "Max threshold", "Green",  &g_max, 360);
			
			//cv::namedWindow ( "Saturation" );
			//cv::createTrackbar ( "Min threshold", "Saturation",  &s_min, 255);
			//cv::createTrackbar ( "Max threshold", "Saturation",  &s_max, 255);
			
			image_valid = false;
			initialized = false;
		
		}
		
		~ImageProcessing () {}
		
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
		
		void segmentImage ()
		{
			
			if ( image_valid )
			{
			
				cv::Mat hsv, h, s;
				std::vector< cv::Mat > channels;
			
				cv::cvtColor ( cv_ptr->image, hsv, CV_BGR2HSV );
			
				cv::split(hsv, channels);
  		  h = channels[0];
  		  //s = channels[2];
  	  
  		  yellow = ( h > y_min ).mul( h < y_max );
  		  blue = ( h > b_min ).mul( h < b_max );
  		  green = ( h > g_min ).mul( h < g_max );
  		  //s = ( s > s_min ).mul( s < s_max );
			
				cv::erode( yellow, yellow, cv::Mat(), cv::Point(-1,-1), 3 );
				cv::erode( blue, blue, cv::Mat(), cv::Point(-1,-1), 3 );
				cv::erode( green, green, cv::Mat(), cv::Point(-1,-1), 3 );
				//cv::erode( s, s, cv::Mat(), cv::Point(-1,-1), 3 );
			
				//yellow = yellow.mul( s );
				//blue = blue.mul( s );
				//green = green.mul( s );
			
				cv::imshow ( "Yellow", yellow );
				cv::imshow ( "Blue", blue );
				cv::imshow ( "Green", green );
				//cv::imshow ( "Saturation", s );
				cv::waitKey ( 3 );
				
			}
		
		}
		
		void initialization ()
		{
		
			if ( image_valid )
			{
			
				//segmentImage();
				filterImage();
				
				if ( ybg_roi_found && pointsValid() )
					initialized = true;
				else
					initialized = false;
				
				/*cv::imshow ( "Yellow", yellow );
				cv::imshow ( "Blue", blue );
				cv::imshow ( "Green", green );
				cv::waitKey ( 3 );*/
				
			}
		
		}
		
		void reset()
		{
		
			initialized = false;
			ybg_roi_found = false;
			yb_roi_found = false;
			yg_roi_found = false;
			bg_roi_found = false;
			image_valid = false;
		
		}

		bool imageValid()
		{

			return image_valid;

		}
		
};

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

void resetTrackerCb( const std_msgs::BoolPtr &msg )
{

	reset_tracker = msg->data;

}

bool validPoints( const std::vector< vpImagePoint > ip )
{

	bool p1_valid = ( ( ip[0].get_i() > 0 ) && ( ip[0].get_i() < 400 ) && ( ip[0].get_j() > 0 ) && ( ip[0].get_j() < 640 ) );
	bool p2_valid = ( ( ip[1].get_i() > 0 ) && ( ip[1].get_i() < 400 ) && ( ip[1].get_j() > 0 ) && ( ip[1].get_j() < 640 ) );
	bool p3_valid = ( ( ip[2].get_i() > 0 ) && ( ip[2].get_i() < 400 ) && ( ip[2].get_j() > 0 ) && ( ip[2].get_j() < 640 ) );
	bool p4_valid = ( ( ip[3].get_i() > 0 ) && ( ip[3].get_i() < 400 ) && ( ip[3].get_j() > 0 ) && ( ip[3].get_j() < 640 ) );

	return ( p1_valid && p2_valid && p3_valid && p4_valid );

}

int main(int argc, char **argv)
{

	ros::init( argc, argv, "right_camera");
	ros::NodeHandle nh;
	
	ros::Publisher object_transform_pub = nh.advertise < geometry_msgs::Pose > ( "right_object_transform", 1 );
	
	ros::Subscriber left_shoulder_button_sub = nh.subscribe( "robot/digital_io/left_shoulder_button/state", 1, &leftShoulderButtonCb );
	ros::Subscriber reset_tracker_sub = nh.subscribe( "reset_tracker", 1, &resetTrackerCb );
	
	ros::Rate r ( 100 );
	
	geometry_msgs::Pose object_pose;
	
	ImageProcessing ip ( nh );
	
	// Tracker Stuff
	
	vpImage<unsigned char> I;
	vpCameraParameters cam;
	vpHomogeneousMatrix cMo;

	vpMbEdgeTracker tracker;
	vpMe me;
							
	double px = 402.539883605;
	double py = 402.539883605;
	double u0 = 323.154823854;
	double v0 = 180.298589253;
	
	cam.initPersProjWithoutDistortion( px, py, u0, v0);
	
	std::vector< vpImagePoint > imagePoints;
	std::vector< vpPoint > points3D;
	
	vpPoint pointO, pointX, pointY, pointZ;
	
	pointO.set_oX( 0 );
	pointO.set_oY( 0 );
	pointO.set_oZ( 0 );
	
	points3D.push_back( pointO );
	
	pointZ.set_oX( 0 );
	pointZ.set_oY( -0.05 );
	pointZ.set_oZ( 0 );
	
	points3D.push_back( pointZ );
	
	pointX.set_oX( 0 );
	pointX.set_oY( 0 );
	pointX.set_oZ( 0.05 );
	
	points3D.push_back( pointX );
	
	pointY.set_oX( 0.05 );
	pointY.set_oY( 0 );
	pointY.set_oZ( 0 );
	
	points3D.push_back( pointY );
	
	bool tracker_init = false;
	
	vpDisplayX display;
	
	// End of Tracker Stuff

	while ( ros::ok() )
	{

		if ( ip.imageValid() )
			break;
			
		//ROS_INFO( "In first loop" );
		
		ros::spinOnce();
		r.sleep();

	}
	
	while ( ros::ok() )
	{
	
		ip.segmentImage();
		
		if ( !ip.initialized && !reset_tracker )
		{
		
			ip.initialization();
		
		}
		else
		{
		
			vpImageConvert::convert( ip.cv_ptr->image, I );
			
			if ( !tracker_init && !reset_tracker )
			{
			
				me.setMaskSize(5);
				me.setMaskNumber(180);
				me.setRange(8);
				me.setThreshold(10000);
				me.setMu1(0.5);
				me.setMu2(0.5);
				me.setSampleStep(4);
				me.setNbTotalSample(250);
							
				tracker.setMovingEdge(me);
							
				tracker.setCameraParameters(cam);
				tracker.setAngleAppear( vpMath::rad(70) );
				tracker.setAngleDisappear( vpMath::rad(80) );
				tracker.setNearClippingDistance(0.1);
				tracker.setFarClippingDistance(100.0);
				tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
				tracker.setOgreVisibilityTest(false);
				//tracker.loadModel("src/baxter_demo_museum/include/baxter_demo_museum/cube.cao");
				tracker.loadModel("cube.cao");
				tracker.setDisplayFeatures(true);
				
				display.init( I );
				
				vpImagePoint i_point;
				i_point.set_i( ( ip.y_centre.y + ip.b_centre.y + ip.g_centre.y )/3 );
				i_point.set_j( ( ip.y_centre.x + ip.b_centre.x + ip.g_centre.x )/3 );
				
				imagePoints.push_back( i_point );
				
				if ( ip.y_centre.y < ip.b_centre.y && ip.y_centre.y < ip.g_centre.y )
				{
				
					// Z axis
					i_point.set_i( ip.y_centre.y );
					i_point.set_j( ip.y_centre.x );
					imagePoints.push_back( i_point );
					
					if ( ip.b_centre.x > ip.g_centre.x )
					{
					
						// X axis
						i_point.set_i( ip.b_centre.y );
						i_point.set_j( ip.b_centre.x );
						imagePoints.push_back( i_point );
						
						// Y axis
						i_point.set_i( ip.g_centre.y );
						i_point.set_j( ip.g_centre.x );
						imagePoints.push_back( i_point );
					
					}
					else
					{
					
						// X axis
						i_point.set_i( ip.g_centre.y );
						i_point.set_j( ip.g_centre.x );
						imagePoints.push_back( i_point );
						
						// Y axis
						i_point.set_i( ip.b_centre.y );
						i_point.set_j( ip.b_centre.x );
						imagePoints.push_back( i_point );
					
					}
				
				}
				else if ( ip.b_centre.y < ip.y_centre.y && ip.b_centre.y < ip.g_centre.y )
				{
				
					// Z axis
					i_point.set_i( ip.b_centre.y );
					i_point.set_j( ip.b_centre.x );
					imagePoints.push_back( i_point );
					
					if ( ip.y_centre.x > ip.g_centre.x )
					{
					
						// X axis
						i_point.set_i( ip.y_centre.y );
						i_point.set_j( ip.y_centre.x );
						imagePoints.push_back( i_point );
						
						// Y axis
						i_point.set_i( ip.g_centre.y );
						i_point.set_j( ip.g_centre.x );
						imagePoints.push_back( i_point );
					
					}
					else
					{
					
						// X axis
						i_point.set_i( ip.g_centre.y );
						i_point.set_j( ip.g_centre.x );
						imagePoints.push_back( i_point );
						
						// Y axis
						i_point.set_i( ip.y_centre.y );
						i_point.set_j( ip.y_centre.x );
						imagePoints.push_back( i_point );
					
					}
				
				}
				else if ( ip.g_centre.y < ip.y_centre.y && ip.g_centre.y < ip.b_centre.y )
				{
				
					// Z axis
					i_point.set_i( ip.g_centre.y );
					i_point.set_j( ip.g_centre.x );
					imagePoints.push_back( i_point );
					
					if ( ip.y_centre.x > ip.b_centre.x )
					{
					
						// X axis
						i_point.set_i( ip.y_centre.y );
						i_point.set_j( ip.y_centre.x );
						imagePoints.push_back( i_point );
						
						// Y axis
						i_point.set_i( ip.b_centre.y );
						i_point.set_j( ip.b_centre.x );
						imagePoints.push_back( i_point );
					
					}
					else
					{
					
						// X axis
						i_point.set_i( ip.b_centre.y );
						i_point.set_j( ip.b_centre.x );
						imagePoints.push_back( i_point );
						
						// Y axis
						i_point.set_i( ip.y_centre.y );
						i_point.set_j( ip.y_centre.x );
						imagePoints.push_back( i_point );
					
					}
				
				}
				
				try
				{
				
					if ( validPoints( imagePoints ) )
					{

						tracker.initFromPoints( I, imagePoints, points3D );
				
						tracker_init = true;

					}
					
				}
				catch ( vpException e )
				{
				
					std::cout << "Catch an exception: " << e << std::endl;
					
					tracker_init = false;
				
				}
			
			}
			
			/*cv::Point test_point;
			test_point.x = imagePoints[0].get_j();
			test_point.y = imagePoints[0].get_i();
			cv::circle( ip.cv_ptr->image, test_point, 2, cv::Scalar( 255, 255, 255 ), -1 );
			test_point.x = imagePoints[1].get_j();
			test_point.y = imagePoints[1].get_i();
			cv::circle( ip.cv_ptr->image, test_point, 2, cv::Scalar( 255, 0, 0 ), -1 );
			test_point.x = imagePoints[2].get_j();
			test_point.y = imagePoints[2].get_i();
			cv::circle( ip.cv_ptr->image, test_point, 2, cv::Scalar( 0, 0, 255 ), -1 );
			test_point.x = imagePoints[3].get_j();
			test_point.y = imagePoints[3].get_i();
			cv::circle( ip.cv_ptr->image, test_point, 2, cv::Scalar( 0, 255, 0 ), -1 );
			cv::imshow( "Centroids", ip.cv_ptr->image );
			cv::waitKey( 3 );*/
			
			if ( tracker_init && !reset_tracker )
			{
			
				try
				{

					vpDisplay::display( I );
							
					tracker.track( I );
					tracker.getPose( cMo );
					tracker.getCameraParameters( cam );
							
					tracker.display( I, cMo, cam, vpColor::red, 2 );
							
					vpDisplay::displayFrame( I, cMo, cam, 0.05, vpColor::none, 2 );
					vpDisplay::flush(I);
				
					vpTranslationVector t;
					vpQuaternionVector q;
				
					cMo.extract( t );
					cMo.extract( q );
				
					object_pose.position.x = t[0];
					object_pose.position.y = t[1];
					object_pose.position.z = t[2];
				
					object_pose.orientation.x = q.x();
					object_pose.orientation.y = q.y();
					object_pose.orientation.z = q.z();
					object_pose.orientation.w = q.w();
				
					object_transform_pub.publish( object_pose );
					
					ROS_INFO_STREAM( "Object Pose: " << object_pose );
				
				}
				catch	(	vpException e	)
				{
				
					std::cout << "Catch an exception: " << e << std::endl;
					vpDisplay::close( I );
					ip.reset();
					imagePoints.clear();
					tracker.resetTracker();
					tracker_init = false;
					
					object_pose.position.x = -1;
					object_pose.position.y = -1;
					object_pose.position.z = -1;
				
					object_pose.orientation.x = -1;
					object_pose.orientation.y = -1;
					object_pose.orientation.z = -1;
					object_pose.orientation.w = -1;
					
					object_transform_pub.publish( object_pose );
					
					ROS_INFO_STREAM( "Object Pose: " << object_pose );
			
				}
		
			}
			
		}
		
		if ( reset_tracker )
		{
		
			vpDisplay::close( I );
			ip.reset();
			imagePoints.clear();
			tracker.resetTracker();
			tracker_init = false;
					
			object_pose.position.x = -1;
			object_pose.position.y = -1;
			object_pose.position.z = -1;
				
			object_pose.orientation.x = -1;
			object_pose.orientation.y = -1;
			object_pose.orientation.z = -1;
			object_pose.orientation.w = -1;
					
			object_transform_pub.publish( object_pose );
		
		}
		
		ros::spinOnce();
		r.sleep();

	}
	
	return 0;

}
