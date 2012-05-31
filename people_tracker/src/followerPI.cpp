// openni_tracker.cpp
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <sound_play/SoundRequest.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/callback_queue.h>
using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

ros::Publisher pubvel;
ros::Publisher pubsound;
ros::Publisher pubperson;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

double lin_speed = 0.4;
double ang_speed = 0.3;
double angular_vel_correction = 1.0;
double following_distance = 1.0;
double iterate_rate = 20;
double Kv=1.0;
double Ki=1.0;
double Kh=2.0;
double max_forward_velocity = 1.2;
double max_angular_velocity = 0.8;
double last_x;
double last_z;
bool SAFE = 1;
double SAFE_DISTANCE = 0.4;

int currently_tracking = 0;
bool am_tracking = 0;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  	ROS_INFO("New User %d", nId);

/*	if (g_bNeedPose)
    	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    	}
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
*/
		char sayBuf[50];
		g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, "./lizcalibration.bin");
		g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		currently_tracking = nId;
		ROS_INFO("Starting Tracking");
		sound_play::SoundRequest soundmsg;
		soundmsg.sound = sound_play::SoundRequest::SAY;
		soundmsg.command = sound_play::SoundRequest::PLAY_START;
		sprintf(sayBuf,"Now tracking user %d", nId);
		soundmsg.arg = sayBuf;
		pubsound.publish(soundmsg);
		am_tracking = 1;
		currently_tracking = nId;
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
    char sayBuf[50];
    		sound_play::SoundRequest soundmsg;
    		soundmsg.sound = sound_play::SoundRequest::SAY;
		soundmsg.command = sound_play::SoundRequest::PLAY_STOP;
		sprintf(sayBuf,"Now tracking user %d", nId);
		soundmsg.arg = sayBuf;
		pubsound.publish(soundmsg);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
        g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(nId, "lizcalibration.bin");
        printf("saved data\n");
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

geometry_msgs::Twist DetermineVelocity(double x, double z, double lin_speed)
{
    geometry_msgs::Twist tw;

    double e = sqrt(x*x+z*z) - following_distance;
    double v=Kv*e + Ki*e*(1.0/iterate_rate);
    double theta = -atan2(x,z);
    double w = Kh*theta;

    if (e > 1e-4)
    {

	    if(v < max_forward_velocity)
		tw.linear.x = v;
	    else
		tw.linear.x = max_forward_velocity;
	    
	    if(w < max_angular_velocity)
		tw.angular.z = w;
	    else
		tw.angular.z = max_angular_velocity;
    }
    else
    {
	tw.linear.x = 0.0;
	tw.angular.z = 0.0;
    }
    return tw;
    /*tw.linear.x = lin_speed;
    tw.angular.z = 0.0;

    // If X component is essentially zero, then just go straight
    if (fabs(x) < 1e-4)
        return tw;

    // Calculate Turning Radius
    double r = fabs((x*x + z*z)/(2*x));

    // Calculate Turning Vel
    tw.angular.z = -tw.linear.x/r; // a = r0, v = rw
    //if (x > 0.0)
     //   tw.angular.z = -tw.angular.z;

    return tw;*/
}


void publishTransforms() {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    bool got_one = 0;
/*
    if(users_count == 0)
    {
            geometry_msgs::Twist tw2;
            tw2.angular.z = 0.0;
            pubvel.publish(tw2);
	    return;
	}
*/

    for(int i=0; i < users_count; ++i)
    {
        XnUserID user = users[i];
  
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        // track the first user only
        if(!got_one)
        {
            got_one = 1;
            XnSkeletonJointPosition joint_position;
            g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, joint_position);
            double x = -joint_position.position.X / 1000.0;
            double y = joint_position.position.Y / 1000.0;
            double z = joint_position.position.Z / 1000.0;

            ROS_INFO("Target is at %f,%f,%f", x,y,z);
            // This is a hard-coded transform into the base_link frame of the torso of the person being tracked
            geometry_msgs::PointStamped pLoc;
            pLoc.header.frame_id = "/base_link";
            pLoc.header.stamp= ros::Time::now();
            pLoc.point.x = z;
            pLoc.point.y = x;
            pLoc.point.z = 1.33+y;
            pubperson.publish(pLoc);

            geometry_msgs::Twist tw2;
	    if(x != last_x && z != last_z)
            	tw2 = DetermineVelocity(x,z,lin_speed);
	    else
	    {
	        tw2.linear.x = 0.0;
	        tw2.angular.z = 0.0;
		}
            // Convert Twist to ROS-Twist msg
            if(SAFE)
                pubvel.publish(tw2);
	    last_x = x;
	    last_z = z;
           return ;
        }
        else
            break;

    }
	return ;
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    bool allSafe = 1;

    for(int i=0; i<msg->ranges.size(); i++)
    {
        if(msg->ranges[i] < SAFE_DISTANCE)
        {
            geometry_msgs::Twist tw2;
	        tw2.linear.x = 0.0;
            tw2.angular.z = 0.0;
            pubvel.publish(tw2);
            SAFE = 0;
            allSafe = 0;
        }
    }
    if(allSafe)
    {
        SAFE=1;
    }
    else
	ROS_INFO("NOT SAFE");

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;

    ROS_INFO("Starting Up");

    pubvel = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    pubsound = nh.advertise<sound_play::SoundRequest> ("/robotsound", 1);
    pubperson = nh.advertise<geometry_msgs::PointStamped>("/person_track",1);

    //ros::Subscriber sub = nh.subscribe("/laser",10,laserCallback);

    string configFilename = ros::package::getPath("people_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

	XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
    
	ros::Rate r(iterate_rate);
        
    ros::NodeHandle pnh("~");
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);
	while (ros::ok()) {
	   g_Context.WaitAndUpdateAll();
	   publishTransforms();
	   //ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	}

	g_Context.Shutdown();
	return 0;
}
