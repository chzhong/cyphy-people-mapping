#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "Trajectory.h"
#include "ClusterTree.h"
#include <vector>
#include <limits>
#include "trajectory_clustering/cluster_msg.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "colors.h"

enum status_type{CREATING, UPDATING, NEW_TRAJ, SPLITTING}; 
double resolution;
double origin_x;
double origin_y;
double map_width;
double map_height;

ros::Time last_pose; int Traj_count;
int lastCluster=-1;
Trajectory currentTraj;
bool creatingCluster;
bool firstCluster;
ClusterTree clusters;
int currentCluster;
const double BRANCH_THRESH = 10.0;
double delta,alpha,ccThresh,sigma,step,drift;
bool newCluster=0;
double last_dist = std::numeric_limits<double>::max();
int Cluster::s_nIDGenerator=0;
double Cluster::s_dfDelta=0.5;
double Cluster::s_dfAlpha=0.5;
double Cluster::s_defaultSigma=0.3;
double ClusterTree::s_ccThresh=3.0;
status_type status = CREATING;
bool SPLIT_FLAG=0;
float robotx,lastx;
float roboty,lasty;
int displaycount=0;
ros::Publisher marker_pub;

void publishMarkers()
{

    std::vector<double> x,y,sigma;
    std::vector<int> ID;
    clusters.getMsgData(x,y,sigma,ID);
    for(int i=0; i<x.size(); i++)
    {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "clusters";
        marker.id = i;

        marker.type = visualization_msgs::Marker::CYLINDER;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x[i];
        marker.pose.position.y = y[i];
        marker.pose.position.z = 0;

        marker.scale.x = sigma[i];
        marker.scale.y = sigma[i];

        int c = ID[i];
        marker.color.r=colors[c][0];
        marker.color.g=colors[c][1];
        marker.color.b=colors[c][2];
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);
    }
}
void ClusterCB()
{
    displaycount++;
    if(status==NEW_TRAJ)
    {
        currentTraj.addPose(robotx,roboty);
        //if(!(displaycount%200))
        ROS_INFO("Status is NEW_TRAJ");
        if(currentTraj.size() < 10)
            return;  // wait for a few points before doing comparison
        // Are we creating or updating?
        if(clusters.closestNode(currentTraj) > BRANCH_THRESH)
        {
            ROS_INFO("Setting status to creating");
            status=CREATING;
        }
        else
        {
            ROS_INFO("Setting status to updating");
            status=UPDATING;
        }

    }
    if(currentTraj.stepDist(robotx, roboty) > step)  // as in, gone from end of corridor back to lifts
    {
        ROS_INFO("FORCING STEP_DIST NEW TRAJECTORY");
        // Force a new trajectory
        if(status == CREATING && currentTraj.size() > 10)  // put a bound on how small trajectories can be
        {
            ROS_INFO("Current trajectory length %d",currentTraj.size());
            currentTraj.prune();
            ROS_INFO("Pruned trajectory length %d",currentTraj.size());
            Cluster temp = Cluster(currentTraj);
            if(SPLIT_FLAG==1)
            {
                ROS_INFO("Split complete, added %d",temp.getID());
                if(currentTraj.size() > 1)
                    clusters.append_as_child(temp);
            }
            else
            {
                if(currentTraj.size() > 1)
                    clusters.insert(temp);
            }
            SPLIT_FLAG=0;
        }
        currentTraj.clear();
        status = NEW_TRAJ;
        return;
    }
    currentTraj.addPose(robotx,roboty);
    if(status == UPDATING)
    {
        ROS_INFO("Status is UPDATING, current cluster is %d, hasChild is %s", clusters.getCurrentCluster(), clusters.hasChild()?"true":"false");
        bool atEnd, hasChild;
        currentTraj.addPose(robotx,roboty);
        clusters.updateCurrentCluster(robotx,roboty,atEnd,hasChild);
        double cdist = clusters.clusterDistance(currentTraj);
        if(atEnd==true)
        {
            ROS_INFO("Reached End of Cluster");
            currentTraj.clear();
            status = NEW_TRAJ;
        }
        if(cdist > drift || atEnd)
        {
            bool oldHasChild = clusters.hasChild();
            ROS_INFO("Time to split");
            if(clusters.Split(currentTraj))
            {
                ROS_INFO("SPLITTING ... Creating new trajectory");
            }
            else
            {
                ROS_INFO("WARNING: Split done but stub is short - check that concat fixes this");
            }
            ROS_INFO("GOT HERE, Current cluster is %d, hasChild is %s, old hasChild is %s",clusters.getCurrentCluster(),clusters.hasChild()?"true":"false",oldHasChild?"true":"false");
            if(oldHasChild)  // don't recompute here!!!
            {
                ROS_INFO("Trying to match child nodes");
                status = SPLITTING;
            }
            else
            {
                ROS_INFO("No child nodes, straight to creating new node");
                status = CREATING;
                SPLIT_FLAG=1;  // when we've finished with this trajectory, append it as a child node
            }
        }
    }
    else if(status == CREATING)
    {
        //if(!(displaycount%200))
            ROS_INFO("Status is CREATING");
        currentTraj.addPose(robotx,roboty);
    }
    else if(status == SPLITTING)
    {
        ROS_INFO("Trying to match to child nodes of %d", clusters.getCurrentCluster());
        if(currentTraj.size() < 40)
        {
            ROS_INFO("Current trajectory is size %d: Waiting on more nodes", currentTraj.size());
            return;
        }
        // check for a match amongst existing child nodes
        if(clusters.matchChild(currentTraj))
        {
            status = UPDATING;
            SPLIT_FLAG = 0;
        }
        else
        {
            SPLIT_FLAG = 1;
            status = CREATING;
        }
            
    }
    else
    {
        ROS_INFO("Invalid Status");
    }
   
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    resolution = msg->info.resolution;
    origin_x = msg->info.origin.position.x;
    origin_y = msg->info.origin.position.x;
    map_width = msg->info.width;
    map_height= msg->info.height;

    ROS_INFO("Got map, resolution is %f",resolution);
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_clusterer");
    ros::NodeHandle n;
    last_pose=ros::Time::now();
    firstCluster=1;
    Traj_count = 0;
    status = CREATING;
    //ros::Subscriber sub_pose = n.subscribe("amcl_pose",100,poseCallback);
    //ros::Subscriber sub_pose = n.subscribe("person_track",100,personCallback);
    ros::Subscriber sub_map = n.subscribe("map",5,mapCallback);
    ros::Publisher cluster_pub = n.advertise<trajectory_clustering::cluster_msg>("clusters",1);
    ros::Publisher raw_pub= n.advertise<geometry_msgs::PointStamped>("raw_data",1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("delta",delta,double(0.5));
    private_node_handle_.param("step",step,double(1.0));
    private_node_handle_.param("alpha",alpha,double(0.1));
    private_node_handle_.param("ccThresh",ccThresh,double(0.6));
    private_node_handle_.param("drift",drift,double(5.0));
    private_node_handle_.param("sigma",sigma,double(0.3));
    ROS_INFO("Delta is %f, Alpha is %f, Cluster-Cluster Thresh is %f, drift is %f, sigma is %f, max step is %f",delta, alpha, ccThresh,drift,sigma, step);
    Cluster::setDelta(delta);
    Cluster::setAlpha(alpha);
    ClusterTree::setCCThresh(ccThresh);
    Cluster::setSigma(sigma);
    ros::Rate loop_rate(10);
    int count = 0;
    tf::TransformListener listener;
    while(ros::ok())
    {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        //ROS_INFO("Robot at %f,%f",transform.getOrigin().x(),transform.getOrigin().y() );
        robotx = transform.getOrigin().x();
        roboty = transform.getOrigin().y();
        if(lastx!=robotx || lasty!=roboty)
        {
            geometry_msgs::PointStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.point.x = robotx;
            msg.point.y = roboty;
            raw_pub.publish(msg);
            ClusterCB();
        }
        publishMarkers();
        count++;
        if(count==100)
        {
            count = 0;
            trajectory_clustering::cluster_msg msg;
            std::vector<double> x,y,sigma;
            std::vector<int> ID;
            clusters.getMsgData(x,y,sigma,ID);
            msg.x.insert(msg.x.end(),x.begin(),x.end());
            msg.y.insert(msg.y.end(),y.begin(),y.end());
            msg.sigma.insert(msg.sigma.end(),sigma.begin(),sigma.end());
            msg.ID.insert(msg.ID.end(),ID.begin(),ID.end());
            cluster_pub.publish(msg);
            std::cout << "Tree is: \n";
            clusters.printTree();
            std::cout << "\n";
            clusters.merge_visit();
            if(!SPLITTING)
                clusters.concat_visit();
        }
        lastx = robotx;
        lasty = roboty;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
