#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "trajectory_clustering/cluster_msg.h"
#include "map_creation/OccupancyGrid.h"
#include "map_creation/EigenMultivariateNormal.h"

COccupancyGrid m_grid(0.05);
ros::Publisher costmap_pub;
float maxX,maxY,minX,minY;

void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    double resolution = msg->info.resolution;
    /*origin_x = msg->info.origin.position.x;
    origin_y = msg->info.origin.position.x;
    map_width = msg->info.width;
    map_height= msg->info.height;
*/
    ROS_INFO("Got map, resolution is %f",resolution);
}

void clusterCallback(const trajectory_clustering::cluster_msgConstPtr &msg)
{
    ROS_INFO("Got Cluster, has %d elements", msg->x.size());
    m_grid.Clear();
    minX = std::numeric_limits<float>::max();
    maxX = -std::numeric_limits<float>::max();
    minY = minX;
    maxY = maxX;

    for(int i=0; i<msg->x.size(); i++)
    {

        Eigen::Matrix<float,2,1> mean;
        mean(0) = msg->x[i];
        mean(1) = msg->y[i];
        Eigen::Matrix<float,2,2> cov;
        cov(0,0) = 0.5*msg->sigma[i];
        cov(0,1) = 0;
        cov(1,0) = 0;
        cov(1,1) = 0.5*msg->sigma[i];

        EigenMultivariateNormal<float,2> gen(mean,cov);

        for(int i=0;i<3000; i++)
        {
            Eigen::Matrix<float,2,1> sampleVec;
            gen.nextSample(sampleVec);
            m_grid.AddPeopleData(sampleVec(0),sampleVec(1));
        }

    }
    
}

void publishMap()
{
    if(!(m_grid.numElem() > 0))
        return;
    std::vector<signed char> theData(m_grid.width()*m_grid.height(),254);
    double xOrig, yOrig;
    if(!m_grid.GetMsgData(theData,xOrig,yOrig))
        return;
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";
    msg.info.resolution = 0.05;
    msg.info.width = m_grid.width();
    msg.info.height= m_grid.height();
    msg.info.origin.position.x = xOrig;
    msg.info.origin.position.y = yOrig;
    msg.data = theData;
    costmap_pub.publish(msg);

}
int main(int argc, char **argv)
{
    ROS_INFO("Started up");
    ros::init(argc,argv,"map_creation");
    ros::NodeHandle n;
    ros::Subscriber map_sub = n.subscribe("/map",5,mapCallback);
    ros::Subscriber cluster_sub = n.subscribe("/clusters",5,clusterCallback);
    costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("people_map",1);
    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok())
    {
        count++;
        if( count%100 == 0 )
            ROS_INFO("Alive");
        if(count%30 == 0)
            publishMap();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
