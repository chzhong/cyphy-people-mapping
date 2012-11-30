#include "Cluster.h"
#include <Eigen/Dense>
#include "ros/ros.h"
#include <cmath>
#include <limits>
#include <algorithm>

Cluster::Cluster(Trajectory &traj, int parent)
{
    m_nID = s_nIDGenerator++;
    std::vector<double> xtmp;
    traj.getPoses(m_xPoses,m_yPoses);
    m_sigmaPoses.resize(m_xPoses.size(),s_defaultSigma);
    ROS_INFO("Created cluster from vector length %d, cluster length %d",traj.size(),(int)m_xPoses.size());
}

Cluster::Cluster(Trajectory &traj)
{
    m_nID = s_nIDGenerator++;
    std::vector<double> xtmp;
    traj.getPoses(m_xPoses,m_yPoses);
    m_sigmaPoses.resize(m_xPoses.size(),s_defaultSigma);
    ROS_INFO("Created cluster from vector length %d, cluster length %d",traj.size(),(int)m_xPoses.size());
}

bool Cluster::concatenate(Cluster &cluster)
{
    m_xPoses.insert(m_xPoses.end(),cluster.m_xPoses.begin(), cluster.m_xPoses.end());
    m_yPoses.insert(m_yPoses.end(),cluster.m_yPoses.begin(), cluster.m_yPoses.end());
    m_sigmaPoses.insert(m_sigmaPoses.end(),cluster.m_sigmaPoses.begin(), cluster.m_sigmaPoses.end());
    return true;
}

void Cluster::print()
{
    ROS_INFO("Cluster PRINT, cluster has %d elements", (int)m_xPoses.size());
    for(unsigned int i=0; i<m_xPoses.size(); i++)
        ROS_INFO("Pose %d, X pos %f, Y pose %f",i,m_xPoses[i],m_yPoses[i]);
}

double Cluster::windowDistance(double pX, double pY, int nInd, double delta)
{
    int indL = std::floor((1-delta)*nInd);
    int indH = std::ceil((1+delta)*nInd);

    if(indL < 0)
        indL = 0;

    if(indH >= (int)m_xPoses.size())
        indH = m_xPoses.size()-1;

    if(indH-indL == 0)
        indH = indL+2;

     Eigen::Map<Eigen::VectorXd> xs(&m_xPoses[indL],indH-indL);
     Eigen::Map<Eigen::VectorXd> ys(&m_yPoses[indL],indH-indL);
     Eigen::Map<Eigen::VectorXd> sigmas(&m_sigmaPoses[indL],indH-indL);

     Eigen::ArrayXd tPointx(xs.size());
     Eigen::ArrayXd tPointy(xs.size());
     Eigen::ArrayXd diffx(xs.size());
     Eigen::ArrayXd diffy(xs.size());
     Eigen::ArrayXd dist(xs.size());
     tPointx.setConstant(pX);
     tPointy.setConstant(pY); 

     diffx = tPointx-xs.array();
     diffy = tPointy-ys.array();

     dist = diffx*diffx + diffy*diffy;
     dist = dist.sqrt()/sigmas.array(); 
     return dist.minCoeff();
}

double Cluster::clusterDistance(Trajectory &traj)
{
    double theSum = 0.0;
    for(int i=0; i<traj.size(); i++)
    {
        int ind = findClosest(traj.getX(i),traj.getY(i));
        theSum=theSum+windowDistance(traj.getX(i),traj.getY(i),ind,s_dfDelta);
    }
    return theSum/traj.size();
}

double Cluster::clusterDistance(Trajectory &traj, int ind_lim)
{
    double theSum = 0.0;
    for(int i=0; i<ind_lim; i++)
    {
        int ind = findClosest(traj.getX(i),traj.getY(i));
        theSum=theSum+windowDistance(traj.getX(i),traj.getY(i),ind,s_dfDelta);
    }
    return theSum/ind_lim;
}


double Cluster::ccDistance(Cluster &cluster)
{
    double theSum = 0.0;
    for(int i=0; i<cluster.size(); i++)
    {
        int ind = findClosest(cluster.getX(i),cluster.getY(i));
        theSum=theSum+windowDistance(cluster.getX(i),cluster.getY(i),ind,s_dfDelta);
    }
    return theSum/cluster.size();
}


int Cluster::findClosest(double pX, double pY)
{ 
    Eigen::Map<Eigen::VectorXd> xs(&m_xPoses[0],m_xPoses.size());
    Eigen::Map<Eigen::VectorXd> ys(&m_yPoses[0],m_yPoses.size());
    Eigen::Map<Eigen::VectorXd> sigmas(&m_sigmaPoses[0],m_sigmaPoses.size());
    Eigen::ArrayXd tPointx(xs.size());
    Eigen::ArrayXd tPointy(xs.size());
    
    Eigen::ArrayXd diffx(xs.size());
    Eigen::ArrayXd diffy(xs.size());
    Eigen::ArrayXd dist(xs.size());
    
    tPointx.setConstant(pX);
    tPointy.setConstant(pY); 

    diffx = tPointx-xs.array();
    diffy = tPointy-ys.array();

    dist = diffx*diffx + diffy*diffy;
    dist = dist.sqrt()/sigmas.array(); 
    int coeff;
    dist.minCoeff(&coeff);


    return coeff;
}

int Cluster::findClosest(double pX, double pY, double &retdist)
{ 
    Eigen::Map<Eigen::VectorXd> xs(&m_xPoses[0],m_xPoses.size());
    Eigen::Map<Eigen::VectorXd> ys(&m_yPoses[0],m_yPoses.size());
    Eigen::Map<Eigen::VectorXd> sigmas(&m_sigmaPoses[0],m_sigmaPoses.size());
    Eigen::ArrayXd tPointx(xs.size());
    Eigen::ArrayXd tPointy(xs.size());
    
    Eigen::ArrayXd diffx(xs.size());
    Eigen::ArrayXd diffy(xs.size());
    Eigen::ArrayXd dist(xs.size());
    
    tPointx.setConstant(pX);
    tPointy.setConstant(pY); 

    diffx = tPointx-xs.array();
    diffy = tPointy-ys.array();

    dist = diffx*diffx + diffy*diffy;
    dist = dist.sqrt()/sigmas.array(); 
    int coeff;
    retdist=dist.minCoeff(&coeff);

    return coeff;
}

bool Cluster::split(Trajectory &traj, Cluster &newCluster, bool hasChild)
{
    I_DONT_TRUST_YOU=1;
    double dist;
    int traj_ind,ind;
    for(traj_ind=traj.size()-1; traj_ind>0; --traj_ind)
    {
        ind = findClosest(traj.getX(traj_ind),traj.getY(traj_ind),dist);
        ROS_INFO("At trajectory %d, closest on cluster is %d at distance %f", traj_ind, ind, dist);
        if(dist < 2)  // make this a parameter!!!
            break;  // found the closest within reasonable shot of the trajectory
    }

    I_DONT_TRUST_YOU=0;
    if(ind < 1 || ind > m_xPoses.size()-1)
    {
        ROS_INFO("Can't split, closest point is at %d on cluster of length %d",ind,m_xPoses.size());
        return false;

    }
    int iw = traj.findClosest(m_xPoses[ind],m_yPoses[ind]);
    ROS_INFO("Splitting current trajectory at %d, it has length %d",iw,traj.size());
    traj.splitAfter(iw);
    ROS_INFO("Now it has length %d",traj.size());

    if(!hasChild)  // only split off from stub if it hasn't already been done
    {
        newCluster.m_xPoses.resize(m_xPoses.size()-ind);
        newCluster.m_yPoses.resize(m_xPoses.size()-ind);
        newCluster.m_sigmaPoses.resize(m_xPoses.size()-ind);
        copy(m_xPoses.begin()+ind, m_xPoses.end(),newCluster.m_xPoses.begin());
        copy(m_yPoses.begin()+ind, m_yPoses.end(),newCluster.m_yPoses.begin());
        copy(m_sigmaPoses.begin()+ind, m_sigmaPoses.end(),newCluster.m_sigmaPoses.begin());
        if(newCluster.m_xPoses.size() > 1)
        {
            std::cout << "First part of split OK\n";
        }
        else
        {
            std::cout << "First part of split is JUNK!, has size " << newCluster.m_xPoses.size();
            return false;
        }
        m_xPoses.erase(m_xPoses.begin()+ind,m_xPoses.end());
        m_yPoses.erase(m_yPoses.begin()+ind,m_yPoses.end());
        m_sigmaPoses.erase(m_sigmaPoses.begin()+ind,m_sigmaPoses.end());

        if(m_xPoses.size() > 1)
        {
            std::cout << "Second part of split OK\n";
        }
        else
        {
            std::cout << "Second part of split is JUNK!, has size " << m_xPoses.size();
            return false;
        }
    }
 
    return true;
}

bool Cluster::update(double pX, double pY, int i)
{
    m_xPoses[i]=(1-s_dfAlpha)*m_xPoses[i]+s_dfAlpha*pX;
    m_yPoses[i]=(1-s_dfAlpha)*m_yPoses[i]+s_dfAlpha*pY;
    double xdiff = pX-m_xPoses[i];
    double ydiff = pY-m_yPoses[i];
    m_sigmaPoses[i]=(1-s_dfAlpha)*m_sigmaPoses[i]+s_dfAlpha*(xdiff*xdiff+ydiff*ydiff);
    if(m_sigmaPoses[i] > 0.6)
        m_sigmaPoses[i] = 0.6;
    return true;
}

bool Cluster::weightedAv(Cluster &c2)
{
    for(int i=0; i < c2.size(); i++)
    {
        int ind = findClosest(c2.getX(i),c2.getY(i));
        double sigma_sq_1 = m_sigmaPoses[ind]*m_sigmaPoses[ind]; 
        double sigma_sq_2 = c2.getSigma(i)*c2.getSigma(i);
        m_xPoses[ind] = (m_xPoses[ind]/sigma_sq_1+c2.getX(i)/sigma_sq_2)/(1.0/sigma_sq_1 + 1.0/sigma_sq_2);
        m_yPoses[ind] = m_yPoses[ind]/sigma_sq_1+c2.getY(i)/sigma_sq_2/(1.0/sigma_sq_1 + 1.0/sigma_sq_2);
        m_sigmaPoses[ind] = 1.0/(1.0/sigma_sq_1+1.0/sigma_sq_2);
    }
    return true;
}

double Cluster::startDist(double pX, double pY)
{
    double xdiff = pX-m_xPoses[0];
    double ydiff = pY-m_yPoses[0];
    return std::sqrt(xdiff*xdiff+ydiff*ydiff);
}

std::ostream& operator<<(std::ostream& output, Cluster& c)
{
    output << c.getID();
    return output;
}

