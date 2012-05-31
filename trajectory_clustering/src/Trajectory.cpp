#include "Trajectory.h"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <iostream>
bool Trajectory::addPose(float x, float y)
{
    m_xPoses.push_back(x);
    m_yPoses.push_back(y);
    return true;
}

int Trajectory::findClosest(double pX, double pY)
{
    Eigen::Map<Eigen::VectorXd> xs(&m_xPoses[0],m_xPoses.size());
    Eigen::Map<Eigen::VectorXd> ys(&m_yPoses[0],m_yPoses.size());
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
    int coeff;
    dist.minCoeff(&coeff);

    return coeff;
}

bool Trajectory::addPoseFront(float x, float y)
{
    std::vector<double>::iterator it;

    it = m_xPoses.begin();
    m_xPoses.insert(it,1,x);
    it = m_yPoses.begin();
    m_yPoses.insert(it,1,y);
    return true;
}

bool Trajectory::clear()
{
    m_xPoses.clear();
    m_yPoses.clear();
    return true;
}

// do this before adding to cluster
bool Trajectory::prune()
{
    Trajectory prunedTrajectory;
    prunedTrajectory.addPose(m_xPoses[0],m_yPoses[0]);

    double current_x = m_xPoses[0];
    double current_y = m_yPoses[0];
    for(int i=1; i < m_xPoses.size(); i++)
    {
        // calculate mahalanobis distance
        Eigen::MatrixXf sigma(2,2);
        sigma(0,0) = 0.3;
        sigma(1,0) = 0.0;
        sigma(0,1) = 0.0;
        sigma(1,1) = 0.3;
        Eigen::VectorXf y(2,1);
        y(0)=m_xPoses[i];
        y(1)=m_yPoses[i];
        Eigen::VectorXf mu(2,1);
        mu(0) = current_x;
        mu(1) = current_y;
   
         
        Eigen::VectorXf d;
        Eigen::MatrixXf D2;
        d = (y-mu);
        std::cout << "Here is y-mu " << d << std::endl;
        std::cout << "Here is y-mu transposed " << d.transpose() << std::endl;
        std::cout << "Here is sigma " << sigma << std::endl;
        D2 = d.transpose()*sigma;
        std::cout << "Here is D2 " << D2 << std::endl;
        D2*=d;
        std::cout << "Here is D2 final " << D2 << std::endl;

        if(D2(0) < 0.2)  // 
        {
            continue;
        }
        else
        {
            prunedTrajectory.addPose(m_xPoses[i],m_yPoses[i]);
            current_x = m_xPoses[i];
            current_y = m_yPoses[i];
        }
    }

    m_xPoses = prunedTrajectory.m_xPoses;
    m_yPoses = prunedTrajectory.m_yPoses;
    return true;
    

    
}
double Trajectory::euclidean_length()
{
    int last_el = m_xPoses.size()-1;
    double xdiff = m_xPoses[last_el]-m_xPoses[0];
    double ydiff = m_yPoses[last_el]-m_yPoses[0];
    return xdiff*xdiff+ydiff*ydiff;
}

float Trajectory::stepDist(float pX, float pY)
{
    if(m_xPoses.size() > 0)
    {
        int last_el = m_xPoses.size()-1;
        float xdiff = m_xPoses[last_el]-pX;
        float ydiff = m_yPoses[last_el]-pY;
        return std::sqrt(xdiff*xdiff+ydiff*ydiff);
    }
    else
        return 0.0;

}

bool Trajectory::splitAfter(int ind)
{
    //copy(m_xPoses.begin()+ind+1,m_xPoses.end(),m_xPoses.begin());
    //copy(m_yPoses.begin()+ind+1,m_yPoses.end(),m_yPoses.begin());
    m_xPoses.erase(m_xPoses.begin(),m_xPoses.begin()+ind);
    m_yPoses.erase(m_yPoses.begin(),m_yPoses.begin()+ind);
    return true;
}


