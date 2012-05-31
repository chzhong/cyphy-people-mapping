#ifndef CLUSTER_H
#define CLUSTER_H

#include <Eigen/Dense>
#include <iostream>
#include <ostream>

#include "Trajectory.h"
#include "ros/ros.h"
using namespace Eigen;

class Cluster{
    public:
        Cluster(){m_nID = s_nIDGenerator++;};
        Cluster(Trajectory &traj, int parent);
        Cluster(Trajectory &traj);
        void print();
        bool split(Trajectory &traj, Cluster &newCluster, bool hasChild);
        double startX(){return m_xPoses[0];}; 
        double endX(){return m_xPoses[m_xPoses.size()-1];}; 
        double endY(){return m_yPoses[m_yPoses.size()-1];}; 
        double startY(){return m_yPoses[0];}; 
        double clusterDistance(Trajectory &traj);
        double clusterDistance(Trajectory &traj, int ind_lim);
        double ccDistance(Cluster &cluster);
        static bool setDelta(double delta){s_dfDelta=delta;return true;};
        static bool setAlpha(double alpha){s_dfAlpha=alpha;return true;};
        static bool setSigma(double sigma){s_defaultSigma=sigma;return true;};
        int findClosest(double pX, double pY);
        int findClosest(double pX, double pY, double &dist);
        int getID(){return m_nID;};
        bool update(double pX, double pY, int ind);
        double startDist(double pX, double pY);
        double getX(int i){return m_xPoses[i];};
        double getY(int i){return m_yPoses[i];};
        double getSigma(int i){return m_sigmaPoses[i];};
        bool getXs(std::vector<double> &xs){xs=m_xPoses;return true;};
        bool getYs(std::vector<double> &ys){ys=m_yPoses;return true;};
        bool getSigmas(std::vector<double> &sigmas){sigmas=m_sigmaPoses;return true;};
        int size(){return m_xPoses.size();};
        bool weightedAv(Cluster &cluster);
        friend std::ostream& operator <<(std::ostream& output, Cluster &c);
        bool concatenate(Cluster &cluster);
    private:
    static int s_nIDGenerator;
    static double s_defaultSigma;
    static double s_dfDelta;
    static double s_dfAlpha;
    double windowDistance(double pX, double pY,int nInd, double delta);
    int m_nID;
    std::vector<double> m_xPoses;
    std::vector<double> m_yPoses;
    std::vector<double> m_sigmaPoses;
    bool I_DONT_TRUST_YOU;
};
#endif
