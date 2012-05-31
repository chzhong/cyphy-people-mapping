#ifndef TRAJECTORY_H 
#define TRAJECTORY_H 

#include <vector>

class Trajectory{
    public:
        Trajectory(){};
        bool addPose(float x, float y);
        int findClosest(double pX, double pY);
        bool addPoseFront(float x, float y);
        bool clear();
        int size(){return m_xPoses.size();};
        void getPoses(std::vector<double> &x, std::vector<double> &y){x=m_xPoses;y=m_yPoses;return;};
        double getX(int i){return m_xPoses[i];};
        double getY(int i){return m_yPoses[i];};
        double euclidean_length();
        float stepDist(float x, float y);
        bool splitAfter(int ind);
        bool prune();
    private:
        std::vector<double> m_xPoses;
        std::vector<double> m_yPoses;

};
#endif
