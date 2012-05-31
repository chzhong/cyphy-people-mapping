#ifndef CLUSTER_TREE_H
#define CLUSTER_TREE_H

#include "Cluster.h"
#include "ros/ros.h"
#include <tree.hh>
#include <tree_util.hh>

//int Cluster::s_nIDGenerator = 0;

class ClusterTree
{
    public:
        ClusterTree(){};
        int size(){return m_tree.size();};
        bool insert(Cluster &newCluster){m_lastCluster=m_tree.insert(m_tree.begin(),newCluster); return true;};
        bool append_as_child(Cluster &newCluster){m_currentCluster=m_tree.append_child(m_currentCluster,newCluster); return true;}
        bool append_as_branch(Cluster &newCluster){m_lastCluster=m_tree.append_child(m_currentCluster,newCluster); return true;}
        int getCurrent();
        static bool setCCThresh(double thresh){s_ccThresh=thresh; return true;};
        void merge_visit();
        void concat_visit();
        //bool push_back(Cluster &cluster){m_clusters.push_back(cluster); return true;};
        double rootNodeDistance(double poseX, double poseY);
        double clusterDistance(Trajectory &currentTraj){return (*m_currentCluster).clusterDistance(currentTraj);};
        bool updateCurrentCluster(double pX, double pY, bool &atEnd, bool &hasChild);
//        bool deleteCluster(int ind);
        bool getMsgData(std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &sigmas, std::vector<int> &IDs);
        void printTree(){kptree::print_tree_bracketed(m_tree,std::cout);};
        double closestNode(Trajectory &traj);
        bool Split(Trajectory &traj);
        bool matchChild(Trajectory &traj);
        int getCurrentCluster(){return (*m_currentCluster).getID();}; 
        bool hasChild();
    private:
        static double s_ccThresh;
        tree<Cluster> m_tree;
        int m_lastUpdated;
        tree<Cluster>::iterator m_currentCluster; // the one being updated
        tree<Cluster>::iterator m_lastCluster;    // the last cluster added
};
#endif
