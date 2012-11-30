#include "ClusterTree.h"
#include <limits>
#include <cmath>
#include <queue>
#include "ros/ros.h"
#include <limits>
/*bool ClusterTree::deleteCluster(int ind)
{
/    int parent = m_clusters[ind].getParent();
    m_clusters[parent].deleteChild(ind);
    return true;
}*/

void ClusterTree::concat_visit()
{
    ROS_INFO("Trying to concatenate ...");
    // examine all children at a particular level before moving down the tree
    for(tree<Cluster>::pre_order_iterator it=m_tree.begin();it!=m_tree.end();++it)
    {
        if(m_tree.number_of_children(it) == 1)
        {
            tree<Cluster>::sibling_iterator current = it;
            ROS_INFO("Concatenating %d with %d ",(*it).getID(),(*it.begin()).getID());
            (*it).concatenate(*(it.begin()));
            m_tree.reparent(current,it.begin());
            if(m_currentCluster == it.begin())
                m_currentCluster = current;
            m_tree.erase(it.begin());
        }
    }

}

void ClusterTree::merge_visit()
{

    for(tree<Cluster>::sibling_iterator sit=m_tree.begin(); sit!=m_tree.end(); ++sit)
    {
        for(tree<Cluster>::sibling_iterator sit2=sit; sit2!=m_tree.end(); ++sit2)
        {
            if(sit!=sit2)
            {
                ROS_INFO("Attempting to merge %d and %d", (*sit).getID(),(*sit2).getID());
                ROS_INFO("Cluster-Cluster distance is %f", (*sit).ccDistance(*sit2));
                if((*sit).ccDistance(*sit2) < 3.0)
                {
                    ROS_INFO("Merging");
                    if( (*sit).size() > (*sit2).size())
                    {
                        (*sit).weightedAv(*sit2);
                        ROS_INFO("Computed Weighted Av");
                        m_tree.reparent(sit,sit2);  //move children of it+1 to it
                        ROS_INFO("Reparented");
                        ROS_INFO("current cluster is %d, deleting %d",(*m_currentCluster).getID(),(*sit2).getID());
                        if(m_currentCluster == sit2)
                        {
                            m_currentCluster = sit;
                            m_lastCluster = sit;
                        }
                        m_tree.erase(sit2);
                        ROS_INFO("Erased");
                    }
                    else
                    {
                        (*sit2).weightedAv(*sit);
                        m_tree.reparent(sit2,sit);  //move children of it+1 to it
                        if(m_currentCluster == sit)
                        {
                            m_currentCluster = sit2;
                            m_lastCluster = sit2;
                        }
                        m_tree.erase(sit);
                        ROS_INFO("Erased");
                    }
                }
            }
        }
    }

    // examine all children at a particular level before moving down the tree
    for(tree<Cluster>::sibling_iterator sit=m_tree.begin(); sit!=m_tree.end(); ++sit)
    {
        tree<Cluster>::breadth_first_queued_iterator bfq = sit;
        while(bfq!=m_tree.end_breadth_first())
        {
            for(tree<Cluster>::sibling_iterator s2=m_tree.begin(bfq); s2!=m_tree.end(bfq); ++s2)
            {
                for(tree<Cluster>::sibling_iterator s3=s2; s3!=m_tree.end(bfq); ++s3)
                {
                    if(s2!=s3)
                    {
                        ROS_INFO("Attempting to merge %d and %d", (*s2).getID(),(*s3).getID());
                        ROS_INFO("Cluster-Cluster distance is %f", (*s2).ccDistance(*s3));
                        if((*s2).ccDistance(*s3) < s_ccThresh)
                        {
                            ROS_INFO("Merging");
                            if( (*s2).size() > (*s3).size())
                            {
                                (*s2).weightedAv(*s3);
                                ROS_INFO("Computed Weighted Av");
                                m_tree.reparent(s2,s3);  //move children of it+1 to it
                                ROS_INFO("Reparented");
                                ROS_INFO("current cluster is %d, deleting %d",(*m_currentCluster).getID(),(*s3).getID());
                                if(m_currentCluster == s3)
                                {
                                    m_currentCluster = s2;
                                    m_lastCluster = s2;
                                }
                                m_tree.erase(s3);
                                ROS_INFO("Erased");
                            }
                            else
                            {
                                (*s3).weightedAv(*s2);
                                m_tree.reparent(s3,s2);  //move children of it+1 to it
                                if(m_currentCluster == s2)
                                {
                                    m_currentCluster = s3;
                                    m_lastCluster = s3;
                                }
                                m_tree.erase(s3);
                                ROS_INFO("Erased");
                            }
                        }
                    }
                }
            }
            ++bfq;
        }
    }
}


/*void ClusterTree::merge_visit()
{
    // examine all children at a particular level before moving down the tree
    for(tree<Cluster>::breadth_first_queued_iterator it=m_tree.begin_breadth_first();it!=m_tree.end_breadth_first();++it)
    {
        std::cout << "At node " << (*it).getID() << "\n";
        if(m_tree.number_of_siblings(it) > 0 && m_tree.index(it) < m_tree.number_of_siblings(it))
        {
            tree<Cluster>::sibling_iterator nextSib = m_tree.next_sibling(it);
            tree<Cluster>::sibling_iterator current = it;
            ROS_INFO("Attempting to merge %d and %d", (*it).getID(),(*nextSib).getID());
            ROS_INFO("Cluster-Cluster distance is %f", (*it).ccDistance(*nextSib));
            if((*it).ccDistance(*nextSib) < s_ccThresh)
            {
                ROS_INFO("Merging");
                (*it).weightedAv(*nextSib);
                ROS_INFO("Computed Weighted Av");
                m_tree.reparent(current,nextSib);  //move children of it+1 to it
                ROS_INFO("Reparented");
                ROS_INFO("current cluster is %d, deleting %d",(*m_currentCluster).getID(),(*nextSib).getID());
                if(m_currentCluster == nextSib)
                    m_currentCluster = current;
        
                m_tree.erase(nextSib);
                ROS_INFO("Erased");
            }

        }
    }
}*/
    /*
}
    std::queue<int> root_queue;
    if(m_clusters.size() > 0)
        root_queue.push(0);
    else
        return;

    while(!root_queue.empty())
    {
        int current_node = root_queue.front();
        root_queue.pop();
        std::vector<int> tmpChild;
        m_clusters[current_node].getChildren(tmpChild);
        if(tmpChild.size() < 2)
            continue;
        else
        {
            root_queue.push(tmpChild[0]); // always add the first of the next level
            ROS_INFO("It's got %d child nodes", tmpChild.size());
        }
        for(unsigned int i=0; i < tmpChild.size(); i++)
        {
            ROS_INFO("C-C distance is %f",m_clusters[tmpChild[i]].ccDistance(m_clusters[tmpChild[i+1]]));
            if(m_clusters[tmpChild[i]].ccDistance(m_clusters[tmpChild[i+1]]) < ccThresh)
            {
                ROS_INFO("Merging clusters");
                m_clusters[tmpChild[i]].weightedAv(m_clusters[tmpChild[i+1]]); // does weighted average
                m_clusters[current_node].deleteChild(tmpChild[i+1]);
                std::vector<int> newChild;
                m_clusters[tmpChild[i+1]].getChildren(newChild);
                m_clusters[tmpChild[i]].assignChildren(newChild);
                --i;
            }
            else
            {
                root_queue.push(tmpChild[i]);
            }
        }
    }
}
*/
double ClusterTree::closestNode(Trajectory &traj)
{
    tree<Cluster>::iterator root=m_tree.begin();
    tree<Cluster>::iterator lastLeaf=m_tree.end();

    double dmin = std::numeric_limits<double>::max();
    for(tree<Cluster>::iterator it=root; it!=lastLeaf; ++it)
    {
        double tempVal = (*it).clusterDistance(traj);
        if(dmin > tempVal)
        {
            dmin = tempVal;
            m_currentCluster = it;
        }
    }
    ROS_INFO("Closest cluster at distance %f",dmin);
    return dmin;
}


double ClusterTree::rootNodeDistance(double poseX, double poseY)
{
    tree<Cluster>::iterator root=m_tree.begin();
    tree<Cluster>::iterator lastLeaf=m_tree.end();

    double dmin = std::numeric_limits<double>::max();
    for(tree<Cluster>::iterator it=root; it!=lastLeaf; ++it)
    {
        double tempVal = hypot(poseX-(*it).startX(),poseY-(*it).startY());
        if(dmin > tempVal)
        {
            dmin = tempVal;
            m_currentCluster = it;
        }
    }
    return dmin;
}

bool ClusterTree::updateCurrentCluster(double pX, double pY, bool &atEnd, bool &hasChild)
{
    atEnd = false;
    hasChild = false;
    int ind = (*m_currentCluster).findClosest(pX,pY);
    ROS_INFO("Updating %d of cluster length %d", ind, (*m_currentCluster).size()-1);
    if(m_lastUpdated == ind)
    {
        ROS_INFO("Aborting update, was last update");
        return false;
    }
    if(m_lastUpdated > ind)
    {
         ROS_INFO("Going backwards along trajectory, forcing atEnd, doing nothing in reality");
         //return true;
     /*    if(m_tree.number_of_children(m_currentCluster) > 0)
             hasChild = true;
        else
            hasChild = false; 
        atEnd = true;*/
        //return false;
    }

    (*m_currentCluster).update(pX,pY,ind);
    if(ind == ((*m_currentCluster).size()-1))
    {
        atEnd = true;
        ROS_INFO("Reached end of current cluster, at %d of cluster length %d", ind, (*m_currentCluster).size()-1);
    }
    else
        atEnd = false;

    if(m_tree.number_of_children(m_currentCluster) > 0)
        hasChild = true;
    else
        hasChild = false; 
    
    m_lastUpdated = ind;
    return true;
}

bool ClusterTree::hasChild()
{
    if(m_tree.number_of_children(m_currentCluster) > 0)
        return true;
    else
        return false;

}
bool ClusterTree::getMsgData(std::vector<double> &xs, std::vector<double> &ys, std::vector<double> &sigmas, std::vector<int> &IDs)
{
    xs.clear();
    ys.clear();
    sigmas.clear();
    IDs.clear();
    std::vector<double> tmp;
    for(tree<Cluster>::iterator it=m_tree.begin(); it!=m_tree.end(); ++it)
    {
        (*it).getXs(tmp);
        xs.insert(xs.end(),tmp.begin(),tmp.end());
        (*it).getYs(tmp);
        ys.insert(ys.end(),tmp.begin(),tmp.end());
        (*it).getSigmas(tmp);
        sigmas.insert(sigmas.end(),tmp.begin(),tmp.end());
        std::vector<int> ids(tmp.size(),(*it).getID());
        IDs.insert(IDs.end(),ids.begin(),ids.end());

    }
    return true;
}

int ClusterTree::getCurrent()
{
    if(m_tree.size()!=0 && m_currentCluster!= m_tree.end())
    {
        return (*m_currentCluster).getID();
    }
    else
        return -1;
}

bool ClusterTree::Split(Trajectory &traj)
{
    Cluster newCluster;
    bool hasChild = 0;
    if(m_tree.number_of_children(m_currentCluster) > 0)
        hasChild = 1;
    (*m_currentCluster).split(traj,newCluster,hasChild);
    ROS_INFO("New Cluster has size %d", newCluster.size());
    if(newCluster.size() < 2)
        return false;
    else if(!hasChild)
        m_tree.append_child(m_currentCluster,newCluster); // append the remainder of the current cluster as a new child
    return true; 
}

bool ClusterTree::matchChild(Trajectory &traj)
{
    double temp = std::numeric_limits<double>::max();
    tree<Cluster>::iterator newIt;
    //for(tree<Cluster>::sibling_iterator it = m_tree.begin(m_currentCluster); it!=m_tree.end(m_currentCluster); ++it)
    for(tree<Cluster>::pre_order_iterator it = m_tree.begin(m_currentCluster); it!=m_tree.end(m_currentCluster); ++it)
    {
        int ind_lim = traj.findClosest((*it).endX(),(*it).endY());
        ROS_INFO("Matching trajectory of length %d but only up to element %d",traj.size(),ind_lim);
        ROS_INFO("Distance to Child node %d is %f",(*it).getID(),(*it).clusterDistance(traj,ind_lim));
        if((*it).clusterDistance(traj,ind_lim) < temp)
        {
            temp = (*it).clusterDistance(traj,ind_lim);
            newIt = it;
        }
    }
    if(temp < 3.0)
    {
        ROS_INFO("Matched existing child");
        m_currentCluster = newIt;
        int ind_lim = traj.findClosest((*newIt).endX(),(*newIt).endY());
        for(int i=0; i < ind_lim; i++)
        {
            int ind = (*m_currentCluster).findClosest(traj.getX(i),traj.getY(i));
            ROS_INFO("Bulk Updating %d of %d",ind,((*m_currentCluster).size())-1);
            (*m_currentCluster).update(traj.getX(i),traj.getY(i),ind);
            if(ind == (*m_currentCluster).size()-1)
            {
                ROS_INFO("Already at END");
            }
        }
        return true;
    }
    else
    {
        ROS_INFO("Need to split, doesn't match existing child nodes");
        //Cluster newCluster(traj);
        //m_currentCluster = m_tree.append_child(m_currentCluster,newCluster);
        //m_lastCluster = m_currentCluster;
        return false;
    }
}

