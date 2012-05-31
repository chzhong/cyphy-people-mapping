#include "map_creation/OccupancyGrid.h"
#include <stdio.h>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>

// -------------------------------------------------------------------------
COccupancyGrid::COccupancyGrid()
{
	stepSize = 0.05;
	m_lo = 0;
	m_lfree = -0.1386;
	m_locc = 0.1386;
    m_xMin = std::numeric_limits<int>::max();
    m_yMin = std::numeric_limits<int>::max();
    m_xMax = -std::numeric_limits<int>::max();
    m_yMax = -std::numeric_limits<int>::max();
};

// -------------------------------------------------------------------------
COccupancyGrid::COccupancyGrid(double Resolution)
{
	stepSize = Resolution;
	m_lo = 0;
	m_lfree = -0.1386;
    m_locc = 0.1386;
	m_xMin = std::numeric_limits<int>::max();
    m_yMin = std::numeric_limits<int>::max();
    m_xMax = -std::numeric_limits<int>::max();
    m_yMax = -std::numeric_limits<int>::max();
}

// -------------------------------------------------------------------------
bool COccupancyGrid::Clear()
{  
   	m_xMin = std::numeric_limits<int>::max();
    m_yMin = std::numeric_limits<int>::max();
    m_xMax = -std::numeric_limits<int>::max();
    m_yMax = -std::numeric_limits<int>::max();
	theMap.clear();
	return true;
}

bool COccupancyGrid::GetMsgData(std::vector<signed char> &data, double &xOrig, double &yOrig)
{
    int width = m_xMax - m_xMin;
    int height = m_yMax - m_yMin;

    if(width < 1 && height < 1)
        return false;

    OccupancyGridMap::const_iterator end = theMap.end();

    for(OccupancyGridMap::const_iterator it = theMap.begin(); it!=end; ++it)
    {
        int col = it->first.first - m_xMin; // x_loc
        int row = it->first.second - m_yMin;

        int dind = row*width+col;

        double prob = 100*(1-1/(1+exp(it->second)));
        prob = round(prob);
        signed char intProb = (signed char) prob;
        if(dind >= 0 && dind < width*height) 
            data[dind] = intProb;
        else
            std::cout << "Got a rotten index " << dind << " from col " << col << " and row " << row << " size is (" << width << ", " << height << ") " << std::endl;
    }

    xOrig = m_xMin*stepSize;
    yOrig = m_yMin*stepSize;
}
bool COccupancyGrid::AddPeopleData(float x, float y)
{
    // first quantize it
    XYPAIR xy = Quantise(x,y);  // take from map coords into quantised people costmap index

    // now add it to the map, yep, it's harsh
    theMap[xy]=theMap[xy]+m_lfree-m_lo;

    if(xy.first < m_xMin)
        m_xMin = xy.first;
    if(xy.second < m_yMin)
        m_yMin = xy.second;
    if(xy.first > m_xMax)
        m_xMax = xy.first;
    if(xy.second > m_yMax)
        m_yMax = xy.second;

}

/*	for (; n > 0; --n)
	{
		XYPAIR xy(x,y);
		if(n==1)
		{
			end_point=false;
			// check the PoseBucket and only increment if we haven't seen one from this pose before
			if(OccupiedBucket[xy].count(RobotPose)==0 && theMap[xy] < m_lo+20*m_locc)
			{
				// increment
				theMap[xy] = theMap[xy]+m_locc-m_lo;
				OccupiedBucket[xy].insert(RobotPose);
			}
		}
		else
		{
			if(FreeBucket[xy].count(RobotPose)==0 && theMap[xy] > m_lo+20*m_lfree)
			{
				// decrement

				double oldval;
				oldval = 1-1/(1+exp(theMap[xy]));
				if(oldval > 0.5)
					int stophere=1;
				theMap[xy] = theMap[xy]+m_lfree-m_lo;
				FreeBucket[xy].insert(RobotPose);
				double newval = 1-1/(1+exp(theMap[xy]));
				if(newval < 0.5 && oldval>0.5)
					int stophere=1;
			}

		}*/
		//visit(x, y);
//------------------------------------------------------------------
XYPAIR COccupancyGrid::Quantise(double x, double y)
{
	XYPAIR retval;
	retval.first = (int)floor(x/stepSize+0.5);
	retval.second = (int)floor(y/stepSize+0.5);
	return retval;
}

/*
* Return a 3xN matrix of costmap values for logging
*/
// ------------------------------------------------------------------------
/*vnl_matrix<double> COccupancyGrid::GetData()
{

	vnl_matrix<double> retmatrix;

	retmatrix.set_size(3,theMap.size());
		
	OccupancyGridMap::iterator q;
    
	int i=0;

	for(q = theMap.begin();q!=theMap.end();q++)
	{
		retmatrix[0][i]=q->first.first;
		retmatrix[1][i]=q->first.second;
		double val;
		val = 1-1/(1+exp(q->second));
		retmatrix[2][i]=1-1/(1+exp(q->second));  //This is a value between 0 and 1
		i++;
	}

	return retmatrix;
}

// -------------------------------------------------------------------------
std::vector< std::vector<double> > COccupancyGrid::GetDataWithPose( SE2Pose p )
{
	std::vector< std::vector<double> > retmatrix;

	printf("______Pose is %f %f \n", p.X(), p.Y() );

	OccupancyGridMap::iterator q;
    
	int i=0;

	for(q = theMap.begin();q!=theMap.end();q++)
	{
		double xVal = q->first.first;
		double yVal = q->first.second;

		//Check to see if it is in range

		double xDelta = xVal - p.X() ;
		double yDelta = yVal - p.Y() ;

		double dist = sqrt( pow(xDelta,2) + pow(yDelta,2) );
		
		if ( dist > 100.0 ) 
		{
			//continue;
		}

		std::vector<double> tmp;
	
		tmp.push_back( xVal );
		tmp.push_back( yVal );
		
		double val;
		val = 1-1/(1+exp(q->second));
		double newval=1-1/(1+exp(q->second));  //This is a value between 0 and 1
		tmp.push_back( newval );

		retmatrix.push_back( tmp );

	}


	return retmatrix;
	
	
}*/
