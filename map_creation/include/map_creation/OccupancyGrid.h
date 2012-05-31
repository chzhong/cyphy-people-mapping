/* COccupancyGrid.h
 * Computes an occupancy grid given different data sources
 */

#ifndef COCCUPANCYGRID_H
#define COCCUPANCYGRID_H

#include <map>
#include <set>
#include <vector>

/*class CPoseComparator
{
public:
	bool operator()(const SE2Pose &P1, const SE2Pose &P2) const
	{
		if(P1.X() < P2.X())
			return true;
		else if(P1.X() > P2.X())
			return false;
		else if(P1.Y() < P2.Y())
			return true;
		else if(P1.Y() > P2.Y())
			return false;
		else if(P1.A() < P2.A())
			return true;
		else
			return false;

	}
};
*/

typedef std::pair<int,int> XYPAIR;
class COccupancyGrid
{
	public:
		typedef std::map<XYPAIR, float> OccupancyGridMap;
		COccupancyGrid();
		COccupancyGrid(double Resolution);
		XYPAIR Quantise(double x, double y);
		OccupancyGridMap::iterator begin() {return theMap.begin();};
		OccupancyGridMap::iterator end() {return theMap.end();};
		bool SetStepSize(double ss){stepSize = ss; return true;};
        bool AddPeopleData(float x,float y);
		bool Clear();
        bool GetMsgData(std::vector<signed char> &data, double &xOrig, double &yOrig);
        int numElem(){return (m_xMax-m_xMin)*(m_yMax-m_yMin);};
        int width(){return (m_xMax-m_xMin);};
        int height(){return (m_yMax-m_yMin);};
	protected:

	private:
		OccupancyGridMap theMap;
		double m_lo; // prior
		double m_lfree; // returned by 'inverse sensor model' if cell is free
		double m_locc;  // returned by 'inverse sensor model' if cell is occupied
		double stepSize; // same as costmap ... this is the grid resolution
        int m_xMin,m_yMin,m_xMax,m_yMax;

};
#endif
