#ifndef AS_SIMULATE_ROBOT_H
#define AS_SIMULATE_ROBOT_H

#include "def.h"
using namespace std;

namespace Astar
{
	class simulate_robot
	{
	protected:
		int row, col;
		int** grid;
		point rob_pos;
	public:
		simulate_robot(int r = 100, int c = 100, point p = point(0,0) );
		~simulate_robot();
		void init();
		vector<vector< double > > handle_simulator(point p);
	};
};
#endif AS_SIMULATE_ROBOT_H