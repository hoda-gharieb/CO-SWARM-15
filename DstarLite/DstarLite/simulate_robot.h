#ifndef DS_SIMULATE_ROBOT_H
#define DS_SIMULATE_ROBOT_H

#include "def.h"
using namespace std;

namespace DsLite
{
	class simulate_robot
	{
	protected:
		int row, col;
		int** grid;
		point rob_pos;
	public:
		simulate_robot(int r = 100, int c = 100, point p = point(0, 0));
		~simulate_robot();
		void init();
		vector<vector< double > > handle_simulator(point p);
	};
};

#endif