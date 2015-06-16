
#ifndef AS_MAP_NODE_H
#define AS_MAP_NODE_H

#include <functional>
#include <stdlib.h>

#include "node.h"

using namespace std;

namespace Astar
{
	class Map_Node
	{
	
	public:
		

		//check whether a cell is in the grid's range
		bool exist(Uint x, Uint y);

		bool blocked(Uint x, Uint y);

		//constructors
		Map_Node(Uint c, Uint r);
		Map_Node();

		//destructor
		~Map_Node();

		//getters
		Uint get_cols();
		Uint get_rows();

		// ============ for operators
		Node* operator()(const Uint x,const Uint y);

		void init_lists();

		double get_opened(Uint x, Uint y);
		void set_opened(Uint x, Uint y, double b);

		bool get_closed(Uint x, Uint y);
		void set_closed(Uint x, Uint y, bool b);

		int get_dir(Uint x, Uint y);
		void set_dir(Uint x, Uint y, int b);

	protected:
		//Map for the robot
		Node*** grid;

		//Map dimensions
		Uint cols, rows;

		vector< vector<double> > opened;
		vector< vector<int> >  dir_map;
		vector< vector<bool> >  closed;
	};
	
};

#endif AS_MAP_NODE_H

