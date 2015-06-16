
#ifndef DSLITE_MAP_NODE_H
#define DSLITE_MAP_NODE_H

#include <functional>
#include <stdlib.h>


#include "Hashing.h"

using namespace std;

namespace DsLite
{
	class Map_Node
	{
	public:

		//check whether a cell is in the grid's range
		bool exist(int x, int y);
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
		Node* operator()(const Uint x, const Uint y);

	protected:
		//Map for the robot
		Node*** grid;

		//Map dimensions
		Uint cols, rows;
	};
	
};

#endif DSLITE_MAP_NODE_H

