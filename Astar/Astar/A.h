#ifndef AS_ALGO_H
#define AS_ALGO_H

#include "map_node.h"

#define Mnode Node

using namespace std;
using namespace Astar;

namespace Astar
{
	class A
	{
	public:
		A (Map_Node* mp, Mnode* st, Mnode* gol);

		~A();

		list< point > get_path();

		string get_path_s();

		void set_path(string p);

	protected:
		bool path_planner();

		Mnode *start, *goal;

		Map_Node* map;

		string path;

	};
};

#endif