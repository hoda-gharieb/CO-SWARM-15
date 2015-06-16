#ifndef AS_FRONTIER_H
#define AS_FRONTIER_H

#include "Map_Node.h"
#include "def.h"

#define Mnode Node

namespace Astar
{

	class Frontier
	{
	protected:
		Map_Node* map;
		Mnode** Frontiers;
		Uint Frontiers_Num;
		set<point> detected_frontiers;
		static Frontier* instance;
		Frontier();
		bool isFrontier(Mnode* n);
		void Calculate_Frontiers(Uint x, Uint y);
		Frontier(Frontier const&);
		void operator=(Frontier const&);
	public:
		static Frontier* get_Instance();
		~Frontier();
		Mnode** get_Frontiers();
		Mnode* optimal_Frontier(Uint x, Uint y);
		void set_map(Map_Node* m);

	};

};

#endif