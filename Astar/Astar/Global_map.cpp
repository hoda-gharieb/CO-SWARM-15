#include "Global_map.h"

using namespace std;
using namespace Astar;

Global_map* Global_map::get_Instance(int c, int r)
{
	if (instance == NULL)
		instance = new Global_map(c, r);
	return instance;
}

Global_map* Global_map::instance = NULL;

Global_map::Global_map(int c, int r)
{
	map = new Map_Node(c, r);
}

Global_map::Global_map(){}

Global_map::~Global_map()
{
	map->~Map_Node();
}

void Global_map::fuse_local_map(Map_Node* local)
{
	for (int i = 0; i < map->get_rows(); i++)
	{
		for (int j = 0; j < map->get_cols(); j++)
		{
			double val = (*local)(i, j)->get_prob();
			val += (*map)(i, j)->get_prob();
			val /= 2.0;
			(*map)(i, j)->set_prob(val);
		}
	}
}

Map_Node* Global_map::get_global_map()
{
	return map;
}

