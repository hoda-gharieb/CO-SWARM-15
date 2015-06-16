#ifndef DSLITE_GLOBAL_MAP_H
#define DSLITE_GLOBAL_MAP_H

#include "map_node.h"

#define Mnode Map_Node::Node

using namespace std;
using namespace DsLite;

namespace DsLite
{
	class Global_map
	{
	public:
		static Global_map* get_Instance(int c, int r);
		~Global_map();
		void fuse_local_map(Map_Node* local);
		Map_Node* get_global_map();

	protected:
		Global_map(int c, int r);
		Global_map();
		Global_map( Global_map const&);
		void operator=(Global_map const&);
		static Global_map* instance;
		Map_Node* map;

	};
};

#endif