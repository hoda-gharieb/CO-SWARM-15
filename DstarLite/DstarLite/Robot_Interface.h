#ifndef DS_ROBOT_INTERFACE_H
#define DS_ROBOT_INTERFACE_H

#include "Global_map.h"
#include "Frontier.h"
#include "DstarL.h"


using namespace std;

namespace DsLite
{

	class Robot_Interface
	{
	protected:
		Map_Node* map;
		Global_map* Gmap;
		Frontier* frontier;
		DstarLite* dsl;
		bool mode;
		Mnode* Robot_Postion;
		Mnode* goal;
		list<point>::iterator it;
		list<point> route;
	public:
		Robot_Interface(int n = 100, int m = 100, int x = 0, int y = 0);
		~Robot_Interface();
		point handle_interface(double data_recieved[3][3]);
		void set_Robot_Pos(Node*);
		void explore(double data_recieved[][3]);
		void new_path();
		void initPosition(Uint x, Uint y);
		void display_map_prob();
		list<point> navigate();
	};
};
#endif

