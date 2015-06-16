#ifndef AS_ROBOT_INTERFACE_H
#define AS_ROBOT_INTERFACE_H

#include "Global_map.h"
#include "Frontier.h"
#include "A.h"


using namespace std;
namespace Astar
{

	class Robot_Interface
	{
	protected:
		Map_Node* map;
		Mnode* goal;
		Global_map* Gmap;
		Frontier* frontier;
		A* aStar;
		bool mode;
		Mnode* Robot_Postion;
		list<point>::iterator it;
		list<point> route;
	public:
		Robot_Interface(int n = 100, int m = 100, int x = 0, int y = 0);
		~Robot_Interface();
		point handle_interface(double data_recieved[][3]);
		void set_Robot_Pos(Node*);
		void explore(double data_recieved[][3]);
		void initPosition(Uint x, Uint y);
		void new_path();
		void display_map_prob();
		list<point> navigate();
	};
};
#endif

