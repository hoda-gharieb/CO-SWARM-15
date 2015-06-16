#include "Robot_Interface.h"


Robot_Interface::Robot_Interface(int n, int m, int x, int y)
{
	map = new Map_Node(n, m);
	Gmap = Global_map::get_Instance(n, m);
	mode = true;
	Robot_Postion = new Node(x, y, 0, 0.25);
	frontier = Frontier::get_Instance();
}

Robot_Interface::~Robot_Interface()
{
}

void Robot_Interface::explore(double data_recieved[][3])
{
	int x = Robot_Postion->get_X(), y = Robot_Postion->get_Y(), a = 1, b = 1;
	for (int i = 0; i < Robot_Postion->num_nbrs; i += (Robot_Postion->num_nbrs == 8 ? 1 : 2))
	{
		int X = x + xDir[i], Y = y + yDir[i], A = a + xDir[i], B = b + yDir[i];
		if (map->exist(X, Y))
		(*map)(X, Y)->set_prob(((*map)(X, Y)->get_prob() + data_recieved[A][B]) / 2.0);
	}
	Gmap->fuse_local_map(map);
	map = Gmap->get_global_map();
}

void Robot_Interface::initPosition(Uint x, Uint y)
{
	Robot_Postion->set_Y(y);
	Robot_Postion->set_X(x);
}

list<point> Robot_Interface::navigate()
{
	list<point> route;
	frontier->set_map(map);
	Mnode* goal = frontier->optimal_Frontier(Robot_Postion->get_X(), Robot_Postion->get_Y());

	if (goal->get_X() == -1 && goal->get_Y() == -1)// exploration ended
		return route;
	aStar = new A(map, Robot_Postion, goal);
	list<point>::iterator it;
	route = aStar->get_path();
	return route;
}

void Robot_Interface::new_path()
{
	route.clear();
	frontier->set_map(map);
	goal = frontier->optimal_Frontier(Robot_Postion->get_X(), Robot_Postion->get_Y());

	if (goal->get_X() == -1 && goal->get_Y() == -1)// exploration ended
		return;
	aStar = new A(map, Robot_Postion, goal);
	route = aStar->get_path();
	if (!route.empty())
	it = route.begin();
}

point Robot_Interface::handle_interface(double data_recieved[3][3])
{
	//--- update recieved data in local map
	int x = Robot_Postion->get_X(), y = Robot_Postion->get_Y(), a = 1, b = 1, idx = -1;
	for (int i = 0; i < Robot_Postion->num_nbrs; i += (Robot_Postion->num_nbrs == 8 ? 1 : 2))
	{
		int X = x + xDir[i], Y = y + yDir[i], A = a + xDir[i], B = b + yDir[i];
		if (map->exist(X, Y))
		{
			if (!data_recieved[A][B] && fabsl((*map)(X, Y)->get_prob() - 0.5) < eps && idx == -1)
				idx = i;
			(*map)(X, Y)->set_prob(((*map)(X, Y)->get_prob() + data_recieved[A][B]) / 2.0);
		}
	}

	if (mode)//-- exploration mode
	{
		if (idx != -1)
		{
			int X = x + xDir[idx], Y = y + yDir[idx], A = a + xDir[idx], B = b + yDir[idx];
			Robot_Postion->set_X(X);
			Robot_Postion->set_Y(Y);
			//return make_pair(A, B);
			return make_pair(X, Y);
		}

		Gmap->fuse_local_map(map);
		map = Gmap->get_global_map();
		mode = false;
		this->new_path();
		if (route.empty())
			return make_pair(-1, -1);
	}

	if (!mode)//-- navigation mode
	{
		int X = 1 + (it->first - x), Y = 1 + (it->second - y);
		double rec_data = data_recieved[X][Y]; // data recieved from robot;
		if (rec_data - eps > 0.5)
		{
			aStar = new A(map, Robot_Postion, goal);
			route = aStar->get_path();
			it = route.begin();
			if (route.empty())
				return point(-1, -1);
		}
		Robot_Postion = (*map)(it->first, it->second);
		list<point>::iterator tmp_it = it;
		if (it != route.end())
		it++;
		if (it == route.end())
			mode = true;
		return make_pair(tmp_it->first , tmp_it->second );
	}
}

void Robot_Interface::display_map_prob()
{
	cout << endl << "Grid :" << endl;
	rep(i, 0, map->get_rows())
	{
		rep(j, 0, map->get_cols())
			cout << (*map)(i, j)->get_prob() << " ";
		cout << endl;
	}

	cout << endl << endl;

	rep(i, 0, map->get_rows())
	{
		rep(j, 0, map->get_cols())
			cout << (((*map)(i, j)->get_prob() < 0.5 - eps) ? '.' : 'O');
		cout << endl;
	}
}