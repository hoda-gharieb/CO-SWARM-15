#include "A.h"

using namespace std;
using namespace Astar;

A::A(Map_Node* mp, Mnode* st, Mnode* gol)
{
	map = mp;
	start = st;
	goal = gol;

}

A::~A(){}


bool A::path_planner()
{
	// Q : list of open nodes
	// temp : temporary queue to save certain nodes and return them back to Q
	priority_queue< Mnode > Q, temp;
	Mnode *n, *child;
	int x, y, newX, newY;

	//create start node and push it in the queue and mark its position as opened one
	n = new Mnode(start->get_X(), start->get_Y(), 0, 0);
	n->Update_prt(goal->get_X(), goal->get_Y());
	Q.push((*n));
	map->set_opened(start->get_X(),start->get_Y(), n->get_priority());

	while (!Q.empty())
	{
		// get current node with highest priority and remove it from open set
		n = new Mnode(Q.top().get_X(), Q.top().get_Y(), Q.top().get_cost(), Q.top().get_priority(), Q.top().get_prob());
		Q.pop();
		x = n->get_X();
		y = n->get_Y();
		map->set_opened(x,y,0);
		map->set_closed(x,y,true);

		if (x == goal->get_X() && y == goal->get_Y()) // if the goal node is reached
		{
			char c;
			int d;
			//trace back the path from the start to goal node
			while (!(x == start->get_X() && y == start->get_Y()))
			{
				d = map->get_dir(x,y);
				c = '0' + d;
				path += c;
				x -= xDir[d];
				y -= yDir[d];
			}

			delete n;
			reverse(path.begin(), path.end());
			return true;
		}

		// check for new children for current node
		for (int i = 0; i < n->num_nbrs; i += (n->num_nbrs == 8 ? 1 : 2))
		{
			newX = x + xDir[i]; newY = y + yDir[i];
			// Map position = 1 if it has an obstacle

			if ( map->exist(newX, newY) && !map->blocked(newX, newY) && !map->get_closed(newX,newY))
			{
				child = new Mnode(newX, newY, n->get_cost(), n->get_priority(), n->get_prob());
				child->next_cost(i);
				child->Update_prt(goal->get_X(), goal->get_Y());
				double xxx = map->get_opened(newX, newY);
				if (map->get_opened(newX,newY) == 0.0 ) // if node not visited yet push it in open nodes
				{
					map->set_opened(newX,newY, child->get_priority());
					xxx = map->get_opened(newX, newY);
					Q.push(*child);
					map->set_dir(newX,newY, i);
				}
				else if (map->get_opened(newX, newY) > child->get_priority()) // if visited and a new path found with less cost update it
				{
					map->set_opened(newX, newY, child->get_priority());
					xxx = map->get_opened(newX, newY);
					map->set_dir(newX,newY, i);
					while (!(Q.top().get_X() == newX && Q.top().get_Y() == newY)) // save all nodes before child node
					{
						temp.push(Q.top());
						Q.pop();
					}
					Q.pop();
					while (!temp.empty()) // return the nodes back to open queue
					{
						Q.push(temp.top());
						temp.pop();
					}
					Q.push(*child);
				}
				delete child; // for memory leakage
			}
		}
		delete n; // for memory leakage
	}

	return false; // no path was found
}

list< point > A::get_path()
{
	map->init_lists();
	path = "";
	list<point> L;

	if (!path_planner())
		return L;
	int j; 
	char c;
	int x = start->get_X();
	int y = start->get_Y();
	string path = get_path_s();
	rep(i, 0, (int)path.size())
	{
		c = path[i];
		j = atoi(&c);
		x = x + xDir[j];
		y = y + yDir[j];
		L.push_back(mp(x, y));		
	}
	return L;
}

string A::get_path_s(){ return path; }

void A::set_path(string p){ path = p; }