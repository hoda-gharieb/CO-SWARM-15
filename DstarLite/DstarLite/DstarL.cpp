#include "DstarL.h"

using namespace std;
using namespace DsLite;

const int DstarLite::MaxSteps = 1000000;

//======== constructor
DstarLite::DstarLite(Map_Node* mp, Mnode* st, Mnode* gol)
{
	map = mp;
	start = (*map)(st->get_X(),st->get_Y());//->
	goal = (*map)(gol->get_X(), gol->get_Y());//->
	last = (*map)(st->get_X(), st->get_Y());//->
	Km = 0;

	openList.clear();
	openl_hash.clear();
	path.clear();

	rhs(goal, 0.0);
	insert_olist(PD(calc_h(start, goal), 0.0), goal);

	//calc_g(goal, 0.0);

	/*insert_olist( PD( calc_h(start, goal), 0.0), goal);
	cells_h[goal] = PD(0.0,0.0);*/

	//rhs(start, calc_h(start, goal));
	//calc_g(start, calc_h(start, goal));

	//cells_h[start] = PD(calc_h(start, goal), calc_h(start, goal));
}

DstarLite::~DstarLite(){}

list< Mnode* > DstarLite::get_path(){ return path; }

//======= return list of points for robot navigation
list< point > DstarLite::get_path_points()
{
	list< pair<Uint, Uint> > L;
	list< Mnode* >::iterator it;
	for (it = path.begin(); it != path.end(); it++)
		L.push_back(make_pair((*it)->get_X(), (*it)->get_Y()));
	return L;
}

bool DstarLite::replan()
{
	path.clear();

	// No solution exist
	if (!ComputeShortestPath())
		return false;

	Mnode* current = (*map)(start->get_X(), start->get_Y());
	path.push_back(current);

	// search for the path with least cost to the goal
	while (current != goal)
	{
		double hoda = calc_g(current);
		// No solution exist
		if (current == NULL || calc_g(current) == DBL_MAX  )
			return false;

		current = get_min_succ(current).first;
		path.push_back(current);
	}
}

Mnode* DstarLite::GS_start(Mnode* n)
{
	if (n != NULL)
		start = n;
	return start;
}

Mnode* DstarLite::GS_goal(Mnode* n)
{
	if (n != NULL)
		goal = n;
	return goal;
}

void DstarLite::update(Mnode* n, double cost)
{
	if (n == goal )
		return;

	Km += calc_h(last, start);
	last = start;

	generate_node(n);

	double oldCost = n->get_cost();
	n->set_cost(cost);

	/*if ( map->blocked(n->get_X(), n->get_Y()))
		return;*/

	Mnode** nbrs = n->get_nbrs();
	double tmpOldCost, tmpNewCost, tmp_rhs, tmp_g;

	for (Uint i = 0; i < Mnode::num_nbrs; i += (Mnode::num_nbrs == 8 ? 1 : 2))
	{
		if (nbrs[i] != NULL /*&& !map->blocked(nbrs[i]->get_X(), nbrs[i]->get_Y() )*/ )
		{
			n->set_cost(oldCost);
			tmpOldCost = calc_cost(n, nbrs[i]);
			n->set_cost(cost);
			tmpNewCost = calc_cost(n, nbrs[i]);

			tmp_rhs = rhs(n);
			tmp_g = calc_g(nbrs[i]);

			if (tmpOldCost != DBL_MAX && tmpNewCost != DBL_MAX )
			{
				if (tmpOldCost - eps > tmpNewCost  && n != goal)
					rhs(n, min(tmp_rhs, tmpNewCost + tmp_g));
			}			
			else if (tmp_rhs != DBL_MAX && (tmpOldCost + tmp_g) != DBL_MAX )
			{
				if (fabsl(tmp_rhs - (tmpOldCost + tmp_g)) < eps)
					rhs(n, get_min_succ(n).second);
			}

		}
	}

	update_node(n);

	for (Uint i = 0; i < Mnode::num_nbrs; i += (Mnode::num_nbrs == 8 ? 1 : 2))
	{
		if (nbrs[i] != NULL /*&& !map->blocked(nbrs[i]->get_X(), nbrs[i]->get_Y())*/ )
		{
			n->set_cost(oldCost);
			tmpOldCost = calc_cost(n, nbrs[i]);
			n->set_cost(cost);
			tmpNewCost = calc_cost(n, nbrs[i]);

			tmp_rhs = rhs(nbrs[i]);
			tmp_g = calc_g(n);

			if (tmpOldCost != DBL_MAX && tmpNewCost != DBL_MAX)
			{
				if (tmpOldCost - eps > tmpNewCost  && nbrs[i] != goal)
					rhs(nbrs[i], min(tmp_rhs, tmpNewCost + tmp_g));
			}
			else if (tmp_rhs != DBL_MAX && (tmpOldCost + tmp_g) != DBL_MAX)
			{
				if (fabsl(tmp_rhs - (tmpOldCost + tmp_g)) < eps)
					rhs(nbrs[i], get_min_succ(nbrs[i]).second);
			}
			update_node(nbrs[i]);
		}
	}

}

bool DstarLite::ComputeShortestPath()
{ 
	if (openList.empty())
		return false;
	
	key_compare key_comp;
	Mnode* n;
	PD old_k, new_k;
	Mnode** nbrs;
	double tmp_g, tmp_rhs, old_g;
	int steps = 0;

	bool a = key_comp(openList.begin()->first, calc_key(start));
	a = !openList.empty();
	double b = rhs(start);
	b = calc_g(start);

	while ( (!openList.empty() && key_comp(openList.begin()->first, calc_key(start))) || !( rhs(start) - calc_g(start) < eps) )
	{
		if (++steps > MaxSteps)
			return false;

		n = openList.begin()->second;
		old_k = openList.begin()->first;
		new_k = calc_key(n);

		tmp_rhs = rhs(n);
		tmp_g = calc_g(n);

		if (key_comp(old_k, new_k))
			update_olist(new_k, n);
		else if (tmp_g - eps > tmp_rhs)
		{
			calc_g(n, tmp_rhs);
			tmp_g = tmp_rhs;

			remove_olist(n);
			nbrs = n->get_nbrs();
			for (Uint i = 0; i < Mnode::num_nbrs; i += (Mnode::num_nbrs == 8 ? 1 : 2) )
			{
				if (nbrs[i] != NULL/* && !map->blocked(nbrs[i]->get_X(), nbrs[i]->get_Y() ) */)
				{
					if (nbrs[i] != goal)
						rhs(nbrs[i], min(rhs(nbrs[i]), calc_cost(nbrs[i], n) + tmp_g));
					update_node(nbrs[i]);
				}
			}
		}
		else
		{
			old_g = tmp_g;
			calc_g(n, DBL_MAX);

			if (n != goal)
				rhs(n, get_min_succ(n).second);

			update_node(n);

			nbrs = n->get_nbrs();

			for (Uint i = 0; i < Mnode::num_nbrs; i += (Mnode::num_nbrs == 8 ? 1 : 2))
			{
				if (nbrs[i] != NULL /*&& !map->blocked(nbrs[i]->get_X(), nbrs[i]->get_Y()) */)
				{
					if ( fabsl( rhs(nbrs[i]) - (calc_cost(nbrs[i],n) + old_g ) ) < eps && nbrs[i] != goal)
						rhs(nbrs[i], get_min_succ(nbrs[i]).second);
					update_node(nbrs[i]);
				}
			}
		}
	}

	if (rhs(start) - calc_g(start) < eps)
		calc_g(start, rhs(start));

	return true;  
}

pair<Mnode*, double> DstarLite::get_min_succ(Mnode* n)
{
	Mnode** nbrs = n->get_nbrs();
	double tmpG, tmpCost;

	Mnode* minSucc = NULL;
	double minCost = DBL_MAX;

	for (Uint i = 0; i < Mnode::num_nbrs; i += (Mnode::num_nbrs == 8 ? 1 : 2))
	{
		if (nbrs[i] != NULL /*&& !map->blocked(nbrs[i]->get_X(), nbrs[i]->get_Y())*/)
		{
			int x = nbrs[i]->get_X(), y = nbrs[i]->get_Y();
			tmpCost = calc_cost(n, nbrs[i]);
			tmpG = calc_g(nbrs[i]);

			if (tmpCost == DBL_MAX || tmpG == DBL_MAX)
				continue;

			tmpCost += tmpG;
			if (tmpCost < minCost)
			{
				minSucc = nbrs[i];
				minCost = tmpCost;
			}
		}
	}
	return pair<Mnode*, double>(minSucc, minCost);
}

double DstarLite::calc_cost(Mnode* a, Mnode* b)
{
	if (a->get_cost() == DBL_MAX || b->get_cost() == DBL_MAX)
		return DBL_MAX;
	Uint x = labs(a->get_X() - b->get_X());
	Uint y = labs(a->get_Y() - b->get_Y());
	double scale = (x + y > 1 ? Sqrt2 : 1.0);
	/*if (map->blocked(a->get_X(), a->get_Y()) || map->blocked(b->get_X(), b->get_Y()))
		scale = 10000000;*/
	return scale * (a->get_cost() + b->get_cost()) / 2.0;
}

double DstarLite::calc_g(Mnode* n, double cost)
{
	generate_node(n);
	PD* g_rhs = &cells_h[n];
	if (cost != DBL_MIN) //
		g_rhs->first = cost;
	return g_rhs->first;
}

double DstarLite::calc_h(Mnode* c, Mnode* e)
{
	Uint dim1 = labs(c->get_X() - e->get_X());
	Uint dim2 = labs(c->get_Y() - e->get_Y());
	if (dim1 > dim2)
		swap(dim1, dim2);
	return ((Sqrt2 - 1.0)*dim1 + dim2);

	//return sqrtl(dim1*dim1 + dim2*dim2);
}

double DstarLite::rhs(Mnode* n, double cost)
{
	if (n == goal)
		return 0;
	generate_node(n);
	PD* g_rhs = &cells_h[n];
	if (cost != DBL_MIN) //
		g_rhs->second = cost;
	return g_rhs->second;
}

PD DstarLite::calc_key(Mnode* c)
{
	double g = calc_g(c);
	double Rhs = rhs(c);
	double g_rhs = min(g, Rhs);
	return PD((g_rhs + calc_h(start,c) + Km), g_rhs);
}

void DstarLite::generate_node(Mnode* n)
{
	if (cells_h.find(n) != cells_h.end())
		return;

	//cells_h[n] = PD(calc_h(n, goal), calc_h(n, goal));
	cells_h[n] = PD(DBL_MAX, DBL_MAX);
}

void DstarLite::update_node(Mnode* n)
{
	bool dif = (calc_g(n) != rhs(n));
	bool exist = (openl_hash.find(n) != openl_hash.end());
	/*if (map->blocked(n->get_X(), n->get_Y()))
	{
		if (exist)
			remove_olist(n);
		return;
	}*/
	if (dif && exist)
		update_olist( calc_key(n), n);
	else if ( dif && !exist )
		insert_olist(calc_key(n), n);
	else if (!dif && exist )
		remove_olist( n );
}

void DstarLite::insert_olist(PD ki, Mnode* n)
{
	OPENLIST::iterator it = openList.insert(openPair(ki, n));
	openl_hash[n] = it;
}

void DstarLite::remove_olist(Mnode* n)
{
	openList.erase(openl_hash[n]);
	openl_hash.erase(openl_hash.find(n));
}

void DstarLite::update_olist(PD ki, Mnode* n)
{
	OPENLIST::iterator temp = openl_hash[n];
	//OPENLIST::iterator pos = temp;
	//pos = (temp == openList.end() ? temp : pos++);
	openList.erase(temp);
	openl_hash[n] = openList.insert(openList.end(), openPair(ki, n));
}

bool DstarLite::key_compare::operator()(const PD& a, const PD& b) const
{
	if ( a.first + eps < b.first)
		return true;
	else if ( a.first - eps > b.first)
		return false;
	else if ( a.second + eps < b.second)
		return true;
	else if ( a.second - eps > b.second)
		return false;

	return false;
}

//bool DstarLite::key_compare::operator()(const PD& a, const PD& b) const
//{
//	if (a.first != DBL_MAX && b.first != DBL_MAX && a.first + eps < b.first)
//		return true;
//	else if (a.first != DBL_MAX && b.first != DBL_MAX && a.first - eps > b.first)
//		return false;
//	else if (a.second != DBL_MAX && b.second != DBL_MAX && a.second + eps < b.second)
//		return true;
//	else if (a.second != DBL_MAX && b.second != DBL_MAX && a.second - eps > b.second)
//		return false;
//
//	return false;
//}