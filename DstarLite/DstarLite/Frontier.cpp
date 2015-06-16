#include "Frontier.h"

using namespace DsLite;

Frontier* Frontier::instance = NULL;

Frontier* Frontier::get_Instance()
{
	if (instance == NULL)
		instance = new Frontier();
	return instance;
}

Frontier::Frontier(){ detected_frontiers.clear(); }

Frontier::~Frontier(){}

Mnode** Frontier::get_Frontiers(){ return Frontiers; }

void Frontier::set_map(Map_Node* m){ this->map = m; }

bool Frontier::isFrontier(Mnode* nd)
{
	if (detected_frontiers.find(point(nd->get_X(), nd->get_Y())) != detected_frontiers.end())
		return false;
	Mnode** Neighbours = nd->get_nbrs();
	rep(n, 0, nd->num_nbrs)
	{
		if (Neighbours[n] == NULL)
			continue;
		else
		{
			if (fabsl(Neighbours[n]->get_prob() - 0.5) < eps)
			{
				Frontiers[Frontiers_Num] = new Node();
				Frontiers[Frontiers_Num++] = (*map)(nd->get_X(), nd->get_Y());
				return true;
			}
		}
	}
	return false;
}

void Frontier::Calculate_Frontiers(Uint x, Uint y)
{
	Frontiers_Num = 0;
	Frontiers = new Mnode*[(map->get_rows() * map->get_cols()) / 2];
	queue<point> Q;

	bool** vis;
	vis = new bool*[map->get_rows()];
	rep(i, 0, map->get_rows())
		vis[i] = new bool[map->get_cols()];
	rep(i, 0, map->get_rows())
		rep(j, 0, map->get_cols())
		vis[i][j] = false;

	Q.push(point(x, y));
	vis[x][y] = true;
	while (!Q.empty())
	{
		point p = Q.front();
		Q.pop();
		Mnode* n = (*map)(p.first, p.second);
		Mnode** nbrs = n->get_nbrs();
		for (int i = 0; i < n->num_nbrs; i += (n->num_nbrs == 8 ? 1 : 2))
		{
			if (nbrs[i] != NULL && !vis[nbrs[i]->get_X()][nbrs[i]->get_Y()] && nbrs[i]->get_prob() < 0.5 - eps)
			{
				if (!isFrontier(nbrs[i]))
				{
					vis[nbrs[i]->get_X()][nbrs[i]->get_Y()] = true;
					Q.push(point(nbrs[i]->get_X(), nbrs[i]->get_Y()));
				}
			}
		}
	}
}

Mnode*  Frontier::optimal_Frontier(Uint x, Uint y)
{
	Calculate_Frontiers(x, y);
	vector<pair<Mnode*, double> > distances;
	double distance;
	rep(i, 0, Frontiers_Num)
	{
		pair<Node*, double> pr;
		distance = sqrt(pow(abs((int)(Frontiers[i]->get_X() - x)), 2) + pow(abs((int)(Frontiers[i]->get_Y() - y)), 2));
		pr.first = Frontiers[i];
		pr.second = distance;
		distances.push_back(pr);
	}
	struct sort_pred
	{
		bool operator()(const std::pair<Node*, Uint> &left, const std::pair<Node*, Uint>&right)
		{
			return left.second < right.second;
		}
	};
	sort(distances.begin(), distances.end(), sort_pred());

	if (Frontiers_Num != 0)
	{
		detected_frontiers.insert(point(distances[0].first->get_X(), distances[0].first->get_Y()));
		return distances[0].first;
	}

	Mnode* mn = new Mnode(-1, -1);
	return mn;
}
