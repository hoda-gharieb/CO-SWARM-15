#include "Node.h"

//========================= Astar::Node_class_functions_declarations ============================//
const Uint Astar::Node::num_nbrs = 8;

//constructors

Astar::Node::Node(int x, int y, double c, double pr, double p) : X(x), Y(y), cost(c), prob(p), priority(pr)
{
	init = false;
	neighbours = NULL;
}

Astar::Node::Node()
{
	Astar::Node::Node(0, 0, 0.0, 0.0, 0.5);
}

//destructor

Astar::Node::~Node()
{
	if (neighbours != NULL)
		delete[] neighbours;
}

//=============== getters

int Astar::Node::get_X() const { return X; }

int Astar::Node::get_Y() const { return Y; }

double Astar::Node::get_cost() const { return cost; }

double Astar::Node::get_priority() const{ return priority; }

double Astar::Node::get_prob() { return prob; }

Astar::Node** Astar::Node::get_nbrs()
{
	return neighbours;
}

//============= setters

void Astar::Node::set_X(int x) { X = x; }

void Astar::Node::set_Y(int y) { Y = y; }

void Astar::Node::set_cost(double c){ cost = c; }

void Astar::Node::set_priority(double p){ priority = p; }

void Astar::Node::set_prob(double p){ prob = p; }

void Astar::Node::set_nbrs(Astar::Node** nbrs)
{
	if (init)
		return;
	init = true;
	neighbours = nbrs;
}

//============== other class functions

void  Astar::Node::Update_prt(const int& x, const int& y)
{
	priority = cost + heuristic(x, y) * 10;
}

void  Astar::Node::next_cost(const int &dir)
{
	cost += ((dir % 2) ? 14 : 10);// higher priority for moving straight
}


// the heuristic here is an estimation for the distance from current Astar::Node to goal Astar::Node
double  Astar::Node::heuristic(const int &x, const int &y)
{
	return  (int)(sqrt((x - X)*(x - X) + (y - Y)*(y - Y))); // euclidean distance
}

// ============ operators

bool Astar::Node::operator<(const Astar::Node & a)const
{
	return (priority - eps > a.get_priority());
}

bool operator==(const Astar::Node& a, const Astar::Node& b)
{
	return ((a.get_X() == b.get_X()) && (a.get_Y() == b.get_Y()));
}

bool operator!=(const Astar::Node& a, const Astar::Node& b)
{
	return ((a.get_X() != b.get_X()) || (a.get_Y() != b.get_Y()));
}

bool operator<(const Astar::Node & a, const Astar::Node & b)
{
	return (a.get_priority() - eps > b.get_priority());
}

bool operator>(const Astar::Node & a, const Astar::Node & b)
{
	return (a.get_priority() < b.get_priority() - eps);
}

