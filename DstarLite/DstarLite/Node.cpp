#include "node.h"

using namespace std;
using namespace DsLite;


//========================= Node_class_functions_declarations ============================//
const Uint Node::num_nbrs = 8;

//constructors

Node::Node(int x, int y, double c, double p) : X(x), Y(y), cost(c), prob(p)
{
	init = false;
	neighbours = NULL;
}

Node::Node()
{
	Node::Node(0, 0, 0.0, 0.5);
}

//destructor

Node::~Node()
{
	if (neighbours != NULL)
		delete[] neighbours;
}

//=============== getters

int Node::get_X() const { return X; }

int Node::get_Y() const { return Y; }

double Node::get_cost() const { return cost; }

Node** Node::get_nbrs(){ return neighbours; }

double Node::get_prob() { return prob; }


//============= setters

void Node::set_X(int x) { X = x; }

void Node::set_Y(int y) { Y = y; }

void Node::set_cost(double c){ cost = c; }

void Node::set_nbrs(Node** nbrs)
{
	if (init)
		return;
	init = true;
	neighbours = nbrs;
}

void Node::set_prob(double p){ prob = p; }


// ============ operators

bool operator==(const Node& a, const Node& b)
{
	return ((a.get_X() == b.get_X()) && (a.get_Y() == b.get_Y()));
}

bool operator!=(const Node& a, const Node& b)
{
	return ((a.get_X() != b.get_X()) || (a.get_Y() != b.get_Y()));
}

bool operator<(const Node & a, const Node & b)
{
	return (a.get_cost() - eps > b.get_cost());
}

bool operator>(const Node & a, const Node & b)
{
	return (a.get_cost() < b.get_cost() - eps);
}