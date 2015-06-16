#include "map_node.h"

using namespace std;
using namespace Astar;


//========================= Map_Node_class_functions_declarations ============================//

//check whether a cell is in the grid's range
bool Map_Node::exist(Uint x, Uint y)
{
	return  (x >= 0 && x < rows && y >= 0 && y < cols);
}

bool Map_Node::blocked(Uint x, Uint y)
{
	//return grid[x][y]->get_prob() - eps > 0.5;
	return !(grid[x][y]->get_prob() < 0.5 - eps);
}

//constructors
Map_Node::Map_Node(Uint c, Uint r)
{
	rows = r ;
	cols = c ;
	grid = new Node**[rows];

	// creating cells
	rep(i, 0, (int)rows)
	{
		grid[i] = new Node*[cols];
		rep(j, 0, (int)cols)
			grid[i][j] = new Node(i, j);
	}

	rep(i, 0, (int)rows)
	{
		rep(j, 0, (int)cols)
		{
			Node** nbrs = new Node*[Node::num_nbrs];
			for (int k = 0; k < Node::num_nbrs; k += (Node::num_nbrs == 8 ? 1 : 2))
			{
				Uint x = i + xDir[k], y = j + yDir[k];
				if (Map_Node::exist(x, y))
					nbrs[k] = grid[x][y];
				else
					nbrs[k] = NULL;
			}
			grid[i][j]->set_nbrs(nbrs);
		}
	}

	init_lists();
}

Map_Node::Map_Node()
{
	Map_Node::Map_Node(100, 100);
}

//destructor
Map_Node::~Map_Node()
{
	rep(i, 0, (int)rows)
	{
		rep(j, 0, (int)cols)
			delete grid[i][j];
		delete[] grid[i];
	}
	delete[] grid;
}

//getters
Uint Map_Node::get_cols(){ return cols; }

Uint Map_Node::get_rows(){ return rows; }

//=============== operators ==============//

Node* Map_Node::operator()(const Uint x, const Uint y){ return grid[x][y]; }


// ==================== manipulating open, closed and direction lists ======================//

void Map_Node::init_lists()
{
	opened.resize(rows);
	dir_map.resize(rows);
	closed.resize(rows);

	rep(i, 0, (int)rows)
	{
		opened[i].resize(cols);
		dir_map[i].resize(cols);
		closed[i].resize(cols);
		rep(j, 0, (int)cols)
		{
			opened[i][j] = 0.0;
			closed[i][j] = false;
			dir_map[i][j] = 0;
		}
	}

}

double Map_Node::get_opened(Uint x, Uint y){ return opened[x][y]; }
void Map_Node::set_opened(Uint x, Uint y, double b){ opened[x][y] = b; }

bool Map_Node::get_closed(Uint x, Uint y){ return closed[x][y]; }
void Map_Node::set_closed(Uint x, Uint y, bool b){ closed[x][y] = b; }

int Map_Node::get_dir(Uint x, Uint y){ return dir_map[x][y]; }
void Map_Node::set_dir(Uint x, Uint y, int b){ dir_map[x][y] = b; }