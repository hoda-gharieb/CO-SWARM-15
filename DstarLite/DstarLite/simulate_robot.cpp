#include "simulate_robot.h"

DsLite::simulate_robot::simulate_robot(int r, int c, point p)
{
	row = r;
	col = c;

	grid = new int*[row];
	rep(i, 0, row)
		grid[i] = new int[col];

	rob_pos = p;
}

DsLite::simulate_robot::~simulate_robot()
{
	rep(i, 0, row)
		delete[] grid[i];
	delete[] grid;
}

void DsLite::simulate_robot::init()
{
	rep(i, 0, row)
		rep(j, 0, col)
		grid[i][j] = 0;

	for (int i = row / 8; i < row * 7 / 8; i++)
		grid[i][col / 2] = 1;
	for (int i = col / 8; i < col * 7 / 8; i++)
		grid[row / 2][i] = 1;

	cout << "Map Size (X,Y): " << row << "," << col << endl;
	cout << "Start: " << rob_pos.first << "," << rob_pos.second << endl;

	rep(i, 0, row)
	{
		rep(j, 0, col)
		{
			if (rob_pos.first == i && rob_pos.second == j)
				cout << "S";
			else if (grid[i][j] == 0)
				cout << ".";
			else
				cout << "O";
		}
		cout << endl;
	}
}

vector<vector< double > > DsLite::simulate_robot::handle_simulator(point p)
{
	rob_pos = p;
	vector<vector< double > > V(3);
	rep(i, 0, 3)
		V[i].resize(3, -1);
	int x = 1, y = 1, X, Y, a, b;
	rep(i, 0, 8)
	{
		X = x + xDir[i]; Y = y + yDir[i]; a = rob_pos.first + xDir[i]; b = rob_pos.second + yDir[i];
		if (a >= 0 && b >= 0 && a < row && b < col)
			V[X][Y] = grid[a][b];
	}
	return V;
}