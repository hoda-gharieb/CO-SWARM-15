#include "Robot_Interface.h"
#include "simulate_robot.h"

using namespace DsLite;

int gcd(int a, int b) { return (b == 0 ? a : gcd(b, a%b)); }

const int row = 100, col = 100;




int main()
{
	freopen("output.out", "w", stdout);
	srand(time(NULL));


	clock_t start = clock();

	Robot_Interface *rob = new Robot_Interface(row, col, 0, 0);
	simulate_robot *sim = new simulate_robot(row, col, point(0, 0));
	sim->init();
	point p = point(0, 0);
	while (p.first != -1 || p.second != -1)
	{
		vector<vector< double > > V = sim->handle_simulator(p);
		double arr[3][3];
		rep(i, 0, 3)
			rep(j, 0, 3)
			arr[i][j] = V[i][j];

		p = rob->handle_interface(arr);
	}
	rob->display_map_prob();
	clock_t end = clock();
	double time_elapsed = double(end - start);
	cout << "Time to calculate the route (ms): " << time_elapsed << endl;

}




