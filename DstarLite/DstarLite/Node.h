
#ifndef DSLITE_NODE_H
#define DSLITE_NODE_H

#include <functional>
#include <stdlib.h>

#include "def.h"

using namespace std;

namespace DsLite
{
	
		class Node
		{

		protected:
			//node coordinates
			int X, Y;
			//node cost
			double cost;
			//node initated or not
			bool init;
			//all node neighbours
			Node** neighbours;
			//probability needed in frontier based algorithm to determine whether it's occupied or not
			double prob;

		public:
			static const Uint num_nbrs;
			//constructors
			Node(int x, int y, double c = 1.0, double p = 0.5);
			Node();

			//destructor
			~Node();

			//getters
			int get_X() const;
			int get_Y() const;
			double get_cost() const;
			Node** get_nbrs();
			double get_prob();

			//setters
			void set_X(int x);
			void set_Y(int y);
			void set_cost(double c);
			void set_prob(double p);
			void set_nbrs(Node** nbrs);


		};

};

#endif DSLITE_NODE_H

