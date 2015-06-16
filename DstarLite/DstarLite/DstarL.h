#ifndef DSLITE_ALGO_H
#define DSLITE_ALGO_H

#include "map_node.h"

#define Mnode Node

using namespace std;
using namespace DsLite;

namespace DsLite
{
	class DstarLite
	{
	public:
		struct key_compare : public binary_function< PD, PD, bool>
		{
			bool operator()(const PD& a, const PD& b) const;
		};

		static const int MaxSteps;

		DstarLite(Map_Node* mp, Mnode* st, Mnode* gol);

		~DstarLite();

		list< Mnode* > get_path();

		list< point > get_path_points();

		bool replan();

		Mnode* GS_start(Mnode* n);

		Mnode* GS_goal(Mnode* n);

		void update(Mnode* n, double cost);	

	protected:
		typedef unordered_map<Mnode*, PD, Hashing> HASH;
		typedef pair< PD, Mnode* > openPair;
		typedef multimap<PD, Mnode*, key_compare> OPENLIST;
		typedef unordered_map<Mnode*, OPENLIST::iterator, Hashing> OPENLhash;

		Mnode *start, *goal, *last;

		Map_Node* map;

		double Km;

		list< Mnode* > path;
		
		OPENLIST openList;

		HASH cells_h;
		
		OPENLhash openl_hash;


		bool ComputeShortestPath();

		pair<Mnode*, double> get_min_succ(Mnode* n);

		double calc_cost(Mnode* a, Mnode* b);
		double calc_g(Mnode* n, double cost = DBL_MIN);
		double calc_h(Mnode* c, Mnode* e);
		double rhs(Mnode* n, double cost = DBL_MIN);
		PD calc_key(Mnode* c);


		void generate_node(Mnode* n);
		void update_node(Mnode* n);

		void insert_olist(PD ki, Mnode* n);
		void remove_olist(Mnode* n);
		void update_olist(PD ki, Mnode* n);



	};
};

#endif