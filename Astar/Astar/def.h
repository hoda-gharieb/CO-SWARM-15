#ifndef AS_DEF_H
#define AS_DEF_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <math.h>
#include <vector>
#include <sstream>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <iomanip>
#include <cstring>
#include <limits.h>
#include <queue>

using namespace std;

#define PD pair<double,double>
#define Set(a, s) memset(a, s, sizeof (a))
#define rep(i, x, y) for(int i = x; i < y; i++)
#define Rep(i, x, y) for(int i = x; i <= y; i++)
#define vi vector<int>
#define vvi vector<vector<int> >
#define vp vector< pair< int, int > >
#define point pair<Uint, Uint >
#define pp push_back
#define mp make_pair
#define eps pow(10.0,-14.0)
#define MOD 1000000007
#define PI 3.14159265
#define Sqrt2 1.414213562373095048
#define oo 1e18

typedef unsigned int Uint;
typedef unsigned long long ull;
typedef long long ll;

// direction : odd indices for moving diagonally, even ones for moving straight
const int xDir[] = { 0, 1, 1, 1, 0, -1, -1, -1 };
const int yDir[] = { 1, 1, 0, -1, -1, -1, 0, 1 };

#endif AS_DEF_H