
#ifndef DSLITE_HASHING_H
#define DSLITE_HASHING_H

#include <functional>
#include <stdlib.h>

#include "node.h"

using namespace std;

namespace DsLite
{
		//for hashing a cell
		class Hashing : public unary_function<Node*, size_t>
		{
		protected:
			static const int hashing_factor;
		public:
			size_t operator()(Node* n) const;
		};

};

#endif DSLITE_HASHING_H

