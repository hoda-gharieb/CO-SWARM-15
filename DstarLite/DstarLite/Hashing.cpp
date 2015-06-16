#include "Hashing.h"

//========================== Hashing_class_functions_declarations ========================//

const int DsLite::Hashing::hashing_factor = 1000000;

size_t DsLite::Hashing::operator()(Node* n) const
{
	return n->get_X() * n->get_Y() * DsLite::Hashing::hashing_factor;
}
