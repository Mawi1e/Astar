#include "Initializer.h"

int main() {
	std::cin.tie(0);
	std::cout.tie(0);
	std::ios_base::sync_with_stdio(0);

	mawile::Initializer* _Initializer = new mawile::Initializer(mawile::LanguageSelector::Japanese);

	if (_Initializer->Run() == false) {
		return (-1);
	}

	delete _Initializer;

	return (0);
}