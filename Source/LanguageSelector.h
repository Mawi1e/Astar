#pragma once
#include <locale>
#include <fstream>
#include <iostream>

#include <fcntl.h>
#include <io.h>

namespace mawile {
	enum class LanguageSelector {
		English = 1,
		Japanese = 2,
		Korean = 3,
	};

	std::string LScout(mawile::LanguageSelector, std::string, std::string, std::string);
	std::wstring LSwcout(mawile::LanguageSelector, std::wstring, std::wstring, std::wstring);
	std::string LSstream(std::ofstream&, mawile::LanguageSelector, std::string, std::string, std::string);
	std::wstring LSwstream(std::wofstream, mawile::LanguageSelector, std::wstring, std::wstring, std::wstring);
}