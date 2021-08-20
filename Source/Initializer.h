#pragma once

#include <vector>
#include <fstream>
#include <iostream>

#include <Windows.h>
#include <conio.h>

#include "Astar.h"
#include "LanguageSelector.h"

#ifdef _WIN32
#define ClearConsoleScreen() std::system("cls")
#elif _WIN64
#define ClearConsoleScreen() std::system("cls")
#else
#define ClearConsoleScreen() std::system("clear")
#endif

namespace mawile {
	class Initializer {
	public:
		bool Run() noexcept;

		explicit(true) Initializer(mawile::LanguageSelector) noexcept;
		~Initializer() noexcept;

	private:
		mawile::Astar*					_Astar = nullptr;
		mawile::LanguageSelector		_LanguageSelector = mawile::LanguageSelector::English;
		std::vector<std::vector<int>>	Map;
		int								rows, cols;

	};
}