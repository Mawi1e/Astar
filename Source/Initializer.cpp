#include "Initializer.h"

namespace mawile {
	bool Initializer::Run() noexcept {
		while (true) {
			ClearConsoleScreen();
			mawile::LSwcout(this->_LanguageSelector,
				L"Choose a way to get information about the map.\n1. Direct Input\n2. File Input\n> ",
				L"マップに関する情報の入力方法を選んでください。\n1. 直接入力\n2. ファイル入力\n> ",
				L"맵에 대한정보를 입력받을 방법을 고르세요.\n1. 직접 입력\n2. 파일 입력\n> "
			);

			Coordinate Src = { 0, 0 };
			Coordinate Dst = { 0, 0 };
			int _Input;
			std::cin >> _Input;

			if (_Input == 1) {
				mawile::LSwcout(this->_LanguageSelector,
					L"Enter the cols of the map.> ",
					L"マップの横の長さを入力してください。> ",
					L"맵의 가로길이를 입력하세요.> "
				);
				std::cin >> (this->cols);

				mawile::LSwcout(this->_LanguageSelector,
					L"Enter the rows of the map.> ",
					L"マップの縦の長さを入力してください。> ",
					L"맵의 세로길이를 입력하세요.> "
				);
				std::cin >> (this->rows);

				(this->Map) = std::vector<std::vector<int>>((this->rows), std::vector<int>((this->cols)));

				mawile::LSwcout(this->_LanguageSelector,
					L"Enter the map.(0: Empty space, 1: Wall, 2: Starting point, 3: Arrival point)\n",
					L"マップを入力してください。 (0: 空いたスペース、 1: 壁、 2:出発地点、 3: 到着地点)\n",
					L"맵을 입력하세요. (0: 빈 공간, 1: 벽, 2: 출발지점, 3: 도착지점)\n"
				);

				for (int i = 0; i < (this->rows); ++i) {
					for (int j = 0; j < (this->cols); ++j) {
						std::cin >> (this->Map[i][j]);

						if ((this->Map[i][j]) == 2) {
							Src = { i, j };
							(this->Map[i][j]) = 0;
						}
						else if ((this->Map[i][j]) == 3) {
							Dst = { i, j };
							(this->Map[i][j]) = 0;
						}
					}
				}
			}
			else if(_Input == 2) {
				std::string _FilePath;
				mawile::LSwcout(this->_LanguageSelector,
					L"Enter a file path.> ",
					L"ファイルパスを入力してください。> ",
					L"파일경로를 입력하세요.> "
				);
				std::cin >> _FilePath;

				std::ifstream ifs(_FilePath);

				if (ifs.is_open()) {
					ifs >> (this->cols) >> (this->rows);
					(this->Map) = std::vector<std::vector<int>>((this->rows), std::vector<int>((this->cols)));

					for (int i = 0; i < (this->rows); ++i) {
						for (int j = 0; j < (this->cols); ++j) {
							ifs >> (this->Map[i][j]);

							if ((this->Map[i][j]) == 2) {
								Src = { i, j };
								(this->Map[i][j]) = 0;
							}
							else if ((this->Map[i][j]) == 3) {
								Dst = { i, j };
								(this->Map[i][j]) = 0;
							}
						}
					}

					ifs.close();
				}
			}
			else {
				continue;
			}

			if (!((this->_Astar)->Execute(cols, rows, Src, Dst, Map))) {
				continue;
			}

			mawile::LSwcout(this->_LanguageSelector,
				L"Choose a way to get information about the map printed.\n1. Direct Output\n2. File Output\n> ",
				L"マップ情報の出力方法を選んでください。\n1. 直接出力\n2. ファイル出力\n> ",
				L"맵에 대한정보를 출력받을 방법을 고르세요.\n1. 직접 출력\n2. 파일 출력\n> "
			);
			int _Output;
			std::cin >> _Output;

			ClearConsoleScreen();

			if (_Output == 1) {
				mawile::LSwcout(this->_LanguageSelector,
					L"[ The shortest path searched ]\n",
					L"[ 探索された最短経路 ]\n",
					L"[ 탐색된 최단경로 ]\n"
				);

				for (auto const& v : (this->_Astar)->GetPath()) {
					std::cout << '[' << v.x << ", " << v.y << ']' << '\n';
				}
				std::cout << '\n';

				mawile::LSwcout(this->_LanguageSelector,
					L"[ The maps searched ]\n",
					L"[ 探索されたマップ ]\n",
					L"[ 탐색된 맵 ]\n"
				);

				for (int i = 0; i < (this->rows); ++i) {
					for (int j = 0; j < (this->cols); ++j) {
						if ((this->Map[i][j]) == 4) std::cout << '*' << ' ';
						else std::cout << (this->Map[i][j]) << ' ';
					}
					std::cout << '\n';
				}
				std::cout << '\n';
			}
			else if (_Output == 2) {
				std::ofstream ofs("Map_Output.txt");

				if (ofs.is_open()) {
					mawile::LSstream(ofs, this->_LanguageSelector, 
						"[ The shortest path searched ]\n",
						"[ 探索された最短経路 ]\n",
						"[ 탐색된 최단경로 ]\n"
					);

					for (auto const& v : (this->_Astar)->GetPath()) {
						ofs << '[' << v.x << ", " << v.y << ']' << '\n';
					}
					ofs << '\n';

					mawile::LSstream(ofs, this->_LanguageSelector,
						"[ The maps searched ]\n",
						"[ 探索されたマップ ]\n",
						"[ 탐색된 맵 ]\n"
					);

					for (int i = 0; i < (this->rows); ++i) {
						for (int j = 0; j < (this->cols); ++j) {
							if((this->Map[i][j]) == 4) ofs << '*' << ' ';
							else ofs << (this->Map[i][j]) << ' ';
						}
						ofs << '\n';
					}

					ofs.close();
				}
			}
			else {
				continue;
			}

			mawile::LSwcout(this->_LanguageSelector,
				L"Press any key. ",
				L"どのキーでも押してください。 ",
				L"아무키나 눌러주세요. "
			);
			_getch();
		}

		return true;
	}

	Initializer::Initializer(mawile::LanguageSelector _LanguageSelector = mawile::LanguageSelector::English) noexcept {
		SetConsoleTitleW(mawile::LSwcout(_LanguageSelector,
			L"Astar ( Drakk. )",
			L"アスタール ( Drakk. )",
			L"에이스타 ( Drakk. )"
		).c_str());

		if ((this->_Astar) == nullptr) {
			(this->_Astar) = new mawile::Astar(false);
		}

		(this->_LanguageSelector) = _LanguageSelector;
	}

	Initializer::~Initializer() noexcept {
		if ((this->_Astar) != nullptr) {
			delete (this->_Astar);
		}

		(this->_Astar) = nullptr;
	}
}