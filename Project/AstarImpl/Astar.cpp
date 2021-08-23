#include "Astar.h"

namespace mawile {
	namespace AstarFunc {
		constexpr bool isDestination(int Current_y, int Current_x, Coordinate Dst) {
			return (Dst.y == Current_y && Dst.x == Current_x);
		}

		constexpr bool isUnBlocked(std::vector<std::vector<int>>& Map, int Current_y, int Current_x) {
			return (Map[Current_y][Current_x] == 0);
		}

		constexpr bool isInRange(int Rows, int Cols, int Current_y, int Current_x) {
			return (Current_y >= 0 && Current_x >= 0 && Current_y < Rows && Current_x < Cols);
		}

		double Distance(int X1, int Y1, int X2, int Y2) {
			return (double)(std::sqrt(std::pow(X1 - X2, 2) + std::pow(Y1 - Y2, 2)));
		}
	}

	bool Astar::Execute(int Cols, int Rows, Coordinate Src, Coordinate Dst, std::vector<std::vector<int>>& Map) {
		(this->Cols) = Cols;
		(this->Rows) = Rows;
		(this->Map) = &Map;

		if (!AstarFunc::isInRange(Rows, Cols, Src.y, Src.x) ||
			!AstarFunc::isInRange(Rows, Cols, Dst.y, Dst.x)) return false;
		if (!AstarFunc::isUnBlocked(Map, Src.y, Src.x) ||
			!AstarFunc::isUnBlocked(Map, Dst.y, Dst.x)) return false;
		if (AstarFunc::isDestination(Src.y, Src.x, Dst)) return true;

		std::vector<std::vector<bool>> closedList(Rows, std::vector<bool>(Cols));
		std::vector<std::vector<Cell>> CellDetails(Rows, std::vector<Cell>(Cols));

		for (int i = 0; i < Rows; ++i) {
			for (int j = 0; j < Cols; ++j) {
				CellDetails[i][j].f = CellDetails[i][j].g = CellDetails[i][j].h = AstarFunc::INF;
				CellDetails[i][j].parent_y = CellDetails[i][j].parent_x = -1;
				closedList[i][j] = false;
			}
		}

		int start_x = Src.x;
		int start_y = Src.y;

		CellDetails[start_y][start_x].f
			= CellDetails[start_y][start_x].g
			= CellDetails[start_y][start_x].h
			= 0.0;
		CellDetails[start_y][start_x].parent_y = start_y;
		CellDetails[start_y][start_x].parent_x = start_x;

		std::set<CostCoordinate> openList;
		openList.insert({ 0.0, { start_y, start_x } });

		while (!openList.empty()) {
			CostCoordinate Current = *openList.begin();
			openList.erase(openList.begin());

			int Currentx = Current.coord.x;
			int Currenty = Current.coord.y;

			closedList[Currenty][Currentx] = true;

			double Newf, Newg, Newh;

			for (std::size_t i = 0; i < 4; ++i) {
				int Nextx = Currentx + AstarFunc::Dx1[i];
				int Nexty = Currenty + AstarFunc::Dy1[i];

				if (AstarFunc::isInRange(Rows, Cols, Nexty, Nextx)) {
					if (AstarFunc::isDestination(Nexty, Nextx, Dst)) {
						CellDetails[Nexty][Nextx].parent_x = Currentx;
						CellDetails[Nexty][Nextx].parent_y = Currenty;
						BacktrackingMap(CellDetails, Dst);
						return true;
					}
					else if (closedList[Nexty][Nextx] == false &&
						AstarFunc::isUnBlocked(Map, Nexty, Nextx)) {
						Newg = CellDetails[Currenty][Currentx].g + 1.000;
						Newh = AstarFunc::Distance(Nextx, Nexty, Dst.x, Dst.y);
						Newf = Newg + Newh;

						if (CellDetails[Nexty][Nextx].f == AstarFunc::INF ||
							CellDetails[Nexty][Nextx].f > Newf) {
							CellDetails[Nexty][Nextx].g = Newg;
							CellDetails[Nexty][Nextx].h = Newh;
							CellDetails[Nexty][Nextx].f = Newf;

							CellDetails[Nexty][Nextx].parent_x = Currentx;
							CellDetails[Nexty][Nextx].parent_y = Currenty;

							openList.insert({ Newf, { Nexty, Nextx } });
						}
					}
				}
			}

			if ((this->Flag) == false) {
				for (std::size_t i = 0; i < 4; ++i) {
					int Nextx = Currentx + AstarFunc::Dx2[i];
					int Nexty = Currenty + AstarFunc::Dy2[i];

					if (AstarFunc::isInRange(Rows, Cols, Nexty, Nextx)) {
						if (AstarFunc::isDestination(Nexty, Nextx, Dst)) {
							CellDetails[Nexty][Nextx].parent_x = Currentx;
							CellDetails[Nexty][Nextx].parent_y = Currenty;
							BacktrackingMap(CellDetails, Dst);
							return true;
						}
						else if (closedList[Nexty][Nextx] == false &&
							AstarFunc::isUnBlocked(Map, Nexty, Nextx)) {
							Newg = CellDetails[Currenty][Currentx].g + 1.414;
							Newh = AstarFunc::Distance(Nextx, Nexty, Dst.x, Dst.y);
							Newf = Newg + Newh;

							if (CellDetails[Nexty][Nextx].f == AstarFunc::INF ||
								CellDetails[Nexty][Nextx].f > Newf) {
								CellDetails[Nexty][Nextx].g = Newg;
								CellDetails[Nexty][Nextx].h = Newh;
								CellDetails[Nexty][Nextx].f = Newf;

								CellDetails[Nexty][Nextx].parent_x = Currentx;
								CellDetails[Nexty][Nextx].parent_y = Currenty;

								openList.insert({ Newf, { Nexty, Nextx } });
							}
						}
					}
				}
			}
		}

		return false;
	}

	void Astar::BacktrackingMap(std::vector<std::vector<Cell>>& cellDetails, Coordinate Dst) {
		(this->Path).clear();
		std::stack<Coordinate> Path;
		Path.push({ Dst.y, Dst.x });
		(this->Path).push_back({ Dst.y, Dst.x });

		int Currentx = Dst.x;
		int Currenty = Dst.y;

		while (!(cellDetails[Currenty][Currentx].parent_x == Currentx &&
			cellDetails[Currenty][Currentx].parent_y == Currenty)) {
			int tempX = cellDetails[Currenty][Currentx].parent_x;
			int tempY = cellDetails[Currenty][Currentx].parent_y;

			Currentx = tempX;
			Currenty = tempY;

			Path.push({ Currenty, Currentx });
			(this->Path).push_back({ Currenty, Currentx });
		}

		while (!Path.empty()) {
			Coordinate Coord = Path.top();
			Path.pop();

			(this->Map[0][Coord.y][Coord.x]) = 4;
		}

		std::reverse((this->Path).begin(), (this->Path).end());
	}

	std::vector<Coordinate> Astar::GetPath() {
		return (this->Path);
	}

	Astar::Astar(bool Flag = false) noexcept {
		(this->Flag) = Flag;
	}
}