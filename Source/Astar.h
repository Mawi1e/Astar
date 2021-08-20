#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <set>
#include <stack>

#undef max

namespace mawile {
	struct Cell {
		double		f, g, h;
		int			parent_x, parent_y;
	};

	struct Coordinate {
		int y, x;
	};

	struct CostCoordinate {
		double			cost;
		Coordinate		coord;

		friend bool operator<(CostCoordinate const& lhs, CostCoordinate const& rhs) {
			return lhs.cost < rhs.cost;
		}
	};

	namespace AstarFunc {
		constexpr double INF = std::numeric_limits<double>::max();
		constexpr int Dx1[4] = { 0, 0, 1, -1 };
		constexpr int Dy1[4] = { -1, 1, 0, 0 };
		constexpr int Dx2[4] = { 1, -1, -1, 1 };
		constexpr int Dy2[4] = { -1, 1, -1, 1 };

		constexpr bool isDestination(int, int, Coordinate);
		constexpr bool isUnBlocked(std::vector<std::vector<int>>&, int, int);
		constexpr bool isInRange(int, int, int, int);
		double Distance(int, int, int, int);
	}

	class Astar {
	public:
		bool Execute(int, int, Coordinate, Coordinate, std::vector<std::vector<int>>&);
		void BacktrackingMap(std::vector<std::vector<Cell>>&, Coordinate);
		std::vector<Coordinate> GetPath();

		explicit(true) Astar(bool) noexcept;

	private:
		std::vector<Coordinate>			Path;
		std::vector<std::vector<int>>*	Map;
		Coordinate						Src, Dst;
		int								Cols, Rows;
		bool							Flag = false;

	};
}