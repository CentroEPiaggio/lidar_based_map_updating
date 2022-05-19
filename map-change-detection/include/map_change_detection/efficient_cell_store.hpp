#pragma once

#include "map_change_detection/vector2.hpp"
#include <vector>

class SortedCellContainer
{
	public:
		SortedCellContainer() {};

		const bool isPresent(const Vector2<int32_t>& cell) const {
			int32_t	i = 0, j = 0;
			while (i < stored_cells.size() && stored_cells.at(i).at(0).getX() < cell.getX())
				i++;
			if (i == stored_cells.size() || stored_cells.at(i).at(0).getX() > cell.getX())
				return false;
			while (j < stored_cells.at(i).size() && stored_cells.at(i).at(j).getY() < cell.getY())
				j++;
			if (j == stored_cells.at(i).size() || stored_cells.at(i).at(j).getX() > cell.getX())
				return false;
			if (Vector2<int32_t>::equal(cell, stored_cells.at(i).at(j)))
				return true;

			return false;
		}

		void insertElement(const Vector2<int32_t>& cell) {
			int32_t	i = 0, j = 0;
			while (i < stored_cells.size() && stored_cells.at(i).at(0).getX() < cell.getX())
				i++;
			if (i == stored_cells.size()) {
				std::vector<Vector2<int32_t>>	vec;
				vec.push_back(cell);
				stored_cells.push_back(vec);
				return;
			}
			if (stored_cells.at(i).at(0).getX() > cell.getX()) {
				std::vector<Vector2<int32_t>>	vec;
				vec.push_back(cell);
				stored_cells.insert(stored_cells.begin() + i, vec);
				return;
			}
			while (j < stored_cells.at(i).size() && stored_cells.at(i).at(j).getY() < cell.getY())
				j++;
			if (j == stored_cells.at(i).size()) {
				stored_cells.at(i).push_back(cell);
				return;
			}
			if (Vector2<int32_t>::equal(cell, stored_cells.at(i).at(j)))
				return;

			stored_cells.at(i).insert(stored_cells.at(i).begin() + j, cell);
			return;
		}

		void removeElement(const Vector2<int32_t>& cell) {
			int32_t	i = 0, j = 0;
			while (i < stored_cells.size() && stored_cells.at(i).at(0).getX() < cell.getX())
				i++;
			if (i == stored_cells.size() || stored_cells.at(i).at(0).getX() > cell.getX())
				return;

			while (j < stored_cells.at(i).size() && stored_cells.at(i).at(j).getY() < cell.getY())
				j++;
			if (j == stored_cells.at(i).size() || stored_cells.at(i).at(j).getX() > cell.getX())
				return;

			if (Vector2<int32_t>::equal(cell, stored_cells.at(i).at(j))) {
				if (stored_cells.at(i).size() == 1) {
					stored_cells.erase(stored_cells.begin() + i);
					return;
				}
				stored_cells.at(i).erase(stored_cells.at(i).begin() + j);
				return;
			}
			return;
		}

		inline const std::vector<std::vector<Vector2<int32_t>>>& getCells() const	{	return stored_cells;	}

	private:
		std::vector<std::vector<Vector2<int32_t>>>	stored_cells;

};