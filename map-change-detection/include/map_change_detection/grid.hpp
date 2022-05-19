#pragma once

#include "map_change_detection/vector2.hpp"
#include "map_change_detection/rolling_buffer.hpp"
#include "map_change_detection/efficient_cell_store.hpp"

#include <map_change_detection/ChangedCells.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2/utils.h>
#include <vector>
#include <boost/optional.hpp>

class Grid
{
	public:
		/** Create a Grid object starting from a given map
			@param map
			@param saturation - counter max allowed value  */
		Grid(ros::NodeHandle* nodehandle, const nav_msgs::OccupancyGrid& map, bool preprocessing=true);

		inline const uint32_t& getWidth() const     {   return width;				}
		inline const uint32_t& getHeight() const    {   return height;				}
		inline const float& getResolution() const   {   return resolution;			}
		inline const double& getOriginX() const		{   return originPos.getX();	}
		inline const double& getOriginY() const		{   return originPos.getY();	}
		inline const double& getOriginYaw() const	{   return originYaw;			}

		inline void setOriginPos(const Vector2<double>& pos) {	originPos = pos;	}
		inline void setOriginX(const double& x) {	originPos.setX(x);	}
		inline void setOriginY(const double& y) {	originPos.setY(y);	}

		inline void setOriginYaw(const double& y) {		originYaw = y;	}

		inline const float& getProbOcc(const Vector2<int32_t>& cell) const   		{   return probOcc.at(cell.getX()).at(cell.getY()); }
		inline const float& getProbOcc(const int32_t& x, const int32_t& y) const	{   return probOcc.at(x).at(y); 					}

		/** Set occupancy probability of the given cell
		 * @param cell
		 * @param p */
		inline void setProbOcc(const Vector2<int32_t>& cell, const float& p)		{   probOcc.at(cell.getX()).at(cell.getY()) = p;	}
		inline void setProbOcc(const int32_t& x, const int32_t& y, const float& p)	{   probOcc.at(x).at(y) = p; 						}

		inline const int& getBuffersSize() const	{	return buffer_size;	}

		/** Check whether the given coordinates are valid grid indices
			@param gridPos 
			@return validity bool */
		inline bool isValidGridIndex(const Vector2<int32_t>& gridPos) const {  return ((gridPos.getX() < width) && (gridPos.getY() < height) && (gridPos.getX() >= 0) && (gridPos.getY() >= 0)); }
		inline bool isValidGridIndex(const int32_t& x, const int32_t& y) const {  return ((x < width) && (y < height) && (x >= 0) && (y >= 0)); }

		/** Convert from world coordinates to grid coordinates
		 * @param worldPosition
		 * @return grid coordinate */
		Vector2<int32_t> worldToGrid(const Vector2<double>& worldPosition) const;
		Vector2<int32_t> worldToGrid(const double& x, const double& y) const;

		/** Convert from grid coordinate to world coordinate
		 * @param gridPosition
		 * @return world coordinate */
		Vector2<double> gridToWorld(const Vector2<int32_t>& gridPosition) const;
		Vector2<double> gridToWorld(const int32_t& x, const int32_t& y) const;

		/** update actual_map attribute and publish the map */
		void updateMap();

		inline void publishChangedCells(const map_change_detection::ChangedCells& msg) {	pub_cells.publish(msg);	}

		inline const bool isChanged(const Vector2<int32_t>& cell) {
			return (countChangedFlags(cell) >= change_threshold  || ((countChangedFlags(cell) + countChangedNeighbours(cell)) >= change_threshold));
		}
		inline const bool isChanged(const int32_t& x, const int32_t& y) {
			return (countChangedFlags(x,y) >= change_threshold  || ((countChangedFlags(x,y) + countChangedNeighbours(x,y)) >= change_threshold));
		}

		/** Compute the valid cells adjacent to the input cell (the input cell is included)
		 * @param cell */
		std::vector<Vector2<int32_t>> getAdjacentCells(const Vector2<int32_t>& cell) const;

		/** Return true if there is at least one occupied cell inside the circle centered in 'cell' with radius 'radius'
		 * @param cell
		 * @param radius */
		bool neighborsOccupied(const Vector2<int32_t>& cell, const int32_t& radius) const;

		// TO DO:	le due funzioni di ray casting fanno cose quasi uguali, si possono accorpare? Se no, si può riorganizzare meglio il codice di queste due funzioni?

		/** Return the coordinates of the first occupied cell encountered starting from cell start and moving towards cell end.
		 * If no occupied cell is found, an empty optional is returned
		 * @param start
		 * @param end
		 * @return cell */
		boost::optional<Vector2<int32_t>> virtualRayCasting(const Vector2<int32_t>& start, const Vector2<int32_t>& end) const;

		/** Return the coordinates of the 'interesting' cells encountered starting from cell start and moving towards cell end (cell end is not included!).
		 * If trace_cells is true, occupied cells are returned, otherwise free cells previously seen as occupied are returned
		 * @param start
		 * @param end
		 * @param trace_cells
		 * @return cells */
		std::vector<Vector2<int32_t>> rayCasting(const Vector2<int32_t>& start, const Vector2<int32_t>& end, const bool& trace_cells) const;

		inline void insertChangedFlag(const Vector2<int32_t>& cell) {
			int64_t	lin_idx = cell.getY() * width + cell.getX();
			if(cellBuffers.at(lin_idx).getCount() == 0)
				cellBuffers.at(lin_idx).setSize(buffer_size);
			cellBuffers.at(lin_idx).insert(true);
		}

		inline void insertUnchangedFlag(const Vector2<int32_t>& cell) {
			int64_t	lin_idx = cell.getY() * width + cell.getX();
			if(cellBuffers.at(lin_idx).getCount() == 0)
				return;
			cellBuffers.at(lin_idx).insert(false);
		}

		// inline const int8_t& countChangedFlags(const Vector2<int32_t>& cell) const	{	return cellBuffers.at(cell.getX()).at(cell.getY()).getCount();	}
		// inline const int8_t& countChangedFlags(const int& x, const int& y) const 	{	return cellBuffers.at(x).at(y).getCount();						}
		inline const int8_t& countChangedFlags(const Vector2<int32_t>& cell) const	{	return cellBuffers.at(cell.getX() + width * cell.getY()).getCount();	}
		inline const int8_t& countChangedFlags(const int& x, const int& y) const 	{	return cellBuffers.at(x + width * y).getCount();						}

		
		SortedCellContainer	toOcc, toFree;	// keep track of the potentially changed cells in an efficient (hope so) way

	private:

		ros::NodeHandle 		nh_;
		ros::Publisher			pub_cells;


		uint32_t	width, height;
		float		resolution;
		Vector2<double>	originPos;
		double			originYaw;

		std::vector<std::vector<float>>	probOcc;	// [0, 100], 0 se la cella è sicuramente libera, 100 se la cella è sicuramente occupata

		int	buffer_size, change_threshold;

		std::vector<RollingBuffer>	cellBuffers;

		int8_t countChangedNeighbours(const Vector2<int32_t>& cell);
		int8_t countChangedNeighbours(const int32_t& x, const int32_t& y);
};
