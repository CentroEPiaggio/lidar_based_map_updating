#include "map_change_detection/grid.hpp"

#include <math.h>
#include <functional>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

Grid::Grid(ros::NodeHandle* nodehandle, const nav_msgs::OccupancyGrid& map, bool preprocessing)	:	nh_(*nodehandle)
{
	nh_.getParam("buffer_size", buffer_size);
	ROS_ASSERT_MSG(buffer_size > 0, "Buffer size must be greater than 0, but %d was given!", buffer_size);
	nh_.getParam("counters_change_threshold", change_threshold);
	ROS_ASSERT_MSG(change_threshold > 0 && change_threshold <= buffer_size, "Invalid change threshold!");

	width = map.info.width;
	height = map.info.height;
	resolution = map.info.resolution;
	originPos.setX(map.info.origin.position.x);
	originPos.setY(map.info.origin.position.y);

	originYaw = tf2::getYaw(map.info.origin.orientation);

	probOcc.resize(width);
	for (int k=0; k<map.data.size(); k++){
		if (preprocessing && map.data.at(k) < 0)
			probOcc.at(k % width).push_back(100);
		else
			probOcc.at(k % map.info.width).push_back(map.data.at(k));
	}

	cellBuffers.resize(map.info.width * map.info.height);
	bool fill_buffers;
	nh_.getParam("init_buffers", fill_buffers);
	if(fill_buffers) {
		ROS_INFO("Fill empty buffers");
		for(int i = 0; i < cellBuffers.size(); i++)
			cellBuffers.at(i).setSize(buffer_size);
	}

	std::string	cells_topic;
	nh_.getParam("/changed_cells_topic", cells_topic);
	pub_cells = nh_.advertise<map_change_detection::ChangedCells>(cells_topic, 1, true);
	updateMap();

	ROS_INFO("Grid object created!");
	ROS_INFO("Width: %d; Height: %d; Resolution: %f", this->getWidth(), this->getHeight(), this->getResolution());
    ROS_INFO("Origin: x = %f, y = %f, yaw = %f", this->getOriginX(), this->getOriginY(), this->getOriginYaw());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void Grid::updateMap() {	
	// nav_msgs::OccupancyGrid map = initial_map;
	map_change_detection::ChangedCells	changed_msg;
	int64_t	lin_idx;
	for (auto cells : toFree.getCells()) {
		for (auto cell : cells) {
			lin_idx = cell.getX() + cell.getY() * getWidth();
			if (isChanged(cell)) {
				// map.data.at(lin_idx) = 0;
				changed_msg.toFree.push_back(lin_idx);
			}
		}
	}

	for (auto cells : toOcc.getCells()) {
		for (auto cell : cells) {
			lin_idx = cell.getX() + cell.getY() * getWidth();
			if (isChanged(cell)) {
				// map.data.at(lin_idx) = 100;
				changed_msg.toOcc.push_back(lin_idx);
			}
		}
	}
	
	// actual_map = map;

	changed_msg.header.stamp = ros::Time::now();
	pub_cells.publish(changed_msg);
	publishChangedCells(changed_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

Vector2<int32_t> Grid::worldToGrid(const Vector2<double>& worldPosition) const {

	double gridX = (worldPosition.getX() - originPos.getX()) / resolution;
	double gridY = (worldPosition.getY() - originPos.getY()) / resolution;

	return Vector2<int32_t>(static_cast<int32_t>(floor(gridX)), static_cast<int32_t>(floor(gridY)));
}

Vector2<int32_t> Grid::worldToGrid(const double& x, const double& y) const {

	double gridX = (x - originPos.getX()) / resolution;
	double gridY = (y - originPos.getY()) / resolution;

	return Vector2<int32_t>(static_cast<int32_t>(floor(gridX)), static_cast<int32_t>(floor(gridY)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

Vector2<double> Grid::gridToWorld(const Vector2<int32_t>& gridPosition) const {
	
	// This gives the center of the cell
	double worldX = originPos.getX() + (gridPosition.getX() + 0.5) * resolution;
	double worldY = originPos.getY() + (gridPosition.getY() + 0.5) * resolution;

	return Vector2<double>(worldX, worldY);
}

Vector2<double> Grid::gridToWorld(const int32_t& x, const int32_t& y) const {
	
	// This gives the center of the cell
	double worldX = originPos.getX() + (x + 0.5) * resolution;
	double worldY = originPos.getY() + (y + 0.5) * resolution;

	return Vector2<double>(worldX, worldY);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2<int32_t>> Grid::getAdjacentCells(const Vector2<int32_t>& cell) const {
	std::vector<Vector2<int32_t>>	cells;
	Vector2<int32_t>				tmp_cell(cell.getX(), cell.getY());

	bool x_valid = cell.getX() >= 0 && cell.getX() < width;
	bool y_valid = cell.getY() >= 0 && cell.getY() < height;

	if (x_valid && y_valid) {
		cells.push_back(tmp_cell);

		if (cell.getX() != 0) {
			tmp_cell.setX(cell.getX() - 1);
			cells.push_back(tmp_cell);
		}
		if (cell.getX() != (width - 1)) {
			tmp_cell.setX(cell.getX() + 1);
			cells.push_back(tmp_cell);
		}
		tmp_cell.setX(cell.getX());

		if (cell.getY() != 0) {
			tmp_cell.setY(cell.getY() - 1);
			cells.push_back(tmp_cell);
		}
		if (cell.getY() != (height - 1)) {
			tmp_cell.setY(cell.getY() + 1);
			cells.push_back(tmp_cell);
		}
	} else if (x_valid) {
		if (cell.getY() == -1) {
			tmp_cell.setY(0);
			cells.push_back(tmp_cell);
		} else if (cell.getY() == height) {
			tmp_cell.setY(height - 1);
			cells.push_back(tmp_cell);
		}
	} else if (y_valid) {
		if (cell.getX() == -1) {
			tmp_cell.setX(0);
			cells.push_back(tmp_cell);
		} else if (cell.getX() == width) {
			tmp_cell.setX(width - 1);
			cells.push_back(tmp_cell);
		}
	}
	return cells;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Adapted from:  https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles

bool Grid::neighborsOccupied(const Vector2<int32_t>& cell, const int32_t& radius) const {
	Vector2<int32_t>	tmp;
	for (int32_t x = -radius; x <= radius ; x++) {
		int32_t height = static_cast<int32_t>(std::sqrt(radius * radius - x * x));
		tmp.setX(x + cell.getX());
		for (int32_t y = -height; y <= height; y++){
			tmp.setY(y + cell.getY());
			if (isValidGridIndex(tmp)) {
				if (getProbOcc(tmp) >= 90)
					return true;
			}
		}
	}
	return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

// Algorithm based on: http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
// boost optional: https://stackoverflow.com/questions/22227839/how-to-use-boostoptional

boost::optional<Vector2<int32_t>> Grid::virtualRayCasting(const Vector2<int32_t>& start, const Vector2<int32_t>& end) const {
	int32_t	dx, dy, x, y, n, x_inc, y_inc, error;
	Vector2<int32_t> cell;

	x = start.getX();
    y = start.getY();

	if (x > end.getX())
		dx = x - end.getX();
	else
		dx = end.getX() - x;

	if (y > end.getY())
		dy = y - end.getY();
	else
		dy = end.getY() - y;

    n = 1 + dx + dy;
    x_inc = (end.getX() > x) ? 1 : -1;
    y_inc = (end.getY() > y) ? 1 : -1;
    error = static_cast<long int>(dx) - static_cast<long int>(dy);
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
		cell.setX(x);
		cell.setY(y);
		if (!isValidGridIndex(cell)) {
			ROS_WARN("Skipping beam from (%d, %d) to (%d, %d). It goes out of map in (%d, %d)", start.getX(), start.getY(), end.getX(), end.getY(), cell.getX(), cell.getY());
			return boost::none;
		}

		if (getProbOcc(cell) >= 90)
			return cell;

        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
	}

	return boost::none;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2<int32_t>> Grid::rayCasting(const Vector2<int32_t>& start, const Vector2<int32_t>& end, const bool& trace_cells) const {
	int32_t	dx, dy, x, y, n, x_inc, y_inc, error;
	Vector2<int32_t> 				actual_cell;
	std::vector<Vector2<int32_t>>	cells;
	std::function<void(const Vector2<int32_t>&)>	lambda;

	if (trace_cells) {
		lambda = [&] (Vector2<int32_t> tmp_cell) {
			if (getProbOcc(tmp_cell) >= 90 && !Vector2<int32_t>::equal(tmp_cell, end)) 
				cells.push_back(tmp_cell);
		};
	} else {
		lambda = [&] (Vector2<int32_t> tmp_cell) {
			if (getProbOcc(tmp_cell) == 0 && countChangedFlags(tmp_cell) > 0)
				cells.push_back(tmp_cell);
		};
	}

	x = start.getX();
    y = start.getY();

	if (x > end.getX())
		dx = x - end.getX();
	else
		dx = end.getX() - x;

	if (y > end.getY())
		dy = y - end.getY();
	else
		dy = end.getY() - y;

    n = 1 + dx + dy;
    x_inc = (end.getX() > x) ? 1 : -1;
    y_inc = (end.getY() > y) ? 1 : -1;
    error = static_cast<long int>(dx) - static_cast<long int>(dy);
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
		actual_cell.setX(x);
		actual_cell.setY(y);

		if (isValidGridIndex(actual_cell))
			lambda(actual_cell);

        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
	}

	return cells;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

int8_t Grid::countChangedNeighbours(const Vector2<int32_t>& cell) {
	int32_t x_valid, y_valid;

	bool	x_border = cell.getX() == 0 || cell.getX() == (getWidth() - 1);
	bool	y_border = cell.getY() == 0 || cell.getY() == (getHeight() - 1);

	int32_t x_prev = cell.getX() - 1;
	int32_t x_next = cell.getX() + 1;
	int32_t y_prev = cell.getY() - 1;
	int32_t y_next = cell.getY() + 1;

	int32_t count = 0;

	if (!x_border && !y_border) {
		// visit cells
		if (countChangedFlags(x_prev, y_prev) >= change_threshold && getProbOcc(x_prev, y_prev) == getProbOcc(cell))
			count++;
		if (countChangedFlags(cell.getX(), y_prev) >= change_threshold && getProbOcc(cell.getX(), y_prev) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, y_prev) >= change_threshold && getProbOcc(x_next, y_prev) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_prev, cell.getY()) >= change_threshold && getProbOcc(x_prev, cell.getY()) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, cell.getY()) >= change_threshold && getProbOcc(x_next, cell.getY()) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_prev, y_next) >= change_threshold && getProbOcc(x_prev, y_next) == getProbOcc(cell))
			count++;
		if (countChangedFlags(cell.getX(), y_next) >= change_threshold && getProbOcc(cell.getX(), y_next) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, y_next) >= change_threshold && getProbOcc(x_next, y_next) == getProbOcc(cell))
			count++;
	} else if (x_border) {
		if (cell.getX() == 0)
			x_valid = x_next;
		else
			x_valid = x_prev;
		if (y_border) {
			if (cell.getY() == 0)
				y_valid = y_next;
			else
				y_valid = y_prev;
			
			// visit cells
			if (countChangedFlags(x_valid, y_valid) >= change_threshold && getProbOcc(x_valid, y_valid) == getProbOcc(cell))
				count++;
			if (countChangedFlags(cell.getX(), y_valid) >= change_threshold && getProbOcc(cell.getX(), y_valid) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, cell.getY()) >= change_threshold && getProbOcc(x_valid, cell.getY()) == getProbOcc(cell))
				count++;
		} else {
			// visit cells
			if (countChangedFlags(cell.getX(), y_prev) >= change_threshold && getProbOcc(cell.getX(), y_prev) == getProbOcc(cell))
				count++;
			if (countChangedFlags(cell.getX(), y_next) >= change_threshold && getProbOcc(cell.getX(), y_next) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, cell.getY()) >= change_threshold && getProbOcc(x_valid, cell.getY()) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, y_next) >= change_threshold && getProbOcc(x_valid, y_next) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, y_prev) >= change_threshold && getProbOcc(x_valid, y_prev) == getProbOcc(cell))
				count++;
		}
	} else {
		if (cell.getY() == 0)
			y_valid = y_next;
		else
			y_valid = y_prev;

		// visit cells
		if (countChangedFlags(x_prev, y_valid) >= change_threshold && getProbOcc(x_prev, y_valid) == getProbOcc(cell))
			count++;
		if (countChangedFlags(cell.getX(), y_valid) >= change_threshold && getProbOcc(cell.getX(), y_valid) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, y_valid) >= change_threshold && getProbOcc(x_next, y_valid) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, cell.getY()) >= change_threshold && getProbOcc(x_next, cell.getY()) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_prev, cell.getY()) >= change_threshold && getProbOcc(x_prev, cell.getY()) == getProbOcc(cell))
			count++;
	}

	return count;

}

int8_t Grid::countChangedNeighbours(const int32_t& x, const int32_t& y) {
	Vector2<int32_t>	cell(x, y);
	int32_t				x_valid, y_valid;

	bool	x_border = cell.getX() == 0 || cell.getX() == (getWidth() - 1);
	bool	y_border = cell.getY() == 0 || cell.getY() == (getHeight() - 1);

	int32_t x_prev = cell.getX() - 1;
	int32_t x_next = cell.getX() + 1;
	int32_t y_prev = cell.getY() - 1;
	int32_t y_next = cell.getY() + 1;

	int32_t count = 0;

	if (!x_border && !y_border) {
		// visit cells
		if (countChangedFlags(x_prev, y_prev) >= change_threshold && getProbOcc(x_prev, y_prev) == getProbOcc(cell))
			count++;
		if (countChangedFlags(cell.getX(), y_prev) >= change_threshold && getProbOcc(cell.getX(), y_prev) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, y_prev) >= change_threshold && getProbOcc(x_next, y_prev) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_prev, cell.getY()) >= change_threshold && getProbOcc(x_prev, cell.getY()) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, cell.getY()) >= change_threshold && getProbOcc(x_next, cell.getY()) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_prev, y_next) >= change_threshold && getProbOcc(x_prev, y_next) == getProbOcc(cell))
			count++;
		if (countChangedFlags(cell.getX(), y_next) >= change_threshold && getProbOcc(cell.getX(), y_next) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, y_next) >= change_threshold && getProbOcc(x_next, y_next) == getProbOcc(cell))
			count++;
	} else if (x_border) {
		if (cell.getX() == 0)
			x_valid = x_next;
		else
			x_valid = x_prev;
		if (y_border) {
			if (cell.getY() == 0)
				y_valid = y_next;
			else
				y_valid = y_prev;
			
			// visit cells
			if (countChangedFlags(x_valid, y_valid) >= change_threshold && getProbOcc(x_valid, y_valid) == getProbOcc(cell))
				count++;
			if (countChangedFlags(cell.getX(), y_valid) >= change_threshold && getProbOcc(cell.getX(), y_valid) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, cell.getY()) >= change_threshold && getProbOcc(x_valid, cell.getY()) == getProbOcc(cell))
				count++;
		} else {
			// visit cells
			if (countChangedFlags(cell.getX(), y_prev) >= change_threshold && getProbOcc(cell.getX(), y_prev) == getProbOcc(cell))
				count++;
			if (countChangedFlags(cell.getX(), y_next) >= change_threshold && getProbOcc(cell.getX(), y_next) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, cell.getY()) >= change_threshold && getProbOcc(x_valid, cell.getY()) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, y_next) >= change_threshold && getProbOcc(x_valid, y_next) == getProbOcc(cell))
				count++;
			if (countChangedFlags(x_valid, y_prev) >= change_threshold && getProbOcc(x_valid, y_prev) == getProbOcc(cell))
				count++;
		}
	} else {
		if (cell.getY() == 0)
			y_valid = y_next;
		else
			y_valid = y_prev;

		// visit cells
		if (countChangedFlags(x_prev, y_valid) >= change_threshold && getProbOcc(x_prev, y_valid) == getProbOcc(cell))
			count++;
		if (countChangedFlags(cell.getX(), y_valid) >= change_threshold && getProbOcc(cell.getX(), y_valid) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, y_valid) >= change_threshold && getProbOcc(x_next, y_valid) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_next, cell.getY()) >= change_threshold && getProbOcc(x_next, cell.getY()) == getProbOcc(cell))
			count++;
		if (countChangedFlags(x_prev, cell.getY()) >= change_threshold && getProbOcc(x_prev, cell.getY()) == getProbOcc(cell))
			count++;
	}

	return count;

}
