#include "grid.hpp"

#include <map_change_detection/ChangedCells.h>

class MapProcesser
{
	public:
		MapProcesser(ros::NodeHandle* nodehandle);

	private:

		ros::NodeHandle nh_;
		ros::Subscriber	sub;
		ros::Publisher	pub;

		nav_msgs::OccupancyGrid initial_map;

		void subscriberCallback(const map_change_detection::ChangedCells& msg);
};