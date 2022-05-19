#include "map_change_detection/map_processer.hpp"

MapProcesser::MapProcesser(ros::NodeHandle* nodehandle) :	nh_(*nodehandle) {
	std::string map_topic, map_processed_topic, cells_topic;

	nh_.getParam("/map_topic", map_topic);
	nh_.getParam("/processed_map_topic", map_processed_topic);
	nh_.getParam("/changed_cells_topic", cells_topic);

	sub = nh_.subscribe(cells_topic, 1, &MapProcesser::subscriberCallback, this);
	pub = nh_.advertise<nav_msgs::OccupancyGrid>(map_processed_topic, 5, true);

	while(ros::ok()){
		boost::shared_ptr<nav_msgs::OccupancyGrid const> map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, ros::Duration(4.0));
		if(map){
			initial_map = *map;
			break;
		} else {
			ROS_INFO("Waiting for map...");
		}
	}
}

void MapProcesser::subscriberCallback(const map_change_detection::ChangedCells& msg) {
	nav_msgs::OccupancyGrid actual_map = initial_map;

	for(auto idx : msg.toOcc)
		actual_map.data.at(idx) = 100;

	for(auto idx : msg.toFree)
		actual_map.data.at(idx) = 0;

	for(auto idx : msg.toFree) {
		bool below = idx >= actual_map.info.width;
		bool above = idx < (actual_map.data.size() - actual_map.info.width);
		int remainder = idx % actual_map.info.width;
		bool right = remainder != actual_map.info.width - 1;
		bool left = remainder != 0;
		if(below && above && right && left) {
			auto tmp_idx = idx - actual_map.info.width; // below
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx++;	// below right
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx -= 2;	// below left
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx += actual_map.info.width; // left
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx += 2; // right
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx += actual_map.info.width; // above right
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx--;	// above
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
			tmp_idx--;	// above left
			if(actual_map.data.at(tmp_idx) == -1)
				actual_map.data.at(tmp_idx) = 100;
		} else {
			int64_t	tmp_idx;
			if(below) {
				tmp_idx = idx - actual_map.info.width;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(above) {
				tmp_idx = idx + actual_map.info.width;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(right) {
				tmp_idx = idx + 1;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(left) {
				tmp_idx = idx - 1;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(above && right) {
				tmp_idx = idx + actual_map.info.width + 1;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(above && left) {
				tmp_idx = idx + actual_map.info.width - 1;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(below && right) {
				tmp_idx = idx - actual_map.info.width + 1;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
			if(below && left) {
				tmp_idx = idx - actual_map.info.width - 1;
				if(actual_map.data.at(tmp_idx) == -1)
					actual_map.data.at(tmp_idx) = 100;
			}
		}
	}

	actual_map.header.stamp = ros::Time::now();

	pub.publish(actual_map);
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "updated_map_processer_node");
	ros::NodeHandle nh("~");

	MapProcesser	node(&nh);

	ros::spin();

	return 0;
}