#include "map_change_detection/map_change_detection.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "map_checker_node");
	ros::NodeHandle nh("~");

	MapChangeDetection node(&nh);

	ROS_INFO("Node initializated. Entering the loop");

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();

		if (node.scan_handler->getNewScanFlag()) {
			node.detectChanges();
		}

		if (node.readyForUpdate())
			node.scan_handler->setUpdateScanFlag(true);

		// Update time-related flag
		node.checkElapsedTime();

		if(node.getUpdateOnlineFlag())
			node.grid->updateMap();

		loop_rate.sleep();

	}

	return 0;
}