#include "map_change_detection/scan_handler.hpp"

#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

ScanHandler::ScanHandler(ros::NodeHandle* nodehandle, std::string fixed_frame) : nh_(*nodehandle), map_frame(fixed_frame), new_scan(false), update_scan(false)
{
	std::string	scan_topic;
	ROS_INFO("Calling ScanHandler constructor");

	nh_.getParam("base_frame", base_frame);
	nh_.getParam("scan_topic", scan_topic);
	nh_.getParam("max_beam_range", beam_range);

	sub = nh_.subscribe(scan_topic, 1, &ScanHandler::subscriberCallback, this);

	tfListener.reset(new tf2_ros::TransformListener(tfBuffer));

	// Wait for scan to be published
	while (ros::ok()){
		boost::shared_ptr<sensor_msgs::LaserScan const> first_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>(scan_topic, ros::Duration(4.0));
		if(first_scan){
			ROS_INFO("Got scan!");

			scan = *first_scan;

			uint64_t beams_count = ((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
			ROS_ASSERT_MSG(beams_count == scan.ranges.size(), "%lu beams expected, %lu given", beams_count, first_scan->ranges.size());

			double	min_delta;
			nh_.param("min_angle_increment", min_delta, 1.0);
			min_delta = min_delta * M_PI / 180.0;

			beam_step = ceil(min_delta / scan.angle_increment);
			ROS_ASSERT_MSG(beam_step > 0, "Request min angle increment is %f, lidar's angle increment is %f. Computed step is %d", min_delta,
																											scan.angle_increment, beam_step);

			scan.angle_increment = scan.angle_increment * beam_step;
			ROS_DEBUG("Lidar's angle increment is %f. Requested min angle increment is %f. Resulting angle increment is %f", first_scan->angle_increment * 180.0/M_PI, min_delta * 180.0/M_PI, scan.angle_increment * 180.0/M_PI);
			uint32_t	considered_beams = static_cast<uint32_t>(((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1);
			ROS_INFO("Beams considered: %u. Angle increment: %f deg", considered_beams, scan.angle_increment * 180 / M_PI);
			hitPoints.resize(considered_beams);
			valid_meas.resize(considered_beams);

			scan.intensities.clear();	//clear unused vector
			break;
		} else {
			ROS_INFO("Waiting for scan...");
		}
	}

	// Wait for tf to be published
	geometry_msgs::TransformStamped transformStamped;
	bool exist = false;
	while (ros::ok() && !exist) {
		try {
			transformStamped = tfBuffer.lookupTransform(map_frame, base_frame, ros::Time(0), ros::Duration(4.0));
			exist = true;
		}
		catch (tf2::TransformException &ex) {
			ROS_INFO("Waiting for transform from frame %s to frame %s...", map_frame.c_str(), base_frame.c_str());
   		}
	}
	ROS_INFO("ScanHandler object ready!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void ScanHandler::subscriberCallback(const sensor_msgs::LaserScan& msg) {

	if (update_scan) {
		try {
			// Localize scan
			geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(map_frame, base_frame, msg.header.stamp, ros::Duration(1.0));

			ROS_DEBUG("Scan localized!");
			robot_pose.theta = tf2::getYaw(transformStamped.transform.rotation);
			robot_pose.x = transformStamped.transform.translation.x;
			robot_pose.y = transformStamped.transform.translation.y;

			scan.header = msg.header;
			scan.ranges.clear();
			valid_beams = 0;
			double	th;
			int		i;
			Vector2<double>	point;
			for (int j = 0; j < hitPoints.size(); j++) {
				// i indexes the raw scan, j indexes the downsampled scan
				i = j * beam_step;
				scan.ranges.push_back(msg.ranges.at(i));

				// compute hit point
				if (msg.ranges.at(i) < beam_range) {
					th = msg.angle_min + i * msg.angle_increment + robot_pose.theta;
					point.setX(robot_pose.x + msg.ranges.at(i) * cos(th));
					point.setY(robot_pose.y + msg.ranges.at(i) * sin(th));
					hitPoints.at(j) = point;
					valid_meas.at(j) = true;
					valid_beams++;
				} else {
					valid_meas.at(j) = false;
				}
			}

			new_scan = true;
			update_scan = false;

		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("Transform from frame %s to frame %s not found, ignoring scan", map_frame.c_str(), base_frame.c_str());
   		}
	}
}
