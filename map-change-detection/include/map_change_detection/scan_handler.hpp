#pragma once

#include "map_change_detection/vector2.hpp"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

class ScanHandler
{
	public:
		/** Create a ScanHandler object.
		 * @param nodehandle
		 * @param fixed_frame */
		ScanHandler(ros::NodeHandle* nodehandle, std::string fixed_frame);

		inline const sensor_msgs::LaserScan& getScan() const	{	return scan;		}
		inline const geometry_msgs::Pose2D& getPose() const		{	return robot_pose;	}

		inline const int32_t& getBeamStep() const	{	return beam_step;	}
		inline const int32_t& getValidBeams() const	{	return valid_beams;	}
		inline const float& getMaxBeamRange() const	{	return beam_range;	}
		inline const bool& getNewScanFlag() const	{	return new_scan;	}
		inline const std::vector<Vector2<double>>& getHitPoints() const	{	return hitPoints;			}
		inline const bool isValidMeasurement(const int32_t& idx) const 	{	return valid_meas.at(idx);	}

		inline void setNewScanFlag(const bool& flag)	{	new_scan = flag;	}
		inline void setUpdateScanFlag(const bool& flag)	{	update_scan = flag;	}

	private:

		ros::NodeHandle nh_;

		ros::Subscriber	sub;

		// tf listener needed in order to get the robot pose
		tf2_ros::Buffer 							tfBuffer;
		std::unique_ptr<tf2_ros::TransformListener>	tfListener;

		sensor_msgs::LaserScan			scan;
		geometry_msgs::Pose2D			robot_pose;
		std::vector<Vector2<double>>	hitPoints;
		std::vector<bool>				valid_meas;
		int32_t							valid_beams;

		float	beam_range;
		int32_t	beam_step;

		bool	new_scan;		// indicates whether this object has processed a new scan
		bool	update_scan;	// indicates whether this object has to process a new scan

		std::string	map_frame, base_frame;

		/** LaserScan callback. Read the scan and get the estimated robot pose corresponding to the measurement
		 * @param msg */
		void subscriberCallback(const sensor_msgs::LaserScan& msg);

};