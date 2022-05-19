#pragma once

#include "map_change_detection/grid.hpp"
#include "map_change_detection/scan_handler.hpp"

#include <map_change_detection/UpdateMap.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class MapChangeDetection
{
	public:
		MapChangeDetection(ros::NodeHandle* nodehandle);

		inline const bool readyForUpdate() const	{	return (robot_moved_enough && enough_time_elapsed && !w_idle);	}

		inline const bool& getUpdateOnlineFlag() const	{	return online_update;	}

		/** Here the true work is done. This function calls findAnomalousMeasurements function in order to detect anomalous beams. Then anomalous beams
		 * are analyzed looking for changed cells, while not anomalous beams are analyzed looking for false positives introduced in the previous iterations.
		 * The result of this function is the update of the counters inside grid object */
		void detectChanges();

		/** If enough time is elapsed since last scan processing, the corresponding flag is set to true */
		void checkElapsedTime();

		std::unique_ptr<ScanHandler>	scan_handler;

		std::unique_ptr<Grid>	grid;

	private:
		bool first_time = true;	// only used for time consumption monitoring

		bool online_update;

		//ROS stuff
		ros::NodeHandle nh_;
		ros::Publisher	pub_marker;
		ros::Subscriber	sub_odom;
		ros::ServiceServer update_map_service;

		bool serviceCallback(map_change_detection::UpdateMap::Request& request, map_change_detection::UpdateMap::Response& response);

		// Anomalous beams detection
		double	m_pairing, q_pairing, dist_saturation;	// parameters for computing the pairing distance threshold as a function of the measured range
		int	vm_ratio;		// ratio between virtual beam and measured beam (consider vm_ratio virtual beams for each measured beam)
		int8_t	search_window;	// number of virtual beams evaluated for pairing in both direction (thus the number of evaluated beams is 2*search_width+1)
		float	v_max_range;		// max beam range considered for virtual measurements
		float	max_anom_beam_frac;	// max allowed anomalous beams (expressed as a fraction of the total number of valid beams). The idea is that 
									//		if too many beams are anomalous, maybe the robot is lost

		// 'Normal' beams analysis
		float	skip_fraction, skip_fraction_fp;	// final part of the beam that has to be skipped during ray casting (for anomalous beams analysis and false positive detection, respectively)
		float	min_skip_length;					// min length of the skipped part of the beam during anomalous beams analysis
		double	m_hitSearch, q_hitSearch;			// parameters for computing the circle that has to be considered when searching for occupied cells
		int		N_skip_anom;							// number of beams that has to be skipped near to an anomalous beam

		bool	visualization_mode, debug_visualization;	// whether to compute rviz-related stuff

		/** Detect anomalous beams and return related indexes
		 * @param pose_world - robot pose in world coordinates
		 * @param pose_grid - robot pose in grid coordinates
		 * @return beam indexes */
		std::vector<int> findAnomalousMeasurements(const Vector2<double>& pose_world, const Vector2<int32_t>& pose_grid);

		/** Compute the hit point of the virtual beam with index idx
		 * @param idx - beam index (it's involved in the angle computation)
		 * @param pose_world - robot pose in world coordinates
		 * @param pose_grid - robot pose in grid coordinates */
		inline boost::optional<Vector2<double>> computeVirtualHitPoint(const int& idx, const Vector2<double>& pose_world, const Vector2<int32_t>& pose_grid);

		/** ---------------------------------------------------------------- 
		 * Update logic
		 * ----------------------------------------------------------------- */
		// Attributes related to the idle state induced by a quick rotation
		double				max_w;					// max angular velocity allowed
		double				w_idle_duration;		// an angular velocity greater than max_w puts the system in an idle state for at least w_idle_duration seconds
		ros::Time			w_idle_end;				// time at which the system exit from idle state
		bool				w_idle;					// whether the system is in idle state (induced by an angular velocity greater than max_w)
		geometry_msgs::Pose	w_idle_last_pose;		// robot pose when the idle state was triggered
		float				w_idle_squared_dist;	// min (squared) distance from w_idle_last_pose required in order to exit from idle state


		// Attributes related to normal conditions (new update is triggered based on elapsed time and traveled distance since last update)
		nav_msgs::Odometry	last_odom;			// a certain minimum displacement from this pose is required in order to process a new scan
		float				min_linear_dist_squared, min_angular_dist;	// minimum displacement required in order to set 'robot_moved_enough' to true
		bool				robot_moved_enough;	// whether the robot has moved enough since last scan processing. If true, a new scan MAY be processed
		bool				update_last_odom;	// whether a new odom pose has to be accepted (normally set to true if a scan has been processed)
		
		ros::Time	last_processed_time;	// time w.r.t. which the elapsed time is measured. It is normally given by the timestamp of the last processed scan
		bool 		enough_time_elapsed;	// whether enough time is elapsed since last scan processing. If true, a new scan MAY be processed
		double		min_elapsed_time;		// minimum amount of time that has to elapse in order to set 'enough_time_elapsed' to true


		// Functions 
		/** Reset the update logic
		 * @param time - time instant from which elapsed time will be measured after the reset */
		inline void resetUpdateLogic(const ros::Time& time);

		/** Odom subscriber callback. It's the update logic core. Read the msgs received on odom_topic and change the update flags accordingly
		 * @param msg */
		void odomCallback(const nav_msgs::Odometry& msg);

		/** ---------------------------------------------------------------- 
		 * Rviz functions
		 * ----------------------------------------------------------------- */

		visualization_msgs::Marker m_marker, v_marker, lines, unpaired_marker, evaluated_marker, occ_marker, free_marker;

		std_msgs::ColorRGBA	col_free, col_occ;

		/** Initialize marker objects
		 * @param fixed_frame */
		void initializeRvizStuff(const std::string& fixed_frame);

		/** Clear 'points' field of marker objects */
		inline void clearRvizMarkers();

		/** Clear 'points' field of marker objects used only for debugging purposes */
		inline void clearRvizDebugMarkers();

		/** Compute markers points and colors */
		void updateRvizMarkers();

		/** Publish marker messages
		 * @param timestamp */
		inline void publishRvizMarkers(const ros::Time& timestamp);

		/** Publish marker debug messages
		 * @param timestamp */
		inline void publishRvizDebugMarkers(const ros::Time& timestamp);

};