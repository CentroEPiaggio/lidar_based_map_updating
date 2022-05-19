#include "map_change_detection/map_change_detection.hpp"
#include "map_change_detection/efficient_cell_store.hpp"

#include <map_change_detection/ChangedCells.h>

#include <tf2/LinearMath/Quaternion.h>
#include <numeric>

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

MapChangeDetection::MapChangeDetection(ros::NodeHandle* nodehandle) :	nh_(*nodehandle), update_last_odom(true), robot_moved_enough(false),
																		enough_time_elapsed(false), w_idle(false)
{
	std::string	map_topic, map_frame, odom_topic;
	double		min_linear_dist, search_width_ang, v_range_tol, w_idle_dist, range1, range2, pair_dist1, pair_dist2, radius1, radius2;

	ROS_INFO("Calling node constructor...");

	// Retrieve parameters
	nh_.getParam("/map_topic", map_topic);
	nh_.getParam("odom_topic", odom_topic);
	nh_.getParam("map_frame", map_frame);

	nh_.getParam("visualize_on_rviz", visualization_mode);
	nh_.getParam("visualize_debug_stuff", debug_visualization);

	nh_.getParam("scan_update_interval", min_elapsed_time);
	nh_.getParam("min_linear_displacement", min_linear_dist);
	nh_.getParam("min_angular_displacement", min_angular_dist);
	min_linear_dist_squared = min_linear_dist * min_linear_dist;
	min_angular_dist = min_angular_dist * M_PI / 180.0;

	nh_.getParam("virtual_to_measured_beam_ratio", vm_ratio);
	nh_.getParam("min_search_window_halfwidth_degrees", search_width_ang);

	nh_.getParam("max_range_tol", v_range_tol);

	nh_.getParam("max_trusted_angular_velocity", max_w);
	nh_.getParam("min_idle_duration", w_idle_duration);
	nh_.getParam("min_idle_distance", w_idle_dist);
	w_idle_squared_dist = w_idle_dist * w_idle_dist;
	max_w *= M_PI / 180;	// from deg to rad

	nh_.getParam("range_1_pair", range1);
	nh_.getParam("pairing_distance_1", pair_dist1);
	nh_.getParam("range_2_pair", range2);
	nh_.getParam("pairing_distance_2", pair_dist2);
	nh_.getParam("pairing_distance_saturation", dist_saturation);
	m_pairing = (pair_dist1 - pair_dist2) / (range1 - range2);
	q_pairing = pair_dist1 - m_pairing * range1;
	
	nh_.getParam("final_chunk_skip_fraction", skip_fraction);
	nh_.getParam("skip_fraction_false_positive", skip_fraction_fp);
	nh_.getParam("min_chunk_skip_length", min_skip_length);
	if (skip_fraction < 0 || skip_fraction > 1) {
		ROS_WARN("Parameter 'final_chunk_skip_fraction' has to be greater than 0 and smaller than 1. %f was given. Setting it to 0.1...", skip_fraction);
		skip_fraction = 0.1;
	}
	if (skip_fraction_fp < 0 || skip_fraction_fp > 1) {
		ROS_WARN("Parameter 'skip_fraction_false_positive' has to be greater than 0 and smaller than 1. %f was given. Setting it to 0.1...", skip_fraction_fp);
		skip_fraction_fp = 0.1;
	}

	nh_.getParam("range_1_search", range1);
	nh_.getParam("pairing_distance_1", radius1);
	nh_.getParam("range_2_search", range2);
	nh_.getParam("pairing_distance_2", radius2);
	m_hitSearch = (radius1 - radius2) / (range1 - range2);
	q_hitSearch = radius1 - m_hitSearch * range1;

	nh_.getParam("N_skip_anomalous", N_skip_anom);

	nh_.getParam("max_anomalous_beam_fraction", max_anom_beam_frac);

	sub_odom = nh_.subscribe(odom_topic, 1, &MapChangeDetection::odomCallback, this);  

	nh_.getParam("update_map_online", online_update);

	update_map_service = nh_.advertiseService("update_map", &MapChangeDetection::serviceCallback, this);  

	if (visualization_mode || debug_visualization)
		initializeRvizStuff(map_frame);

	// Create grid object
	while(ros::ok()){
		boost::shared_ptr<nav_msgs::OccupancyGrid const> map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, ros::Duration(4.0));
		if(map){
			ROS_INFO("Got map! Creating grid object...");
			grid.reset(new Grid(nodehandle, *map));
			break;
		} else {
			ROS_INFO("Waiting for map...");
		}
	}

	// Create scan handler object
	scan_handler.reset(new ScanHandler(nodehandle, map_frame));

	last_processed_time = scan_handler->getScan().header.stamp;

	v_max_range = v_range_tol + scan_handler->getMaxBeamRange();

	search_width_ang = search_width_ang * M_PI / 180; 	// deg to rad
	double v_angle_increment = scan_handler->getScan().angle_increment / vm_ratio;
	search_window = ceil(search_width_ang / v_angle_increment);
	ROS_INFO("Virtual angle increment is %.4f deg. Search width is %d steps and %.4f deg", v_angle_increment / M_PI * 180, search_window, search_window * v_angle_increment / M_PI * 180);
	ROS_ASSERT_MSG(search_window >= 0, "Invalid search width! Requested min search width is %f. Scan angle increment is %f. Ratio between virtual and real\
					 measurement is %d. Resulting search width is %d", search_width_ang, scan_handler->getScan().angle_increment, vm_ratio, search_window);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

bool MapChangeDetection::serviceCallback(map_change_detection::UpdateMap::Request& request, map_change_detection::UpdateMap::Response& response) {
	ROS_INFO("Service called! Updating the map...");
	map_change_detection::ChangedCells	msg;
	for(int i = 0; i < grid->getWidth(); i++) {
		for(int j = 0; j < grid->getHeight(); j++) {
			if(grid->isChanged(i, j)) {
				if(grid->getProbOcc(i,j) == 100)
					msg.toFree.push_back(j * grid->getWidth() + i);
				else if(grid->getProbOcc(i,j) == 0)
					msg.toOcc.push_back(j * grid->getWidth() + i);
			}
		}
	}
	msg.header.stamp = ros::Time::now();
	grid->publishChangedCells(msg);
	ROS_INFO("...Done! Cells changed to free = %lu, cells changed to occ = %lu, total map cells = %u", msg.toFree.size(), msg.toOcc.size(), grid->getWidth() * grid->getHeight());
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::initializeRvizStuff(const std::string& fixed_frame) {
	pub_marker = nh_.advertise<visualization_msgs::Marker>("/marker", 10, true);

	if(debug_visualization) {
		m_marker.header.frame_id = fixed_frame;
		m_marker.ns = "hit_visualization";
		m_marker.action = visualization_msgs::Marker::ADD;
		m_marker.id = 0;
		m_marker.type = visualization_msgs::Marker::SPHERE_LIST;
		m_marker.pose.orientation.w = 1;
		m_marker.scale.x = 0.15;
		m_marker.scale.y = 0.15;
		m_marker.scale.z = 0.15;
		m_marker.color.r = 1.0;
		m_marker.color.g = 0.8;
		m_marker.color.b = 0.2;
		m_marker.color.a = 0.4;

		v_marker.header.frame_id = fixed_frame;
		v_marker.ns = "hit_visualization";
		v_marker.action = visualization_msgs::Marker::ADD;
		v_marker.id = 1;
		v_marker.type = visualization_msgs::Marker::SPHERE_LIST;
		v_marker.pose.orientation.w = 1;
		v_marker.scale.x = 0.15;
		v_marker.scale.y = 0.15;
		v_marker.scale.z = 0.15;
		v_marker.color.r = 0.0;
		v_marker.color.g = 1.0;
		v_marker.color.b = 0.0;
		v_marker.color.a = 0.4;

		lines.header.frame_id = fixed_frame;
		lines.ns = "hit_visualization";
		lines.action = visualization_msgs::Marker::ADD;
		lines.id = 2;
		lines.type = visualization_msgs::Marker::LINE_LIST;
		lines.pose.orientation.w = 1;
		lines.scale.x = 0.03;
		lines.color.r = 1.0;
		lines.color.g = 0.0;
		lines.color.b = 0.0;
		lines.color.a = 1.0;

		unpaired_marker.header.frame_id = fixed_frame;
		unpaired_marker.ns = "hit_visualization";
		unpaired_marker.action = visualization_msgs::Marker::ADD;
		unpaired_marker.id = 3;
		unpaired_marker.type = visualization_msgs::Marker::SPHERE_LIST;
		unpaired_marker.pose.orientation.w = 1;
		unpaired_marker.scale.x = 0.25;
		unpaired_marker.scale.y = 0.25;
		unpaired_marker.scale.z = 0.25;
		unpaired_marker.color.r = 1.0;
		unpaired_marker.color.g = 0.8;
		unpaired_marker.color.b = 0.2;
		unpaired_marker.color.a = 1.0;

		evaluated_marker.header.frame_id = fixed_frame;
		evaluated_marker.ns = "hit_visualization";
		evaluated_marker.action = visualization_msgs::Marker::ADD;
		evaluated_marker.id = 4;
		evaluated_marker.type = visualization_msgs::Marker::SPHERE_LIST;
		evaluated_marker.pose.orientation.w = 1;
		evaluated_marker.scale.x = 0.25;
		evaluated_marker.scale.y = 0.25;
		evaluated_marker.scale.z = 0.25;
		evaluated_marker.color.r = 0.0;
		evaluated_marker.color.g = 1.0;
		evaluated_marker.color.b = 0.0;
		evaluated_marker.color.a = 1.0;
	}

	if(visualization_mode) {
		occ_marker.header.frame_id = fixed_frame;
		occ_marker.ns = "to_occupied_cell_transitions";
		occ_marker.action = visualization_msgs::Marker::ADD;
		occ_marker.id = 0;
		occ_marker.type = visualization_msgs::Marker::POINTS;
		occ_marker.pose.orientation.w = 1;
		occ_marker.scale.x = 0.05;
		occ_marker.scale.y = 0.05;
		occ_marker.color.r = 1.0;
		occ_marker.color.g = 0.0;
		occ_marker.color.b = 0.0;
		occ_marker.color.a = 1.0;

		free_marker.header.frame_id = fixed_frame;
		free_marker.ns = "to_free_cell_transitions";
		free_marker.action = visualization_msgs::Marker::ADD;
		free_marker.id = 1;
		free_marker.type = visualization_msgs::Marker::POINTS;
		free_marker.pose.orientation.w = 1;
		free_marker.scale.x = 0.05;
		free_marker.scale.y = 0.05;
		free_marker.color.r = 0.0;
		free_marker.color.g = 1.0;
		free_marker.color.b = 0.0;
		free_marker.color.a = 1.0;

		col_free.g = 1.0;
		col_occ.r = 1.0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::clearRvizMarkers() {
	free_marker.points.clear();
	occ_marker.points.clear();

	free_marker.colors.clear();
	occ_marker.colors.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::clearRvizDebugMarkers() {
	m_marker.points.clear();
	v_marker.points.clear();
	unpaired_marker.points.clear();
	evaluated_marker.points.clear();
	lines.points.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::updateRvizMarkers() {
	uint32_t	counter;
	Vector2<double>			world_point;
	geometry_msgs::Point	p;
	p.z = 0;

	std::vector<std::vector<Vector2<int32_t>>> tmp = grid->toFree.getCells();
	for (auto cells : tmp) {
		for (auto cell : cells) {
			// counter = grid->getFreeCounter(cell);
			counter = grid->countChangedFlags(cell);
			if (counter > 0) {
				world_point = grid->gridToWorld(cell);
				p.x = world_point.getX();
				p.y = world_point.getY();
				free_marker.points.push_back(p);
				col_free.a = counter /(double)grid->getBuffersSize();
				free_marker.colors.push_back(col_free);
			}
		}
	}

	tmp = grid->toOcc.getCells();
	for (auto cells : tmp) {
		for (auto cell : cells) {
			// counter = grid->getOccCounter(cell);
			counter = grid->countChangedFlags(cell);
			if (counter > 0) {
				world_point = grid->gridToWorld(cell);
				p.x = world_point.getX();
				p.y = world_point.getY();
				occ_marker.points.push_back(p);
				col_occ.a = counter /(double)grid->getBuffersSize();
				occ_marker.colors.push_back(col_occ);
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::publishRvizMarkers(const ros::Time& timestamp) {
	free_marker.header.stamp = timestamp;
	occ_marker.header.stamp = timestamp;

	pub_marker.publish(free_marker);
	pub_marker.publish(occ_marker);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::publishRvizDebugMarkers(const ros::Time& timestamp) {
	m_marker.header.stamp = timestamp;
	v_marker.header.stamp = timestamp;
	unpaired_marker.header.stamp = timestamp;
	evaluated_marker.header.stamp = timestamp;
	lines.header.stamp = timestamp;

	pub_marker.publish(v_marker);
	pub_marker.publish(m_marker);
	pub_marker.publish(unpaired_marker);
	pub_marker.publish(evaluated_marker);
	pub_marker.publish(lines);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::checkElapsedTime() {
	if (!enough_time_elapsed) {
		if (ros::Time::now().toSec() - last_processed_time.toSec() >= min_elapsed_time) {
			enough_time_elapsed = true;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::resetUpdateLogic(const ros::Time& time) {

	// if scan saved in scan_handler is going to be processed (or discarded), is no longer 'new'
	scan_handler->setNewScanFlag(false);

	// time taken as input is considered in order to trigger the next transition of 'enough_time_elapsed' flag
	last_processed_time = time;
	enough_time_elapsed = false;
	
	// new odom pose has to be retrieved, and the robot has to move in order to trigger the next scan processing
	update_last_odom = true;
	robot_moved_enough = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

boost::optional<Vector2<double>> MapChangeDetection::computeVirtualHitPoint(const int& idx, const Vector2<double>& pose_world, const Vector2<int32_t>& pose_grid) {
					
	double th = idx * scan_handler->getScan().angle_increment / vm_ratio + scan_handler->getScan().angle_min + scan_handler->getPose().theta;
	Vector2<double> rotation(cos(th), sin(th));
	Vector2<double> max_hitPoint = Vector2<double>::sum(rotation.scale(v_max_range), pose_world);
	boost::optional<Vector2<int32_t>> hitOptional = grid->virtualRayCasting(pose_grid, grid->worldToGrid(max_hitPoint));
	if (hitOptional)
		return grid->gridToWorld(hitOptional.get());
	else
		return boost::none;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<int> MapChangeDetection::findAnomalousMeasurements(const Vector2<double>& pose_world, const Vector2<int32_t>& pose_grid) {

	Vector2<double>	rotation;
	std::vector<boost::optional<Vector2<double>>>	v_hitPoints;
	std::vector<int>								anomalous_idxs;
	std::vector<std::vector<Vector2<double>>>		tried_pairing_points;
	std::vector<Vector2<double>>					tmp_points;
	double	dist_squared, th;
	bool	pairing_found;
	int		idx;	// temporary index

	// boost optional has fixed size. If a measurement is invalid, the corresponding optional will be empty
	v_hitPoints.resize(vm_ratio * scan_handler->getHitPoints().size());

	std::vector<bool> v_hitPoint_computed (v_hitPoints.size(), false);	//true if the virtual hit point at index i has been computed

	for (int i = 0; i < scan_handler->getHitPoints().size(); i++) {
		int j = i * vm_ratio;
		// central beam comparison:
		if (scan_handler->isValidMeasurement(i)) {
			double threshold = std::min(scan_handler->getScan().ranges.at(i) * m_pairing + q_pairing, dist_saturation);
			threshold *= threshold;
			if (!v_hitPoint_computed.at(j)) {
				v_hitPoint_computed.at(j) = true;
				v_hitPoints.at(j) = computeVirtualHitPoint(j, pose_world, pose_grid);
			}
			if (v_hitPoints.at(j)) {
				dist_squared = Vector2<double>::squaredDistance(scan_handler->getHitPoints().at(i), v_hitPoints.at(j).get());
				if (dist_squared < threshold)
					continue;
			}

			pairing_found = false;
			for (int k = 1; k <= search_window; k++) {
				// check beam from one side
				if (j - k >= 0)					// check out of range index
					idx = j - k;
				else
					idx = v_hitPoints.size() + j - k;	// assuming a 360 lidar. If it's not the case, the following two 'if' should be skipped
				
				if (!v_hitPoint_computed.at(idx)) {
					v_hitPoint_computed.at(idx) = true;
					v_hitPoints.at(idx) = computeVirtualHitPoint(idx, pose_world, pose_grid);
				}
				if (v_hitPoints.at(idx)) {
					dist_squared = Vector2<double>::squaredDistance(scan_handler->getHitPoints().at(i), v_hitPoints.at(idx).get());
					if (dist_squared < threshold) {
						pairing_found = true;
						break;
					}
				}
				// check beam from the other side
				if (j + k < v_hitPoints.size())	// check out of range index
					idx = j + k;
				else
					idx = j + k - v_hitPoints.size();	// assuming a 360 lidar. If it's not the case, the following two 'if' should be skipped
				
				if (!v_hitPoint_computed.at(idx)) {
					v_hitPoint_computed.at(idx) = true;
					v_hitPoints.at(idx) = computeVirtualHitPoint(idx, pose_world, pose_grid);
				}
				if (v_hitPoints.at(idx)) {
					dist_squared = Vector2<double>::squaredDistance(scan_handler->getHitPoints().at(i), v_hitPoints.at(idx).get());
					if (dist_squared < threshold) {
						pairing_found = true;
						break;
					}
				}
			}
			if (!pairing_found) {
				anomalous_idxs.push_back(i);

				if (debug_visualization) {
					if (v_hitPoints.at(j))
						tmp_points.push_back(v_hitPoints.at(j).get());

					for (int k = 1; k <= search_window; k++) {
						if (j - k >= 0)
							idx = j - k;
						else
							idx = v_hitPoints.size() + j - k;
						
						if (v_hitPoints.at(idx))
							tmp_points.push_back(v_hitPoints.at(idx).get());
						
						if (j + k < v_hitPoints.size())
							idx = j + k;
						else
							idx = j + k - v_hitPoints.size();
						
						if (v_hitPoints.at(idx))
							tmp_points.push_back(v_hitPoints.at(idx).get());
					}
					tried_pairing_points.push_back(tmp_points);
					tmp_points.clear();
				}
			}
		}
	}

	// Outlier rejection: anomalous beams are inliers only if there are at least 3 consecutive anomalous beams
	if (anomalous_idxs.size() < 3) {
		anomalous_idxs.clear();
		if (debug_visualization)
			tried_pairing_points.clear();
	} else {
		std::vector<bool> outliers (anomalous_idxs.size(), true);	// element i is true if the beam with index anomalous_idxs.at(i) is an outlier
		int	next, prev;
		for (int curr = anomalous_idxs.size() - 2; curr > 0; curr--) {
			next = curr + 1;
			prev = curr - 1;
			if ((anomalous_idxs.at(next) - anomalous_idxs.at(curr)) == 1 && (anomalous_idxs.at(curr) - anomalous_idxs.at(prev)) == 1) {
				outliers.at(curr) = false;
				outliers.at(next) = false;
				outliers.at(prev) = false;
			}
		}
		// handle vector begin:
		prev = anomalous_idxs.size() - 1;
		if ((anomalous_idxs.at(1) - anomalous_idxs.at(0)) == 1 && (anomalous_idxs.at(0) + scan_handler->getHitPoints().size() - anomalous_idxs.at(prev)) == 1) {
				outliers.at(0) = false;
				outliers.at(1) = false;
				outliers.at(prev) = false;
		}
		// handle vector end:
		int curr = anomalous_idxs.size() - 1;
		prev = curr - 1;
		if ((anomalous_idxs.at(0) + scan_handler->getHitPoints().size() - anomalous_idxs.at(curr)) == 1 && (anomalous_idxs.at(curr) - anomalous_idxs.at(prev)) == 1) {
				outliers.at(curr) = false;
				outliers.at(next) = false;
				outliers.at(prev) = false;
		}
		// delete outliers:
		for (int i = anomalous_idxs.size() - 1; i >= 0; i--) {
			if (outliers.at(i)) {
				anomalous_idxs.erase(anomalous_idxs.begin() + i);
				if (debug_visualization)
					tried_pairing_points.erase(tried_pairing_points.begin() + i);
			}
		}
	}

	if (debug_visualization) {

		clearRvizDebugMarkers();

		geometry_msgs::Point	p, v_p;
		p.z = 0;
		v_p.z = 0;

		// Update markers
		for (int i = 0; i < scan_handler->getHitPoints().size(); i++) {
			if (scan_handler->isValidMeasurement(i)) {
				p.x = scan_handler->getHitPoints().at(i).getX();
				p.y = scan_handler->getHitPoints().at(i).getY();
				m_marker.points.push_back(p);
			}
		}
		for (auto v_hit : v_hitPoints) {
			if (v_hit) {
				p.x = v_hit.get().getX();
				p.y = v_hit.get().getY();
				v_marker.points.push_back(p);
			}
		}
		for (int i = 0; i < anomalous_idxs.size(); i++) {
			p.x = scan_handler->getHitPoints().at(anomalous_idxs.at(i)).getX();
			p.y = scan_handler->getHitPoints().at(anomalous_idxs.at(i)).getY();
			unpaired_marker.points.push_back(p);
			for (int j = 0; j < tried_pairing_points.at(i).size(); j++) {
				v_p.x = tried_pairing_points.at(i).at(j).getX();
				v_p.y = tried_pairing_points.at(i).at(j).getY();
				lines.points.push_back(p);
				lines.points.push_back(v_p);
				evaluated_marker.points.push_back(v_p);
			}
		}
		publishRvizDebugMarkers(scan_handler->getScan().header.stamp);
	}

	return anomalous_idxs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::detectChanges() {

	double start_time, end_time, tmp_start, time_find, time_normal, time_anom;
	start_time = ros::Time::now().toSec();

	resetUpdateLogic(scan_handler->getScan().header.stamp);

	if (scan_handler->getValidBeams() <= 0) {
		ROS_WARN("Skipping scan with 0 valid beams");
		return;
	}

	Vector2<double>	world_pose;
	world_pose.setX(scan_handler->getPose().x);
	world_pose.setY(scan_handler->getPose().y);

	Vector2<int32_t>		grid_pose = grid->worldToGrid(world_pose);

	tmp_start = ros::Time::now().toSec();
	std::vector<int>	anomalous_idxs = findAnomalousMeasurements(world_pose, grid_pose);
	time_find = ros::Time::now().toSec() - tmp_start;

	if (anomalous_idxs.size() > max_anom_beam_frac * scan_handler->getValidBeams()) {
		ROS_WARN("Too many anomalous beams! Is the robot lost? Skipping map change detection...");
		return;
	}

	// Anomalous points analysis: detect cell transistions that explain the measured range
	tmp_start = ros::Time::now().toSec();
	std::vector<bool> column (grid->getHeight(), false);
	std::vector<std::vector<bool>>	seen_occ(grid->getWidth(), column), seen_free(grid->getWidth(), column), seen_fp_occ(grid->getWidth(), column), seen_fp_free(grid->getWidth(), column);
	Vector2<double>		fake_hitPoint;
	Vector2<int32_t>	grid_m_hitPoint;
	for (auto idx : anomalous_idxs) {
		// search for cells changed to occupied
		grid_m_hitPoint = grid->worldToGrid(scan_handler->getHitPoints().at(idx));
		if (grid->isValidGridIndex(grid_m_hitPoint)) {
			int32_t	Ncell = ceil((m_hitSearch * scan_handler->getScan().ranges.at(idx) + q_hitSearch) / grid->getResolution());
			if (!grid->neighborsOccupied(grid_m_hitPoint, Ncell)) {
				if(!seen_occ.at(grid_m_hitPoint.getX()).at(grid_m_hitPoint.getY())) {
					// grid->incrementOccCounter(grid_m_hitPoint);
					grid->insertChangedFlag(grid_m_hitPoint);
					seen_occ.at(grid_m_hitPoint.getX()).at(grid_m_hitPoint.getY()) = true;
					// if (grid->getOccCounter(grid_m_hitPoint) == 1)
					// 	grid->toOcc.insertElement(grid_m_hitPoint);
					if (grid->countChangedFlags(grid_m_hitPoint) == 1 && online_update)
						grid->toOcc.insertElement(grid_m_hitPoint);
				}
			}
		}

		// search for cells changed to free
		if (skip_fraction * scan_handler->getScan().ranges.at(idx) >= min_skip_length)
			fake_hitPoint = Vector2<double>::sum(world_pose.scale(skip_fraction), scan_handler->getHitPoints().at(idx).scale(1 - skip_fraction));
		else {
			double tmp_skip = min_skip_length / scan_handler->getScan().ranges.at(idx);
			if (tmp_skip < 0)
				tmp_skip = 0;
			else if (tmp_skip > 1)
				tmp_skip = 1;
			fake_hitPoint = Vector2<double>::sum(world_pose.scale(tmp_skip), scan_handler->getHitPoints().at(idx).scale(1 - tmp_skip));
		}
		std::vector<Vector2<int32_t>> tmp_cells = grid->rayCasting(grid_pose, grid->worldToGrid(fake_hitPoint), true);
		for (auto cell : tmp_cells) {
			if(grid->getProbOcc(cell) == 100 && !seen_free.at(cell.getX()).at(cell.getY())) {
				// grid->incrementFreeCounter(cell);
				grid->insertChangedFlag(cell);
				seen_free.at(cell.getX()).at(cell.getY()) = true;
				// if (grid->getFreeCounter(cell) == 1)
				// 	grid->toFree.insertElement(cell);
				if (grid->countChangedFlags(cell) == 1 && online_update)
						grid->toFree.insertElement(cell);
			}
		}
	}
	time_anom = ros::Time::now().toSec() - tmp_start;

	tmp_start = ros::Time::now().toSec();
	// 'Normal' points analysis: search for false positives. Beams adjacent to anomalous beams are skipped
	int idx = 0;
	for (int i = 0; i < scan_handler->getHitPoints().size(); i++) {
		if (scan_handler->isValidMeasurement(i)) {
			if ((idx >= anomalous_idxs.size()) || (i < (anomalous_idxs.at(idx) - N_skip_anom))) {
				// not anomalous beam
				auto grid_hitPoint = grid->worldToGrid(scan_handler->getHitPoints().at(i));

				// check for cells wrongly judged as changed to free
				std::vector<Vector2<int32_t>> tmp_cells = grid->getAdjacentCells(grid_hitPoint);
				for (auto cell : tmp_cells) {
					// if (!seen_fp_free.at(cell.getX()).at(cell.getY()) && grid->getFreeCounter(cell) > 0) {
					// 	grid->decrementFreeCounter(cell);
					// 	seen_fp_free.at(cell.getX()).at(cell.getY()) = true;
					// 	if (grid->getFreeCounter(cell) == 0)
					// 		grid->toFree.removeElement(cell);
					// }
					if (!seen_fp_free.at(cell.getX()).at(cell.getY()) && grid->getProbOcc(cell) == 100) {
						grid->insertUnchangedFlag(cell);
						seen_fp_free.at(cell.getX()).at(cell.getY()) = true;
						if (grid->countChangedFlags(cell) == 0 && online_update)
							grid->toFree.removeElement(cell);
					}
				}

				// check for cells wrongly judged as changed to occupied
				fake_hitPoint = Vector2<double>::sum(world_pose.scale(skip_fraction_fp), scan_handler->getHitPoints().at(i).scale(1 - skip_fraction_fp));
				tmp_cells = grid->rayCasting(grid_pose, grid->worldToGrid(fake_hitPoint), false);
				for (auto cell : tmp_cells) {
					// if (!seen_fp_occ.at(cell.getX()).at(cell.getY()) && grid->getOccCounter(cell) > 0) {
					// 	grid->decrementOccCounter(cell);
					// 	seen_fp_free.at(cell.getX()).at(cell.getY()) = true;
					// 	if (grid->getOccCounter(cell) == 0)
					// 		grid->toOcc.removeElement(cell);
					// }

					if (!seen_fp_occ.at(cell.getX()).at(cell.getY())) {
						grid->insertUnchangedFlag(cell);
						seen_fp_occ.at(cell.getX()).at(cell.getY()) = true;
						if (grid->countChangedFlags(cell) == 0 && online_update)
							grid->toOcc.removeElement(cell);
					}
				}
			} else if (i == (anomalous_idxs.at(idx) + N_skip_anom)) {
				// skipping anomalous beam
				idx++;
			}
		}
	}
	time_normal = ros::Time::now().toSec() - tmp_start;

	double start_viz, end_viz, dt_viz;
	start_viz = ros::Time::now().toSec();
	if (visualization_mode) {
		clearRvizMarkers();

		updateRvizMarkers();
		
		publishRvizMarkers(scan_handler->getScan().header.stamp);
	}
	end_viz = ros::Time::now().toSec();
	dt_viz = end_viz - start_viz;

	// Time consumption monitoring
	end_time = ros::Time::now().toSec();
	static uint64_t counter;
	static double cum_time, max_time;
	if (first_time) {
		first_time = false;
		counter = 0;
		cum_time = 0.0;
		max_time = 0.0;
	}
	double dt = end_time - start_time;
	counter++;
	cum_time += dt;
	double mean_time = cum_time / counter;
	if (dt > max_time)
		max_time = dt;

	ROS_INFO("Total: %fs (find: %fs, anom: %fs, normal: %fs, plot: %fs). Mean: %fs; max: %fs", dt, time_find, time_anom, time_normal, dt_viz, mean_time, max_time);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

void MapChangeDetection::odomCallback(const nav_msgs::Odometry& msg) {

	if (abs(msg.twist.twist.angular.z) >= max_w) {
		ROS_INFO("Angular velocity too big! Going idle...");

		if(debug_visualization && !w_idle) {
			clearRvizDebugMarkers();
			publishRvizDebugMarkers(msg.header.stamp);
		}

		w_idle_end = msg.header.stamp + ros::Duration(w_idle_duration);
		w_idle_last_pose = msg.pose.pose;
		w_idle = true;
		resetUpdateLogic(msg.header.stamp);

		return;
	}

	if (w_idle) {
		// If enough time is elapsed...
		if (msg.header.stamp > w_idle_end) {
			double dx = msg.pose.pose.position.x - w_idle_last_pose.position.x;
			double dy = msg.pose.pose.position.y - w_idle_last_pose.position.y;
			// ...and enough distance has been traveled...
			if ((dx * dx + dy * dy) >= w_idle_squared_dist) {
				// ...reset the update logic one last time (one more is better than one less!) and exit from idle state
				ROS_INFO("Exiting from idle state!");
				resetUpdateLogic(msg.header.stamp);
				w_idle = false;
			}
		}
		return;
	}

	if (update_last_odom) {
		last_odom = msg;
		update_last_odom = false;
		return;
	}

	if (!robot_moved_enough) {
		double	dx, dy, dist_squared, dth;
		dx = last_odom.pose.pose.position.x - msg.pose.pose.position.x;
		dy = last_odom.pose.pose.position.y - msg.pose.pose.position.y;
		dist_squared = dx * dx + dy * dy;
		if (dist_squared >= min_linear_dist_squared) {
			robot_moved_enough = true;
		} else {
			// difference between two angles: https://stackoverflow.com/questions/1878907/how-can-i-find-the-difference-between-two-angles/52432897#52432897
			dth = M_PI - fabs(fmod(fabs(tf2::getYaw(last_odom.pose.pose.orientation) - tf2::getYaw(msg.pose.pose.orientation)), 2.0 * M_PI) - M_PI);
			if (dth >= min_angular_dist)
				robot_moved_enough = true;
		}
	}
}
