/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : local_trajectory_builder_2d.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月07日 星期五 19时50分28秒
* Description : 
*******************************************************************************/
LocalTrajectoryBuilder2D() {
	ActiveSubmaps2D active_submaps_;
	scan_matching::RealTimeCorrelativeScanMatcher2D real_time_correlative_scan_matcher_;
	MotionFilter motion_filter_;
	scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
	RangeDataCollator range_data_collator_;
}

InitializeExtrapolator() {
	// 收到imu或者laser的数据的时候会调用
	// 位姿器为空的时候会执行
	extrapolator_ = make_unique<PoseExtrapolator>(
			pose_queue_duration,
			imu_gravity_time_constant);
	extrapolator_->AddPose();
}

AddOdometryData() {
	extrapolator_->AddOdometryData();
}

AddImuData() {
	InitializeExtrapolator(imu_data.time);
	extrapolator_->AddImuData();
}

AddRangeData() {
	// 参数是未同步时间的带戳点云数据
	auto synchronized_data = range_data_collator_.AddRangeData();

	if(do_not_use_imu_data) InitializeExtrapolator(synchronized_data.time);

	// 位姿器的最后位姿的时间大于激光数据的第一个点的时间，所以无法提供初始位姿
	if(the_fisrt_point_time_of_synchronized_data < exptrapolator_->GetLastPoseTime()) return nullptr;

	std::vector<transform::Rigid3f> range_data_poses; // 记录每个激光雷达点的位姿
	for(auto& range : synchronized_data.ranges) {
		if(time_of_range_point < extrapolator_->GetLastExtrapolatedTime()) {
			time_of_range_point	= extrapolator_->GetLastExtrapolatedTime();
		}
		range_data_poses.push_back(extrapolator_->ExtrapolatePose();
	}

	for(auto& point : synchronized_data) {
		if(the_range_of_point < options_.max_range()){
			accumulated_range_data_.returns.push_back();
		} else {
			accumulated_range_data_.misses.push_back();
		}
	}

	++num_accumulated_;

	if(num_accumulated_ >= options_.num_accumulated_range_data()) {
		auto sensor_duration = current_sensor_time - last_sensor_time_;
		last_sensor_time_ = current_sensor_time;
		num_accumulated_ = 0;
		auto gravity_alignment = extrapolator_->EstimateGravityOrientation();
		accumulated_range_data_.origin = range_data_poses.back();
		return AddAccumulatedRangeData(
				time,
				TransfromToGravityAlignedFrameAndFilter(),
				gravity_alignment,
				sensor_duration);
	}
	return nullptr;
}

AddAccumulatedRangeData() {
	auto& non_gravity_aligned_pose_rediction = extrapolator_->ExtrapolatePose();
	auto& pose_prediction = transfrom::Project2D();
	auto& filtered_gravity_aligned_point_cloud = sensor::AdaptiveVoxelFilter();

	auto& pose_estimate_2d = ScanMatch(
			time,
			pose_prediction,
			filtered_gravity_aligned_point_cloud);

	auto& pose_estimate = transform::Embed3D();
	extrapolator_->AddPose(time, pose_estimate);

	auto range_data_in_local = TransfromRangeData(gravity_aligned_range_data, pose_estimate_2d);

	auto insertion_result = InsertIntoSubmap();

	// metric things
	// last_wall_time_
	// last_thread_cpu_time_seconds_
	
	return make_unique<MatchingResult>(
			time,
			pose_estimate,
			range_data_in_local,
			insertion_result);
}

ScanMatch() {
	if(active_submaps_.submaps().empty()) return pose_prediction;

	auto& matching_submap = active_submaps_.submaps().front();
	auto initial_ceres_pose = pose_prediction;

	if(user_online_correlative_scan_matching)
		auto score = real_time_correlative_scan_matcher_.Match();

	auto pose_observation;
	ceres::Solver::Summary summary;
	ceres_scan_matcher_.Match();

	if(pose_observation) {
		//metric things
	}
	return pose_observation;
}

InsertIntoSubmap() {
	if(motion_filter_.IsSimilar()) return nullptr;

	auto insertion_submaps = active_submaps_.InsertRangeData(range_data_in_local);

	return make_unique<InsertionResult>(
			make_shared<TrajectoryNode::Data(time, gravity_alignment, filtered_gravity_aligned_point_cloud, pose_estimate),
			insertion_submaps);
}
