/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : global_trajectory_builder.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月08日 星期六 14时46分59秒
* Description : 
*******************************************************************************/
// 这是一个模板类，需要指明LocalTrajectoryBuilder和PoseGraph，同时继承于TrajectoryBuilderInterface
//
CreateGlobalTrajectoryBuilder2D() {
	return make_unique<GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
			local_trajectory_builder,
			trajectory_id,
			&pose_graph,
			local_slam_result_callback, //From map_builder_.AddTrajectoryBuilder()
			pose_graph_odometry_motion_filter);
}

AddSensorData() {	// TimedPointCloudData
	auto matching_result = local_trajectory_builder_->AddRangeData();

	// metric things
	kLocalSlamMatchingResults->Increment();

	if(matching_result->insertion_result != nullptr) {
		//metric things
		kLocalSlamInsertionResults->Increment();

		auto node_id = pose_graph_->AddNode();
		auto insertion_result = make_unique<InsertionResult>();
	}

	//local slam result callback things
	local_slam_result_callback_(
			trajectory,
			time,
			local_pose,
			range_data_in_local,
			insertion_result);
}

AddSensorData() { // imu
	local_trajectory_builder_->AddImuData();
	pose_graph_->AddImuData();
}

AddSensorData() { // odometry_data
	local_trajectory_builder_->AddOdometryData();

	if(pose_graph_odometry_motion_filter_.value() == odometry_data.pose) return;

	pose_graph_->AddOdometryData();
}

AddSensorData() { // fixed_frame_pose_data
	pose_graph_->AddFixedFramePoseData();
	
}

AddSensorData() { // landmark_data
	pose_graph_->AddLandmarkData();
}

AddLocalSlamResultData() { //mapping::LocalSlamResultData
	local_slam_result_data->AddToPoseGraph();
}
