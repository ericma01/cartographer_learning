/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : map_builder.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月07日 星期五 14时11分03秒
* Description :
*******************************************************************************/
MapBuilder() {
	//创建两个指针对象（非指针对象自动创建）
	pose_graph_ = make_unique<PoseGraph2D>(
			make_unique<optimization::OptimizationProblem2D>(
				&thread_pool_));

	if (set_collate_by_trajectory) {
		sensor_collator_ = make_unique<sensor::TrajectoryCollator>();
	} else {
		sensor_collator_ = make_unique<sensor::Collator>();
	}
}

AddTrajectoryBuilder() {
	// 添加轨迹builder，需要前端builder、后端、前端回调等作为参数
	// 添加纯定位trimmer
	// 添加轨迹初始位姿
	// 添加轨迹options
	const int trajectory_id = trajectory_builders_.size(); //确定轨迹ID

	if(has_pose_graph_odometry_motion_filter) {
		// emplace MotionFilter
	}

	unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
	local_trajectory_builder = make_unique<LocalTrajectoryBuilder2D>();

	//这是关键哟
	trajectory_builders_.push_back(
			make_unique<CollatedTrajectoryBuilder>(
				&sensor_collator_,
				CreateGlobalTrajectoryBuilder2D( //wrapped_trajectory_builder TrajectoryBuilderInterface
					local_trajectory_builder,
					&pose_graph_,
					local_slam_result_callback,
					pose_graph_odometry_motion_filter)));

	MaybeAddPureLocalizationTrimmer(
			&pose_graph_);

	if(has_initial_trajectory_pose) {
		pose_graph_->SetInitialTrajectoryPose(
				trajectory_id, // 不同轨迹有不同的初始位姿
				initial_trajectory_pose,
				timestamp);
	}

	all_trajectory_builder_options_.push_back();

}

MaybeAddPureLocalizationTrimmer() {
	if(set_pure_localization) {
		pose_graph_->AddTrimmer(
				make_unique<PureLocalizationTrimmer>();
	} //Trimmer默认设置

	if(has_pure_localization_trimmer) {
		pose_graph_->AddTrimmer(
				make_unique<PureLocalizationTrimmer>();
	} //Trimmer自定义配置
}

CreateGlobalTrajectoryBuilder2D() {
	return make_unique<GlobalTrajectoryBuilder<LocalTrajectory2D, mapping::PoseGraph2D>> (
			local_trajectory_builder,
			&pose_graph_,
			local_slam_result_callback,
			pose_graph_odometry_motion_filter);
}

FinishTrajectory() {
	sensor_collator_->FinishTrajectory();
	pose_graph_->FinishTrajectory();
}

LoadState() {
	proto::PoseGraph pose_graph_proto;

	std::map<int, int> trajectory_remapping; //trajectory_id_proto and new_trajectory_id
	for(int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
		const int new_trajectory_id = AddTrajectoryForDeserialization();

		pose_graph_->FreezeTrajectory();
	}

	//设置约束中节点和子图的轨迹id为新的轨迹id
	for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id();
    constraint_proto.mutable_node_id()->set_trajectory_id();
  }

	//从pose_graph_proto中读取下列对象
	MapById<SubmapId, transform::Rigid3d> submap_poses;
	MapById<NodeId, transform::Rigid3d> node_poses;


	SerializedData proto;
	//遍历ProtoStreamDeserializer中的全部数据
	while(has_proto) {
		if(has_PoseGraph) LOG(ERROR);
		if(has_AllTrajectoryBuilderOptions) LOG(ERROR);
		if(has_Submap) {
			pose_graph_->AddSubmapFromProto();
		}
		if(has_Node) {
			pose_graph_->AddNodeFromProto();
		}
		if(has_TrajectoryData) {
			pose_graph_->SetTrajectoryDataFromProto();
		}
		if(has_imu_odometry_fixedframepose_landmark_data) {
			sensor::FromProto();
		}
	}

	if(load_frozen_state_trajectory) {
		for( auto& constraint : pose_graph_proto)
			pose_graph_->AddNodeToSubmap();
	} else {
		pose_graph_->AddSerializedConstraints();
	}

}






