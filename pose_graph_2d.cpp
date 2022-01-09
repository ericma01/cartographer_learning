/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : pose_graph_2d.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月08日 星期六 16时27分36秒
* Description :
*******************************************************************************/
PoseGraph2D() {
	auto optimization_problem_;
	auto constraint_builder_;
	auto thread_pool_;

	if(has_overlapping_submaps_trimmer_2d)
		AddTrimmer(
				make_unique<OverlappingSubmapsTrimmer2D>);
}

InitializeGlobalSubmapPoses() {
	auto time;
	auto insertion_submaps;

	auto& submap_data = optimization_problem_->submap_data()
	if(insertion_submaps.size() == 1) {
		if(submap_data.SizeOfTrajectoryOrZero() == 0) {
			if(data_.initial_trajectory_poses.count() > 0) {
				data_.trajectory_connectivity_state.Connect();
			}
			optimization_problem_->AddSubmap(
					transform::Project2D(
						ComputeLocalToGlobalTransform()));
		}
		data_.submap_data.at(submap_id).submap = insertion_submaps.front();
		return submap_id;
	}

	if(insertion_submaps.size() == 2) {
		auto end_it = submap_data.EndOfTrajectory();
		auto last_submap_id = end_it->id;
		if(data_.submap_data.at(last_submap_id).submap == insertion_submaps.front()) {
			auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;
			optimization_problem_->AddSubmap();
			return {last_submap_id, SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
		}

		if(data_.submap_data.at(last_submap_id).submap == insertion_submaps.back()) {
			auto front_submap_id = SubmapId{trajectory_id, last_submap_id.submap_index - 1};

			if(data_.submap_data.at(front_submap_id).submap == insertion_submaps.front())
				return {front_submap_id, last_submap_id};
		}

	}
}

AddNode() {
	auto& constant_data;
	auto& trajectory_id;
	auto& insertion_submaps;

	auto optimized_pose(
			GetLocalToGlobalTransfrom() * constrant_data->local_pose);
	auto node_id = AppendNode();
	bool newly_finished_submap = insertion_submaps.front()->insertion_finished();

	AddWorkItem([=]() {
			return ComputeConstraintsForNode();
	});
	return node_id;
}

AddImuData() {
	AddWorkItem([=](){
		if(CanAddWorkItemModifying())
			optimization_problem_->AddImuData();
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddOdometryData() {
	AddWorkItem([=](){
		if(CanAddWorkItemModifying())
			optimization_problem_->AddOdometryData();
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddFixedFramePoseData() {
	AddWorkItem([=](){
		if(CanAddWorkItemModifying())
			optimization_problem_->AddFixedFramePoseData();
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddLandmarkData() {
	AddWorkItem([=](){
		if(CanAddWorkItemModifying()) {
			for(auto& observation : landmark_data.landkmark_observations) {
				data_.landmark_nodes[observation.id].landmark_observations.emplace_back();
			}
		}
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

ComupteConstraint() {
	auto node_id;
	auto submap_id;

	CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
	if(data_.submap_data.at(submap_id).submap->insertion_finished != true) return;

	auto node_time = GetLatestNodeTime();
	auto last_connection_time = data_.trajectory_connectivity_state.LastConnectionTime();

	if(node_id.trajectory_id == submap_id.trajectory_id ||
			node_time < last_connection_time + options_.global_constraint_search_after_n_seconds)
		auto maybe_add_local_constranint = true;
	else if(global_localization_smaplers_[node_id.trajectory_id]->Pluse())
		auto maybe_add_global_constraint = true;

	auto constrant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
	auto submap = data.submap_data.at(submap_id).submap.get();

	if(maybe_add_local_constraint) {
		auto initial_relative_pose = optimization_problem_->submap_data().at(submpa_id).global_pose.inverse() *
																 optimization_problem_->node_data().at(node_id).global_pose_id;
		constraint_builder_.MaybeAddConstraint();
	} else if(maybe_add_global_constraint) {
		constraint_builder_.MaybeAddGlobalConstraint();
	}
}

DeleteTrajectory() {
	auto it = data_.trajectories_state.find(trajectory_id);

	it->second.deletion_state = InternalTrajectoryState::DeletionState::SCHEDULED_FOR_DELETION;
	AddWorkItem([this, trajectory_id]() {
		// Firstly, CHECK something to make sure that this trajectory state is SCHEDULED_FOR_DELETION.

		// Secondly, change this trajectory state is WAIT_FOR_DELETION.
		data_.trajectories_state.at(trajectory_id).deletion_state = InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION;
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

FinishTrajectory() {
	AddWorkItem([this, trajectory_id]() {
		//Fistly, make sure that this trajectory is NOT finished.

		data_.trajectories_state[trajectory_id].state = TrajectoryState::FINISHED;
		for(auto& submap : data_.submap_data.trajectory(trajectory_id))
			data_.submap_data.at(submap.id).state = SubmapState::kFinished;
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

FreezeTrajectory() {
	data_.trajectory_connectivity_state.Add();
	AddWorkItem([this, trajectory_id]() {
		for(auto& entry : data_.trajectories_state) {
			if(data_.trajectory_connectivity_state.TransitivelyConnectd() continue; // these trajectories have connected

			data_.trajectory_connectivity_state.Connect();

		}
		data_.trajectories_state[trajectory_id].state = TrajectoryState::FROZEN;
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

DeleteTrajectoriesIfNeeded() {
	TrimmingHandle trimming_handle(this);
	for(auto& it : data_.trajectories_state) {
		if(it.second.deletion_state = InternalTrajectoryState::DeletionState::WAIT_FOR_DELETION) {
			auto submap_ids = trimming_handle.GetSubmapIds(it.first);
		}
		for (auto& submap_id : submap_ids) {
        trimming_handle.TrimSubmap(submap_id);
    }
    it.second.state = TrajectoryState::DELETED;
		it.second.deletion_state = InternalTrajectoryState::DeletionState::NORMAL;
	}
}

AppendNode() {
	AddTrajectoryIfNeeded(trajectory_id);
	if(!CanAddWorkItemModifying()) LOG(WARNING) << "This trajectory is finished or deleted and cannot add node.";

	auto node_id = data_.trajectory_nodes.Append(); //trajectory_id constant_data optimized_pose
	++data_.num_trajectory_nodes;

	if(data_.submap_data.SizeOfTrajectoryOrZero() == 0 || data_.submap_data.EndOfTrajectory()->data.submap != insertion_submaps.back() {
		auto submap_idd = data_.submap_data.Append(trajectory_id, InternalSubmapData());
		data_.submap_data.at(submap_id).submap = insertion_submaps.back();
		kActiveSubmapsMetric->Increment();
	}
	return node_id;
}

AddWorkItem() {
	if(work_queue_ == nullptr) {
		work_queue_ = make_unique<WorkQueue>();
		auto task = make_unique<common::Task>();
		task->SetWorkItem([this]() {
				DrainWorkQueue();
		});
		thread_pool->Schedule(task);
	}

	work_queue_->push_back({now_time, wori_item});
	//metric things
}

AddTrajectoryIfNeeded() {
	// CHECK to make sure trajectory state is not FINISHED/DELETED and deletion state is NORMAL.
	data_trajectory_connectivity_state.Add();

	if(has_no_smapler_of_this_trajectory) {
		global_loclaization_smaplers_[trajectory_id] = make_unique<common::FixedRatioSampler>();
	}
}

ComputeConstraintsForNode() {
	auto insertion_submaps;

	auto& constant_data = data_.trajectory_nodes.at(node_id).constant_data;
	auto submpa_ids = InitializeGlobalSubmapPoses();

	auto matching_id = submap_ids.front();
	auto local_pose_2d = transfrom::Project2D(constant_data->local_pose, constant_data->gravity_alignment.inverse());
	auto global_pose_2d = optimization_problem_->submap_data().at(matching_id).global_pose *
			 constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *
			 local_pose_2d;
	optimization_problem_->AddTrajectoryNode();

	for (size_t i = 0; i < insertion_submaps.size(); ++i) {
		auto submap_id = submap_ids[i];
		CHECK(data_.submap_data.at(submpa_id).state == SubmapState::kNoConstraintSearch);

		auto constranint_transform = constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() * local_pose_2d;
		data_.constraints.push_back();
	}

	for(auto submap_id_data : data_.submap_data) {
		if(submap_id_data.data.state == SubmapState::kFinished)
			finished_submap_ids.emplace_back();
	}

	if(newly_finished_submap) {
		auto newly_finished_submap_id = submpa_ids.front();
		auto finished_submap_data = data_.submap_data.at(newly_finished_submpa_id);
		finished_submap_data.state = SubmapState::kFinished;
		newly_finished_submap_node_ids = finished_submap_data.node_ids;
	}

	for (const auto& submap_id : finished_submap_ids) {
    ComputeConstraint(node_id, submap_id);
  }

	if (newly_finished_submap) {
    const SubmapId newly_finished_submap_id = submap_ids.front();
    // We have a new completed submap, so we look into adding constraints for
    // old nodes.
    for (const auto& node_id_data : optimization_problem_->node_data()) {
      const NodeId& node_id = node_id_data.id;
      if (newly_finished_submap_node_ids.count(node_id) == 0) {
        ComputeConstraint(node_id, newly_finished_submap_id);
      }
    }
  }
  constraint_builder_.NotifyEndOfNode();
  absl::MutexLock locker(&mutex_);
  ++num_nodes_since_last_loop_closure_;
  if (options_.optimize_every_n_nodes() > 0 &&
      num_nodes_since_last_loop_closure_ > options_.optimize_every_n_nodes()) {
    return WorkItem::Result::kRunOptimization;
  }
  return WorkItem::Result::kDoNotRunOptimization;

}

UpdateTrajectoryConnectivity() {
	data_.trajectory_connectivity_state.Connect();
}

HandleWorkQueue() {
	data_.constraints.insert();
	RunOptimization();

	global_slam_optimization_callback_();

	for(auto constraint : result)
		UpdateTrajectoryConnectivity();

	DeleteTrajectoriesIfNeeded();

	for(auto trimmer : trimmers_)
		trimmer->Trim();

	trimmers_erase();

	//metric things

	DrainWorkQueue();
}

DrainWorkQueue() {
	while(process_work_queue) {
		if(work_queue_->empty())
			work_queue_.reset();

		auto work_item = work_queue_->front().task;
		work_queue_->pop_front();
		process_work_queue = work_item() == WorkItem::Result::kDoNotRunOptimization;
	}

	constraint_builder_.WhenDone([this]() {
		HandleWorkQueue();
	});
}

WaitForAllComputations() {
	auto num_trajectory_nodes = data_.numtrajectory_nodes;
	auto num_finished_nodes_at_start = constraint_builder_.GetNumFinishedNodes();

	// First wait for the work queue to drain
	while(!work_queue_mutex_.AwaitWithTimeout()) {}

	constraint_builder_.WhenDone([this](){
		data_.constraints.insert();
	});

	while(!mutex_.AwaitWithTimeout()) {}
}

AddSubmapFromProto() {
	auto global_submap_pose;
	auto submap;

	const auto submap_id;
	auto global_submap_pose_2d = transform::Project2D(global_submap_pose);

	auto submap_ptr = make_shared<const Submap2D>(&conversion_tables_);

	AddTrajectoryIfNeeded();
	if(!CanAddWorkItemModifying()) return;

	data_.submap_data.Insert(submap_id, InternalSubmapData());
	data_.submap.at(submap_id).submap = submap_ptr;
	data_.global_submap_poses_2d.Insert();

	AddWorkItem([this]() {
		data_.submap_data.at(submap_id).state = SubmapState::kFinished;
    optimization_problem_->InsertSubmap(submap_id, global_submap_pose_2d);
    return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddNodeFromProto() {
	auto global_pose;
	auto node;

	auto constant_data = make_shared<TrajectoryNode::Data>(node.node_data());

	AddTrajectoryIfNeeded();
	if(!CanAddWorkItemModifying()) return;

	AddWorkItem([this]() {
		auto& constant_data;
		auto&gravity_alignment_inverse;

		optimization_problem_->InsertTrajectoryNode();
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

SetTrajectoryDataFromProto() {
	auto trajectory_data;
	AddWorkItem([this]() {
		if(CanAddWorkItemModifying()) {
			optimization_problem_.SetTrajectoryData();
		}
		return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddNodeToSubmap() {
	AddWorkItem([this](){
		if (CanAddWorkItemModifying(submap_id.trajectory_id)) {
      data_.submap_data.at(submap_id).node_ids.insert(node_id);
    }
    return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddSerializedConstraints() {	
	auto constraints;

	AddWorkItem([this](){
		for(auto constraint : constraints) {
			if(constraint.tag == Constraint::Tag::INTER_SUBMAP)
				UpdateTrajectoryConnectivity();

			auto pose;
			data_.constraints.push_back(pose);
		}
    return WorkItem::Result::kDoNotRunOptimization;
	});
}

AddTrimmer() {
	AddWorkItem([this]() {
    trimmers_.emplace_back(trimmer_ptr);
    return WorkItem::Result::kDoNotRunOptimization;
  });
}

RunFinalOptimization() {
	AddWorkItem([this]() {
		optimization_problem_->SetMaxNumIterations();
    return WorkItem::Result::kRunOptimization;
	});

	AddWorkItem([this]() {
		optimization_problem_->SetMaxNumIterations();
    return WorkItem::Result::kDoNotRunOptimization;
	});

	WaitForAllComputations();
}

RunOptimization() {
	if(optimization_problem_->submap_data().empty()) return;

	optimization_problem_->Solve();

	auto submap_data;
	auto node_data;

	for(auto trajectory_id : node_data.trajectory_ids()) {
		for(auto node : node_data.trajectory(trajectory_id)) {
			auto mutable_trajectory_node = data_.trajectory_nodes.at(node_id);
			mutable_trajectory_node.global_pose = node.data.global_pose_2d * mutable_trajectory_node.constant_data->gravity_alignment;
		}
		auto local_to_new_global = ComputeLocalToGlobalTransform();
		auto local_to_old_global = ComputeLocalToGlobalTransform();
		auto old_global_to_new_global = local_to_new_global * local_to_old_global.inverse();

		// Extrapolate all point cloud poses that were not included in the 'optimization_problem_' yet.

	}
	for (const auto& landmark : optimization_problem_->landmark_data()) {
    data_.landmark_nodes[landmark.first].global_landmark_pose = landmark.second;
  }
  data_.global_submap_poses_2d = submap_data;
}

CanAddWorkItemModifying() {
	auto it = data_.trajectories_state.find(trajectory_id);
	if(it == end()) return true;
	if(trajectory_state == Finished/Deleted || trajectory_deletion_state == Normal) return false;
	else return true;
}

SetInitialTrajectoryPose() {
	data_.initial_trajectory_poses[from_trajectory_id] =
      InitialTrajectoryPose{to_trajectory_id, pose, time};
}

ComputeLocalToGlobalTransform() {
	auto global_submap_poses;

	auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);

	if(begin_it == end_it) {
		const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    if (it != data_.initial_trajectory_poses.end()) {
      return GetInterpolatedGlobalTrajectoryPose() * it->second.relative_pose;
    } else {
      return transform::Rigid3d::Identity();
    }
	}

	return transform::Embed3D(
			global_submpa_poses.at(end_it_->id).global_pose,
			data_.submap_data.at(end_it->id).submap->local_pose);
}

TrimmingHandle() {
	parent_ = parent;
}

TrimSubmap() {
	//Make sure that the state of submap_id is kFinished
	//
	
	auto nodes_to_retain;
	for(auto submap_data : parent_->data_.submap_data)
		if(submap_data.id != submap_id)
			nodes_to_remain.insert();

	auto nodes_to_remove;
	auto constraints_related_to_submap_id_to_remain;

	auto other_submap_ids_losing_constraints;
	auto constraints_related_to_nodes_to_remove_to_remain;

	//update parent_->data_.constraints
	
	//make sure that have no inter-submap constraints left
	
	//Delete scan matchers of the submaps that lost all constraints
	for (const SubmapId& submap_id : other_submap_ids_losing_constraints) {
    parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  }
	
	// Mark the submap with 'submap_id' as trimmed and remove its data
	parent_->data_.submap_data.Trim(submap_id);
  parent_->constraint_builder_.DeleteScanMatcher(submap_id);
  parent_->optimization_problem_->TrimSubmap(submap_id);

	// Remove the 'nodes_to_remove' from the pose graph and the optimization problem
	for (const NodeId& node_id : nodes_to_remove) {
    parent_->data_.trajectory_nodes.Trim(node_id);
    parent_->optimization_problem_->TrimTrajectoryNode(node_id);
  }
}
