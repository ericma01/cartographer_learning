/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : trajectory_collator.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月07日 星期五 17时01分48秒
* Description : 
*******************************************************************************/
AddTrajectory() {
	// 创建每条轨迹的有序sensor队列
	// 'Callback' will be called whenever data from this queue can be dispatched
	if(trajectory_to_queue_.count(trajectroy_id)){
		for(auto& sensor_id : sensor_ids) {
			QueueKey queue_key;
			trajectory_to_queue_[trajectory_id].AddQueue(
					queue_key,
					Callback);
			trajectory_to_queue_keys_[trajectory_id].push_back(queue_key);
		}
	}
}

FinishTrajectory() {
	//The queue will be removed once the last piece of data from it has been dispatched
	for(auto& queue_key : trajecotry_to_queue_keys_[trajectory_id]) 
		trajectory_to_queue_.at(trajectory_id).MarkQueueAsFinished(queue_key);
}

AddSensorData() {
	QueueKey queue_key{trajectory_id, data->GetSensorId()};

	auto* metric = GetOrCreateSensorMetric();
	metric->Increment();

	trajectory_to_queue_.at(trajectory_id).Add();
}

Flush() {
	//Dispatch all queue values
	for(auto& it : trajectory_to_queue_) 
		it.second.Flush();
}



