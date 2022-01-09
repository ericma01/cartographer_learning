/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : collated_trajectory_builder.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月08日 星期六 14时10分59秒
* Description : 
*******************************************************************************/
CollatedTrajectoryBuilder() {
	sensor_collator_;
	collate_landmarks_; //options
	collate_finxed_frame_; //options
	trajectory_id_;
	wrapped_trajectory_builder_; //CreateGlobalTrajectoryBuilder2D()
	last_logging_time_; //now()

	sensor_collator_->AddTrajectory(
			trajectory_id,
			expected_sensor_id_strings,
			[this] (sensor_id, unique_ptr<sensor::Data> data) {
				HandleCollatedSensorData();
			}
}

AddData() {
	sensor_collator_->AddSensorData();
}

HandleCollatedSensorData() {
	auto it = rate_timers_.find(sensor_id);
	it->second.Pulse(data->GetTime());

	// update last_logging_time_
	
	data->AddToTrajectoryBuilder();
	
}
