/*******************************************************************************
* Copyright (C) Other Robotics(Shenzheng) Co., Ltd. All Rights Reserved.
* Department  : R&D SW.
********************************************************************************
* File Name   : ordered_multi_queue.cpp
* Author      : Eric Ma
* Version     : v0.00
* Date        : 2022年01月07日 星期五 17时42分59秒
* Description : 
*******************************************************************************/
AddQueue() {
	queues_[queue_key].callback = std::move(callback);
}

MarkQueueAsFinished() {
	auto it = queues_.find(queue_key);
	it->second.finished = true;
	Dispatch();
}

Add() {
	auto it = queues_.find(queue_key);
	it->second.queue.Push(std::move(data));
	Dispatch();
}

Flush() {
	//Mark unfinished queue in queues_ as finished
}

Dispatch() {
	while(true) {
		
	}
}

