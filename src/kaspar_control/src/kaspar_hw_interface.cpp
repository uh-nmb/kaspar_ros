#include <herkulex_driver/herkulex.h>
#include <kaspar_control/kaspar_hw_interface.h>

namespace kaspar_control {

void KASPARHWInterface::load_params() {
	KASPARInterface::load_params();
	if (!nh_.getParam("joint_ids", joint_ids)) {
		ROS_WARN_STREAM_NAMED(name_, "Missing joint_ids param");
		exit(0);
	}

	if (!nh_.getParam("port_name", port_name_)) {
		ROS_WARN_STREAM_NAMED(name_, "Missing port_name param");
		exit(0);
	}

	nh_.param("port_speed", port_speed_, 115200);
	nh_.param("timeout", timeout_, 250);
}

void KASPARHWInterface::load_joints() {
	KASPARInterface::load_joints();

	// Add the servoID
	for (std::vector<Joint>::iterator it = joints_.begin(); it != joints_.end();
			++it) {
		if (joint_ids.find(it->name) != joint_ids.end()) {
			it->id = joint_ids[it->name];
		}
	}
}

void KASPARHWInterface::read(ros::Duration &elapsed_time) {
	for (std::vector<Joint>::iterator it = joints_.begin(); it != joints_.end();
			++it) {
		herkulex::HerkuleX::StatusMsg status;
		it->eff = servo_controller->get_speed(it->id, status);
		it->pos = servo_controller->get_position(it->id, status);
	}
}

void KASPARHWInterface::write(ros::Duration &elapsed_time) {
	for (std::vector<Joint>::iterator it = joints_.begin(); it != joints_.end();
			++it) {
		herkulex::HerkuleX::StatusMsg status;
//		servo_controller->set_speed(it->id, it->eff, 0,
//				herkulex::HerkuleX::LedColor::LED_OFF, status);
		servo_controller->set_position(it->id, it->pos, 0,
				herkulex::HerkuleX::LedColor::LED_OFF, status);
	}
}

}  // kaspar_control
