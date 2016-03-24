#include <kaspar_control/kaspar_interface.h>

namespace kaspar_control {

KASPARInterface::KASPARInterface(const ros::NodeHandle nh, std::string name) :
		hardware_interface::RobotHW(), name_(name), nh_(nh) {
	load_params();
}

void KASPARInterface::load_params() {
	nh_.param("update_frequencey", update_freq_, 10);
}

void KASPARInterface::start() {
	main_loop_ = boost::thread(&KASPARInterface::controller_loop, this);
}

void KASPARInterface::stop() {
	main_loop_.interrupt();
	main_loop_.join();
}

void KASPARInterface::controller_loop() {
	static ros::Rate rate(update_freq_);

	ros::Time last_time = ros::Time::now();
	while (ros::ok() && !main_loop_.interruption_requested()) {
		ros::Time time = ros::Time::now();
		ros::Duration elapsed = time - last_time;

		read(elapsed);
		controller_manager_->update(time, elapsed);

		enforceLimits(elapsed);
		write(elapsed);

		last_time = time;
		rate.sleep();
	}
}

void KASPARInterface::load_joints() {
	for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it =
			urdf_model_->joints_.begin(); it != urdf_model_->joints_.end();
			++it) {
		switch (it->second->type) {
		case urdf::Joint::PRISMATIC:
		case urdf::Joint::REVOLUTE:
			Joint j(it->second->name);
			joints_.push_back(j);

			hardware_interface::JointStateHandle handle(j.name, &j.pos, &j.vel,
					&j.eff);

			joint_state_interface_.registerHandle(handle);

			hardware_interface::JointHandle joint_handle_position =
					hardware_interface::JointHandle(handle, &j.pos);
			position_joint_interface_.registerHandle(joint_handle_position);

			hardware_interface::JointHandle joint_handle_effort =
					hardware_interface::JointHandle(handle, &j.eff);
			effort_joint_interface_.registerHandle(joint_handle_effort);

			joint_limits_interface::getJointLimits(it->second, j.limits);
			bool soft = joint_limits_interface::getSoftJointLimits(it->second,
					j.soft_limits);

			// Load from ros param
			getJointLimits(j.name, nh_, j.limits);

			if (soft) {
				jnt_pos_soft_limits_interface_.registerHandle(
						joint_limits_interface::PositionJointSoftLimitsHandle(
								joint_handle_position, j.limits,
								j.soft_limits));
				jnt_eff_soft_limits_interface_.registerHandle(
						joint_limits_interface::EffortJointSoftLimitsHandle(
								joint_handle_effort, j.limits, j.soft_limits));
			} else {
				jnt_pos_limits_interface_.registerHandle(
						joint_limits_interface::PositionJointSaturationHandle(
								joint_handle_position, j.limits));
				jnt_eff_limits_interface_.registerHandle(
						joint_limits_interface::EffortJointSaturationHandle(
								joint_handle_effort, j.limits));
			}
		}
	}
}

void KASPARInterface::loadURDF(ros::NodeHandle &nh, std::string urdf_param) {
	urdf_model_ = boost::make_shared<urdf::Model>();
	std::string urdf_string;

	ros::Rate check(1);
	do {
		if (urdf_model_->initParam(urdf_param)) {
			ROS_INFO_STREAM_NAMED(name_,
					"Waiting for robot description on " << urdf_param);
			check.sleep();
		}
	} while (urdf_string.empty() && ros::ok());

	if (!urdf_model_->initString(urdf_string)) {
		ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
	} else {
		ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
	}
}

void KASPARInterface::enforceLimits(ros::Duration &period) {
	jnt_pos_limits_interface_.enforceLimits(period);
	jnt_eff_limits_interface_.enforceLimits(period);
	jnt_eff_soft_limits_interface_.enforceLimits(period);
	jnt_pos_soft_limits_interface_.enforceLimits(period);
}

}
