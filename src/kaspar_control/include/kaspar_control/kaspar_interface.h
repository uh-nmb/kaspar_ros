#ifndef KASPAR_CONTROL__KASPAR_INTERFACE_H
#define KASPAR_CONTROL__KASPAR_INTERFACE_H

// Use ours due to https://github.com/ros-controls/ros_control/issues/182 (fixed in indigo)
#include <kaspar_control/joint_limits_urdf.h>
#include <kaspar_control/joint_limits_rosparam.h>

#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <urdf/model.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>

namespace kaspar_control {

struct Joint {
	std::string name;
	uint8_t id;
	double cmd;
	double pos;
	double vel;
	double eff;
	joint_limits_interface::JointLimits limits;
	joint_limits_interface::SoftJointLimits soft_limits;

	Joint(std::string name) :
			name(name), id(0), cmd(0), pos(0), vel(0), eff(0) {
	}
	Joint(std::string name, uint8_t id) :
			name(name), id(id), cmd(0), pos(0), vel(0), eff(0) {
	}
};

class KASPARInterface: public hardware_interface::RobotHW {
public:
	KASPARInterface(const ros::NodeHandle nh, std::string name);

	virtual ~KASPARInterface() {
	}
	virtual void initialise() = 0;
	virtual void start();
	virtual void stop();

protected:
	ros::NodeHandle nh_;

	std::string name_;
	boost::thread main_loop_;

	std::vector<Joint> joints_;
	boost::shared_ptr<urdf::Model> urdf_model_;

	int update_freq_;

	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

	// Hardware interfaces
	hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::PositionJointInterface position_joint_interface_;
	hardware_interface::EffortJointInterface effort_joint_interface_;

	// Joint limits interfaces - Saturation
	joint_limits_interface::PositionJointSaturationInterface jnt_pos_limits_interface_;
	joint_limits_interface::EffortJointSaturationInterface jnt_eff_limits_interface_;

	// Joint limits interfaces - Soft limits
	joint_limits_interface::PositionJointSoftLimitsInterface jnt_pos_soft_limits_interface_;
	joint_limits_interface::EffortJointSoftLimitsInterface jnt_eff_soft_limits_interface_;

	virtual void load_params();
	virtual void load_joints();
	virtual void loadURDF(ros::NodeHandle &nh, std::string desc_param);

	virtual void controller_loop();

	virtual void enforceLimits(ros::Duration &period);

	virtual void read(ros::Duration &elapsed_time) = 0;
	virtual void write(ros::Duration &elapsed_time) = 0;

};

} // kaspar_control

#endif // KASPAR_CONTROL__KASPAR_INTERFACE_H
