#ifndef KASPAR_CONTROL__KASPAR_HW_INTERFACE_H
#define KASPAR_CONTROL__KASPAR_HW_INTERFACE_H

#include <kaspar_control/kaspar_interface.h>
#include <herkulex_driver/herkulex.h>

namespace kaspar_control {

class KASPARHWInterface: public KASPARInterface {
public:
	KASPARHWInterface(const ros::NodeHandle nh, std::string name) :
			KASPARInterface(nh, name), port_speed_(0), port_name_(""), timeout_(
					0) {
	}

	virtual ~KASPARHWInterface() {
	}
	virtual void initialise();

protected:
	std::map<std::string, int> joint_ids;

	boost::shared_ptr<herkulex::HerkuleX> servo_controller;
	int port_speed_;
	std::string port_name_;
	int timeout_;

	virtual void load_params();
	virtual void load_joints();
	virtual void read(ros::Duration &elapsed_time);
	virtual void write(ros::Duration &elapsed_time);
};

} // kaspar_control

#endif
