#include <kaspar_control/kaspar_hw_interface.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "kaspar_hw_interface");
	ros::NodeHandle nh;

	// NOTE: We run the ROS loop in a separate thread as external calls such
	// as service callbacks to load controllers can block the (main) control loop
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Create the hardware interface specific to your robot
	boost::shared_ptr<kaspar_control::KASPARHWInterface> kaspar_hw_interface =
			boost::make_shared<kaspar_control::KASPARHWInterface>(nh, "kaspar3");

	kaspar_hw_interface->initialise();

	// Start the control loop
	kaspar_hw_interface->start();

	// Wait until shutdown signal received
	ros::waitForShutdown();

	return 0;
}
