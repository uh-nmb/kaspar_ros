#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>

#include <herkulex_driver/herkulex.h>

namespace herkulex {
const T_ADC::ADC_MAP HerkuleX::t_adc = T_ADC::get_map();

HerkuleX::HerkuleX(ros::NodeHandle *nh) {
	//Read launch file params
	nh->getParam("herkulex/portname", port_name_);
	nh->getParam("herkulex/baudrate", baudrate_);
	nh->getParam("herkulex/timeout", timeout_);

	ack_policy_ = AckPolicy::Unknown;

	serial_port_ = new serial::Serial(port_name_, (uint32_t) baudrate_,
			serial::Timeout::simpleTimeout(timeout_));

	if (serial_port_->isOpen()) {
		ROS_INFO("HerkuleX: Serial port %s opened", port_name_.c_str());
	} else {
		ROS_ERROR("Herkulex: Serial port %s not opened", port_name_.c_str());
	}
}

HerkuleX::~HerkuleX() {
	serial_port_->close();
	delete serial_port_;
}

void HerkuleX::initialize() {
	StatusMsg status;
	initialize(BROADCAST_ID, status);
}

void HerkuleX::initialize(uint8_t servo_id, StatusMsg& status) {
	clear_error(servo_id, status);
	if (status.status == Packet_Status::Error) {
		return;
	} else {
		ros::Duration(WAIT_TIME_BY_ACK / 1000.0).sleep();
	}

	set_torque_state(servo_id, TorqueState::On, status);
	if (status.status == Packet_Status::Error) {
		return;
	} else {
		ros::Duration(WAIT_TIME_BY_ACK / 1000.0).sleep();
	}
}

std::vector<uint8_t> HerkuleX::perform_id_scan() {
	std::vector<uint8_t> ids;
	for (uint8_t id = 0; id < BROADCAST_ID; id++) {
		StatusMsg status;
		get_position(id, status);
		if (status.status != Packet_Status::Error) {
			ids.push_back(id);
		}
	}
	return ids;
}

HerkuleX::Model::e HerkuleX::get_model(uint8_t servo_id, StatusMsg& status) {
	uint8_t result = read_registry1_eep(servo_id, Address_EEP::MODEL_NO1, status);

	if (status.status == Packet_Status::Error) {
		return Model::Unknown;
	}

	if (result == Model::DRS_0101) {
		return Model::DRS_0101;
	} else if (result == Model::DRS_0201) {
		return Model::DRS_0201;
	} else {
		return Model::Unknown;
	}
}

void HerkuleX::set_ackpolicy(AckPolicy::e ack_policy) {
	StatusMsg status;
	write_registry_ram(BROADCAST_ID, Address_RAM::ACK_POLICY,
			(uint8_t) ack_policy, status);
}

void HerkuleX::clear_error(uint8_t servo_id, StatusMsg& status) {
	std::vector<uint8_t> data(2, 0x00);
	write_registry_ram(servo_id, Address_RAM::STATUS_ERROR, data, status);
}

float HerkuleX::get_voltage(uint8_t servo_id, StatusMsg& status) {
	uint8_t result = read_registry1_ram(servo_id, Address_RAM::TEMPERATURE,
			status);
	return result * 0.07407451;
}

T_ADC::ADC_MAP::mapped_type HerkuleX::get_temperature(uint8_t servo_id,
		StatusMsg& status) {
	uint16_t result = read_registry2_ram(servo_id, Address_RAM::TEMPERATURE,
			status);
	uint8_t adc = result & 0xFF;
	return t_adc.at(adc);
}

uint8_t HerkuleX::get_torque(uint8_t servo_id, StatusMsg& status) {
	std::vector<uint8_t> result = read_registry_ram(servo_id, Address_RAM::PWM,
			2, status);

	if (status.status == Packet_Status::Error) {
		return 0;
	}

	int16_t torque = ((result[10] & 0x03) << 8) | (result[9] & 0xFF);

	if ((result[10] & 0x40) == 0x40) {
		torque *= -1;
	}

	return torque;
}

HerkuleX::TorqueState::e HerkuleX::get_torque_state(uint8_t servo_id,
		StatusMsg& status) {
	uint8_t result = read_registry1_ram(servo_id, Address_RAM::TORQUE_CONTROL,
			status);

	if (status.status == Packet_Status::Error) {
		return TorqueState::Unknown;
	}
	if (result == TorqueState::On) {
		return TorqueState::On;
	}
	if (result == TorqueState::Off) {
		return TorqueState::Off;
	}
	return TorqueState::Unknown;
}

void HerkuleX::set_torque_state(uint8_t servo_id, TorqueState::e state,
		StatusMsg& status) {
	write_registry_ram(servo_id, Address_RAM::TORQUE_CONTROL, (uint8_t) state,
			status);
}

int16_t HerkuleX::get_speed(uint8_t servo_id, StatusMsg& status) {
	// TODO: PWM is torque, but herkuleX arduino drivers return torque as speed??
	return get_torque(servo_id, status);
}

void HerkuleX::set_speed(uint8_t servo_id, int16_t goal_speed,
		uint16_t play_time, LedColor::e led_color, StatusMsg& status) {
	if (goal_speed < 0) {
		goal_speed = (-1) * goal_speed;
		goal_speed |= 0x4000;
	}

	std::vector<uint8_t> data(5, 0);
	data[0] = uint8_t(play_time / 11.2);  // Execution time in (ms / 11.2)
	data[1] = goal_speed & 0X00FF;
	data[2] = (goal_speed & 0XFF00) >> 8;
	data[3] = (led_color << 2) & 0xFD; // position control mode LED colours
	data[4] = servo_id;

	send_data(build_packet(servo_id, Command::HSJOG, data), status);
}

uint16_t HerkuleX::get_position(uint8_t servo_id, StatusMsg& status) {
	return read_registry2_ram(servo_id, Address_RAM::CALIBRATED_POSITION,
			status);
}

void HerkuleX::set_position(uint8_t servo_id, uint16_t goal_position,
		uint16_t play_time, LedColor::e led_color, StatusMsg& status) {

	std::vector<uint8_t> data(5, 0);
	data[0] = uint8_t(play_time / 11.2);  // Execution time in (ms / 11.2)
	data[1] = goal_position & 0X00FF;
	data[2] = (goal_position & 0XFF00) >> 8;
	data[3] = (led_color << 2) & 0xFD; // position control mode LED colours
	data[4] = servo_id;

	send_data(build_packet(servo_id, Command::HSJOG, data), status);
}

void HerkuleX::set_led(uint8_t servo_id, LedColor::e led_color,
		StatusMsg& status) {
	write_registry_ram(servo_id, Address_RAM::LED_CONTROL, (uint8_t) led_color,
			status);
}

void HerkuleX::set_id(uint8_t old_id, uint8_t new_id, StatusMsg& status) {
	if (get_position(old_id, status) == -1
			|| status.status == Packet_Status::Error) {
		return;
	}

	write_registry_eep(old_id, Address_EEP::SERVO_ID, new_id, status);
	if (status.status == Packet_Status::Error) {
		return;
	}

	ros::Duration(WAIT_TIME_BY_ACK / 1000.0).sleep();

	reboot(old_id, status);
}

void HerkuleX::reboot(uint8_t servo_id, StatusMsg& status) {
	send_data(build_packet(servo_id, Command::HREBOOT), status);
}

void HerkuleX::rollback(uint8_t servo_id, StatusMsg& status) {
	send_data(build_packet(servo_id, Command::HROLLBACK), status);
}

void HerkuleX::status(uint8_t servo_id, StatusMsg& status) {
	send_data_for_result(build_packet(servo_id, Command::HSTAT), status);
}

std::vector<std::string> HerkuleX::status_text(StatusMsg& status) {
	std::vector<std::string> status_msgs;
	Packet_Status::e status_code = status.status;
	Packet_Detail::e detail_code = status.details;

	if (status_code == Packet_Status::Error) {
		status_msgs.push_back("Invalid response received.");
		return status_msgs;
	}

	if ((detail_code & Packet_Detail::H_DETAIL_INPOSITION)
			== Packet_Detail::H_DETAIL_INPOSITION) {
		status_msgs.push_back("In Position");
	}

	if ((detail_code & Packet_Detail::H_DETAIL_MOTORON)
			== Packet_Detail::H_DETAIL_MOTORON) {
		status_msgs.push_back("Motor On");
	}

	if ((detail_code & Packet_Detail::H_DETAIL_MOVING)
			== Packet_Detail::H_DETAIL_MOVING) {
		status_msgs.push_back("Moving");
	}

	if (status_code == Packet_Status::H_STATUS_OK) {
		return status_msgs;
	}

	if ((status_code & Packet_Status::H_ERROR_INPUT_VOLTAGE)
			== Packet_Status::H_ERROR_INPUT_VOLTAGE) {
		status_msgs.push_back("Exceeded Input Voltage");
	}

	if ((status_code & Packet_Status::H_ERROR_POS_LIMIT)
			== Packet_Status::H_ERROR_POS_LIMIT) {
		status_msgs.push_back("Exceeded Position Limit");
	}

	if ((status_code & Packet_Status::H_ERROR_TEMPERATURE_LIMIT)
			== Packet_Status::H_ERROR_TEMPERATURE_LIMIT) {
		status_msgs.push_back("Exceeded Temperature Limit");
	}

	if ((status_code & Packet_Status::H_ERROR_INVALID_PKT)
			== Packet_Status::H_ERROR_INVALID_PKT) {
		std::string packet_errors = "";

		if ((detail_code & Packet_Detail::H_PKTERR_CHECKSUM)
				== Packet_Detail::H_PKTERR_CHECKSUM) {
			packet_errors += "Checksum Error, ";
		}
		if ((detail_code & Packet_Detail::H_PKTERR_UNKNOWN_CMD)
				== Packet_Detail::H_PKTERR_UNKNOWN_CMD) {
			packet_errors += "Unknown Command, ";
		}
		if ((detail_code & Packet_Detail::H_PKTERR_EXCEED_REG_RANGE)
				== Packet_Detail::H_PKTERR_EXCEED_REG_RANGE) {
			packet_errors += "Exceed REG range, ";
		}
		if ((detail_code & Packet_Detail::H_PKTERR_GARBAGE)
				== Packet_Detail::H_PKTERR_GARBAGE) {
			packet_errors += "Garbage detected, ";
		}

		if (!packet_errors.empty()) {
			// Remove trailing comma
			packet_errors = packet_errors.substr(0, packet_errors.size() - 2);
		} else {
			packet_errors += "Unknown Error";
		}

		status_msgs.push_back("Invalid Packet Received: " + packet_errors);
	}

	if ((status_code & Packet_Status::H_ERROR_OVERLOAD)
			== Packet_Status::H_ERROR_OVERLOAD) {
		status_msgs.push_back("Overload");
	}

	if ((status_code & Packet_Status::H_ERROR_DRIVER_FAULT)
			== Packet_Status::H_ERROR_DRIVER_FAULT) {
		status_msgs.push_back("Driver Fault");
	}

	if ((status_code & Packet_Status::H_ERROR_EEPREG_DISTORT)
			== Packet_Status::H_ERROR_EEPREG_DISTORT) {
		status_msgs.push_back("EEP Registry Distorted");
	}

	return status_msgs;
}

bool HerkuleX::is_right_packet(std::vector<uint8_t> buffer) {
	if (buffer.size() < 7) {
		return false;
	}

	if (buffer.size() != buffer[2]) {
		//ROS_WARN("Invalid packet length! %s", buffer);
		return false;
	}

	if (checksum1(buffer) != buffer[5]) {
		//ROS_WARN("Invalid packet checksum1! %s", buffer);
		return false;
	}

	if (checksum2(buffer[5]) != buffer[6]) {
		//ROS_WARN("Invalid packet checksum2! %s", buffer);
		return false;
	}

	return true;
}

std::vector<uint8_t> HerkuleX::build_packet(uint8_t servo_id,
		Command::e command) {
	std::vector<uint8_t> data_pkt(0, 0);

	return build_packet(servo_id, command, data_pkt);
}

std::vector<uint8_t> HerkuleX::build_packet(uint8_t servo_id,
		Command::e command, uint8_t optional_data) {
	std::vector<uint8_t> data_pkt(1, 0);
	data_pkt[0] = optional_data & 0xFF;

	return build_packet(servo_id, command, data_pkt);
}

std::vector<uint8_t> HerkuleX::build_packet(uint8_t servo_id,
		Command::e command, uint16_t optional_data) {
	std::vector<uint8_t> data_pkt(2, 0);
	data_pkt[0] = optional_data & 0X00FF;
	data_pkt[1] = (optional_data & 0XFF00) >> 8;

	return build_packet(servo_id, command, data_pkt);
}

std::vector<uint8_t> HerkuleX::build_packet(uint8_t servo_id,
		Command::e command, std::vector<uint8_t> optional_data) {

	uint8_t pkt_size = BASIC_PKT_SIZE + optional_data.size();
	std::vector<uint8_t> buffer(pkt_size, 0);

	buffer[0] = 0xFF;  // Packet Header
	buffer[1] = 0xFF;  // Packet Header
	buffer[2] = pkt_size;  // Packet Size
	buffer[3] = servo_id;  // Servo ID
	buffer[4] = command;  // Command

	std::vector<uint8_t>::iterator insert = buffer.begin();
	std::advance(insert, BASIC_PKT_SIZE);
	buffer.insert(insert, optional_data.begin(), optional_data.end());

	buffer[5] = checksum1(buffer);
	buffer[6] = checksum2(buffer[5]);

	return buffer;
}

uint8_t HerkuleX::checksum1(std::vector<uint8_t> buffer) {
	uint8_t chksum1 = 0x00;
	// 0 and 1 are header
	chksum1 = chksum1 ^ buffer[2]; // packet size
	chksum1 = chksum1 ^ buffer[3]; // servo id
	chksum1 = chksum1 ^ buffer[4]; // command

	// Data starts at index == 7
	for (std::vector<uint8_t>::size_type i = 7; i < buffer.size(); i++) {
		chksum1 = chksum1 ^ buffer[i];
	}
	return chksum1 & 0xFE;
}

uint8_t HerkuleX::checksum2(uint8_t checksum1) {
	return (~checksum1) & 0xFE;
}

uint8_t HerkuleX::read_registry1_ram(uint8_t servo_id, Address_RAM::e address,
		StatusMsg& status) {
	std::vector<uint8_t> result = read_registry_ram(servo_id, address, 1,
			status);
	if (status.status == Packet_Status::Error) {
		return 0;
	} else {
		return result[2];
	}
}

uint16_t HerkuleX::read_registry2_ram(uint8_t servo_id, Address_RAM::e address,
		StatusMsg& status) {
	std::vector<uint8_t> result = read_registry_ram(servo_id, address, 2,
			status);
	if (status.status == Packet_Status::Error) {
		return 0;
	} else {
		return ((result[3] & 0x03) << 8) | (result[2] & 0xFF);
	}
}

std::vector<uint8_t> HerkuleX::read_registry_ram(uint8_t servo_id,
		Address_RAM::e address, uint8_t length, StatusMsg& status) {
	std::vector<uint8_t> data(2, 0);
	data[0] = address;
	data[1] = length;

	std::vector<uint8_t> result = send_data_for_result(
			build_packet(servo_id, Command::HRAMREAD, data), status);

	if (status.status == Packet_Status::Error) {
		return std::vector<uint8_t>();
	} else {
		return std::vector<uint8_t>(result.begin() + BASIC_PKT_SIZE,
				result.end());
	}
}

uint8_t HerkuleX::read_registry1_eep(uint8_t servo_id, Address_EEP::e address,
		StatusMsg& status) {
	std::vector<uint8_t> result = read_registry_eep(servo_id, address, 1,
			status);
	if (status.status == Packet_Status::Error) {
		return 0;
	} else {
		return result[2];
	}
}

uint16_t HerkuleX::read_registry2_eep(uint8_t servo_id, Address_EEP::e address,
		StatusMsg& status) {
	std::vector<uint8_t> result = read_registry_eep(servo_id, address, 2,
			status);
	if (status.status == Packet_Status::Error) {
		return 0;
	} else {
		return ((result[3] & 0x03) << 8) | (result[2] & 0xFF);
	}
}

std::vector<uint8_t> HerkuleX::read_registry_eep(uint8_t servo_id,
		Address_EEP::e address, uint8_t length, StatusMsg& status) {
	std::vector<uint8_t> data(2, 0);
	data[0] = address;
	data[1] = length;

	std::vector<uint8_t> result = send_data_for_result(
			build_packet(servo_id, Command::HEEPREAD, data), status);

	if (status.status == Packet_Status::Error) {
		return std::vector<uint8_t>();
	} else {
		std::vector<uint8_t>::iterator data_begin = result.begin();
		std::advance(data_begin, BASIC_PKT_SIZE);
		return std::vector<uint8_t>(data_begin, result.end());
	}
}

void HerkuleX::write_registry_ram(uint8_t servo_id, Address_RAM::e address,
		uint8_t data, StatusMsg& status) {
	std::vector<uint8_t> data_pkt(1, 0);
	data_pkt[0] = data;

	write_registry_ram(servo_id, address, data_pkt, status);
}

void HerkuleX::write_registry_ram(uint8_t servo_id, Address_RAM::e address,
		uint16_t data, StatusMsg& status) {
	std::vector<uint8_t> data_pkt(2, 0);
	data_pkt[0] = data & 0X00FF;
	data_pkt[1] = (data & 0XFF00) >> 8;

	write_registry_ram(servo_id, address, data_pkt, status);
}

void HerkuleX::write_registry_ram(uint8_t servo_id, Address_RAM::e address,
		std::vector<uint8_t> data, StatusMsg& status) {

	std::vector<uint8_t> optional_data(2 + data.size(), 0);
	optional_data[0] = address;  // Address
	optional_data[1] = data.size();  // Length
	optional_data.insert(optional_data.end(), data.begin(), data.end());

	send_data(build_packet(servo_id, Command::HRAMWRITE, optional_data),
			status);
}

void HerkuleX::write_registry_eep(uint8_t servo_id, Address_EEP::e address,
		uint8_t data, StatusMsg& status) {
	std::vector<uint8_t> data_pkt(1, 0);
	data_pkt[0] = data;

	write_registry_eep(servo_id, address, data_pkt, status);
}

void HerkuleX::write_registry_eep(uint8_t servo_id, Address_EEP::e address,
		uint16_t data, StatusMsg& status) {
	std::vector<uint8_t> data_pkt(2, 0);
	data_pkt[0] = data & 0X00FF;
	data_pkt[1] = (data & 0XFF00) >> 8;

	write_registry_eep(servo_id, address, data_pkt, status);
}

void HerkuleX::write_registry_eep(uint8_t servo_id, Address_EEP::e address,
		std::vector<uint8_t> data, StatusMsg& status) {

	std::vector<uint8_t> optional_data(2 + data.size(), 0);
	optional_data[0] = address;  // Address
	optional_data[1] = data.size();  // Length
	optional_data.insert(optional_data.end(), data.begin(), data.end());

	send_data(build_packet(servo_id, Command::HEEPWRITE, optional_data),
			status);
}

void HerkuleX::send_data(std::vector<uint8_t> buffer, StatusMsg& status) {
	if (ack_policy_ == AckPolicy::All) {
		send_data_for_result(buffer, status);
	} else {
		serial_port_->flush();
		serial_port_->write(buffer);
	}
}

std::vector<uint8_t> HerkuleX::send_data_for_result(std::vector<uint8_t> buffer,
		StatusMsg& status) {

	serial_port_->flush();
	serial_port_->write(buffer);

	// Broadcast requests return no data
	if (buffer[3] != BROADCAST_ID) {
		return std::vector<uint8_t>();
	}

	ros::Duration(WAIT_TIME_BY_ACK / 1000.0).sleep();
	std::vector<uint8_t> result(2, 0xFF);

	/* Extra checking and filtering to handle noisy serial lines */
	// Locate the start of the header
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout((timeout_ + WAIT_TIME_BY_ACK) / 1000.0);
	while (ros::Time::now() - start_time < timeout) {
		std::vector<uint8_t> tmp_buffer;
		serial_port_->read(tmp_buffer, 1);
		if (tmp_buffer.empty()) {
			continue;
		}

		uint8_t byte = tmp_buffer[0] & 0xFF;
		if (byte > 0xE9) {  // max valid packet length
			continue;
		}

		result.push_back(byte);
		break;
	}

	// If we have a size byte
	if (result.size() > 2) {
		std::vector<uint8_t> tmp_buffer;
		serial_port_->read(tmp_buffer, result[2] - 3);
		result.insert(result.end(), tmp_buffer.begin(), tmp_buffer.end());
	}

	// If we have data, but less than size byte, and bytes are waiting on the line
	if (result.size() > 2 && result.size() < result[2]
			&& serial_port_->available() > 0) {
		status.status == Packet_Status::Timeout;
		std::vector<uint8_t> tmp_buffer;
		int to_read =
				serial_port_->available() > result[2] - result.size() ?
						result[2] - result.size() : serial_port_->available();
		serial_port_->read(tmp_buffer, to_read);
	}

	if (result.size() == 2) {
		result.clear();
	} else {
		if (!is_right_packet(result)) {
			status.status = Packet_Status::Error;
		} else {
			status.status = static_cast<Packet_Status::e>(result.at(
					result.size() - 2));
			status.details = static_cast<Packet_Detail::e>(result.at(
					result.size() - 1));
		}
	}

	return result;
}
}
