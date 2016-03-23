#ifndef HERKULEX_H
#define HERKULEX_H

#include <ros/ros.h>
#include <serial/serial.h>

#include "t_adc.h"

namespace herkulex {

class HerkuleX {
public:
	HerkuleX(ros::NodeHandle *nh);
	~HerkuleX();

	struct Command {
		enum e {
			// SERVO HERKULEX COMMAND - See Manual p40
			HEEPWRITE = 0x01, // Rom write
			HEEPREAD = 0x02, // Rom read
			HRAMWRITE = 0x03, // Ram write
			HRAMREAD = 0x04, // Ram read
			HIJOG = 0x05, // Write n servo with different timing
			HSJOG = 0x06, // Write n servo with same time
			HSTAT = 0x07, // Read error
			HROLLBACK = 0x08, // Back to factory value
			HREBOOT = 0x09 // Reboot
		};
	};

	struct LedColor {
		enum e {
			// HERKULEX LED - See Manual p29
			LED_OFF = 0x00,
			LED_GREEN = 0x01,
			LED_BLUE = 0x02,
			LED_RED = 0x04
		};
	};

	struct Packet_Status {
		enum e {
			// HERKULEX STATUS ERROR - See Manual p39
			H_STATUS_OK = 0x00,

			H_ERROR_INPUT_VOLTAGE = 0x01,
			H_ERROR_POS_LIMIT = 0x02,
			H_ERROR_TEMPERATURE_LIMIT = 0x04,
			H_ERROR_INVALID_PKT = 0x08,
			H_ERROR_OVERLOAD = 0x10,
			H_ERROR_DRIVER_FAULT = 0x20,
			H_ERROR_EEPREG_DISTORT = 0x40,

			Timeout = 0xFD,
			Unknown = 0xFE,
			Error = 0xFF
		};
	};

	struct Packet_Detail {
		enum e {
			H_DETAIL_MOVING = 0x01,
			H_DETAIL_INPOSITION = 0x02,

			H_PKTERR_CHECKSUM = 0x04,
			H_PKTERR_UNKNOWN_CMD = 0x08,
			H_PKTERR_EXCEED_REG_RANGE = 0x10,
			H_PKTERR_GARBAGE = 0x20,

			H_DETAIL_MOTORON = 0x40,

			Unknown = 0xFE,
		};
	};

	struct StatusMsg {
		Packet_Status::e status;
		Packet_Detail::e details;

		StatusMsg() :
				status(Packet_Status::Unknown), details(Packet_Detail::Unknown) {
		}

		StatusMsg(Packet_Status::e status) :
				status(status), details(Packet_Detail::Unknown) {
		}

		StatusMsg(Packet_Status::e status, Packet_Detail::e details) :
				status(status), details(details) {
		}
	};

	struct Address_EEP {
		enum e {
			MODEL_NO1 = 0,
			MODEL_NO2 = 1,
			VERSION1 = 2,
			VERSION2 = 3,
			BAUD_RATE = 4,
			SERVO_ID = 6,
			ACK_POLICY = 7,
			ALARM_LED_POLICY = 8,
			TORQUE_POLICY = 9,
			MAX_TEMP = 11,
			MIN_VOLTAGE = 12,
			MAX_VOLTAGE = 13,
			ACCELERATION_RATIO = 14,
			MAX_ACCELERATION_TIME = 15,
			DEAD_ZONE = 16,
			SATURATOR_OFFSET = 17,
			SATURATOR_SLOPE = 18,
			PWM_OFFSET = 20,
			MIN_PWM = 21,
			MAX_PWM = 22,
			OVERLOAD_PWM_THRESHOLD = 24,
			MIN_POSITION = 26,
			MAX_POSITION = 28,
			POSITION_KP = 30,
			POSITION_KD = 32,
			POSITION_KI = 34,
			POSITION_FEEDFORWARD_GAIN1 = 36,
			POSITION_FEEDFORWARD_GAIN2 = 38,
			LED_BLINK_PERIOD = 44,
			ADC_FAULT_CHECK_PERIOD = 45,
			PACKET_GARBAGE_CHECK_PERIOD = 46,
			STOP_DETECTION_PERIOD = 47,
			OVERLOAD_DETECTION_PERIOD = 48,
			STOP_THRESHOLD = 49,
			INPOSITION_MARGIN = 50,
			CALIBRATION_DIFF = 53
		};
	};

	struct Address_RAM {
		enum e {
			SERVO_ID = 0,
			ACK_POLICY = 1,
			ALARM_LED_POLICY = 2,
			TORQUE_POLICY = 3,
			MAX_TEMP = 5,
			MIN_VOLTAGE = 6,
			MAX_VOLTAGE = 7,
			ACCELERATION_RATIO = 8,
			MAX_ACCELERATION_TIME = 9,
			DEAD_ZONE = 10,
			SATURATOR_OFFSET = 11,
			SATURATOR_SLOPE = 12,
			PWM_OFFSET = 14,
			MIN_PWM = 15,
			MAX_PWM = 16,
			OVERLOAD_PWM_THRESHOLD = 18,
			MIN_POSITION = 20,
			MAX_POSITION = 22,
			POSITION_KP = 24,
			POSITION_KD = 26,
			POSITION_KI = 28,
			POSITION_FEEDFORWARD_GAIN1 = 30,
			POSITION_FEEDFORWARD_GAIN2 = 32,
			LED_BLINK_PERIOD = 38,
			ADC_FAULT_CHECK_PERIOD = 39,
			PACKET_GARBAGE_CHECK_PERIOD = 40,
			STOP_DETECTION_PERIOD = 41,
			OVERLOAD_DETECTION_PERIOD = 42,
			STOP_THRESHOLD = 43,
			INPOSITION_MARGIN = 44,
			CALIBRATION_DIFF_UP = 46,
			STATUS_ERROR = 48,
			STATUS_DETAIL = 49,
			TORQUE_CONTROL = 52,
			LED_CONTROL = 53,
			VOLTAGE = 54,
			TEMPERATURE = 55,
			CURRENT_CONTROL_MODE = 56,
			TICK = 57,
			CALIBRATED_POSITION = 58,
			ABSOLUTE_POSITION = 60,
			DIFFERENTIAL_POSITION = 62,
			PWM = 64,
			ABSOLUTE_GOAL_POSITION = 68,
			ABSOLUTE_DESIRED_TRAJECTORY_POSITION = 70,
			DESIRED_VELOCITY = 72
		};
	};

	struct Model {
		enum e {
			DRS_0101 = 0x01, DRS_0201 = 0x02, Unknown = 0xFE
		};
	};

	struct AckPolicy {
		enum e {
			None = 0x00, Read = 0x01, All = 0x02, Unknown = 0xFE
		};
	};

	struct TorqueState {
		enum e {
			On = 0x60, Off = 0x00, Break = 0x40, Unknown = 0xFE
		};
	};

	static const T_ADC::ADC_MAP t_adc;

	void initialize();
	void initialize(uint8_t servo_id, StatusMsg& status);
	std::vector<uint8_t> perform_id_scan();

	Model::e get_model(uint8_t servo_id, StatusMsg& status);

	void set_ackpolicy(AckPolicy::e ackpolicy);

	void clear_error(uint8_t servo_id, StatusMsg& status);
	void status(uint8_t servo_id, StatusMsg& status);
	std::vector<std::string> status_text(StatusMsg& status);

	float get_voltage(uint8_t servo_id, StatusMsg& status);
	T_ADC::ADC_MAP::mapped_type get_temperature(uint8_t servo_id,
			StatusMsg& status);

	uint8_t get_torque(uint8_t servo_id, StatusMsg& status);
	TorqueState::e get_torque_state(uint8_t servo_id, StatusMsg& status);
	void set_torque_state(uint8_t servo_id, TorqueState::e state,
			StatusMsg& status);

	int16_t get_speed(uint8_t servo_id, StatusMsg& status);
	void set_speed(uint8_t servo_id, int16_t goal_speed, uint16_t play_time,
			LedColor::e led_color, StatusMsg& status);

	uint16_t get_position(uint8_t servo_id, StatusMsg& status);
	void set_position(uint8_t servo_id, uint16_t goal_position,
			uint16_t play_time, LedColor::e led_color, StatusMsg& status);

	void set_led(uint8_t servo_id, LedColor::e led_color, StatusMsg& status);
	void set_id(uint8_t old_id, uint8_t new_id, StatusMsg& status);

	void reboot(uint8_t servo_id, StatusMsg& status);
	void rollback(uint8_t servo_id, StatusMsg& status);

private:
	static const uint8_t BROADCAST_ID = 0xFE;
	static const uint8_t BASIC_PKT_SIZE = 7;
	static const uint8_t WAIT_TIME_BY_ACK = 30;

	AckPolicy::e ack_policy_;

	//Serial variables
	std::string port_name_;
	int baudrate_;
	int timeout_;
	serial::Serial *serial_port_;

	void write_registry_ram(uint8_t servo_id, Address_RAM::e address, uint8_t data,
			StatusMsg& status);
	void write_registry_ram(uint8_t servo_id, Address_RAM::e address,
			uint16_t data, StatusMsg& status);
	void write_registry_ram(uint8_t servo_id, Address_RAM::e address,
			std::vector<uint8_t> data, StatusMsg& status);
	void write_registry_eep(uint8_t servo_id, Address_EEP::e address, uint8_t data,
			StatusMsg& status);
	void write_registry_eep(uint8_t servo_id, Address_EEP::e address,
			uint16_t data, StatusMsg& status);
	void write_registry_eep(uint8_t servo_id, Address_EEP::e address,
			std::vector<uint8_t> data, StatusMsg& status);

	uint8_t read_registry1_ram(uint8_t servo_id, Address_RAM::e address,
			StatusMsg& status);
	uint16_t read_registry2_ram(uint8_t servo_id, Address_RAM::e address,
			StatusMsg& status);
	std::vector<uint8_t> read_registry_ram(uint8_t servo_id,
			Address_RAM::e address, uint8_t length, StatusMsg& status);
	uint8_t read_registry1_eep(uint8_t servo_id, Address_EEP::e address,
			StatusMsg& status);
	uint16_t read_registry2_eep(uint8_t servo_id, Address_EEP::e address,
			StatusMsg& status);
	std::vector<uint8_t> read_registry_eep(uint8_t servo_id,
			Address_EEP::e address, uint8_t length, StatusMsg& status);

	std::vector<uint8_t> build_packet(uint8_t servo_id, Command::e command);
	std::vector<uint8_t> build_packet(uint8_t servo_id, Command::e command,
			uint8_t optional_data);
	std::vector<uint8_t> build_packet(uint8_t servo_id, Command::e command,
			uint16_t optional_data);
	std::vector<uint8_t> build_packet(uint8_t servo_id, Command::e command,
			std::vector<uint8_t> optional_data);

	bool is_right_packet(std::vector<uint8_t> buffer);

	uint8_t checksum1(std::vector<uint8_t> buffer);
	uint8_t checksum2(uint8_t checksum1);
	void send_data(std::vector<uint8_t> buffer, StatusMsg& status);
	std::vector<uint8_t> send_data_for_result(std::vector<uint8_t> buffer,
			StatusMsg& status);
};
}

#endif //HERKULEX_H
