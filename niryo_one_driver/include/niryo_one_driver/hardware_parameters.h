#ifndef HARDWARE_PARAMETERS_H
#define HARDWARE_PARAMETERS_H

#include<stdio.h>
#include<stdlib.h>
#include<vector>

class hardware_parameters{
private:
    int hardware_version;

    double hv_1_stepper_1_gear_ratio = 6.0625;
    double hv_1_stepper_2_gear_ratio = 8.3125;
    double hv_1_stepper_3_gear_ratio = 7.875;
    double hv_1_stepper_4_gear_ratio = 5.0;

    double hv_1_stepper_1_offset_position = 3.05433;
    double hv_1_stepper_2_offset_position = 0.628319;
    double hv_1_stepper_3_offset_position = -1.401;
    double hv_1_stepper_4_offset_position = 2.791;

    double hv_1_stepper_1_home_position = 0.0;
    double hv_1_stepper_2_home_position = 0.628319;
    double hv_1_stepper_3_home_position = -1.401;
    double hv_1_stepper_4_home_position = 0.0;

    double hv_1_stepper_1_direction = -1.0;
    double hv_1_stepper_2_direction = -1.0;
    double hv_1_stepper_3_direction = 1.0;
    double hv_1_stepper_4_direction = -1.0;

    double hv_1_stepper_1_max_effort = 90;
    double hv_1_stepper_2_max_effort = 140;
    double hv_1_stepper_3_max_effort = 140;
    double hv_1_stepper_4_max_effort = 90;

    std::vector<int> hv_1_can_required_motors = {1,2,3,4};
    std::vector<int> hv_1_dxl_required_motors = {4,5,6};
    std::vector<int> hv_1_dxl_authorized_motors = {4,5,6,11,12,13,31};

    double hv_2_stepper_1_gear_ratio = 6.0625;
    double hv_2_stepper_2_gear_ratio = 8.3125;
    double hv_2_stepper_3_gear_ratio = 7.875;
    double hv_2_stepper_4_gear_ratio = 5.0;

    double hv_2_stepper_6_gear_ratio = 5.0;
    double hv_2_stepper_7_gear_ratio = 5.0;

    double hv_2_stepper_1_offset_position = 3.05433;
    double hv_2_stepper_2_offset_position = 0.640187;
    double hv_2_stepper_3_offset_position = -1.397485;
    double hv_2_stepper_4_offset_position = 2.791;

    double hv_2_stepper_1_home_position = 0.0;
    double hv_2_stepper_2_home_position = 0.640187;
    double hv_2_stepper_3_home_position = -1.397485;
    double hv_2_stepper_4_home_position = 0.0;

    double hv_2_stepper_1_direction = -1.0;
    double hv_2_stepper_2_direction = -1.0;
    double hv_2_stepper_3_direction = 1.0;
    double hv_2_stepper_4_direction = -1.0;

    double hv_2_stepper_6_direction = -1.0;
    double hv_2_stepper_7_direction = -1.0;

    double hv_2_stepper_1_max_effort = 90;
    double hv_2_stepper_2_max_effort = 130;
    double hv_2_stepper_3_max_effort = 120;
    double hv_2_stepper_4_max_effort = 90;

    double hv_2_stepper_6_max_effort = 90;
    double hv_2_stepper_7_max_effort = 90;

    std::vector<int> hv_2_can_required_motors = {1,2,3};
    std::vector<int> hv_2_can_authorized_motors = {1,2,3,6,7};
    std::vector<int> hv_2_dxl_required_motors = {2,3,6};
    std::vector<int> hv_2_dxl_authorized_motors = {2,3,6,11,12,13,31};

public:
    hardware_parameters(int hardware_version=2){this->hardware_version = hardware_version;}
    double & stepper_1_gear_ratio() { return hardware_version==1?hv_1_stepper_1_gear_ratio:hv_2_stepper_1_gear_ratio; }
    double & stepper_2_gear_ratio() { return hardware_version==1?hv_1_stepper_2_gear_ratio:hv_2_stepper_2_gear_ratio; }
    double & stepper_3_gear_ratio() { return hardware_version==1?hv_1_stepper_3_gear_ratio:hv_2_stepper_3_gear_ratio; }
    double & stepper_4_gear_ratio() { return hardware_version==1?hv_1_stepper_4_gear_ratio:hv_2_stepper_4_gear_ratio; }

    double & stepper_6_gear_ratio() { return hv_2_stepper_6_gear_ratio; }
    double & stepper_7_gear_ratio() { return hv_2_stepper_7_gear_ratio; }

    double & stepper_1_offset_position() { return hardware_version==1?hv_1_stepper_1_offset_position:hv_2_stepper_1_offset_position; }
    double & stepper_2_offset_position() { return hardware_version==1?hv_1_stepper_2_offset_position:hv_2_stepper_2_offset_position; }
    double & stepper_3_offset_position() { return hardware_version==1?hv_1_stepper_3_offset_position:hv_2_stepper_3_offset_position; }
    double & stepper_4_offset_position() { return hardware_version==1?hv_1_stepper_4_offset_position:hv_2_stepper_4_offset_position; }

    double & stepper_1_home_position() { return hardware_version==1?hv_1_stepper_1_home_position:hv_2_stepper_1_home_position; }
    double & stepper_2_home_position() { return hardware_version==1?hv_1_stepper_2_home_position:hv_2_stepper_2_home_position; }
    double & stepper_3_home_position() { return hardware_version==1?hv_1_stepper_3_home_position:hv_2_stepper_3_home_position; }
    double & stepper_4_home_position() { return hardware_version==1?hv_1_stepper_4_home_position:hv_2_stepper_4_home_position; }

    double & stepper_1_direction() { return hardware_version==1?hv_1_stepper_1_direction:hv_2_stepper_1_direction; }
    double & stepper_2_direction() { return hardware_version==1?hv_1_stepper_2_direction:hv_2_stepper_2_direction; }
    double & stepper_3_direction() { return hardware_version==1?hv_1_stepper_3_direction:hv_2_stepper_3_direction; }
    double & stepper_4_direction() { return hardware_version==1?hv_1_stepper_4_direction:hv_2_stepper_4_direction; }

    double & stepper_6_direction() { return hv_2_stepper_6_direction; }
    double & stepper_7_direction() { return hv_2_stepper_7_direction; }

    double & stepper_1_max_effort() { return hardware_version==1?hv_1_stepper_1_max_effort:hv_2_stepper_1_max_effort; }
    double & stepper_2_max_effort() { return hardware_version==1?hv_1_stepper_2_max_effort:hv_2_stepper_2_max_effort; }
    double & stepper_3_max_effort() { return hardware_version==1?hv_1_stepper_3_max_effort:hv_2_stepper_3_max_effort; }
    double & stepper_4_max_effort() { return hardware_version==1?hv_1_stepper_4_max_effort:hv_2_stepper_4_max_effort; }

    double & stepper_6_max_effort() { return hv_2_stepper_6_max_effort; }
    double & stepper_7_max_effort() { return hv_2_stepper_7_max_effort; }

    std::vector<int> & can_required_motors(){return hardware_version==1?hv_1_can_required_motors:hv_2_can_required_motors;}
    std::vector<int> & can_authorized_motors(){return hv_2_can_authorized_motors;}
    std::vector<int> & dxl_required_motors(){return hardware_version==1?hv_1_dxl_required_motors:hv_2_dxl_required_motors;}
    std::vector<int> & dxl_authorized_motors(){return hardware_version==1?hv_1_dxl_authorized_motors:hv_2_dxl_authorized_motors;}

    long dxl_baudrate=                1000000;
    std::string dxl_uart_device_name= "/dev/serial0";

    float dxl_hardware_control_loop_frequency=     100.0;
    float dxl_hw_write_frequency=                  50.0;
    float dxl_hw_data_read_frequency=              15.0;
    float dxl_hw_status_read_frequency=            0.5;

    float can_hardware_control_loop_frequency=     1500.0;
    float can_hw_write_frequency=                  50.0;
    float can_hw_check_connection_frequency=       3.0;

    int spi_channel=          0;
    long spi_baudrate=        1000000;
    int gpio_can_interrupt=   25;
    int calibration_timeout=  40;

    float niryo_one_hw_check_connection_frequency= 2.0;
};

#endif