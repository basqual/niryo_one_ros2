/*
    rpi_diagnostics.cpp
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "niryo_one_driver/rpi_diagnostics.h"

RpiDiagnostics::RpiDiagnostics(rclcpp::Node::SharedPtr node)
{
    this->node = node;
    cpu_temperature = 0;

    startReadingData();
}

int RpiDiagnostics::getRpiCpuTemperature()
{
    return cpu_temperature;
}

void RpiDiagnostics::readCpuTemperature()
{
#ifdef __aarch64__
    std::fstream cpu_temp_file("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);
    
    int read_temp;
    cpu_temp_file >> read_temp;
    if (read_temp > 0) {
        cpu_temperature = read_temp / 1000;
    }
#endif
}

void RpiDiagnostics::startReadingData()
{
   read_hardware_data_thread.reset(new std::thread(std::bind(&RpiDiagnostics::readHardwareDataLoop, this))); 
}

void RpiDiagnostics::readHardwareDataLoop()
{
    double read_rpi_diagnostics_frequency;
    node->get_parameter("read_rpi_diagnostics_frequency", read_rpi_diagnostics_frequency);
    rclcpp::Rate read_rpi_diagnostics_rate(read_rpi_diagnostics_frequency);

    while (rclcpp::ok()) {
        readCpuTemperature();

        // check if Rpi is too hot
        if (cpu_temperature > 75) {
            RCLCPP_ERROR(node->get_logger(),"Rpi temperature is really high !");
        }
        if (cpu_temperature > 85) {
            RCLCPP_ERROR(node->get_logger(),"Rpi is too hot, shutdown to avoid any damage");
            std::system("sudo shutdown now");
        }

        read_rpi_diagnostics_rate.sleep();
    }
}
