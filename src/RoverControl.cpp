#include "RoverControl.h"

#include <iostream>
#include <cstdio>
#include <stdint.h>
#include <string>


#include "dln/dln_adc.h"

#include "analog.h"
#include "util.h"
#include "thermistor.h"

const int SADL_PIN = 0;
const int R_MOTOR_PIN = 1;
const int L_MOTOR_PIN = 2;
const int BRAKE_PIN = 3;
const int CAM_PAN_PIN = 4;
const int CAM_TILT_PIN = 5;
const int BLADE_PIN = 6;

////////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(double c) : last_time(0) {
    this->cooldown = static_cast<std::clock_t>(c*((double)CLOCKS_PER_SEC));
}

bool Timer::tick() {
    std::clock_t cur_time = std::clock();
    if (cur_time - this->last_time > this->cooldown) {
        this->last_time = cur_time;
        return true;
    } else {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

RoverControl::RoverControl(HDLN& _handle) : handle(_handle), socket("0.0.0.0", 30001),
                                            adxl(_handle),
                                            l_motor(0.0), r_motor(0.0),
                                            fwd_cam_pan(0.0), fwd_cam_tilt(0.0),
                                            sadl(0.0), blade(0.0),
                                            tele_packet_timer(0.5),
                                            tele_timer(0.25),
                                            cmd_start(std::clock()), cmd(CMD_NONE) {
    adxl.initialize();
    adxl.setOffsetZ(7);
    pwm.begin(handle);
    pwm.set_pwm_freq(handle, 50);
    set_l_motor(0);
    set_r_motor(0);
    set_sadl(0);
    set_brake(0);
    set_cam_pan(90);
    set_cam_tilt(130);
}

void RoverControl::update() {
    update_telemetry();
    try {
        // Read a datagram from a client on the server socket
        uint8_t buffer[64];
        std::string source_addr;
        uint16_t source_port;
        this->socket.recvFrom(buffer, 64, source_addr, source_port);

        if (strncmp((char*)buffer, "connect me plz", 14) == 0) {
            std::cout << source_addr << ":" << source_port << " has connected.\n";
            this->clients.push_back(Client(source_addr, source_port));
            return;
        }

        char packet_id = buffer[0];
        switch (packet_id) {
            case 'A':
            {
                // Left motor
                sscanf((char*)buffer+1, "%d|", &this->l_motor);
                this->set_l_motor(this->l_motor);
                std::cout << "Got l motor speed: " << this->l_motor << std::endl;
                break;
            }
            case 'B':
            {
                // Right motor
                sscanf((char*)buffer+1, "%d|", &this->r_motor);
                this->set_r_motor(this->r_motor);
                std::cout << "Got r motor speed: " << this->r_motor << std::endl;
                break;
            }
            case 'H':
            {
                // LR motor
                sscanf((char*)buffer+1, "%d|%d|", &this->l_motor, &this->r_motor);
                this->set_l_motor(this->l_motor);
                this->set_r_motor(this->r_motor);
                std::cout << "Got lr motor speed: " << this->l_motor << " " << this->r_motor << std::endl;
                break;
            }
            case 'C':
            {
                // Pan
                sscanf((char*)buffer+1, "%f|", &this->fwd_cam_pan);
                this->set_cam_pan(this->fwd_cam_pan);
                break;
            }
            case 'D':
            {
                // Tilt
                sscanf((char*)buffer+1, "%f|", &this->fwd_cam_tilt);
                this->set_cam_tilt(this->fwd_cam_tilt);
                break;
            }
            case 'E':
            {
                // SADL
                sscanf((char*)buffer+1, "%d|", &this->sadl);
                this->set_sadl(this->sadl);
                break;
            }
            case 'F':
            {
                // Blade
                sscanf((char*)buffer+1, "%d|", &this->blade);
                this->set_blade(this->sadl);
                break;
            }
            case 'G':
            {
                // Brake
                int brake = 0;
                sscanf((char*)buffer+1, "%d|", &brake);
                this->brake = (brake == 0 ? false : true);
                this->set_blade(this->brake);
                break;
            }
            case 'Z':
            {
                // Text command
                break;
            }
        }
    } catch (SocketException e) {
        //std::cout << "Failed to receive on socket: " << e.what() << std::endl;
    }
}

void RoverControl::update_telemetry() {
    if (this->tele_timer.tick()) {
        // Read telemetry data

        uint16_t value;

        DLN_RESULT result = DlnAdcGetValue(this->handle, 0, 0, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 0\n";
        }
        float voltage_48v = get_48v_voltage(value);

        result = DlnAdcGetValue(this->handle, 0, 1, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 1: " << result << std::endl;
        }
        double l_motor_temp = get_thermistor_temp(value);

        result = DlnAdcGetValue(this->handle, 0, 2, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 2: " << result << std::endl;
        }
        std::cout << "Right motor amp: " << value << std::endl;

        result = DlnAdcGetValue(this->handle, 0, 3, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 3: " << result << std::endl;
        }
        std::cout << "Left motor amp: " << value << std::endl;

        result = DlnAdcGetValue(this->handle, 0, 6, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 6: " << result << std::endl;
        }
        std::cout << "24v amp: " << value << std::endl;

        result = DlnAdcGetValue(this->handle, 0, 4, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 4: " << result << std::endl;
        }
        std::cout << "temp: " << value << std::endl;
        std::cout << "temp: " << get_ambient_temperature(value) << std::endl;
        //float upper_avionics_temp = get_avionics_temperature(value);
        float ambient_temp = get_ambient_temperature(value);

        std::cout << "48v: " << voltage_48v << std::endl;
        std::cout << "L motor temp: " << l_motor_temp << std::endl;
        this->telemetry_bundle += "VOLT:"+std::to_string(voltage_48v)+":0.0:0.0:0.0"+"|";
        this->telemetry_bundle += "L_MOTOR_TEMP:"+std::to_string(l_motor_temp)+"|";
        //this->telemetry_bundle += "UPR_A_TEMP:"+std::to_string(upper_avionics_temp)+"|";
        this->telemetry_bundle += "AMBIENT_TEMP:"+std::to_string(ambient_temp)+"|";

        std::cout << "X accel: " << this->adxl.getAccelerationX() << std::endl;
        std::cout << "Y accel: " << this->adxl.getAccelerationY() << std::endl;
        std::cout << "Z accel: " << this->adxl.getAccelerationZ() << std::endl;

        this->telemetry_bundle += "IMU:"+std::to_string(this->adxl.getAccelerationX())+":"
                                        +std::to_string(this->adxl.getAccelerationY())+":"
                                        +std::to_string(this->adxl.getAccelerationZ())+":"
                                        +"0:0:0:0:0:0"+"|";
    }

    // Time to send telemetry packet bundle?
    if (this->tele_packet_timer.tick()) {
        for (auto& client : this->clients) {
            this->socket.sendTo(this->telemetry_bundle.c_str(), this->telemetry_bundle.size(),
                                client.address, client.port);
        }
        this->telemetry_bundle.clear();
    }
}

void RoverControl::update_command() {
    // Nothing to update if there isn't a command executing
    if (this->cmd == CMD_NONE)
        return;

    // Time since command start in seconds
    double time_since_start = (std::clock()-this->cmd_start)/(double)CLOCKS_PER_SEC;
    if (time_since_start > this->cmd_duration) {
        // Remaining command duration is 0, stop executing the command
    }

    switch (this->cmd) {
        case CMD_FWD:
        {
            break;
        }
        case CMD_REV:
        {
            break;
        }
        case CMD_LEFT:
        {
            break;
        }
        case CMD_RIGHT:
        {
            break;
        }
        case CMD_NONE:
        {
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Motor controls

// Out of 4096
const int PWM_SERVO_MIN = 150;
const int PWM_SERVO_MAX = 450;

void RoverControl::set_cam_pan(float angle) {
    int duty_cycle = map(angle, 0.0, 180.0, PWM_SERVO_MIN, PWM_SERVO_MAX);
    pwm.set_pin(this->handle, CAM_PAN_PIN, duty_cycle);
}

void RoverControl::set_cam_tilt(float angle) {
    int duty_cycle = map(angle, 0.0, 180.0, PWM_SERVO_MIN, PWM_SERVO_MAX);
    pwm.set_pin(this->handle, CAM_TILT_PIN, duty_cycle);
}

////////////////////////

// Out of 4096
const int PWM_MOTOR_MIN = 186;
const int PWM_MOTOR_MAX = 372;

void RoverControl::set_l_motor(int power) {
    // NOTE: Left motor is wonky, so has slightly different min/max values, and has directions flipped
    int duty_cycle = map(-power, -100, 100, 157, 344);
    pwm.set_pin(this->handle, L_MOTOR_PIN, duty_cycle);
}

void RoverControl::set_r_motor(int power) {
    int duty_cycle = map(power, -100, 100, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    pwm.set_pin(this->handle, R_MOTOR_PIN, duty_cycle);
}

void RoverControl::set_sadl(int power) {
    int duty_cycle = map(power, -100, 100, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    pwm.set_pin(this->handle, SADL_PIN, duty_cycle);
}

void RoverControl::set_blade(int power) {
    int duty_cycle = map(power, -100, 100, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    pwm.set_pin(this->handle, BLADE_PIN, duty_cycle);
}

void RoverControl::set_brake(bool on) {
    int duty_cycle = on ? PWM_MOTOR_MAX : PWM_MOTOR_MIN;
    pwm.set_pin(this->handle, BRAKE_PIN, duty_cycle);
}
