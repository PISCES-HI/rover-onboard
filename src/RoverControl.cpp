#include "RoverControl.h"

#include <iostream>
#include <cstdio>
#include <stdint.h>
#include <string>

#include "util.h"

const int L_MOTOR_PIN = 0;
const int R_MOTOR_PIN = 1;
const int CAM_PAN_PIN = 2;
const int CAM_TILT_PIN = 3;
const int SADL_PIN = 4;
const int BLADE_PIN = 5;
const int BRAKE_PIN = 6;

RoverControl::RoverControl(HDLN& _handle) : handle(_handle), socket("0.0.0.0", 30001),
                                            l_motor(0.0), r_motor(0.0),
                                            fwd_cam_pan(0.0), fwd_cam_tilt(0.0),
                                            sadl(0.0), blade(0.0),
                                            last_telemetry(std::clock()),
                                            cmd_start(std::clock()), cmd(CMD_NONE) {
    pwm.begin(handle);
    pwm.set_pwm_freq(handle, 50);
}

void RoverControl::update() {
    try {
        // Read a datagram from a client on the server socket
        uint8_t buffer[64];
        std::string source_addr;
        uint16_t source_port;
        this->socket.recvFrom(buffer, 64, source_addr, source_port);

        if ((char*)buffer == "connect me plz") {
            std::cout << source_addr << ":" << source_port << " has connected.\n";
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
                this->set_cam_pan(this->fwd_cam_tilt);
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
        std::cout << "Failed to receive on socket: " << e.what() << std::endl;
    }
}

void RoverControl::update_telemetry() {
    // Time since last update in seconds
    double time_since_update = (std::clock()-this->last_telemetry)/(double)CLOCKS_PER_SEC;
    if (time_since_update > 0.5) {
        this->last_telemetry = std::clock();
    }
}

void RoverControl::update_command() {
    // Nothing to update if there isn't a command executing
    if (this->cmd == CMD_NONE)
        return;

    // Time since command start in seconds
    double time_since_start = (std::clock()-this->last_telemetry)/(double)CLOCKS_PER_SEC;
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
    int duty_cycle = map(power, -100, 100, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
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
