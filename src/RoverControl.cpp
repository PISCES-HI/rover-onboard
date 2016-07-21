#include "RoverControl.h"

#include <iostream>
#include <cstdio>
#include <stdint.h>
#include <string>
#include <ctime>

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
const int STEREO_CAM_PAN_PIN = 6;
const int STEREO_CAM_TILT_PIN = 7;
const int BLADE_PIN = 8;

////////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(double c) : last_time(0) {
    this->cooldown = static_cast<std::clock_t>(c*((double)CLOCKS_PER_SEC));
}

void Timer::set_cooldown(double c) {
    this->cooldown = static_cast<std::clock_t>(c*((double)CLOCKS_PER_SEC));
}

void Timer::reset() {
    this->last_time = std::clock();
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
                                            mag(_handle),
                                            barometer(_handle),
                                            gps(Serial("/dev/ttyUSB0")),
                                            l_motor(0.0), r_motor(0.0),
                                            fwd_cam_pan(90.0), fwd_cam_tilt(130.0),
                                            stereo_cam_pan(90.0), stereo_cam_tilt(90.0),
                                            sadl(0.0), blade(0.0),
                                            tele_packet_timer(0.5),
                                            tele_timer(0.25),
                                            cmd_start(std::clock()), cmd(CMD_NONE),
                                            taking_panorama(false), panorama_timer(0.5) {
    adxl.initialize();
    adxl.setOffsetZ(7);
    mag.initialize();
    barometer.initialize();
    barometer.setControl(BMP085_MODE_PRESSURE_3);

    // Initialize GPS
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    pwm.begin(handle);
    pwm.set_pwm_freq(handle, 50);

    set_l_motor(0);
    set_r_motor(0);
    set_sadl(0);
    set_brake(0);
    set_cam_pan(90);
    set_cam_tilt(130);

    set_stereo_cam_pan(90);
    set_stereo_cam_tilt(90);
}

void RoverControl::update() {
    update_telemetry();
    update_stereo_panorama();
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
            case 'I':
            {
                // Stereo Pan
                sscanf((char*)buffer+1, "%f|", &this->stereo_cam_pan);
                this->set_stereo_cam_pan(this->stereo_cam_pan);
                break;
            }
            case 'J':
            {
                // Stereo Tilt
                sscanf((char*)buffer+1, "%f|", &this->stereo_cam_tilt);
                this->set_stereo_cam_tilt(this->stereo_cam_tilt);
                break;
            }
            case 'K':
            {
                this->stereo_snapshot();
            }
            case 'L':
            {
                std::cout << "PANORAMA!!!!!!\n\n";
                this->start_stereo_panorama();
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
    // Update GPS
    gps.read();
    if (gps.newNMEAreceived() && gps.parse(gps.lastNMEA())) {
        if (gps.fix) {
            // GPS:<latitude>:<longitude>:<speed>:<altitude>:<angle>
            this->telemetry_bundle += "GPS:"+std::to_string(this->gps.lat)+":"
                                      +std::to_string(this->gps.lon)+":"
                                      +std::to_string(this->gps.speed)+":"
                                      +std::to_string(this->gps.altitude)+":"
                                      +std::to_string(this->gps.angle)+"|";
        }
    }

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
        //std::cout << "Right motor amp: " << value << std::endl;

        result = DlnAdcGetValue(this->handle, 0, 3, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 3: " << result << std::endl;
        }
        //std::cout << "Left motor amp: " << value << std::endl;

        result = DlnAdcGetValue(this->handle, 0, 6, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 6: " << result << std::endl;
        }
        //std::cout << "24v amp: " << value << std::endl;

        result = DlnAdcGetValue(this->handle, 0, 4, &value);
        if (DLN_FAILED(result)) {
            std::cout << "Failed to read analog channel 4: " << result << std::endl;
        }
        //std::cout << "ambient temp: " << value << std::endl;
        //std::cout << "ambient temp: " << get_ambient_temperature(value) << std::endl;
        //float upper_avionics_temp = get_avionics_temperature(value);
        float ambient_temp = get_ambient_temperature(value);

        //std::cout << "48v: " << voltage_48v << std::endl;
        //std::cout << "L motor temp: " << l_motor_temp << std::endl;
        this->telemetry_bundle += "VOLT:"+std::to_string(voltage_48v)+":0.0:0.0:0.0"+"|";
        this->telemetry_bundle += "L_MOTOR_TEMP:"+std::to_string(l_motor_temp)+"|";
        //this->telemetry_bundle += "UPR_A_TEMP:"+std::to_string(upper_avionics_temp)+"|";
        this->telemetry_bundle += "AMBIENT_TEMP:"+std::to_string(ambient_temp)+"|";

        /*std::cout << "X accel: " << this->adxl.getAccelerationX() << std::endl;
        std::cout << "Y accel: " << this->adxl.getAccelerationY() << std::endl;
        std::cout << "Z accel: " << this->adxl.getAccelerationZ() << std::endl;

        std::cout << "X heading: " << this->mag.getHeadingX() << std::endl;
        std::cout << "Y heading: " << this->mag.getHeadingY() << std::endl;
        std::cout << "Z heading: " << this->mag.getHeadingZ() << std::endl;*/

        this->telemetry_bundle += "IMU:"+std::to_string(this->adxl.getAccelerationX())+":"
                                        +std::to_string(this->adxl.getAccelerationY())+":"
                                        +std::to_string(this->adxl.getAccelerationZ())+":"
                                        +"0:0:0:"
                                        +std::to_string(this->mag.getHeadingX())+":"
                                        +std::to_string(this->mag.getHeadingY())+":"
                                        +std::to_string(this->mag.getHeadingZ())+"|";

        if (barometer.getControl() == BMP085_MODE_PRESSURE_3) {
            float pressure = barometer.getPressure();
            float altitude = barometer.getAltitude(pressure);
            this->telemetry_bundle += "W_PR_ALT:"+std::to_string(pressure)+":"+std::to_string(altitude)+"|";
            barometer.setControl(BMP085_MODE_TEMPERATURE);
        } else if (barometer.getControl() == BMP085_MODE_TEMPERATURE) {
            float temp = barometer.getTemperatureF();
            this->telemetry_bundle += "W_TEMP:"+std::to_string(temp)+"|";
            barometer.setControl(BMP085_MODE_PRESSURE_3);
        }
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

void RoverControl::set_stereo_cam_pan(float angle) {
    int duty_cycle = map(angle, 0.0, 180.0, 125, 425);
    pwm.set_pin(this->handle, STEREO_CAM_PAN_PIN, duty_cycle);
}

void RoverControl::set_stereo_cam_tilt(float angle) {
    int duty_cycle = map(angle, 0.0, 180.0, PWM_SERVO_MIN, PWM_SERVO_MAX);
    pwm.set_pin(this->handle, STEREO_CAM_TILT_PIN, duty_cycle);
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

void RoverControl::stereo_snapshot() {
    time_t t = time(0);
    tm* now = localtime(&t);

    std::string date_time = std::to_string(now->tm_year + 1900) + "-"
                            + std::to_string(now->tm_mon + 1) + "-"
                            + std::to_string(now->tm_mday) + "-"
                            + std::to_string(now->tm_min) + ":"
                            + std::to_string(now->tm_sec);

    system((std::string("ffmpeg -f video4linux2 -i /dev/video0 -vframes 1 ~/stereo_snapshots/") + date_time + "_L.jpg").c_str());
    system((std::string("ffmpeg -f video4linux2 -i /dev/video0 -vframes 1 ~/stereo_snapshots/") + date_time + "_R.jpg").c_str());
}

void RoverControl::start_stereo_panorama() {
    this->taking_panorama = true;
    this->panorama_pos = 0;
    this->panorama_state = WAIT_MOVE;
    this->panorama_timer.set_cooldown(0.5);
    this->panorama_timer.reset();
}

void RoverControl::update_stereo_panorama() {
    if (!this->taking_panorama) return;

    if (this->panorama_timer.tick()) {
        std::cout << "panorama tick\n";
        switch (this->panorama_state) {
            case WAIT_MOVE:
            {
                std::cout << "wait move\n";
                time_t t = time(0);
                tm* now = localtime(&t);

                std::string date_time = std::to_string(now->tm_year + 1900) + "-"
                                        + std::to_string(now->tm_mon + 1) + "-"
                                        + std::to_string(now->tm_mday) + "-"
                                        + std::to_string(now->tm_min) + ":"
                                        + std::to_string(now->tm_sec);
                std::string pos_str = std::to_string(this->panorama_pos);

                system((std::string("ffmpeg -f video4linux2 -i /dev/video0 -vframes 1 ~/stereo_snapshots/") +
                                    date_time + "_" + pos_str + "_L.jpg").c_str());
                system((std::string("ffmpeg -f video4linux2 -i /dev/video0 -vframes 1 ~/stereo_snapshots/") +
                                    date_time + "_" + pos_str + "_R.jpg").c_str());

                // TODO: take photo

                this->panorama_state = WAIT_PHOTO;
                this->panorama_timer.set_cooldown(1.0);
                break;
            }
            case WAIT_PHOTO:
            {
                std::cout << "wait photo\n";
                if (this->panorama_pos == 6) {
                    // Done taking a panorama
                    this->taking_panorama = false;
                    return;
                }
                
                switch (this->panorama_pos) {
                    case 0:
                    {
                        this->set_stereo_cam_pan(10.0);
                        this->set_stereo_cam_tilt(60.0);
                        break;
                    }
                    case 1:
                    {
                        this->set_stereo_cam_pan(90.0);
                        this->set_stereo_cam_tilt(60.0);
                        break;
                    }
                    case 2:
                    {
                        this->set_stereo_cam_pan(170.0);
                        this->set_stereo_cam_tilt(60.0);
                        break;
                    }
                    case 3:
                    {
                        this->set_stereo_cam_pan(170.0);
                        this->set_stereo_cam_tilt(120.0);
                        break;
                    }
                    case 4:
                    {
                        this->set_stereo_cam_pan(90.0);
                        this->set_stereo_cam_tilt(120.0);
                        break;
                    }
                    case 5:
                    {
                        this->set_stereo_cam_pan(10.0);
                        this->set_stereo_cam_tilt(120.0);
                        break;
                    }
                }

                this->panorama_pos += 1;
                this->panorama_state = WAIT_MOVE;
                this->panorama_timer.set_cooldown(0.5);
                break;
            }
        }
    }
}
