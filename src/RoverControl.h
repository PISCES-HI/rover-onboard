#ifndef ROVER_CONTROL_H
#define ROVER_CONTROL_H

#include <ctime>
#include <string>
#include <vector>

#include "dln/dln_generic.h"

#include "AdxlDriver.h"
#include "Bmp085Driver.h"
#include "GpsDriver.h"
#include "Hmc5883lDriver.h"
#include "PwmDriver.h"
#include "Socket.h"

enum Command {
    CMD_FWD,
    CMD_REV,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_NONE
};

enum PanoramaState {
    WAIT_MOVE,
    WAIT_PHOTO
};

struct Timer {
    // c is cooldown in seconds
    Timer(double c);
    void set_cooldown(double c);
    void reset();
    bool tick();

    std::clock_t last_time;
    std::clock_t cooldown;
};

struct Client {
    Client(const std::string& a, const unsigned short p) : address(a), port(p) { }
    std::string address;
    unsigned short port;
};

class RoverControl {
    public:
        RoverControl(HDLN& _handle);

        void update();

    private:
        void update_telemetry();
        void update_command();

        void set_cam_pan(float angle);
        void set_cam_tilt(float angle);
        void set_stereo_cam_pan(float angle);
        void set_stereo_cam_tilt(float angle);
        void set_l_motor(int power);
        void set_r_motor(int power);
        void set_sadl(int power);
        void set_blade(int power);
        void set_brake(bool on);
        void stereo_snapshot();
        void start_stereo_panorama();
        void update_stereo_panorama();

        HDLN& handle;
        UDPSocket socket;
        AdxlDriver adxl;
        Bmp085Driver barometer;
        Hmc5883lDriver mag;
        GpsDriver gps;
        PwmDriver pwm;
        float fwd_cam_pan;
        float fwd_cam_tilt;
        float stereo_cam_pan;
        float stereo_cam_tilt;
        int l_motor;
        int r_motor;
        int sadl;
        int blade;
        bool brake;

        std::vector<Client> clients;

        Timer tele_packet_timer; // Telemetry packet timer
        std::string telemetry_bundle; // Bundle of telemetry packets

        Timer tele_timer;

        Command cmd; // Command currently being executed, if any
        std::clock_t cmd_start; // Command timer
        double cmd_duration; // Command duration in seconds

        bool taking_panorama;
        unsigned int panorama_pos;
        PanoramaState panorama_state;
        Timer panorama_timer;
};

#endif // ROVER_CONTROL_H
