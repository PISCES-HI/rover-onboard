#include <iostream>

#include "dln/dln.h"
#include "dln/dln_generic.h"
#include "dln/dln_gpio.h"

#include "analog.h"
#include "i2c.h"
#include "PwmDriver.h"
#include "RoverControl.h"

#include "Serial.h"
#include "GpsDriver.h"

void gps_loop(GpsDriver& gps);

int main() {
    /*GpsDriver gps(Serial("/dev/ttyUSB0"));

    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Set the update rate
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

    gps.sendCommand(PGCMD_ANTENNA);

    // Ask for firmware version
    gps.sendCommand(PMTK_Q_RELEASE);

    while (true) gps_loop(gps);*/

    DLN_RESULT result;

    DlnConnect("localhost", DLN_DEFAULT_SERVER_PORT);

    // Check device count
    uint32_t device_count;
    result = DlnGetDeviceCount(&device_count);
    if (!DLN_SUCCEEDED(result)) {
        std::cout << "Failed to get DLN device count" << std::endl;
        return -1;
    }
    if (device_count != 1) {
        std::cout << "There should be one and only one DLN device connected. There are "
                  << device_count << std::endl;
        return -1;
    }

    // Try to open our device
    HDLN handle;
    result = DlnOpenDevice(0, &handle);
    if (!DLN_SUCCEEDED(result)) {
        std::cout << "Failed to open DLN device" << std::endl;
        return -1;
    }

    // Print out some info about our DLN device
    DLN_VERSION version;
    uint32_t sn, id;
    DlnGetVersion(handle, &version);
    DlnGetDeviceSn(handle, &sn);
    DlnGetDeviceId(handle, &id);
    std::cout << "Opened DLN device: " << sn << "\t" << id << std::endl;

    // Initialize all the things
    init_i2c(handle);
    init_analog(handle);

    // Instantiate the rover controller
    RoverControl rover(handle);

    while (true) {
        rover.update();
    }

    cleanup_analog(handle);

    DlnCloseHandle(handle);

    DlnDisconnectAll();

    return 0;
}

void gps_loop(GpsDriver& gps) {
    char c = gps.read();

    if (c) std::cout << c << std::endl;

    // if a sentence is received, we can check the checksum, parse it...
    if (gps.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(gps.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        if (!gps.parse(gps.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
            return;  // we can fail to parse a sentence in which case we should just wait for another

        std::cout << "\nTime: ";
        std::cout << gps.hour; std::cout << ':';
        std::cout << gps.minute; std::cout << ':';
        std::cout << gps.seconds; std::cout << '.';
        std::cout << gps.milliseconds << std::endl;
        std::cout << "Date: ";
        std::cout << gps.day; std::cout << '/';
        std::cout << gps.month; std::cout << "/20";
        std::cout << gps.year << std::endl;
        std::cout << "Fix: "; std::cout << (int)gps.fix;
        std::cout << " quality: "; std::cout << (int)gps.fixquality << std::endl; 
        if (gps.fix) {
            std::cout << "Location: ";
            std::cout << gps.latitude; std::cout << gps.lat;
            std::cout << ", "; 
            std::cout << gps.longitude; std::cout << gps.lon << std::endl;     
            std::cout << "Speed (knots): "; std::cout << gps.speed << std::endl;
            std::cout << "Angle: "; std::cout << gps.angle << std::endl;
            std::cout << "Altitude: "; std::cout << gps.altitude << std::endl;
            std::cout << "Satellites: "; std::cout << (int)gps.satellites << std::endl;
        }
    }
}
