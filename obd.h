#ifndef OBD_H
#define OBD_H

#include <Arduino.h>

/*
 * ============================================================
 *  RoadSense OBD-II Interface
 *  ------------------------------------------------------------
 *  Provides vehicle telemetry through an MCP2515 CAN interface.
 *
 *  Current Parameters
 *      - Vehicle Speed
 *      - Engine RPM
 *      - Derived Acceleration
 *      - Brake Event Detection
 *
 *  Author: RoadSense
 * ============================================================
 */

class OBD
{
public:

    /* Standard OBD-II PIDs */
    static constexpr uint8_t PID_ENGINE_RPM       = 0x0C;
    static constexpr uint8_t PID_VEHICLE_SPEED    = 0x0D;
    static constexpr uint8_t PID_ENGINE_LOAD      = 0x04;
    static constexpr uint8_t PID_COOLANT_TEMP     = 0x05;
    static constexpr uint8_t PID_THROTTLE         = 0x11;

    /* Constructor */
    OBD();

    /* Initialize CAN hardware */
    bool begin();

    /* Poll ECU */
    void update();

    /* Getters */
    float getSpeed() const;
    float getRPM() const;
    float getAcceleration() const;

    bool isBraking() const;

    bool connected() const;

private:

    /* Latest Telemetry */
    float vehicleSpeed;
    float previousSpeed;

    float engineRPM;

    float acceleration;

    bool brakeEvent;

    bool canConnected;

    unsigned long previousUpdate;

    /*
     * Request PID
     */
    bool requestPID(uint8_t pid);

    /*
     * Decode ECU Response
     */
    bool parseResponse();

    /*
     * Compute Derived Quantities
     */
    void calculateAcceleration();

    /*
     * Brake Detection
     */
    void detectBraking();
};

#endif
