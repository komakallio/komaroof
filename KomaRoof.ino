/*
 * Copyright (c) 2016 Jari Saukkonen
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <DallasTemperature.h>
#include <OneWire.h>
#include <TaskScheduler.h>

#include "DualVNH5019MotorShield.h"
#include "NMEASerial.h"
#include "MessageHandler.h"
#include "PowerConsumptionLog.h"
#include "Roof.h"
#include "Version.h"

#define BOARD_NAME "KOMAROOF"
#define ONE_WIRE_BUS 2
#define MOTOR_POLARITY 1    // Change to -1 to invert movement direction
#define FULL_SPEED 400
#define RAMP_LENGTH 20      // two seconds

void motorTick();
void temperatureTick();

static OneWire oneWire(ONE_WIRE_BUS);
static DallasTemperature dallasTemperature(&oneWire);
static MessageHandler handler;
static NMEASerial serial(&handler);
static DualVNH5019MotorShield motorShield;
static PowerConsumptionLog powerConsumptionLog;
static Task motorTask(100, TASK_FOREVER, &motorTick);
static Task temperatureTask(1000, TASK_FOREVER, &temperatureTick);
static Scheduler taskScheduler;
static RoofState roofState = STOPPED;
static Phase phase = IDLE;
static int countSincePhase;
static int direction;
static float temperature = DEVICE_DISCONNECTED_C;
static bool temperatureRequested = false;

void setup() {
    Serial.begin(57600);
    handler.registerCommand("TEST", test);
    handler.registerCommand("STATUS", status);
    handler.registerCommand("OPEN", open);
    handler.registerCommand("CLOSE", close);
    handler.registerCommand("STOP", stop);

    taskScheduler.init();
    taskScheduler.addTask(motorTask);

    dallasTemperature.begin();
    if (dallasTemperature.getDeviceCount() > 0) {
        dallasTemperature.setWaitForConversion(false);
        dallasTemperature.setResolution(12);
        taskScheduler.addTask(temperatureTask);
    }

    motorTask.enable();
    test("");
}

void loop() {
    taskScheduler.execute();
}

void serialEvent() {
    serial.onSerialEvent();
}

void motorTick() {
    // called @ 10hz rate
    countSincePhase++;

    powerConsumptionLog.append(motorShield.getM1CurrentMilliamps());
    if (countSincePhase % 10 == 0) {
        status("");
        powerConsumptionLog.report(serial);
    }

    switch (phase) {
        case RAMP_UP: {
            int power = FULL_SPEED * countSincePhase / RAMP_LENGTH;
            if (power > FULL_SPEED)
                power = FULL_SPEED;
            motorShield.setM1Speed(power * direction);
            if (power == FULL_SPEED)
                phase = MOVE_UNTIL_NEAR;
            break;
        }
        case RAMP_DOWN: {
            int power = FULL_SPEED - FULL_SPEED * countSincePhase / RAMP_LENGTH;
            if (power < 0)
                power = 0;
            motorShield.setM1Speed(power * direction);
            if (power == 0) {
                phase = IDLE;
                roofState = STOPPED;
            }
            break;
        }
    }
}

void temperatureTick() {
    // called @ 1hz

    // alternate between sending requests and reading temperature
    if (!temperatureRequested) {
        dallasTemperature.requestTemperaturesByIndex(0);
        temperatureRequested = true;
    } else {
        temperature = dallasTemperature.getTempCByIndex(0);
        temperatureRequested = false;
    }
}

void test(const String&) {
    serial.print(String(BOARD_NAME) + String(",VER=") + String(KOMAROOF_VERSION));
}

void open(const String&) {
    if (roofState == CLOSED || roofState == STOPPED) {
        roofState = OPENING;
        phase = RAMP_UP;
        countSincePhase = 0;
        direction = MOTOR_POLARITY;
        serial.print("OPEN,OK");
    } else {
        serial.print("OPEN,ERR");
    }
}

void close(const String&) {
    if (roofState == OPEN || roofState == STOPPED) {
        roofState = CLOSING;
        phase = RAMP_UP;
        countSincePhase = 0;
        direction = -MOTOR_POLARITY;
        serial.print("CLOSE,OK");
    } else {
        serial.print("CLOSE,ERR");
    }
}

void stop(const String&) {
    if (phase != IDLE) {
        phase = RAMP_DOWN;
        countSincePhase = 0;
    }
    serial.print("STOP,OK");
}

void status(const String&) {
    static const char* roofStateNames[] = { "STOPPED", "OPEN", "CLOSED", "OPENING", "CLOSING", "ERROR" };
    static const char* phaseNames[] = { "IDLE", "RAMP_UP", "MOVE_UNTIL_NEAR", "RAMP_DOWN", "CLOSE_TIGHTLY" };

    String message = "STATUS,";
    message += "ROOF=";
    message += roofStateNames[roofState];
    message += ",PHASE=";
    message += phaseNames[phase];
    if (temperature != (float)DEVICE_DISCONNECTED_C) {
        message += ",TEMP1=";
        message += (int)(temperature);
        message += ".";
        message += (int)(roundf(temperature*10)) % 10;
    }
    serial.print(message);
}
