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
#define PIN_ONE_WIRE_BUS 44
#define PIN_BUTTON_CLOSE 42
#define PIN_BUTTON_OPEN 40
#define PIN_BUTTON_EMERGENCYSTOP 3
#define PIN_LIMITSWITCH_OPEN 18
#define PIN_LIMITSWITCH_CLOSE 19
#define PIN_ENCODER_GATE_1 20
#define PIN_ENCODER_GATE_2 21
#define MOTOR_CURRENT_LIMIT_MILLIAMPS 2500
#define MOTOR_POLARITY 1    // Change to -1 to invert movement direction
#define FULL_SPEED 400
#define CLOSING_SPEED 50
#define RAMP_LENGTH 20      // two seconds

void currentMeasurementTick();
void motorTick();
void temperatureTick();
void buttonTick();
void emergencyStop();

static OneWire oneWire(PIN_ONE_WIRE_BUS);
static DallasTemperature dallasTemperature(&oneWire);
static MessageHandler handler;
static NMEASerial serial(&handler);
static DualVNH5019MotorShield motorShield;
static PowerConsumptionLog powerConsumptionLog;
static Task motorTask(100, TASK_FOREVER, &motorTick);
static Task temperatureTask(1000, TASK_FOREVER, &temperatureTick);
static Task buttonTask(100, TASK_FOREVER, &buttonTick);
static Task currentMeasurementTask(10, TASK_FOREVER, &currentMeasurementTick);
static Scheduler taskScheduler;
static RoofState roofState = STOPPED;
static Phase phase = IDLE;
static int countSincePhase;
static int direction;
static float temperature = DEVICE_DISCONNECTED_C;
static bool temperatureRequested = false;
static volatile bool emergencyStopPressed = false;
static volatile bool limitSwitchOpenActive = false;
static volatile bool limitSwitchCloseActive = false;
static volatile int encoderState = 0;
static volatile int encoderPosition = 0;

void setup() {

    pinMode(PIN_BUTTON_CLOSE, INPUT);
    pinMode(PIN_BUTTON_OPEN, INPUT);
    pinMode(PIN_BUTTON_EMERGENCYSTOP, INPUT);
    pinMode(PIN_LIMITSWITCH_OPEN, INPUT);
    pinMode(PIN_LIMITSWITCH_CLOSE, INPUT);
    pinMode(PIN_ENCODER_GATE_1, INPUT);
    pinMode(PIN_ENCODER_GATE_2, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_EMERGENCYSTOP), emergencyStopISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LIMITSWITCH_CLOSE), limitSwitchCloseISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LIMITSWITCH_OPEN), limitSwitchOpenISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_GATE_1), encoderGate1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_GATE_2), encoderGate2ISR, CHANGE);

    Serial.begin(57600);
    handler.registerCommand("TEST", test);
    handler.registerCommand("STATUS", status);
    handler.registerCommand("OPEN", open);
    handler.registerCommand("CLOSE", close);
    handler.registerCommand("STOP", stop);

    powerConsumptionLog.setOverloadThreshold(MOTOR_CURRENT_LIMIT_MILLIAMPS);

    taskScheduler.init();
    taskScheduler.addTask(motorTask);
    taskScheduler.addTask(buttonTask);
    taskScheduler.addTask(currentMeasurementTask);

    dallasTemperature.begin();
    if (dallasTemperature.getDeviceCount() > 0) {
        dallasTemperature.setWaitForConversion(false);
        dallasTemperature.setResolution(12);
        taskScheduler.addTask(temperatureTask);
        temperatureTask.enable();
    }

    motorTask.enable();
//    buttonTask.enable();
    currentMeasurementTask.enable();
    test("");
}

void loop() {
    taskScheduler.execute();
}

void serialEvent() {
    serial.onSerialEvent();
}

void emergencyStopISR() {
    emergencyStopPressed = (digitalRead(PIN_BUTTON_EMERGENCYSTOP) == LOW);
}

void limitSwitchOpenISR() {
    limitSwitchOpenActive = (digitalRead(PIN_LIMITSWITCH_OPEN) == LOW);
}

void limitSwitchCloseISR() {
    limitSwitchCloseActive = (digitalRead(PIN_LIMITSWITCH_CLOSE) == LOW);
}

void encoderGate1ISR() {
    int previousEncoderState = encoderState;
    encoderState = (encoderState & 0x01) | (digitalRead(PIN_ENCODER_GATE_1) == LOW ? 2 : 0);
    updateEncoder(previousEncoderState, encoderState);
}

void encoderGate2ISR() {
    int previousEncoderState = encoderState;
    encoderState = (encoderState & 0x2) | (digitalRead(PIN_ENCODER_GATE_2) == LOW ? 1 : 0);
    updateEncoder(previousEncoderState, encoderState);
}

void updateEncoder(int previous, int current) {
    static int grayToBinary[4] = { 0, 1, 3, 2 };
    previous = grayToBinary[previous & 0x3];
    current = grayToBinary[current & 0x3];

    if (current == ((previous+1) & 0x3))
        encoderPosition++;
    if (current == ((previous-1) & 0x3))
        encoderPosition--;
}

void currentMeasurementTick() {
    powerConsumptionLog.measure(motorShield.getM1CurrentMilliamps());
}

void motorTick() {
    // called @ 10hz rate
    countSincePhase++;

    powerConsumptionLog.appendCurrentMeasurement();
    if (countSincePhase % 10 == 0) {
        status("");
        powerConsumptionLog.report(serial);
    }

    if (powerConsumptionLog.isOverload()) {
        motorShield.setM1Speed(0);
        if (roofState == CLOSING && phase == CLOSE_TIGHTLY) {
            roofState = CLOSED;
            phase = IDLE;
            encoderPosition = 0;
        } else {
            roofState = ERROR;
            phase = IDLE;
        }
    }

    if (emergencyStopPressed) {
        motorShield.setM1Speed(0);
        motorShield.setM2Speed(0);
        phase = IDLE;
        roofState = STOPPED;
        return;
    }

    if (limitSwitchOpenActive) {
        if (phase != IDLE && roofState == OPENING) {
            phase = RAMP_DOWN;
            countSincePhase = 0;
        }
        limitSwitchOpenActive = false;
    }

    if (limitSwitchCloseActive) {
        if (phase != IDLE && roofState == CLOSING) {
            phase = RAMP_DOWN;
            countSincePhase = 0;
        }
        limitSwitchCloseActive = false;
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
            int minimumPower = (roofState == CLOSING ? CLOSING_SPEED : 0);
            if (power < minimumPower)
                power = minimumPower;
            motorShield.setM1Speed(power * direction);
            if (power == minimumPower) {
                phase = IDLE;
                if (roofState == STOPPING)
                    roofState = STOPPED;
                else if (roofState == OPENING)
                    roofState = OPEN;
                else if (roofState == CLOSING)
                    phase = CLOSE_TIGHTLY;
            }
            break;
        }
        case CLOSE_TIGHTLY:
            // just coast ahead, until we trigger current limit when the roof hits the wall
            break;
    }
}

void buttonTick() {
    if (digitalRead(PIN_BUTTON_OPEN) == LOW && phase == IDLE) {
        open("");
    }
    if (digitalRead(PIN_BUTTON_CLOSE) == LOW && phase == IDLE) {
        close("");
    }
}

void temperatureTick() {
    // called @ 1hz

    if (temperatureRequested) {
        temperature = dallasTemperature.getTempCByIndex(0);
    }
    dallasTemperature.requestTemperaturesByIndex(0);
    temperatureRequested = true;
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
        roofState = STOPPING;
        phase = RAMP_DOWN;
        // start mid-ramp if we were not running at full speed
        countSincePhase = RAMP_LENGTH - RAMP_LENGTH * motorShield.getM1Speed() / FULL_SPEED;
    }
    serial.print("STOP,OK");
}

void status(const String&) {
    static const char* roofStateNames[] = { "STOPPED", "OPEN", "CLOSED", "OPENING", "CLOSING", "STOPPING", "ERROR" };
    static const char* phaseNames[] = { "IDLE", "RAMP_UP", "MOVE_UNTIL_NEAR", "RAMP_DOWN", "CLOSE_TIGHTLY" };

    String message = "STATUS,";
    message += "ROOF=";
    message += roofStateNames[roofState];
    message += ",PHASE=";
    message += phaseNames[phase];
    message += ",ENCODER=";
    message += encoderPosition;
    if (temperature != (float)DEVICE_DISCONNECTED_C) {
        message += ",TEMP1=";
        message += (int)(temperature);
        message += ".";
        message += (int)(temperature*10) % 10;
        message += (int)(roundf(temperature*100)) % 10;
    }
    serial.print(message);
}
