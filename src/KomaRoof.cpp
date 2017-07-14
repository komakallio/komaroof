#include <Arduino.h>

/*
 * Copyright (c) 2017 Jari Saukkonen
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

#include "Config.h"
#include "DualVNH5019MotorShield.h"
#include "NMEASerial.h"
#include "MessageHandler.h"
#include "PowerConsumptionLog.h"
#include "Roof.h"
#include "Settings.h"
#include "Version.h"

void currentMeasurementTick();
void motorTick();
void temperatureTick();
void voltageTick();
void buttonTick();
void timerTick();
void emergencyStop();

static OneWire oneWire(PIN_ONE_WIRE_BUS);
static DallasTemperature dallasTemperature(&oneWire);
static MessageHandler handler;
static NMEASerial serial(&handler);
static DualVNH5019MotorShield motorShield;
static PowerConsumptionLog roofPowerConsumptionLog;
static PowerConsumptionLog lockPowerConsumptionLog;
static Task motorTask(100, TASK_FOREVER, &motorTick);
static Task temperatureTask(1000, TASK_FOREVER, &temperatureTick);
static Task voltageTask(1000, TASK_FOREVER, &voltageTick);
static Task buttonTask(100, TASK_FOREVER, &buttonTick);
static Task currentMeasurementTask(10, TASK_FOREVER, &currentMeasurementTick);
static Task timerTask(100, TASK_FOREVER, &timerTick);
static Scheduler taskScheduler;
static RoofState roofState = STOPPED;
static Phase phase = IDLE;
static Settings settings;
static int roofSpeed = 0;
static int targetRoofSpeed = 0;
static int tickCount = 0;
static int direction;
static float temperature = DEVICE_DISCONNECTED_C;
static bool temperatureRequested = false;
static float batteryVoltage = 0.0f;
static volatile bool emergencyStopPressed = false;
static volatile bool limitSwitchOpenActive = false;
static volatile bool limitSwitchCloseActive = false;
static volatile int encoderState = 0;
static volatile int encoderPosition = 0;
static volatile bool emergencyStopInterrupt = false;
static unsigned long moveStartTime = 0;
static unsigned long lockStartTime = 0;
static bool lockCurrentDetected = false;

void emergencyStopISR();
void limitSwitchCloseISR();
void limitSwitchOpenISR();
void encoderGate1ISR();
void encoderGate2ISR();
void stop(const String&);
void open(const String&);
void close(const String&);
void lock(const String&);
void unlock(const String&);
void setspeed(const String&);
void status(const String&);
void test(const String&);
void encoderstatus();

void setup() {

    pinMode(PIN_UNUSED, INPUT);
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
    handler.registerCommand("LOCK", lock);
    handler.registerCommand("UNLOCK", unlock);
    handler.registerCommand("SETSPEED", setspeed);

    taskScheduler.init();
    taskScheduler.addTask(motorTask);
    taskScheduler.addTask(buttonTask);
    taskScheduler.addTask(currentMeasurementTask);
    taskScheduler.addTask(voltageTask);
    taskScheduler.addTask(timerTask);

    settings.load();
    roofState = settings.roofState;
    encoderPosition = settings.encoderPosition;

    motorShield.init();

    dallasTemperature.begin();
    if (dallasTemperature.getDeviceCount() > 0) {
        dallasTemperature.setWaitForConversion(false);
        dallasTemperature.setResolution(12);
        taskScheduler.addTask(temperatureTask);
        temperatureTask.enable();
    }

    motorTask.enable();
    buttonTask.enable();
    currentMeasurementTask.enable();
    voltageTask.enable();
    timerTask.enable();
    test("");
}

void logger(String msg, unsigned long timeMillis=0) {
    if (timeMillis == 0) {
        timeMillis = millis();
    }

    String logMessage = "LOG,TIME=";
    logMessage += timeMillis;
    logMessage += ",ENCODER=";
    logMessage += encoderPosition;
    logMessage += ",";
    logMessage += msg;

    serial.print(logMessage);
}

void loop() {
    taskScheduler.execute();
}

void serialEvent() {
    serial.onSerialEvent();
}

void emergencyStopISR() {
    emergencyStopPressed = (digitalRead(PIN_BUTTON_EMERGENCYSTOP) == HIGH);
    emergencyStopInterrupt = true;
}

void limitSwitchOpenISR() {
    limitSwitchOpenActive = (digitalRead(PIN_LIMITSWITCH_OPEN) == LOW);
}

void limitSwitchCloseISR() {
    limitSwitchCloseActive = (digitalRead(PIN_LIMITSWITCH_CLOSE) == LOW);
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

void encoderGate1ISR() {
    int previousEncoderState = encoderState;
    encoderState = (encoderState & 0x1) | (digitalRead(PIN_ENCODER_GATE_1) == LOW ? 0x2 : 0);
    updateEncoder(previousEncoderState, encoderState);
}

void encoderGate2ISR() {
    int previousEncoderState = encoderState;
    encoderState = (encoderState & 0x2) | (digitalRead(PIN_ENCODER_GATE_2) == LOW ? 0x1 : 0);
    updateEncoder(previousEncoderState, encoderState);
}

void currentMeasurementTick() {
    roofPowerConsumptionLog.measure(motorShield.getM1CurrentMilliamps());
    lockPowerConsumptionLog.measure(motorShield.getM2CurrentMilliamps());
}

void timerTick() {
    unsigned long t = millis();
    if ((roofState == OPENING || roofState == CLOSING) && (t - moveStartTime > MAX_MOVE_DURATION)) {
        motorShield.setM1Speed(0);
        motorShield.setM2Speed(0);
        roofSpeed = targetRoofSpeed = 0;
        roofState = ERROR;
        phase = IDLE;
        settings.save(roofState, encoderPosition);
        logger("ERROR=DURATION");
    }
}

void motorTick() {
    // called @ 10hz rate
    tickCount++;

    roofPowerConsumptionLog.appendCurrentMeasurement();
    lockPowerConsumptionLog.appendCurrentMeasurement();
    if (tickCount == 10) {
        status("");
        roofPowerConsumptionLog.report(serial);
        tickCount = 0;
    } else {
        if (roofState == OPENING || roofState == CLOSING) {
            encoderstatus();
        }
    }

    if (emergencyStopInterrupt && emergencyStopPressed) {
        if (digitalRead(PIN_BUTTON_CLOSE) == LOW) {
            encoderPosition = ENCODER_RESET_CLOSED;
            roofState = CLOSED;
            settings.save(roofState, encoderPosition);
        } else if (digitalRead(PIN_BUTTON_OPEN) == LOW) {
            encoderPosition = ENCODER_RESET_OPEN;
            roofState = OPEN;
            settings.save(roofState, encoderPosition);
        }
        emergencyStopInterrupt = false;
    }

    unsigned int currentThreshold = (phase == CLOSE_TIGHTLY ? MOTOR_CLOSING_CURRENT_LIMIT_MILLIAMPS : MOTOR_CURRENT_LIMIT_MILLIAMPS);
    if (roofPowerConsumptionLog.isOverload(currentThreshold)) {
        motorShield.setM1Speed(0);
        roofSpeed = targetRoofSpeed = 0;
        if (roofState == CLOSING && phase == CLOSE_TIGHTLY) {
            phase = LOCKING;
            encoderPosition = 0;
            lockStartTime = millis();
            lockCurrentDetected = false;
        } else if (roofState != CLOSED && roofState != OPEN) {
            roofState = ERROR;
            phase = IDLE;
            logger("ERROR=OVERCURRENT");
            settings.save(roofState, encoderPosition);
        }
    }
    if (lockPowerConsumptionLog.isOverload(LOCK_CURRENT_DETECTION_MILLIAMPS)) {
        lockCurrentDetected = true;
    }
    if (lockPowerConsumptionLog.isOverload(LOCK_CURRENT_LIMIT_MILLIAMPS)) {
        motorShield.setM2Speed(0);
        roofState = ERROR;
        phase = IDLE;
        logger("ERROR=LOCKOVERCURRENT");
        settings.save(roofState, encoderPosition);
    }

    if (motorShield.getM1Fault()) {
        motorShield.setM1Speed(0);
        roofSpeed = targetRoofSpeed = 0;
        if (roofState != ERROR) {
            roofState = ERROR;
            logger("ERROR=MOTOR1FAULT");
            settings.save(roofState, encoderPosition);
        }
        phase = IDLE;
    }

    if (emergencyStopPressed) {
        motorShield.setM1Speed(0);
        motorShield.setM2Speed(0);
        roofSpeed = targetRoofSpeed = 0;
        roofState = STOPPED;
        if (phase != IDLE) {
            logger("CMD=EMERGENCYSTOP");
            settings.save(roofState, encoderPosition);
        }
        phase = IDLE;
        return;
    }

    if (limitSwitchOpenActive) {
        if ((phase == RAMP_UP || phase == MOVE_UNTIL_NEAR) && roofState == OPENING) {
            logger("LIMIT=SWITCH_OPENING");
            phase = RAMP_DOWN;
            targetRoofSpeed = 0;
            if (encoderPosition == 0) {
                logger("ERROR=ENCODERMALFUNCTION");
            }
        }
        limitSwitchOpenActive = false;
    }

    if (encoderPosition >= ENCODER_MAX_POSITION) {
        if ((phase == RAMP_UP || phase == MOVE_UNTIL_NEAR) && roofState == OPENING) {
            logger("LIMIT=ENCODER_OPENING");
            phase = RAMP_DOWN;
            targetRoofSpeed = 0;
        }
    }

    if (limitSwitchCloseActive) {
        if ((phase == RAMP_UP || phase == MOVE_UNTIL_NEAR) && roofState == CLOSING) {
            logger("LIMIT=SWITCH_CLOSING");
            phase = RAMP_DOWN;
            targetRoofSpeed = CLOSING_SPEED;
        }
        limitSwitchCloseActive = false;
    }

    if (encoderPosition <= ENCODER_MIN_POSITION) {
        if ((phase == RAMP_UP || phase == MOVE_UNTIL_NEAR) && roofState == CLOSING) {
            phase = RAMP_DOWN;
            targetRoofSpeed = CLOSING_SPEED;
        }
    }

    if (targetRoofSpeed != roofSpeed) {
        roofSpeed += RAMP_DELTA * (targetRoofSpeed > roofSpeed ? 1 : -1);
        if (abs(roofSpeed-targetRoofSpeed) <= (RAMP_DELTA/2)) {
            roofSpeed = targetRoofSpeed;
        }
    }

    switch (phase) {
        case RAMP_UP: {
            motorShield.setM1Speed(roofSpeed * direction);
            if (roofSpeed == targetRoofSpeed) {
                phase = MOVE_UNTIL_NEAR;
            }
            break;
        }
        case RAMP_DOWN: {
            motorShield.setM1Speed(roofSpeed * direction);
            if (roofSpeed == targetRoofSpeed) {
                phase = IDLE;
                if (roofState == STOPPING)
                {
                    logger("STATE=STOPPED");
                    roofState = STOPPED;
                    settings.save(roofState, encoderPosition);
                }
                else if (roofState == OPENING)
                {
                    logger(String("STATE=OPEN,DURATION=") + (millis()-moveStartTime));
                    roofState = OPEN;
                    settings.save(roofState, encoderPosition);
                }
                else if (roofState == CLOSING)
                {
                    logger("STATE=TIGHTENING");
                    phase = CLOSE_TIGHTLY;
                }
            }
            break;
        }
        case CLOSE_TIGHTLY:
            // just coast ahead, until we trigger current limit when the roof hits the wall
            break;
        case LOCKING: {
            if (millis() - lockStartTime > MAX_LOCK_MOVE_DURATION) {
                motorShield.setM2Speed(0);
                roofState = CLOSED;
                phase = IDLE;
                logger(String("STATE=CLOSED,DURATION=") + (millis()-moveStartTime));
                settings.save(roofState, encoderPosition);
                if (!lockCurrentDetected) {
                    logger("WARN=NOLOCKCURRENT");
                }
            } else {
                // 800ms ramp
                motorShield.setM2Speed(min(400, (millis()-lockStartTime)/2));
            }
            break;
        }
        case UNLOCKING: {
            if (millis() - lockStartTime > MAX_LOCK_MOVE_DURATION) {
                motorShield.setM2Speed(0);
                phase = RAMP_UP;
                roofSpeed = 0;
                targetRoofSpeed = FULL_SPEED;
                if (!lockCurrentDetected) {
                    logger("WARN=NOLOCKCURRENT");
                }
            } else {
                // 800ms ramp
                motorShield.setM2Speed(-(long)min(400, (millis()-lockStartTime)/2));
            }
            break;
        }
        case IDLE:
        case MOVE_UNTIL_NEAR:
            // no need to do anything
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

void voltageTick() {
    batteryVoltage = analogRead(PIN_BATTERY_VOLTAGE) * REFERENCE_VOLTAGE / 1024.0 * BATTERY_RESISTOR_DIVISOR;
}

void test(const String&) {
    serial.print(String(BOARD_NAME) + String(",VER=") + String(KOMAROOF_VERSION));
}

void open(const String&) {
    if (roofState == CLOSED || roofState == STOPPED) {
        lockCurrentDetected = false;
        roofState = OPENING;
        phase = UNLOCKING;
        direction = MOTOR_POLARITY;
        moveStartTime = lockStartTime = millis();
        logger("CMD=OPEN", moveStartTime);
        serial.print("OPEN,OK");
    } else {
        serial.print("OPEN,ERR");
    }
}

void unlock(const String&) {
    serial.print("UNLOCK,OK");
    moveStartTime = millis();
    logger("CMD=UNLOCK", moveStartTime);
    while (millis() - moveStartTime < MAX_LOCK_MOVE_DURATION && motorShield.getM2CurrentMilliamps() < LOCK_CURRENT_LIMIT_MILLIAMPS*1.1) {
        motorShield.setM2Speed(-(long)min(400, (millis()-moveStartTime)/2));
        delay(50);
    }
    motorShield.setM2Speed(0);
}

void lock(const String&) {
    serial.print("LOCK,OK");
    moveStartTime = millis();
    logger("CMD=LOCK", moveStartTime);
    while (millis() - moveStartTime < MAX_LOCK_MOVE_DURATION && motorShield.getM2CurrentMilliamps() < LOCK_CURRENT_LIMIT_MILLIAMPS*1.1) {
        motorShield.setM2Speed(min(400, ((long)millis()-moveStartTime)/2));
        delay(50);
    }
    motorShield.setM2Speed(0);
}

void close(const String&) {
    if (roofState == OPEN || roofState == STOPPED) {
        lockCurrentDetected = false;
        roofState = CLOSING;
        phase = RAMP_UP;
        targetRoofSpeed = FULL_SPEED;
        roofSpeed = 0;
        direction = -MOTOR_POLARITY;
        moveStartTime = millis();
        logger("CMD=CLOSE", moveStartTime);
        serial.print("CLOSE,OK");
    } else {
        serial.print("CLOSE,ERR");
    }
}

void stop(const String&) {
    if (phase != IDLE) {
        roofState = STOPPING;
        phase = RAMP_DOWN;
        targetRoofSpeed = 0;
    }
    if (roofState == ERROR) {
        roofState = STOPPED;
        phase = IDLE;
        settings.save(roofState, encoderPosition);
    }
    logger("CMD=STOP");
    serial.print("STOP,OK");
}

void encoderstatus() {
    String message = "STATUS,";
    message += ",ENCODER=";
    message += encoderPosition;
    serial.print(message);
}

void status(const String&) {
    static const char* roofStateNames[] = { "STOPPED", "OPEN", "CLOSED", "OPENING", "CLOSING", "STOPPING", "ERROR" };
    static const char* phaseNames[] = { "IDLE", "RAMP_UP", "MOVE_UNTIL_NEAR", "RAMP_DOWN", "CLOSE_TIGHTLY", "LOCKING", "UNLOCKING" };

    String message = "STATUS,";
    message += "ROOF=";
    message += roofStateNames[roofState];
    message += ",PHASE=";
    message += phaseNames[phase];
    message += ",ENCODER=";
    message += encoderPosition;
    message += ",BATTERYVOLTAGE=";
    message += batteryVoltage;
    message += ",SPEED=";
    message += roofSpeed;
    if (temperature != (float)DEVICE_DISCONNECTED_C) {
        message += ",TEMP1=";
        message += temperature;
    }
    serial.print(message);
}

void setspeed(const String& speedAsString) {
    targetRoofSpeed = atoi(speedAsString.c_str());
    serial.print("SETSPEED,OK");
}
