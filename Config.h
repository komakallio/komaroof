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

#ifndef CONFIG_H
#define CONFIG_H

#define BOARD_NAME "KOMAROOF"
#define PIN_UNUSED 46
#define PIN_ONE_WIRE_BUS 52
#define PIN_BUTTON_CLOSE 50
#define PIN_BUTTON_OPEN 48
#define PIN_BUTTON_EMERGENCYSTOP 3
#define PIN_LIMITSWITCH_OPEN 18
#define PIN_LIMITSWITCH_CLOSE 19
#define PIN_ENCODER_GATE_1 20
#define PIN_ENCODER_GATE_2 21
#define PIN_BATTERY_VOLTAGE A2
#define REFERENCE_VOLTAGE 4.987
#define BATTERY_RESISTOR_DIVISOR 3.25
#define MOTOR_CURRENT_LIMIT_MILLIAMPS 2500
#define MOTOR_CLOSING_CURRENT_LIMIT_MILLIAMPS 2500
#define MOTOR_POLARITY -1    // Change to -1 to invert movement direction
#define FULL_SPEED 300
#define CLOSING_SPEED 100
#define ENCODER_MAX_POSITION 715  // stop opening at this encoder position
#define ENCODER_MIN_POSITION 15  // start closing tightly at this encoder position
#define ENCODER_RESET_CLOSED 0
#define ENCODER_RESET_OPEN 650
#define MAX_MOVE_DURATION 180000 // 3 minutes
#define MAX_LOCK_MOVE_DURATION 7000 // 7 seconds
#define RAMP_DELTA 10 // amount to change roof speed per motor tick
#define LOCK_CURRENT_DETECTION_MILLIAMPS 100
#define LOCK_CURRENT_LIMIT_MILLIAMPS 500

#endif
