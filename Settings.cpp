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

#include <EEPROM.h>
#include "Settings.h"

void Settings::load() {
    uint8_t* arr = (uint8_t*)this;
    for (int i = 0; i < sizeof(Settings); i++) {
        arr[i] = EEPROM.read(i);
    }

    if (magic != 0x524F4F46) { // 'ROOF'
        // invalid eeprom data, reset to defaults
        magic = 0x524F4F46;
        roofState = STOPPED;
        encoderPosition = 0;
        save();
    }
}

void Settings::save() {
    uint8_t* arr = (uint8_t*)this;
    for (int i = 0; i < sizeof(Settings); i++) {
        if (EEPROM.read(i) != arr[i]) {
            EEPROM.write(i, arr[i]);
        }
    }
}

void Settings::save(RoofState roofState, int encoderPosition) {
    this->roofState = roofState;
    this->encoderPosition = encoderPosition;
    save();
}
