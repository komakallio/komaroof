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

#include "NMEASerial.h"

#ifndef POWERCONSUMPTIONLOG_H
#define POWERCONSUMPTIONLOG_H

class PowerConsumptionLog {
public:
    PowerConsumptionLog();

    void measure(unsigned int milliAmps);
    void appendCurrentMeasurement();
    void report(NMEASerial& serial);
    void setOverloadThreshold(unsigned int milliAmps);
    bool isOverload(unsigned int threshold) const;

private:
    unsigned int m_window[16];
    int m_windowPos;
    unsigned int m_overloadThreshold;
    unsigned int m_data[16];
    unsigned int m_pos;
};

#endif
