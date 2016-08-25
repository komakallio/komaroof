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
#include <Arduino.h>
#include "MessageHandler.h"

void MessageHandler::onMessage(String message) {
    int pos = 0;
    String cmd;
    String params;
    if (message.indexOf(',') != -1) {
        cmd = message.substring(0, message.indexOf(','));
        params = message.substring(message.indexOf(',') + 1);
    } else {
        cmd = message;
        params = "";
    }

    for (int i = 0; i < m_numCommands; i++) {
        if (cmd == m_commands[i].command) {
            m_commands[i].callback(params);
            return;
        }
    }
}

void MessageHandler::registerCommand(const String& command, void(*callback)(const String&))
{
    if (m_numCommands >= MAX_COMMANDS)
        return;

    m_commands[m_numCommands].command = command;
    m_commands[m_numCommands].callback = callback;
    m_numCommands++;
}
