/**
 * Copyright (c) 2025, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may
 * may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file parser.c
 * @brief Implements HID data parsers for specific game controllers.
 *
 * This file contains the implementations of parser functions for
 * the Logitech controllers (F310, F710) in both X-Input and D-Input modes.
 * Each function is responsible for interpreting the controller-specific raw
 * byte array from a HID report and extracting button and analog stick data.
 */
 
#include "parser.h"
#include <stdio.h>
 
// External variable defined in joy_teleop_node.cpp, used for verbose logging if needed.
extern int verbose;
 
// Defines the total number of supported controller models in the lookup table.
#define CONTROLLER_COUNT 4
 
/**
 * @brief A lookup table mapping controller Vendor IDs (VID) and Product IDs (PID)
 * to their corresponding parser functions.
 *
 * This array allows the system to dynamically select the correct parser at
 * runtime when a new controller is connected.
 */
const struct _device_lookup_storage _device_lookup[CONTROLLER_COUNT] = {
    {0x046d, 0xc21d, prs_v046d_pc21d}, // Logitech F310/F710 (X-Input mode)
    {0x046d, 0xc216, prs_v046d_pc216}, // Logitech F310 (D-Input mode)
    {0x046d, 0xc21f, prs_v046d_pc21d}, // Logitech F710 (Wireless, X-Input mode)
    {0x046d, 0xc219, prs_v046d_pc219}  // Logitech F710 (Wireless, D-Input mode)
};
 
/**
 * @brief Retrieves the correct parser function for a given device VID and PID.
 *
 * @param vid The Vendor ID of the connected controller.
 * @param pid The Product ID of the connected controller.
 * @return A function pointer to the specific parser. If no match is found,
 * it returns a pointer to a generic, non-functional parser.
 */
parser_func_t get_parser(int vid, int pid) {
    for (int i = 0; i < CONTROLLER_COUNT; i++) {
        if (_device_lookup[i].vid == vid && _device_lookup[i].pid == pid) {
            return _device_lookup[i].parser;
        }
    }
    return prs_generic; // Return a safe default if the controller is unknown.
}
 
/**
 * @brief Checks if a controller with the given VID and PID is supported.
 *
 * @param vid The Vendor ID of the connected controller.
 * @param pid The Product ID of the connected controller.
 * @return 1 if the controller is supported, -1 otherwise.
 */
int check_allowed(int vid, int pid) {
    for (int i = 0; i < CONTROLLER_COUNT; i++) {
        if (_device_lookup[i].vid == vid && _device_lookup[i].pid == pid) {
            return 1; // Found a match.
        }
    }
    return -1; // No match found.
}
 
/**
 * @brief A generic, default parser that does nothing.
 * @return Always returns 0.
 */
int prs_generic(int mode, int data_len, uint8_t *data) {
    return 0;
}
 
/**
 * @brief Parser for Logitech controllers in X-Input mode (VID 0x046d, PID 0xc21d/0xc21f).
 *
 * This function decodes the 14-byte data packet sent by the controller
 * in X-Input mode.
 *
 * @param mode The type of data to extract (e.g., PARSER_MODE_BUTTON).
 * @param data_len The length of the raw data buffer.
 * @param data Pointer to the raw HID data buffer.
 * @return The parsed integer value (button bitmask or analog axis value).
 */
int prs_v046d_pc21d(int mode, int data_len, uint8_t *data) {
    if (data_len < 14) return 0; // Safety check for data length.
 
    switch (mode) {
        case PARSER_MODE_BUTTON: {
            uint32_t button = 0;
            // Bytes 2 and 3 contain the primary button states as a bitmask.
            button |= (SCREEN_DPAD_UP_GAME_BUTTON * ((data[2] & 0x01) ? 1 : 0));
            button |= (SCREEN_DPAD_DOWN_GAME_BUTTON * ((data[2] & 0x02) ? 1 : 0));
            button |= (SCREEN_DPAD_LEFT_GAME_BUTTON * ((data[2] & 0x04) ? 1 : 0));
            button |= (SCREEN_DPAD_RIGHT_GAME_BUTTON * ((data[2] & 0x08) ? 1 : 0));
            button |= (SCREEN_MENU2_GAME_BUTTON * ((data[2] & 0x10) ? 1 : 0)); // START button
            button |= (SCREEN_MENU1_GAME_BUTTON * ((data[2] & 0x20) ? 1 : 0)); // BACK button
            button |= (SCREEN_L3_GAME_BUTTON * ((data[2] & 0x40) ? 1 : 0)); // Left stick click
            button |= (SCREEN_R3_GAME_BUTTON * ((data[2] & 0x80) ? 1 : 0)); // Right stick click
            button |= (SCREEN_L1_GAME_BUTTON * ((data[3] & 0x01) ? 1 : 0)); // LB
            button |= (SCREEN_R1_GAME_BUTTON * ((data[3] & 0x02) ? 1 : 0)); // RB
            button |= (SCREEN_A_GAME_BUTTON * ((data[3] & 0x10) ? 1 : 0));
            button |= (SCREEN_B_GAME_BUTTON * ((data[3] & 0x20) ? 1 : 0));
            button |= (SCREEN_X_GAME_BUTTON * ((data[3] & 0x40) ? 1 : 0));
            button |= (SCREEN_Y_GAME_BUTTON * ((data[3] & 0x80) ? 1 : 0));
 
            // Bytes 4 and 5 represent analog triggers (LT/RT), treated as buttons here.
            button |= (data[4] > 20 ? SCREEN_L2_GAME_BUTTON : 0);
            button |= (data[5] > 20 ? SCREEN_R2_GAME_BUTTON : 0);
            return button;
        }
        // Analog sticks are 16-bit signed values. Combine two bytes and center the value around 0.
        case PARSER_MODE_ANALOG1x: return (data[6] * 0x100) + data[7] - 32768;
        case PARSER_MODE_ANALOG1y: return ((data[8] * 0x100) + data[9] - 32768) * -1;
        case PARSER_MODE_ANALOG2x: return (data[10] * 0x100) + data[11] - 32768;
        case PARSER_MODE_ANALOG2y: return ((data[12] * 0x100) + data[13] - 32768) * -1;
    }
    return 0;
}
 
/**
 * @brief Internal helper function containing the shared D-Input mode parsing logic.
 *
 * This function is marked 'static' as it's only intended for use within this file.
 * It centralizes the parsing logic for both F310 and F710 D-Input modes.
 */
static int _parse_d_mode_data(int mode, uint8_t *data) {
    switch (mode) {
        case PARSER_MODE_BUTTON: {
            uint32_t buttons = 0;
            // Byte 4 contains the face buttons and D-Pad state.
            if (data[4] & 0x10) buttons |= SCREEN_X_GAME_BUTTON; // Button 1
            if (data[4] & 0x20) buttons |= SCREEN_A_GAME_BUTTON; // Button 2
            if (data[4] & 0x40) buttons |= SCREEN_B_GAME_BUTTON; // Button 3
            if (data[4] & 0x80) buttons |= SCREEN_Y_GAME_BUTTON; // Button 4
 
            // The lower 4 bits of byte 4 represent the D-Pad as a hat switch (0-7 for directions, 8 for neutral).
            uint8_t dpad = data[4] & 0x0F;
            if (dpad == 0) buttons |= SCREEN_DPAD_UP_GAME_BUTTON;
            else if (dpad == 1) buttons |= SCREEN_DPAD_UP_GAME_BUTTON | SCREEN_DPAD_RIGHT_GAME_BUTTON;
            else if (dpad == 2) buttons |= SCREEN_DPAD_RIGHT_GAME_BUTTON;
            else if (dpad == 3) buttons |= SCREEN_DPAD_DOWN_GAME_BUTTON | SCREEN_DPAD_RIGHT_GAME_BUTTON;
            else if (dpad == 4) buttons |= SCREEN_DPAD_DOWN_GAME_BUTTON;
            else if (dpad == 5) buttons |= SCREEN_DPAD_DOWN_GAME_BUTTON | SCREEN_DPAD_LEFT_GAME_BUTTON;
            else if (dpad == 6) buttons |= SCREEN_DPAD_LEFT_GAME_BUTTON;
            else if (dpad == 7) buttons |= SCREEN_DPAD_UP_GAME_BUTTON | SCREEN_DPAD_LEFT_GAME_BUTTON;
            
            // Byte 5 contains shoulder and menu buttons.
            if (data[5] & 0x01) buttons |= SCREEN_L1_GAME_BUTTON;
            if (data[5] & 0x02) buttons |= SCREEN_R1_GAME_BUTTON;
            if (data[5] & 0x04) buttons |= SCREEN_L2_GAME_BUTTON;
            if (data[5] & 0x08) buttons |= SCREEN_R2_GAME_BUTTON;
            if (data[5] & 0x10) buttons |= SCREEN_MENU1_GAME_BUTTON; // BACK
            if (data[5] & 0x20) buttons |= SCREEN_MENU2_GAME_BUTTON; // START
            return buttons;
        }
        // Analog sticks are 8-bit values centered around 128.
        case PARSER_MODE_ANALOG1x: return data[0] - 128;
        case PARSER_MODE_ANALOG1y: return data[1] - 128;
        case PARSER_MODE_ANALOG2x: return data[2] - 128;
        case PARSER_MODE_ANALOG2y: return data[3] - 128;
    }
    return 0;
}
 
/**
 * @brief Parser for the Logitech F310 controller in D-Input mode (8-byte report).
 */
int prs_v046d_pc216(int mode, int data_len, uint8_t *data) {
    if (data_len < 8) return 0;
    // This function is just a wrapper that performs a length check before
    // calling the common helper function.
    return _parse_d_mode_data(mode, data);
}
 
/**
 * @brief Parser for the Logitech F710 controller in D-Input mode (7-byte report).
 */
int prs_v046d_pc219(int mode, int data_len, uint8_t *data) {
    if (data_len < 7) return 0;
    // This function is also a wrapper with a different length check,
    // calling the same common helper function.
    return _parse_d_mode_data(mode, data);
}
 