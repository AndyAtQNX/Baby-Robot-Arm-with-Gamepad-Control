/**
 * Copyright (c) 2025, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file joy_teleop_node.cpp
 * @brief ROS2 node for interfacing with HID joysticks on QNX.
 *
 * This node connects directly to the QNX HIDDI (Human Interface Device Driver
 * Interface) service to read raw data from connected game controllers. It uses
 * a parser system to interpret the device-specific data format and then
 * publishes it as a standardized `sensor_msgs::msg::Joy` message on the `/joy`
 * topic for other ROS2 nodes to collect.
 *
 * It is also designed to be able to handle controller insertions and removals
 * during runtime.
 */
 
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <unistd.h>
#include <pthread.h>
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
 
// QNX-specific headers for the HIDDI service.
#include <sys/hiddi.h>
#include <sys/hidut.h>
 
#include "parser.h"
 
using namespace std::chrono_literals;
 
// This global variable is used by the parser functions if needed.
int verbose = 0;
 
class JoyTeleopNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the JoyTeleopNode.
     *
     * Initializes the ROS2 node, creates the publisher and timer, and
     * initializes the mutex for thread-safe access to joystick state.
     */
    JoyTeleopNode() : Node("joy_teleop_node") {
        instance_ = this; // Store a static pointer to this instance for C-style callbacks.
        pthread_mutex_init(&joy_state_lock_, NULL);
        // Create a publisher for the "/joy" topic.
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
        // Create a wall timer to publish joystick messages at a fixed rate (50 Hz).
        timer_ = this->create_wall_timer(20ms, std::bind(&JoyTeleopNode::publish_joy_message, this));
    }
 
    /**
     * @brief Destructor for the JoyTeleopNode.
     *
     * Cleans up resources, including disconnecting from the HIDDI service
     * and destroying the mutex.
     */
    ~JoyTeleopNode() {
        if (g_hid_conn) {
            hidd_disconnect(g_hid_conn);
        }
        pthread_mutex_destroy(&joy_state_lock_);
    }
 
    /**
     * @brief Initializes the connection to the HIDDI service.
     */
    void initialize_hid_connection() {
        if (!init_hidd()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize HIDDI connection! Shutting down.");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "HIDDI Joy Teleop node started successfully.");
        }
    }
 
private:
    /**
     * @brief Publishes the current joystick state as a Joy message.
     *
     * This function is called by the timer at a regular interval. It copies
     * the latest joystick data in a thread-safe manner and publishes it.
     */
    void publish_joy_message() {
        auto joy_msg = sensor_msgs::msg::Joy();
        joy_msg.header.stamp = this->get_clock()->now();
 
        // Lock the mutex to safely copy the joystick state.
        pthread_mutex_lock(&joy_state_lock_);
        joy_msg.axes.assign(latest_axes_.begin(), latest_axes_.end());
        joy_msg.buttons.assign(latest_buttons_.begin(), latest_buttons_.end());
        pthread_mutex_unlock(&joy_state_lock_);
        
        publisher_->publish(joy_msg);
    }
 
    /**
     * @brief Sets up the connection parameters and connects to the HIDDI service.
     * @return true on success, false on failure.
     */
    bool init_hidd() {
        // Define the C-style callback functions for HIDDI events.
        static hidd_funcs_t hid_funcs = {
            .nentries = _HIDDI_NFUNCS,
            .insertion = &JoyTeleopNode::on_hidd_insert, // Called on device connection
            .removal = &JoyTeleopNode::on_hidd_remove,   // Called on device disconnection
            .report = &JoyTeleopNode::on_hidd_report      // Called when device sends data
        };
        hidd_connect_parm_t hid_parms = {};
        hid_parms.funcs = &hid_funcs;
 
        // Attempt to connect to the HIDDI service.
        if (hidd_connect(&hid_parms, &g_hid_conn) != EOK) {
            perror("hidd_connect");
            return false;
        }
        return true;
    }
 
    /**
     * @brief Static callback executed when a HID report (joystick data) is received.
     *
     * @param data Raw byte buffer containing the report data.
     * @param len Length of the data buffer.
     * @param user_data Pointer to user-defined data, used here to get the device instance.
     */
    static void on_hidd_report(struct hidd_connection *, struct hidd_report *, void *data, uint32_t len, uint32_t, void *user_data) {
        if (!instance_) return;
        hidd_device_instance_t *inst = *(hidd_device_instance_t **)user_data;
        if (!inst) return;
 
        // Get the correct parser function for this specific controller model.
        parser_func_t parser = get_parser(inst->device_ident.vendor_id, inst->device_ident.product_id);
        if (!parser) return;
 
        // Parse the raw button data.
        int raw_buttons = parser(PARSER_MODE_BUTTON, len, (uint8_t*)data);
        
        // Lock the mutex before updating the shared joystick state vectors.
        pthread_mutex_lock(&instance_->joy_state_lock_);
        
        // Determine the correct normalization divisor based on the controller mode (X-Input vs. D-Input).
        bool is_xinput = (inst->device_ident.product_id == 0xc21d || inst->device_ident.product_id == 0xc21f);
        float divisor = is_xinput ? 32768.0f : 128.0f;
 
        // Parse and normalize analog stick values.
        instance_->latest_axes_[0] = parser(PARSER_MODE_ANALOG1x, len, (uint8_t*)data) / divisor* -1;
        instance_->latest_axes_[1] = parser(PARSER_MODE_ANALOG1y, len, (uint8_t*)data) / divisor * -1; // Invert Y-axis
        instance_->latest_axes_[2] = parser(PARSER_MODE_ANALOG2x, len, (uint8_t*)data) / divisor;
        instance_->latest_axes_[3] = parser(PARSER_MODE_ANALOG2y, len, (uint8_t*)data) / divisor; 
        
        // Map D-Pad buttons to axes, as is common in ROS.
        instance_->latest_axes_[4] = (raw_buttons & SCREEN_DPAD_RIGHT_GAME_BUTTON ? 1.0f : (raw_buttons & SCREEN_DPAD_LEFT_GAME_BUTTON ? -1.0f : 0.0f));
        instance_->latest_axes_[5] = (raw_buttons & SCREEN_DPAD_UP_GAME_BUTTON ? 1.0f : (raw_buttons & SCREEN_DPAD_DOWN_GAME_BUTTON ? -1.0f : 0.0f));
 
        // Map raw button bits to the button vector.
        instance_->latest_buttons_[0] = (raw_buttons & SCREEN_A_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[1] = (raw_buttons & SCREEN_B_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[2] = (raw_buttons & SCREEN_X_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[3] = (raw_buttons & SCREEN_Y_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[4] = (raw_buttons & SCREEN_L1_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[5] = (raw_buttons & SCREEN_R1_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[6] = (raw_buttons & SCREEN_L2_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[7] = (raw_buttons & SCREEN_R2_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[8] = (raw_buttons & SCREEN_MENU1_GAME_BUTTON) ? 1 : 0;
        instance_->latest_buttons_[9] = (raw_buttons & SCREEN_MENU2_GAME_BUTTON) ? 1 : 0;
        
        pthread_mutex_unlock(&instance_->joy_state_lock_);
    }
 
    /**
     * @brief Tries to attach to a HID report to start receiving data.
     * @return 0 on success, -1 on failure.
     */
    static int try_attach_to_report(struct hidd_connection *conn, hidd_device_instance_t *inst, struct hidd_collection *collection) {
        struct hidd_report_instance *report_inst = NULL;
        if (hidd_get_report_instance(collection, 0, HID_INPUT_REPORT, &report_inst) == EOK) {
            struct hidd_report *report_handle = NULL;
            if (hidd_report_attach(conn, inst, report_inst, 0, sizeof(void*), &report_handle) == EOK) {
                void **user_data_slot = static_cast<void**>(hidd_report_extra(report_handle));
                if (user_data_slot) { *user_data_slot = inst;} // Store instance pointer for the report callback
                printf("[INFO] [%s]: Successfully attached to controller report.\n", instance_->get_name());
                fflush(stdout);
                return 0;
            }
        }
        return -1;
    }
    
    /**
     * @brief Static callback executed when a supported HID device is inserted.
     */
    static void on_hidd_insert(struct hidd_connection *conn, hidd_device_instance_t *inst) {
        if (!instance_) return;
        // Check if the connected device is one of our supported controllers.
        if (check_allowed(inst->device_ident.vendor_id, inst->device_ident.product_id) != 1) { return; }
        printf("[INFO] [%s]: Supported controller found: VID=0x%04X, PID=0x%04X\n", instance_->get_name(), inst->device_ident.vendor_id, inst->device_ident.product_id);
        fflush(stdout);
        
        // Iterate through the device's collections to find a suitable input report to attach to.
        struct hidd_collection **collections;
        _Uint16t num_collections;
        if (hidd_get_collections(inst, NULL, &collections, &num_collections) == EOK) {
            for (int i = 0; i < num_collections; i++) {
                if (try_attach_to_report(conn, inst, collections[i]) == 0) return;
                // Also check nested collections.
                struct hidd_collection **nested_collections;
                _Uint16t num_nested_collections;
                if (hidd_get_collections(NULL, collections[i], &nested_collections, &num_nested_collections) == EOK) {
                    for (int j = 0; j < num_nested_collections; j++) {
                        if (try_attach_to_report(conn, inst, nested_collections[j]) == 0) return;
                    }
                }
            }
        }
        fprintf(stderr, "[WARN] [%s]: Could not attach to a suitable input report.\n", instance_->get_name());
        fflush(stderr);
    }
    
    /**
     * @brief Static callback executed when a supported HID device is removed.
     */
    static void on_hidd_remove(struct hidd_connection *conn, hidd_device_instance_t *inst) {
        if (!instance_) return;
        if (check_allowed(inst->device_ident.vendor_id, inst->device_ident.product_id) == 1) {
            printf("[INFO] [%s]: Supported controller removed.\n", instance_->get_name());
            fflush(stdout);
            hidd_reports_detach(conn, inst);
        }
    }
 
    // ROS2 and application members
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    pthread_mutex_t joy_state_lock_; // Mutex for thread-safe data access
 
    // Thread-shared vectors for storing the latest joystick state
    std::vector<float> latest_axes_ = std::vector<float>(8, 0.0f);
    std::vector<int32_t> latest_buttons_ = std::vector<int32_t>(17, 0);
 
    // Static members for C-style callbacks
    static JoyTeleopNode* instance_;
    static struct hidd_connection *g_hid_conn;
};
 
    // Initialization of static members
    JoyTeleopNode* JoyTeleopNode::instance_ = nullptr;
    struct hidd_connection* JoyTeleopNode::g_hid_conn = nullptr;
 
/**
 * @brief Main entry point for the C++ node.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyTeleopNode>();
    node->initialize_hid_connection(); // Connect to the HIDDI service
    rclcpp::spin(node); // Start the ROS2 event loop
    rclcpp::shutdown();
    return 0;
}
      
 