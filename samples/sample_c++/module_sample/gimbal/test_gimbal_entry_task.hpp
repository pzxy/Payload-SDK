/**
 ********************************************************************
 * @file    test_gimbal_entry.cpp
 * @version V2.0.0
 * @date    2023/3/28
 * @brief
 *
 * @copyright (c) 2018-2023 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdexcept>
#include "test_gimbal_entry.hpp"
#include "dji_logger.h"
#include "utils/util_misc.h"
#include "dji_gimbal.h"
#include "dji_gimbal_manager.h"
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include <iostream>
#include <random>
#include <string>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <miniocpp/client.h>
#include <fstream>
#include <exception>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
typedef enum {
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_ON_FREE_MODE,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_ON_YAW_FOLLOW_MODE,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_PITCH_RANGE_EXTENSION_MODE,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_CONTROLLER_MAX_SPEED_PERCENTAGE,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_SET_CONTROLLER_SMOOTH_FACTOR,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_RESET_GIMBAL_SETTINGS,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_ROTATE_GIMBAL_BY_KEYBOARD,
    E_DJI_TEST_GIMBAL_MANAGER_SAMPLE_SELECT_QUIT,
} E_DjiTestGimbalManagerSampleSelect;


/* Private functions declaration ---------------------------------------------*/

static int gen_gimbal_action(std::string method) {
    int action = 0;
    if (method == "gimbal_manager_set_mode") {
        action = 1;
    } else if (method == "gimbal_manager_reset") {
        action = 2;
    } else if (method == "gimbal_manager_rotate") {
        action = 3;
    } else if (method == "gimbal_manager_set_pitch_range_extension_enabled") {
        action = 4;
    } else if (method == "gimbal_manager_set_controller_max_speed_percentage") {
        action = 5;
    } else if (method == "gimbal_manager_set_controller_smooth_factor") {
        action = 6;
    } else if (method == "gimbal_manager_restore_factory_settings") {
        action = 7;
    }
    return action;
}

static void *DjiUser_RunGimbalManagerSampleTask(void *arg);

static void *DjiUser_RunGimbalManagerSampleTask(void *arg) {
    const std::string SERVER_ADDRESS{"mqtt://47.97.201.247:1883"};
    std::string CLIENT_ID = "psdk_async_consume_gimbal_2323423";
    const std::string TOPIC{"thing/edge/xxx/services"};
    const int QOS = 2;

    T_DjiReturnCode returnCode;
    T_DjiDataTimestamp flightStatusTimestamp = {0};
    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init gimbal manager error, return code 0x%08X", returnCode);
        return nullptr;
    }

    mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);
    auto connOpts = mqtt::connect_options_builder()
            .clean_session(false)
            .finalize();
    cli.start_consuming();
    std::cout << "Connecting to the MQTT server..." << std::flush;
    auto tok = cli.connect(connOpts);
    auto rsp = tok->get_connect_response();
    if (!rsp.is_session_present())
        cli.subscribe(TOPIC, QOS)->wait();
    std::cout << "Gimbal Waiting for messages on topic: '" << TOPIC << "'" << std::endl;

    while (true) {
        auto msg = cli.consume_message();
        if (!msg) break;
        nlohmann::json j = nlohmann::json::parse(msg->get_payload());
        std::string method = j["method"];
        int flag = true;
        std::string result_msg = "ok";
        int action = gen_gimbal_action(method);
        if (action !=0 ){
            cout << "gimbal manager: "<<msg->get_topic() << ": " << msg->to_string() << endl;
        }
        switch (action) {
            case 1:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    E_DjiGimbalMode gimbal_mode = data["gimbal_mode"];
                    returnCode = DjiGimbalManager_SetMode(mount_position, gimbal_mode);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager set mode failed, errno = 0x%08llX", returnCode);
                        result_msg = "gimbal manager set mode failed,code" + std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager set mode failed,%s", exc.what());
                    result_msg = "gimbal manager set mode failed, " + std::string(exc.what());
                }
                break;
            case 2:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    E_DjiGimbalResetMode gimbal_reset_mode = data["gimbal_reset_mode"];
                    returnCode = DjiGimbalManager_Reset(mount_position, gimbal_reset_mode);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager reset failed, errno = 0x%08llX", returnCode);
                        result_msg = "gimbal manager reset failed,code" + std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager reset failed,%s", exc.what());
                    result_msg = "gimbal manager reset failed, " + std::string(exc.what());
                }
                break;
            case 3:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    nlohmann::json rotation_json = data["rotation"];
                    T_DjiGimbalManagerRotation rotation = {};
                    rotation.rotationMode = rotation_json["gimbal_rotation_mode"];
                    rotation.pitch = rotation_json["pitch"];
                    rotation.roll = rotation_json["roll"];
                    rotation.yaw = rotation_json["yaw"];
                    rotation.time = rotation_json["time"];
                    returnCode = DjiGimbalManager_Rotate(mount_position, rotation);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager rotate, errno = 0x%08llX", returnCode);
                        result_msg = "gimbal manager rotate failed,code" + std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager rotate failed,%s", exc.what());
                    result_msg = "gimbal manager rotate failed, " + std::string(exc.what());
                }
                break;
            case 4:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    bool enabled_flag = data["enabled_flag"];
                    returnCode = DjiGimbalManager_SetPitchRangeExtensionEnabled(mount_position,enabled_flag);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager set pitch range extension enabled fail, errno = 0x%08llX", returnCode);
                    }
                    result_msg = "gimbal manager set pitch range extension enabled fail,code" + std::to_string(returnCode);
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager set pitch range extension enabled fail,%s", exc.what());
                    result_msg = "gimbal manager set pitch range extension enabled fail, " + std::string(exc.what());
                }
                break;
            case 5:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    E_DjiGimbalAxis axis = data["gimbal_axis"];
                    uint8_t max_speed_percentage = data["max_speed_percentage"];
                    returnCode = DjiGimbalManager_SetControllerMaxSpeedPercentage(mount_position,axis,max_speed_percentage);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager set controller max speed percentage fail, errno = 0x%08llX", returnCode);
                        result_msg = "gimbal manager set controller max speed percentage fail,code" + std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager set controller max speed percentage fail,%s", exc.what());
                    result_msg = "gimbal manager set controller max speed percentage fail, " + std::string(exc.what());
                }
                break;
            case 6:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    E_DjiGimbalAxis axis = data["gimbal_axis"];
                    uint8_t smoothing_factor = data["smoothing_factor"];
                    returnCode = DjiGimbalManager_SetControllerSmoothFactor(mount_position,axis,smoothing_factor);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager set controller smooth factor fail, errno = 0x%08llX", returnCode);
                        result_msg = "gimbal manager set controller smooth factor fail,code" + std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager set controller smooth factor fail,%s", exc.what());
                    result_msg = "gimbal manager set controller smooth factor fail, " + std::string(exc.what());
                }
                break;
            case 7:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    returnCode = DjiGimbalManager_RestoreFactorySettings(mount_position);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("gimbal manager restore factory settings fail, errno = 0x%08llX", returnCode);
                        result_msg = "gimbal manager restore factory settings fail,code" + std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("gimbal manager restore factory settings fail,%s", exc.what());
                    result_msg = "gimbal manager restore factory settings fail, " + std::string(exc.what());
                }
                break;
            default:
                flag = false;
        }
        if (flag) {
            int result = 0;
            if (result_msg != "ok") {
                result = -1;
            }
            std::chrono::milliseconds milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
            nlohmann::json ret_json = {
                    {"tid",       j["tid"]},
                    {"bid",       j["bid"]},
                    {"timestamp", milliseconds.count()}, // 使用函数返回值作为表达式
                    {"method",    method},
                    {"data",      {
                                          {"result", result},
                                          {"message", result_msg},
                                  }}
            };
            cli.publish("thing/edge/xxx/services_reply", ret_json.dump(4));
        }
    }
    DjiGimbalManager_Deinit();
    if (cli.is_connected()) {
        std::cout << "\nShutting down and disconnecting from the MQTT server..." << std::flush;
        cli.unsubscribe(TOPIC)->wait();
        cli.stop_consuming();
        cli.disconnect()->wait();
        std::cout << "OK" << std::endl;
    } else {
        std::cout << "\nClient was disconnected" << std::endl;
    }
    return nullptr;
}
