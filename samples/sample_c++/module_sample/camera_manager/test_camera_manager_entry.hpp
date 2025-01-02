/* Private functions declaration ---------------------------------------------*/
#include <stdexcept>
#include "dji_logger.h"
#include "utils/util_misc.h"
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include "dji_camera_manager.h"
#include <iostream>
#include <random>
#include <string>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <miniocpp/client.h>
#include <fstream>
#include <exception>

static int gen_camera_action(std::string method) {
    int action = 0;
    if (method == "camera_set_optical_zoom_param") {
        action = 1;
    }
    return action;
}

static void *DjiUser_RunCameraManagerSampleTask(void *arg);

static void *DjiUser_RunCameraManagerSampleTask(void *arg) {
    const std::string SERVER_ADDRESS{"mqtt://47.97.201.247:1883"};
    std::string CLIENT_ID = "psdk_async_consume_camera_898798u";
    const std::string TOPIC{"thing/edge/xxx/services"};
    const int QOS = 2;

    T_DjiReturnCode returnCode;
    T_DjiDataTimestamp flightStatusTimestamp = {0};
    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init camera manager error, return code 0x%08X", returnCode);
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
    std::cout << "Camera Waiting for messages on topic: '" << TOPIC << "'" << std::endl;

    while (true) {
        auto msg = cli.consume_message();
        if (!msg) break;
        nlohmann::json j = nlohmann::json::parse(msg->get_payload());
        std::string method = j["method"];
        int flag = true;
        std::string result_msg = "ok";
        int action = gen_camera_action(method);
        if (action ==0 ){
            continue;
        }
        cout << "camera manager: "<<msg->get_topic() << ": " << msg->to_string() << endl;
        switch (action) {
            case 1:
                try {
                    nlohmann::json data = j["data"];
                    E_DjiMountPosition mount_position = data["mount_position"];
                    E_DjiCameraZoomDirection zoom_direction = data["camera_zoom_direction"];
                    dji_f32_t factor = data["factor"];
                    returnCode = DjiCameraManager_SetOpticalZoomParam(mount_position, zoom_direction, factor);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("camera manager set mode failed, errno = 0x%08llX", returnCode);
                        result_msg = "camera manager set optical zoom failed,code"+ std::to_string(returnCode);
                    }
                } catch (std::exception &exc) {
                    USER_LOG_ERROR("camera manager set optical zoom failed,%s", exc.what());
                    result_msg = "camera manager set optical zoom failed, " + std::string(exc.what());
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
    DjiCameraManager_DeInit();
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