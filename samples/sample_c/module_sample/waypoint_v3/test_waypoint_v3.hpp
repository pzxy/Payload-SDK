/**
 ********************************************************************
 * @file    test_waypoint_v3.c
 * @brief
 *
 * @copyright (c) 2018 DJI. All rights reserved.
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
#include <utils/util_file.h>
#include <utils/util_misc.h>
#include "test_waypoint_v3.h"
#include "dji_logger.h"
#include "dji_waypoint_v3.h"
#include "waypoint_file_c/waypoint_v3_test_file_kmz.h"
#include "dji_fc_subscription.h"
#include <iostream>
#include <random>
#include <string>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <miniocpp/client.h>
#include <fstream>

using namespace std;

static std::string generateRandomString(size_t length) {
    const std::string characters = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    std::string randomString;
    std::uniform_int_distribution<> distrib(0, characters.size() - 1);
    std::random_device rd;
    std::mt19937 gen(rd());

    for (size_t i = 0; i < length; ++i) {
        randomString += characters[distrib(gen)];
    }
    return randomString;
}


class MyError : public std::error_category {
public:
    const char *name() const noexcept override {
        return "MyError";
    }

    std::string message(int ev) const override {
        // 将错误代码转换为可读的错误消息
        return "My error code " + std::to_string(ev);
    }
};

static std::string downloadObject(std::string const &object) {
    minio::s3::BaseUrl base_url{"192.168.2.159:30092", false};
    minio::creds::StaticProvider provider{
            "lOA5v9mXMLKIjRvGyLmM", "8zVqHEa7oAfpVteqd5alQk5tyD9oyPokMvO55u0o"};
    minio::s3::Client client{base_url, &provider};
    minio::s3::DownloadObjectArgs args;
    args.bucket = "cloud-bucket";
    args.object = object;
    args.filename = "operation.kmz";
    if (std::remove(args.filename.c_str()) == 0) {
        std::cout << "operation.kmz deleted successfully." << std::endl;
    }
    auto resp = client.DownloadObject(args);
    if (resp) {
        std::cout << "cloud-bucket: " + object << "Download successful"
                  << std::endl;
        return "ok";
    } else {
        std::cout << "cloud-bucket: " + object << "Download failed:" << resp.Error().String()
                  << std::endl;
        return resp.Error().String();
    }
}


static void *DjiTest_WaypointV3RunSampleTask(void *arg) {
    const string SERVER_ADDRESS{"mqtt://192.168.2.101:1883"};
    std::string CLIENT_ID = "psdk_async_consume_:" + generateRandomString(10);
    const string TOPIC{"thing/edge/xxx/services"};
    const int QOS = 2;
    mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);
    auto connOpts = mqtt::connect_options_builder()
            .clean_session(false)
            .finalize();
//    T_DjiReturnCode returnCode;
//    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
//    T_DjiFcSubscriptionFlightStatus flightStatus = 0;
//    T_DjiDataTimestamp flightStatusTimestamp = {0};

    try {
        cli.start_consuming();
        cout << "Connecting to the MQTT server..." << flush;
        auto tok = cli.connect(connOpts);
        auto rsp = tok->get_connect_response();
        if (!rsp.is_session_present())
            cli.subscribe(TOPIC, QOS)->wait();
        cout << "Waiting for messages on topic: '" << TOPIC << "'" << endl;
//        returnCode = DjiWaypointV3_Init();
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("Waypoint v3 init failed.");
//            return nullptr;
//        }
        while (true) {
            auto msg = cli.consume_message();
            if (!msg) break;
            cout << msg->get_topic() << ": " << msg->to_string() << endl;
            nlohmann::json j = nlohmann::json::parse(msg->get_payload());
            std::string method = j["method"];
            int flag = false;
            std::string result_msg = "ok";
            if (method == "waypoint_v3_upload_kmz_file") {
                flag = true;
                nlohmann::json data = j["data"];
                std::string object = data["waypoint_kmz_file_name"];
                result_msg = downloadObject(object);
                if (result_msg != "ok") {
                    goto services_reply;
                }
                std::ifstream kmzFile{"operation.kmz", std::ios::binary | std::ios::ate};
                if (kmzFile) {
                    std::streampos fileSize = kmzFile.tellg();
                    kmzFile.seekg(0, std::ios::beg);
                    std::vector<uint8_t> kmzFileBuf(static_cast<size_t>(fileSize));
                    kmzFile.read(reinterpret_cast<char *>(kmzFileBuf.data()), static_cast<std::streamsize>(fileSize));
//                    returnCode = DjiWaypointV3_UploadKmzFile(kmzFileBuf.data(), kmzFileBuf.size());
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        std::cerr << "Upload kmz file failed." << std::endl;
//                        result_msg = "Upload kmz file failed.";
//                    }
                    cout << "kmzFile-size:"<<kmzFileBuf.size() << endl;
                    kmzFile.close();
                } else {
                    std::cerr << "Open kmz file failed." << std::endl;
                    result_msg = "Open kmz file failed.";
                }
            } else if (method == "waypoint_v3_action") {
                flag = true;
                nlohmann::json data = j["data"];
                int waypoint_action = data["waypoint_action"];
                switch (waypoint_action) {
                    case 0:
//                        returnCode = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
//                        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                            USER_LOG_ERROR("Execute start action failed.");
//                            result_msg = "start action failed.";
//                        }
                        break;
                    case 1:
//                        returnCode = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);
//                        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                            USER_LOG_ERROR("Execute stop action failed.");
//                            result_msg = "stop action failed.";
//                        }
                        break;
                    case 2:
//                        returnCode = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE);
//                        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                            USER_LOG_ERROR("Execute start action failed.");
//                            result_msg = "pause action failed.";
//                        }
                        break;
                    case 3:
//                        returnCode = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_RESUME);
//                        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                            USER_LOG_ERROR("Execute start action failed.");
//                            result_msg = "resume action failed.";
//                        }
                        break;
                    default:
                        result_msg = "invalid action";
                        break;
                }
            }
            if (flag) {
                services_reply:
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
                        {"method",    "waypoint_v3_action"},
                        {"data",      {
                                              {"result", result},
                                              {"message", result_msg},
                                      }}
                };
                cli.publish("thing/edge/xxx/services_reply", ret_json.dump(4));
            }
        }
        if (cli.is_connected()) {
            cout << "\nShutting down and disconnecting from the MQTT server..." << flush;
            cli.unsubscribe(TOPIC)->wait();
            cli.stop_consuming();
            cli.disconnect()->wait();
            cout << "OK" << endl;
        } else {
            cout << "\nClient was disconnected" << endl;
        }
    }
    catch (const mqtt::exception &exc) {
        cerr << "\n  " << exc << endl;
    }
//    returnCode = DjiWaypointV3_DeInit();
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Execute start action failed.");
//    }
    return nullptr;
}

