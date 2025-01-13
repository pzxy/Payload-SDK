/**
 ********************************************************************
 * @file    test_flight_controller_command_flying.cpp
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
#include <termios.h>
#include <utils/util_misc.h>
#include <utils/util_file.h>
#include <utils/cJSON.h>
#include <dji_aircraft_info.h>
#include "dji_flight_controller.h"
#include "dji_logger.h"
#include "dji_fc_subscription.h"
#include "cmath"
#include <iostream>
#include <random>
#include <string>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <miniocpp/client.h>
#include <fstream>

#ifdef OPEN_CV_INSTALLED

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
#endif


/* Private constants ---------------------------------------------------------*/
#define DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE                          2048
#define DJI_TEST_COMMAND_FLYING_CTRL_FREQ                                50
#define DJI_TEST_COMMAND_FLYING_GO_HOME_ALTITUDE                         50
#define DJI_TEST_COMMAND_FLYING_CONTROL_SPEED_DEFAULT                    5
#define DJI_TEST_COMMAND_FLYING_RC_LOST_ACTION_STR_MAX_LEN               32
#define DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX                  (256)

using namespace std;

static int gen_action(std::string method) {
    int action = 0;
    if (method == "fc_obtain_joystick_ctrl_authority") {
        action = 1;
    } else if (method == "fc_release_joystick_ctrl_authority") {
        action = 2;
    } else if (method == "fc_start_takeoff") {
        action = 3;
    } else if (method == "fc_start_force_landing") {
        action = 4;
    } else if (method == "fc_start_go_home") {
        action = 5;
    } else if (method == "fc_cancel_go_home") {
        action = 6;
    } else if (method == "fc_start_landing") {
        action = 7;
    } else if (method == "fc_cancel_landing") {
        action = 8;
    } else if (method == "fc_set_joystick_mode") {
        action = 9;
    } else if (method == "fc_execute_joystick_action") {
        action = 10;
    } else if (method == "fc_arrest_flying") {
        action = 11;
    } else if (method == "fc_cancel_arrest_flying") {
        action = 12;
    }else if (method == "fc_start_or_stop") {
        action = 13;
    }
    return action;
}

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
static T_DjiTaskHandle s_commandFlyingTaskHandle;
static T_DjiTaskHandle s_statusDisplayTaskHandle;
static T_DjiFlightControllerJoystickCommand s_flyingCommand = {0};
static T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
};

static uint16_t s_inputFlag = 0;
static bool flight_controller_flag = false;
static dji_f32_t s_flyingSpeed = DJI_TEST_COMMAND_FLYING_CONTROL_SPEED_DEFAULT;
static uint16_t s_goHomeAltitude = DJI_TEST_COMMAND_FLYING_GO_HOME_ALTITUDE;
static char s_rcLostActionString[DJI_TEST_COMMAND_FLYING_RC_LOST_ACTION_STR_MAX_LEN] = {0};
static T_DjiFlightControllerHomeLocation s_homeLocation = {0};
static T_DjiFcSubscriptionGpsPosition s_gpsPosition = {0};
static bool isFirstUpdateConfig = false;
static bool isCommandFlyingTaskStart = false;
static uint32_t s_statusDisplayTaskCnt = 0;
static T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo1 = {0};
static T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo2 = {0};

/* Private functions declaration ---------------------------------------------*/
static void *DjiUser_FlightControllerCommandFlyingTask(void *arg);

static void *DjiUser_FlightControllerStatusDisplayTask(void *arg);

static void DjiUser_ShowFlightStatusByOpenCV(void);

static void DjiUser_FlightControllerVelocityAndYawRateCtrl(T_DjiFlightControllerJoystickCommand command);

static int DjiUser_ScanKeyboardInput(void);

static T_DjiReturnCode
DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);

static T_DjiVector3f DjiUser_FlightControlGetValueOfQuaternion(void);

static T_DjiFcSubscriptionGpsPosition DjiUser_FlightControlGetValueOfGpsPosition(void);

static T_DjiFcSubscriptionAltitudeOfHomePoint DjiUser_FlightControlGetValueOfRelativeHeight(void);

static T_DjiFcSubscriptionPositionVO DjiUser_FlightControlGetValueOfPositionVo(void);

static T_DjiFcSubscriptionControlDevice DjiUser_FlightControlGetValueOfControlDevice(void);

static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1(void);

static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2(void);

static T_DjiReturnCode DjiUser_FlightControlUpdateConfig(void);

static void *DjiUser_RunFlightControllerCommandFlyingSampleTask(void *arg);

static void *DjiUser_RunFlightControllerCommandFlyingSampleTask(void *arg) {
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    returnCode = osalHandler->TaskCreate("command_flying_task", DjiUser_FlightControllerCommandFlyingTask,
                                         DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE, NULL,
                                         &s_commandFlyingTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create command flying task failed, errno = 0x%08llX", returnCode);
        return nullptr;
    }

//    returnCode = osalHandler->TaskCreate("status_display_task", DjiUser_FlightControllerStatusDisplayTask,
//                                         DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE, NULL,
//                                         &s_statusDisplayTaskHandle);
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        USER_LOG_ERROR("Create status display task failed, errno = 0x%08llX", returnCode);
//        return nullptr;
//    }
    const string SERVER_ADDRESS{"mqtt://127.0.0.1:1883"};
    std::string CLIENT_ID = "psdk_async_consume_fc_0123456789";
    const string TOPIC{"thing/edge/xxx/services"};
    const int QOS = 1;
    mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID);
    auto connOpts = mqtt::connect_options_builder()
            .clean_session(false)
            .finalize();
    cli.start_consuming();
    cout << "Connecting to the MQTT server..." << flush;
    auto tok = cli.connect(connOpts);
    auto rsp = tok->get_connect_response();
    if (!rsp.is_session_present())
        cli.subscribe(TOPIC, QOS)->wait();
    cout << "Flight control Waiting for messages on topic: '" << TOPIC << "'" << endl;
    osalHandler->TaskSleepMs(3000);

    while (true) {
        auto msg = cli.consume_message();
        if (!msg) break;
        nlohmann::json j = nlohmann::json::parse(msg->get_payload());
        std::string method = j["method"];
        int flag = true;
        std::string result_msg = "ok";
        int action = gen_action(method);
        if (action == 0) {
            continue;
        }
        cout << "flight control: " << msg->get_topic() << ": " << msg->to_string() << endl;
        switch (action) {
            case 1:
                returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("obtain joystick control authority failed, errno = 0x%08llX", returnCode);
                    result_msg = "obtain joystick control authority failed,code: " + std::to_string(returnCode);
                }
                break;
            case 2:
                returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("release joystick control authority failed, errno = 0x%08llX", returnCode);
                    result_msg = "release joystick control authority failed,code: " + std::to_string(returnCode);
                }
                break;
            case 3:
                returnCode = DjiFlightController_StartTakeoff();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Take off fail, errno = 0x%08llX", returnCode);
                    result_msg = "Take off failed,code: " + std::to_string(returnCode);
                }
                break;
            case 4:
                returnCode = DjiFlightController_StartForceLanding();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("force landing fail, errno = 0x%08llX", returnCode);
                    result_msg = "force landing fail,code: " + std::to_string(returnCode);
                }
                break;
            case 5:
                returnCode = DjiFlightController_StartGoHome();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("start go home fail, errno = 0x%08llX", returnCode);
                    result_msg = "start go home fail,code: " + std::to_string(returnCode);
                }
                break;
            case 6:
                returnCode = DjiFlightController_CancelGoHome();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("cancel go home fail, errno = 0x%08llX", returnCode);
                    result_msg = "cancel go home fail,code: " + std::to_string(returnCode);
                }
                break;
            case 7:
                returnCode = DjiFlightController_StartLanding();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("start landing fail, errno = 0x%08llX", returnCode);
                    result_msg = "start landing fail,code: " + std::to_string(returnCode);
                }
                break;
            case 8:
                returnCode = DjiFlightController_CancelLanding();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("cancel landing fail, errno = 0x%08llX", returnCode);
                    result_msg = "cancel landing fail,code: " + std::to_string(returnCode);
                }
                break;
            case 9:
                try {
                    nlohmann::json data = j["data"];
                    joystickMode.horizontalControlMode = data["horizontal_control_mode"];
                    joystickMode.verticalControlMode = data["vertical_control_mode"];
                    joystickMode.yawControlMode = data["yaw_control_mode"];
                    joystickMode.horizontalCoordinate = data["horizontal_coordinate"];
                    joystickMode.stableControlMode = data["stable_control_mode"];
                    std::cout << "set mode:----->" << joystickMode.horizontalControlMode
                              << joystickMode.stableControlMode << std::endl;
                    s_inputFlag = 25;
                    DjiFlightController_SetJoystickMode(joystickMode);
                } catch (exception &exc) {
                    USER_LOG_ERROR("set control mode fail,%s", exc.what());
                    result_msg = "set control mode fail , " + std::string(exc.what());
                }
                break;
            case 10:
                try {
                    nlohmann::json data = j["data"];
                    s_flyingCommand.x = data["x"];
                    s_flyingCommand.y = data["y"];
                    s_flyingCommand.z = data["z"];
                    s_flyingCommand.yaw = data["yaw"];
                    s_inputFlag = 1;
                } catch (exception &exc) {
                    USER_LOG_ERROR("set control mode fail,%s", exc.what());
                    result_msg = "set control mode fail , " + std::string(exc.what());
                }
                break;
            case 11:
                returnCode = DjiFlightController_ArrestFlying();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("assert flying fail, errno = 0x%08llX", returnCode);
                    result_msg = "assert flying fail,code: " + std::to_string(returnCode);
                }
                break;
            case 12:
                returnCode = DjiFlightController_CancelArrestFlying();
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("cancel assert flying fail fail, errno = 0x%08llX", returnCode);
                    result_msg = "cancel assert flying fail,code: " + std::to_string(returnCode);
                }
                break;
//            case 13:
//                nlohmann::json data = j["data"];
//                int  action = data["action"];
//                if (action ==1 ) {
//                    flight_controller_flag = true;
//                }else{
//                    flight_controller_flag = false;
//                }
//                break;
            default:
                flag = false;
                USER_LOG_ERROR("invalid type");
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
    if (cli.is_connected()) {
        cout << "\nShutting down and disconnecting from the MQTT server..." << flush;
        cli.unsubscribe(TOPIC)->wait();
        cli.stop_consuming();
        cli.disconnect()->wait();
        cout << "OK" << endl;
    } else {
        cout << "\nClient was disconnected" << endl;
    }

    return nullptr;
}

/* Private functions definition-----------------------------------------------*/
static void *DjiUser_FlightControllerCommandFlyingTask(void *arg) {
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFlightControllerRidInfo ridInfo = {0};
    T_DjiFlightControllerGeneralInfo generalInfo = {0};

    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 0;

    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller failed, errno = 0x%08llX", returnCode);
        return NULL;
    }

    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    /*! subscribe fc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    osalHandler->TaskSleepMs(1000);

    returnCode = DjiUser_FlightControlUpdateConfig();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Update config failed, error code:0x%08llX", returnCode);
    }

    returnCode = DjiFlightController_GetGeneralInfo(&generalInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get general info failed, error code:0x%08llX", returnCode);
    }
    USER_LOG_INFO("Get aircraft serial number is: %s", generalInfo.serialNum);

    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
            DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        USER_LOG_ERROR("Register joystick control authority event callback failed, errno = 0x%08llX", returnCode);
        return NULL;
    }

    isCommandFlyingTaskStart = true;
    DjiFlightController_SetJoystickMode(joystickMode);
    while (true) {
        osalHandler->TaskSleepMs(1000 / DJI_TEST_COMMAND_FLYING_CTRL_FREQ);
        if (s_inputFlag <1 || s_inputFlag>25) {
            continue;
        }
        s_inputFlag++;
        if (s_inputFlag > 25) {
            s_flyingCommand.x = 0;
            s_flyingCommand.y = 0;
            s_flyingCommand.z = 0;
            s_flyingCommand.yaw = 0;
            s_inputFlag = 0;
        }
        if (s_inputFlag <1 || s_inputFlag>25) {
            continue;
        }
        DjiUser_FlightControllerVelocityAndYawRateCtrl(s_flyingCommand);
    }
}

static void *DjiUser_FlightControllerStatusDisplayTask(void *arg) {
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    while (1) {
        if (isCommandFlyingTaskStart == false) {
            continue;
        }
#ifdef OPEN_CV_INSTALLED
        DjiUser_ShowFlightStatusByOpenCV();
#endif
        osalHandler->TaskSleepMs(1000 / DJI_TEST_COMMAND_FLYING_CTRL_FREQ);
    }
}

static void DjiUser_ShowFlightStatusByOpenCV(void) {
#ifdef OPEN_CV_INSTALLED
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude = 0;
    T_DjiVector3f aircraftAngles = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeOfHomePoint = {0};
    E_DjiFlightControllerRtkPositionEnableStatus rtkPositionEnableStatus;
    E_DjiFlightControllerRCLostAction rcLostAction = DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_HOVER;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus downwardsVisEnable;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsVisEnable;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalVisEnable;
//    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsRadarEnable;
//    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalRadarEnable;
    T_DjiFcSubscriptionControlDevice controlDevice;
    T_DjiFcSubscriptionPositionVO positionVo;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    T_DjiReturnCode returnCode;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    Mat img(480, 1000, CV_8UC1, cv::Scalar(0));

    // Get latest flight status
    if (aircraftInfoBaseInfo.aircraftSeries != DJI_AIRCRAFT_SERIES_M300) {
        DjiFlightController_GetRCLostAction(&rcLostAction);
    }
    DjiFlightController_GetGoHomeAltitude(&s_goHomeAltitude);
    DjiFlightController_GetRtkPositionEnableStatus(&rtkPositionEnableStatus);
    DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&downwardsVisEnable);
//    DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(&upwardsRadarEnable);
    DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(&upwardsVisEnable);
//    DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(&horizontalRadarEnable);
    DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(&horizontalVisEnable);

    controlDevice = DjiUser_FlightControlGetValueOfControlDevice();
    aircraftAngles = DjiUser_FlightControlGetValueOfQuaternion();
    s_gpsPosition = DjiUser_FlightControlGetValueOfGpsPosition();
    altitudeOfHomePoint = DjiUser_FlightControlGetValueOfRelativeHeight();
    positionVo = DjiUser_FlightControlGetValueOfPositionVo();

    if (s_statusDisplayTaskCnt++ % 20 == 0) {
        singleBatteryInfo1 = DjiUser_FlightControlGetValueOfBattery1();
        singleBatteryInfo2 = DjiUser_FlightControlGetValueOfBattery2();
    }

    // Display latest flight status
    cv::putText(img, "Status: ", cv::Point(30, 20), FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 0, 0));

    cv::putText(img, "Roll: " + cv::format("%.4f", aircraftAngles.y), cv::Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "Pitch: " + cv::format("%.4f", aircraftAngles.x), cv::Point(50, 80), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "Yaw: " + cv::format("%.4f", aircraftAngles.z), cv::Point(50, 110), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "WorldX: " + cv::format("%.4f", positionVo.x), cv::Point(50, 140), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "WorldY: " + cv::format("%.4f", positionVo.y), cv::Point(50, 170), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "WorldZ: " + cv::format("%.4f", altitudeOfHomePoint), cv::Point(50, 200), FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Latitude: " + cv::format("%.4f", (dji_f64_t) s_gpsPosition.y / 10000000), cv::Point(50, 230),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Longitude: " + cv::format("%.4f", (dji_f64_t) s_gpsPosition.x / 10000000), cv::Point(50, 260),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Battery1: " + cv::format("%d%%", singleBatteryInfo1.batteryCapacityPercent), cv::Point(50, 290),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Battery2: " + cv::format("%d%%", singleBatteryInfo2.batteryCapacityPercent), cv::Point(50, 320),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));

    cv::putText(img, "Config: ", cv::Point(300, 20), FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 0, 0));
    cv::putText(img, "-> RcLostAction(Sync APP): " + cv::format("%d  (0-hover 1-landing 2-gohome)", rcLostAction),
                cv::Point(320, 50),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> GoHomeAltitude(Sync APP): " + cv::format("%d", s_goHomeAltitude), cv::Point(320, 80),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> RTK-Enable(Sync APP): " + cv::format("%d", rtkPositionEnableStatus), cv::Point(320, 110),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> HomePointLatitude: " + cv::format("%.4f", s_homeLocation.latitude), cv::Point(320, 140),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> HomePointLongitude: " + cv::format("%.4f", s_homeLocation.longitude), cv::Point(320, 170),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> FlyingSpeed: " + cv::format("%.2f", s_flyingSpeed), cv::Point(320, 200),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> downwardsVisEnable(Sync APP): " + cv::format("%d", downwardsVisEnable), cv::Point(320, 230),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> upwardsVisEnable(Sync APP): " + cv::format("%d", upwardsVisEnable), cv::Point(320, 260),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> horizontalVisEnable(Sync APP): " + cv::format("%d", horizontalVisEnable), cv::Point(320, 290),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> ControlDevice: " + cv::format("%d", controlDevice.deviceStatus), cv::Point(320, 320),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));

    cv::putText(img,
                "[Q]-Up    [W]-Front  [E]-Down   [R]-TakeOff  [T]-CancelLanding  [Y]-CancelGoHome  [I]-ArrestFly  [O]-CancelArrestFly  [P]-EmgStopMotor",
                cv::Point(30, 400), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 0));
    cv::putText(img,
                "[A]-Left   [S]-Near   [D]-Right   [F]-ForceLand   [G]-Landing   [H]-GoHome  [J]-UpdateConfig  [K]-Brake  [L]-CancelBrakeI",
                cv::Point(30, 430), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 0));
    cv::putText(img,
                "[Z]-Yaw-  [X]-RefreshHomePoint   [C]-Yaw+  [V]-ConfirmLanding   [B]-TurnOn  [N]-TurnOff  [M]-ObtainCtrlAuth",
                cv::Point(30, 460), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 0));

    cv::imshow("Payload SDK Command Flying Data Observation Window", img);
    cv::waitKey(1);
#endif
}

static void DjiUser_FlightControllerVelocityAndYawRateCtrl(T_DjiFlightControllerJoystickCommand command) {
    T_DjiReturnCode returnCode;

    USER_LOG_DEBUG("Joystick command: %.2f %.2f %.2f", command.x, command.y, command.z);
    if (s_inputFlag <1 || s_inputFlag>25) {
        return;
    }
    returnCode = DjiFlightController_ExecuteJoystickAction(command);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Execute joystick command failed, errno = 0x%08llX", returnCode);
        return;
    }
}

static int DjiUser_ScanKeyboardInput(void) {
    int input;
    struct termios new_settings;
    struct termios stored_settings;

    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0, &stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    input = getchar();
    tcsetattr(0, TCSANOW, &stored_settings);

    return input;
}

static T_DjiReturnCode
DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData) {
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                USER_LOG_INFO("[Event] Msdk request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event] Msdk request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                USER_LOG_INFO("[Event] Internal request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event] Internal request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery land\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            break;
        default:
            USER_LOG_INFO("[Event] Unknown joystick ctrl authority event\r\n");
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiVector3f DjiUser_FlightControlGetValueOfQuaternion(void) {
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};
    dji_f64_t pitch, yaw, roll;
    T_DjiVector3f vector3F;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", quaternionTimestamp.millisecond,
                       quaternionTimestamp.microsecond);
        USER_LOG_DEBUG("Quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    }

    pitch = (dji_f64_t) asinf(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
                              -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3,
                             -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) *
          57.3;

    vector3F.x = pitch;
    vector3F.y = roll;
    vector3F.z = yaw;

    return vector3F;
}

static T_DjiFcSubscriptionGpsPosition DjiUser_FlightControlGetValueOfGpsPosition(void) {
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                      (uint8_t *) &gpsPosition,
                                                      sizeof(T_DjiFcSubscriptionGpsPosition),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return gpsPosition;
}

static T_DjiFcSubscriptionAltitudeOfHomePoint DjiUser_FlightControlGetValueOfRelativeHeight(void) {
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeOfHomePoint;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                      (uint8_t *) &altitudeOfHomePoint,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return altitudeOfHomePoint;
}

static T_DjiFcSubscriptionPositionVO DjiUser_FlightControlGetValueOfPositionVo(void) {
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionPositionVO positionVo;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                                      (uint8_t *) &positionVo,
                                                      sizeof(T_DjiFcSubscriptionPositionVO),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return positionVo;
}

static T_DjiFcSubscriptionControlDevice DjiUser_FlightControlGetValueOfControlDevice(void) {
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionControlDevice controlDevice;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                      (uint8_t *) &controlDevice,
                                                      sizeof(T_DjiFcSubscriptionControlDevice),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return controlDevice;
}

static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1(void) {
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                      (uint8_t *) &singleBatteryInfo,
                                                      sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic battery1 error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return singleBatteryInfo;
}

static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2(void) {
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                      (uint8_t *) &singleBatteryInfo,
                                                      sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic battery2 error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return singleBatteryInfo;
}

static T_DjiReturnCode DjiUser_FlightControlUpdateConfig(void) {
    T_DjiReturnCode returnCode;
    char curFileDirPath[DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX];
    char tempFileDirPath[DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX];
    uint32_t fileSize = 0;
    uint32_t readRealSize = 0;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint8_t *jsonData = nullptr;
    cJSON *jsonRoot = nullptr;
    cJSON *jsonItem = nullptr;
    cJSON *jsonValue = nullptr;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

#ifdef SYSTEM_ARCH_LINUX
    returnCode = DjiUserUtil_GetCurrentFileDirPath(__FILE__, DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX,
                                                   curFileDirPath);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", returnCode);
        return returnCode;
    }

    snprintf(tempFileDirPath, DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX, "%s/config/flying_config.json",
             curFileDirPath);

    returnCode = UtilFile_GetFileSizeByPath(tempFileDirPath, &fileSize);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file size by path failed, stat = 0x%08llX", returnCode);
        return returnCode;
    }

    USER_LOG_DEBUG("Get config json file size is %d", fileSize);

    jsonData = static_cast<uint8_t *>(osalHandler->Malloc(fileSize + 1));
    if (jsonData == nullptr) {
        USER_LOG_ERROR("Malloc failed.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    memset(jsonData, 0, fileSize);

    UtilFile_GetFileDataByPath(tempFileDirPath, 0, fileSize, jsonData, &readRealSize);

    jsonData[readRealSize] = '\0';

    jsonRoot = cJSON_Parse((char *) jsonData);
    if (jsonRoot == nullptr) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "go_home_altitude");
//    if (jsonItem != nullptr) {
//        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//        if (jsonValue != nullptr) {
//            USER_LOG_INFO("Get go home altitude is [%.2f]", jsonValue->valuedouble);
//            s_goHomeAltitude = (uint16_t) jsonValue->valuedouble;
//            returnCode = DjiFlightController_SetGoHomeAltitude((uint16_t) jsonValue->valuedouble);
//            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                USER_LOG_ERROR("Set go home altitude failed, errno = 0x%08llX", returnCode);
//                return returnCode;
//            }
//        }
//    }

    if (aircraftInfoBaseInfo.aircraftSeries != DJI_AIRCRAFT_SERIES_M300) {
        jsonItem = cJSON_GetObjectItem(jsonRoot, "rc_lost_action");
        if (jsonItem != nullptr) {
            jsonValue = cJSON_GetObjectItem(jsonItem, "value");
            if (jsonValue != nullptr) {
                USER_LOG_INFO("Get rc lost action is [%s]", jsonValue->valuestring);
                strcpy(s_rcLostActionString, jsonValue->valuestring);
                if (strcmp(jsonValue->valuestring, "go_home") == 0) {
                    returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_GOHOME);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Set rc lost action failed, errno = 0x%08llX", returnCode);
                        return returnCode;
                    }
                } else if (strcmp(jsonValue->valuestring, "hover") == 0) {
                    returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_HOVER);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Set rc lost action failed, errno = 0x%08llX", returnCode);
                        return returnCode;
                    }
                } else if (strcmp(jsonValue->valuestring, "landing") == 0) {
                    returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_LANDING);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Set rc lost action failed, errno = 0x%08llX", returnCode);
                        return returnCode;
                    }
                } else {
                    USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
                }
            }
        }
    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "flying_speed");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get flying speed is [%.2f]", jsonValue->valuedouble);
            s_flyingSpeed = jsonValue->valuedouble;
        }
    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "rtk_enable");
//    if (jsonItem != nullptr) {
//        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//        if (jsonValue != nullptr) {
//            USER_LOG_INFO("Get rtk enable is [%s]", jsonValue->valuestring);
//            if (strcmp(jsonValue->valuestring, "true") == 0) {
//                returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
//                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                    USER_LOG_ERROR("Set rtk enable failed, errno = 0x%08llX", returnCode);
//                    return returnCode;
//                }
//            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
//                returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
//                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                    USER_LOG_ERROR("Set rtk enable failed, errno = 0x%08llX", returnCode);
//                    return returnCode;
//                }
//            } else {
//                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
//            }
//        }
//    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "home_point_latitude");
//    if (jsonItem != nullptr) {
//        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//        if (jsonValue != nullptr) {
//            USER_LOG_INFO("Get home_point_latitude  is [%.2f]", jsonValue->valuedouble);
//            s_homeLocation.latitude = jsonValue->valuedouble;
//        }
//    }
//    jsonItem = cJSON_GetObjectItem(jsonRoot, "home_point_longitude");
//    if (jsonItem != nullptr) {
//        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//        if (jsonValue != nullptr) {
//            USER_LOG_INFO("Get home_point_longitude is [%.2f]", jsonValue->valuedouble);
//            s_homeLocation.longitude = jsonValue->valuedouble;
//        }
//    }

//    if (isFirstUpdateConfig == false) {
//        USER_LOG_INFO("Using current aircraft location, not use config home location.");
//        s_gpsPosition = DjiUser_FlightControlGetValueOfGpsPosition();
//        std::cout << "y: " << s_gpsPosition.y << endl;
//        std::cout << "x: " << s_gpsPosition.x << endl;
//
//        s_homeLocation.latitude = (dji_f64_t) s_gpsPosition.y / 10000000;
//        s_homeLocation.longitude = (dji_f64_t) s_gpsPosition.x / 10000000;

//        returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("Set home location failed, errno = 0x%08llX", returnCode);
//        }
//        isFirstUpdateConfig = true;
//    }else {
//        T_DjiFlightControllerHomeLocation homeLocation;
//        homeLocation.latitude = s_homeLocation.latitude * DJI_PI / 180;
//        homeLocation.longitude = s_homeLocation.longitude * DJI_PI / 180;
//
//        returnCode = DjiFlightController_SetHomeLocationUsingGPSCoordinates(homeLocation);
//        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//            USER_LOG_ERROR("Set home location failed, errno = 0x%08llX", returnCode);
//            return returnCode;
//        }
//    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "HorizontalVisualObstacleAvoidanceEnable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get HorizontalVisualObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set HorizontalVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set HorizontalVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "HorizontalRadarObstacleAvoidanceEnable");
//    if (jsonItem != nullptr) {
//        if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
//            jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//            if (jsonValue != nullptr) {
//                USER_LOG_INFO("Get HorizontalRadarObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
//                if (strcmp(jsonValue->valuestring, "true") == 0) {
//                    returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set HorizontalRadarObstacleAvoidanceEnable failed, errno = 0x%08llX",
//                                       returnCode);
//                        return returnCode;
//                    }
//                } else if (strcmp(jsonValue->valuestring, "false") == 0) {
//                    returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set HorizontalRadarObstacleAvoidanceEnable failed, errno = 0x%08llX",
//                                       returnCode);
//                        return returnCode;
//                    }
//                } else {
//                    USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
//                }
//            }
//        }
//    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "UpwardsVisualObstacleAvoidanceEnable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get UpwardsVisualObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set UpwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set UpwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "UpwardsRadarObstacleAvoidanceEnable");
//    if (jsonItem != nullptr) {
//        if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
//            jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//            if (jsonValue != nullptr) {
//                USER_LOG_INFO("Get UpwardsRadarObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
//                if (strcmp(jsonValue->valuestring, "true") == 0) {
//                    returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set UpwardsRadarObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
//                        return returnCode;
//                    }
//                } else if (strcmp(jsonValue->valuestring, "false") == 0) {
//                    returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set UpwardsRadarObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
//                        return returnCode;
//                    }
//                } else {
//                    USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
//                }
//            }
//        }
//    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "DownwardsVisualObstacleAvoidanceEnable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get DownwardsVisualObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set DownwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set DownwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

    osalHandler->Free(jsonData);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
#endif
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
