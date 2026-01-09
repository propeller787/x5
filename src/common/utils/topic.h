#pragma once

constexpr const char *imagesTopic = "/sensor/camera/images";
constexpr const char *cameraFusedImagesTopic = "/perception/cameraFused/images";

constexpr const char *cameraFusedObjectsTopic = "/perception/cameraFused/objects";
constexpr const char *multiFusedObjectsTopic = "/perception/multiFused/objects";
constexpr const char *multiFusedObjectsTopicDecision = "/perception/decision/multiFused/objects";
constexpr const char *obstacleMapTopicDecision = "/perception/obstacle_map";
constexpr const char *eventTopic = "/decision/event";

constexpr const char *danceStatusTopic = "/xhi/status";

constexpr const char *backupCarTopic = "/sensor/backup/car";
constexpr const char *handleControlTopic = "/handle/control";              // 手柄控制
constexpr const char *nav2ControlTopic = "/nav/control";                   // nva2控制
constexpr const char *appControlTopic = "/app/control";                    // app控制
constexpr const char *carWheelSpeedTopic = "/car/speed";                   // 车辆底盘信息
constexpr const char *carWheelSpeedStaTopic = "/wheel/twist";              // 车辆底盘信息
constexpr const char *carEncoderTopic = "/car/encoder";                    // 车辆电机圈数
constexpr const char *healthDataTopic = "/sensor/health/data";             // 健康计信息
constexpr const char *healthTftDataTopic = "/sensor/tft/health/data";      // tft健康计信息
constexpr const char *healthStatusTopic = "/sensor/health/start";          // 健康计启动
constexpr const char *sensorhandleControlTopic = "/sensor/handle/control"; // 上控健康控制
constexpr const char *gpsTopic = "/sensor/gps/data";
constexpr const char *imuTopic = "/sensor/imu/data";
constexpr const char *hrvStartTopic = "/sensor/hrv/start";
constexpr const char *ultrasoundLeftTopic = "/sensor/left/ultrasound";
constexpr const char *ultrasoundRightTopic = "/sensor/right/ultrasound";
constexpr const char *ultrasoundBackTopic = "/sensor/back/ultrasound";
constexpr const char *joystickTopic = "/sensor/joystick";
constexpr const char *carInfoTopic = "/vlago/car/info";
constexpr const char *operationSceTopic = "/service/operation/scenario";
constexpr const char *serviceColourScreenTopic = "/service/colour_screen";
constexpr const char *serviceGearTopic = "/service/gear";
constexpr const char *modeChangeTopic = "/mode/change";
constexpr const char *cameraH264Topic = "/sensor/backup/camera/h264";
constexpr const char *leftLaserTopic = "/scan/left";
constexpr const char *rightLaserTopic = "/scan/right";
constexpr const char *leftTiltLaserTopic = "/scan/left_tilt";
constexpr const char *rightTiltLaserTopic = "/scan/right_tilt";
constexpr const char *backLaserTopic = "/scan/back";
constexpr const char *leftTiltPointTopice = "/perception/left_tilt/pointcloud";
constexpr const char *rightTiltPointTopice = "/perception/right_tilt/pointcloud";

// 场景辅助测试相关topic
constexpr const char *leftLaserROITopic = "/scene_auxiliary/left_laser/roi_filtered";
constexpr const char *rightLaserROITopic = "/scene_auxiliary/right_laser/roi_filtered";
constexpr const char *leftLaserClusterTopic = "/scene_auxiliary/left_laser/clusters";
constexpr const char *rightLaserClusterTopic = "/scene_auxiliary/right_laser/clusters";
constexpr const char *leftTiltClusterTopic = "/scene_auxiliary/left_tilt/clusters";
constexpr const char *rightTiltClusterTopic = "/scene_auxiliary/right_tilt/clusters";
constexpr const char *backClusterTopic = "/scene_auxiliary/back_laser/clusters";

// base_link坐标系转换后的点云话题
constexpr const char *leftLaserBaseLinkTopic = "/scene_auxiliary/left_laser/base_link";
constexpr const char *rightLaserBaseLinkTopic = "/scene_auxiliary/right_laser/base_link";
constexpr const char *leftTiltBaseLinkTopic = "/scene_auxiliary/left_tilt/base_link";
constexpr const char *rightTiltBaseLinkTopic = "/scene_auxiliary/right_tilt/base_link";
constexpr const char *backLaserBaseLinkTopic = "/scene_auxiliary/back_laser/base_link";

constexpr const char *modeStatusTopic = "/vlago/mode/status";
constexpr const char *rechargeStatusTopic = "/vlago/recharge/status";
constexpr const char *sceneServiceModeTopic = "/scene_services/mode";
constexpr const char *rgbPointCloudTopic = "/rgb/pointcloud";

// 自车
// constexpr const char* carInfo = "/sensor/car/information";

// service
constexpr const char *hrvStartService = "/vlago/hrv/start";
constexpr const char *hrvInfoService = "/vlago/hrv/info";
constexpr const char *operationSceService = "/vlago/operation/scenario";
constexpr const char *operationStatusService = "/vlago/operation/status";
constexpr const char *carControlService = "/vlago/car/control";
// constexpr const char* carInfoService = "/vlago/car/info";
constexpr const char *followControlService = "/vlago/follow_me";
constexpr const char *followControlStatusService = "/vlago/follow_me/status";
constexpr const char *summonService = "/valgo/car/summon";
constexpr const char *summonStatusService = "/valgo/car/summon/status";
constexpr const char *highService = "/vaglo/car/high";
constexpr const char *gearsControlService = "/vlago/gear";
constexpr const char *sensorGearControlService = "/sensor/vlago/gear";
constexpr const char *idControlService = "/vlago/car_id";
constexpr const char *narrowControlService = "/vlago/narrow_assistant_mode";
constexpr const char *standardControlService = "/vlago/standard_mode";
constexpr const char *cruiseControlService = "/vlago/cruise_mode";
constexpr const char *auxiliaryControlService = "/vlago/safe_mode";
constexpr const char *tftControlService = "/vlago/lamp_language_screen";
constexpr const char *sensorTftControlService = "/sensor/vlago/lamp_language_screen";
constexpr const char *danceControlService = "/vlago/dance_show";
constexpr const char *modeSwitchService = "/mode/switch";
constexpr const char *vlagoSettingService = "/vlago/setting";
constexpr const char *vlagoModeEnableService = "/vlago/mode/enable";
constexpr const char *rechargeService = "/vlago/auto_charge";
constexpr const char *manageFollowmeService = "/manage/followme";
constexpr const char *manageCruiseService = "/manage/cruise";
constexpr const char *autoindoorService = "/vlago/autoDriveMode";
constexpr const char *autoindoorNavGotoService = "/vlago/autoindoor/goto";
constexpr const char *autoindoorNavCannelService = "/vlago/autoindoor/cannel";

constexpr const char *feedNarrowService = "/feed/narrow/ctrl";
constexpr const char *feedRechargeService = "/feed/recharge/ctrl";
constexpr const char *feedFollwmeService = "/feed/follwme/ctrl";
constexpr const char *feedDanceService = "/feed/dance/ctrl";
//
constexpr const char *ServiceFollowControlTopic = "/service/follow/control";
// 模式
constexpr const char *sceneAuxiliaryService = "/scene/auxiliary";
constexpr const char *sceneNarrowService = "/scene/narrow";
constexpr const char *sceneStandardService = "/scene/standard";
constexpr const char *sceneCruiseService = "/scene/cruise";
constexpr const char *sceneDanceService = "/scene/dance";
constexpr const char *sceneRechargeService = "/scene/recharge";
constexpr const char *sceneFollowmeService = "/scene/followme";
// 德国版本健康健 启动舞蹈秀
constexpr const char *sceneKeyDanceTopic = "/scene/dance/key";

constexpr const char *manageFollowMe = "/manage/followme";
constexpr const char *manageDance = "/manage/dance";
constexpr const char *manageStandard = "/manage/standard";

// 模型
constexpr const char *perTracekerTopic = "/perception/tracker/data";
constexpr const char *perFeatuerTopic = "/perception/localization/data";
constexpr const char *perDetctorTopic = "/perception/detector/data";
constexpr const char *perMonnaiTopic = "/perception/end_points";
constexpr const char *perSpeedbumpTopic = "/speedbump_marker_array";
constexpr const char *perPoDaoTopic = "/perception/podao_detect_status";
// 预警
constexpr const char *auxiliaryAlarmTopic = "/vlago/auxiliary/alarm";
constexpr const char *auxiliaryAlarmBackTopic = "/vlago/backcar/alarm";
constexpr const char *cursorAlarmTopic = "/vlago/cruise/status";
constexpr const char *speedbumpAlarmTopic = "/speed_bump/status";
constexpr const char *downhillAlarmTopic = "/downhill/status";
constexpr const char *uphillAlarmTopic = "/uphill/status";
constexpr const char *footLimiterTopic = "/foot/limiter/status";

// tft模式
constexpr const char *tftModeTopic = "/vlago/tft/mode";

constexpr const char *audioTopic = "/vlago/audio/play";

// 错误码topic
constexpr const char* errorEventTopic = "/error_events";