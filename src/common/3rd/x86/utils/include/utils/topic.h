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
constexpr const char *wheelOdomTopic = "/sensor/wheel_odom";               // 轮速计里程计 nav_msgs/Odometry
constexpr const char *healthDataTopic = "/sensor/health/data";             // 健康计信息
constexpr const char *healthTftDataTopic = "/sensor/tft/health/data";      // tft健康计信息
constexpr const char *sensorTftChargingStatusTopic = "/sensor/recharge/charging_status";
constexpr const char *healthStatusTopic = "/sensor/health/start";          // 健康计启动
constexpr const char *sensorhandleControlTopic = "/sensor/handle/control"; // 上控健康控制
constexpr const char *gpsTopic = "/sensor/gps/data";
constexpr const char *imuTopic = "/sensor/imu/data";
constexpr const char *hrvStartTopic = "/sensor/hrv/start";
constexpr const char *ultrasoundLeftTopic = "/sensor/left/ultrasound";
constexpr const char *ultrasoundRightTopic = "/sensor/right/ultrasound";
constexpr const char *ultrasoundBackTopic = "/sensor/back/ultrasound";
constexpr const char *joystickTopic = "/sensor/joystick";
constexpr const char *laneWarningPointTopic = "/sensor/laneWarningPoint";
constexpr const char *carInfoTopic = "/vlago/car/info";
constexpr const char *iccidControlService = "/vlago/iccid_control";
constexpr const char *bluetoothStatusTopic = "/vlago/bluetooth/status";
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
constexpr const char *leftTiltDebugScanTopic = "/safety/debug/tilt_left";
constexpr const char *rightTiltDebugScanTopic = "/safety/debug/tilt_right";
// 物理按键失效发布 1 驻车失败 2 解除驻车失败 3 助行失败
constexpr const char *physicalButtonFailureTopic = "/vlago/physical/button/failure";
// 安全功能开启/关闭发布 1 开启 0 关闭
constexpr const char *safetyFunctionTopic = "/vlago/safety/function";
// 巡航模式开启/关闭发布 1 开启 0 关闭
constexpr const char *cruiseModeTopic = "/vlago/cruise/mode";
// 挡位
constexpr const char *gearTopic = "/vlago/gear";


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
constexpr const char *rechargeStatusTopic = "/vlago/recharge/status"; // 废弃不用
constexpr const char *rechargeStateTopic = "/vlago/recharge/state";
constexpr const char *sceneServiceModeTopic = "/scene_services/mode";
constexpr const char *rgbPointCloudTopic = "/rgb/pointcloud";
constexpr const char *followmeStatusTopic = "/vlago/followme/status";
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
constexpr const char *manageHelpcarService = "/manage/helpcar";
constexpr const char *manageStandardService = "/manage/standard";
constexpr const char *manageBluetoothService = "/manage/bluetooth";
constexpr const char *manageRechargeService = "/manage/recharge";
constexpr const char *autoindoorService = "/vlago/autoDriveMode";
constexpr const char *autoindoorNavGotoService = "/vlago/autoindoor/goto";
constexpr const char *autoindoorNavCannelService = "/vlago/autoindoor/cannel";

constexpr const char *feedNarrowService = "/feed/narrow/ctrl";
constexpr const char *feedRechargeService = "/feed/recharge/ctrl";
constexpr const char *feedFollwmeService = "/feed/follwme/ctrl";
constexpr const char *feedDanceService = "/feed/dance/ctrl";

// 系统版本
constexpr const char *UCVersionService = "/vlago/uc/version";
constexpr const char *DBVersionService = "/vlago/db/version";
// 蓝牙mac地址
constexpr const char *bluetoothMacService = "/vlago/bluetooth/mac";
constexpr const char *ServiceFollowControlTopic = "/service/follow/control";
// 模式
constexpr const char *sceneAuxiliaryService = "/scene/auxiliary";
constexpr const char *sceneNarrowService = "/scene/narrow";
constexpr const char *sceneStandardService = "/scene/standard";
constexpr const char *sceneCruiseService = "/scene/cruise";
constexpr const char *sceneHelpcarService = "/scene/helpcar";
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
constexpr const char *auxiliaryStatusTopic = "/scene_auxiliary/status";  // 安全功能开启/关闭状态
constexpr const char *cursorAlarmTopic = "/vlago/cruise/status";
constexpr const char *speedbumpAlarmTopic = "/speed_bump/status";
constexpr const char *downhillAlarmTopic = "/downhill/status";
constexpr const char *uphillAlarmTopic = "/uphill/status";
constexpr const char *footLimiterTopic = "/foot/limiter/status";
constexpr const char *collisionWarningTopic = "/vlago/collision/warning";

// tft模式
constexpr const char *tftModeTopic = "/vlago/tft/mode";
constexpr const char *tftCarModeTopic = "/vlago/tft/car/mode";  // 切换车辆模式 1 舒适 2 运动
constexpr const char *tftSoundSwitchTopic = "/vlago/tft/sound/switch"; // 设置声音开关 1 打开 0 关闭
constexpr const char *tftLanguageTopic = "/vlago/tft/language"; // 中英文 0 中文 1 英文
constexpr const char *tftMaxGearTopic = "/vlago/tft/max_gear"; // 最高挡位
// tft屏触摸事件
constexpr const char *tftTouchEventTopic = "/vlago/tft/touch/event";
constexpr const char *tftJoystickEventTopic = "/tft/joystick/event";
constexpr const char *audioTopic = "/vlago/audio/play";
//灯效类型
constexpr const char *lightTypeTopic = "/vlago/light/type";
// 弹窗类型
constexpr const char *popupTypeTopic = "/vlago/popup/type";
// 错误码topic
constexpr const char* errorEventTopic = "/error_events";


//ota相关
constexpr const char *otaSystemUpgradePrepare = "/system/upgrade_prepare";
constexpr const char *otaPackageInfoTopic = "/ota_agent/upgrade/package_info";
constexpr const char *otaDownloadMessageTopic = "/ota_agent/upgrade/download_message";
constexpr const char *otaDownloadProgressTopic = "/ota_agent/upgrade/download_progress";
constexpr const char *otaUpgradeMessageTopic = "/ota_agent/upgrade/upgrade_message";
constexpr const char *otaUpgradeProgressTopic = "/ota_agent/upgrade/upgrade_progress";

//service/upgrade/package_download
constexpr const char *UpgradePackageDownloadService = "/service/upgrade/package_download";
//service/upgrade/package_upgrade
constexpr const char *UpgradePackageUpgradeService = "/service/upgrade/package_upgrade";
//service/upgrade/package_check
constexpr const char *UpgradePackageCheckService = "/service/upgrade/package_check";


//cloud_manager
constexpr const char *cloudManagerAuthInfoTopic = "/cloud_manager/mqtt/auth_info";
constexpr const char *cloudManagerDeviceBindService = "/cloud_manager/device/bind/status";
constexpr const char *cloudManagerDeviceBindPubTopic = "/vehicle/bind/status";

constexpr const char *cloudManagerGetAuthInfoService = "/cloud_manager/mqtt/get_auth_info";
