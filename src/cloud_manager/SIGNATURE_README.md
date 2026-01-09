Signature 配置说明

- 参数名称
  - `signature.private_key`：私钥（hex 编码）。可通过 ROS2 参数注入，例如：

    ros2 run cloud_manager cloud_manager_node --ros-args -p signature.private_key:="<hex_private_key>"

  - `signature.public_key`：公钥（hex 编码）。可注入以用于验签。

- 本地测试行为
  - 若未注入任何密钥，节点会自动生成一个临时 seed 并派生 keypair（仅用于本地测试，不安全）。
  - 生产环境请务必注入私钥或至少公钥，并移除默认回退策略（见代码注释）。

- 验证 MQTT v5 User Properties
  - 我们提供 `tools/inspect_mqtt_v5_props.py` 用于订阅并打印消息中的 MQTT v5 属性（需要 `paho-mqtt >= 1.6.1`）。
  - 安装：`pip3 install paho-mqtt>=1.6.1`
  - 运行示例：

    python3 tools/inspect_mqtt_v5_props.py --broker broker.emqx.io --port 1883 --topic "vehicle/up/#" --duration 12

- 注意
  - 在生产环境，请把密钥作为参数或从安全的密钥存储加载，不要把私钥硬编码到源码。
  - 若要强制所有消息验签失败（安全模式），请修改 `SignatureManager::verify_data`，去掉默认通过策略。
