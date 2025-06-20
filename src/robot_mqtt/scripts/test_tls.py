import paho.mqtt.client as mqtt

# 获取参数
self.mqtt_broker = rospy.get_param('~mqtt_broker', 'pec81f9f.ala.cn-hangzhou.emqxsl.cn')
self.mqtt_port = rospy.get_param('~mqtt_port', 8883)  # 使用8883端口用于TLS加密
self.mqtt_tls = rospy.get_param('~mqtt_tls', True)  # 启用TLS

# 创建MQTT客户端
client = mqtt.Client()

# 如果需要用户名和密码，取消注释并设置
# self.mqtt_user = rospy.get_param('~mqtt_user', 'admin')
# self.mqtt_password = rospy.get_param('~mqtt_password', 'admin')
# client.username_pw_set(self.mqtt_user, self.mqtt_password)

# 启用TLS连接
if self.mqtt_tls:
    client.tls_set()  # 使用默认证书，如果有自定义证书，可以在此传入文件路径
    client.tls_insecure_set(True)  # 如果你不验证服务器证书（不推荐）

# 连接到MQTT服务器
try:
    client.connect(self.mqtt_broker, self.mqtt_port, 60)
    print(f"成功连接到 {self.mqtt_broker} 端口 {self.mqtt_port} (TLS加密连接)")
except Exception as e:
    print(f"连接失败: {str(e)}")
