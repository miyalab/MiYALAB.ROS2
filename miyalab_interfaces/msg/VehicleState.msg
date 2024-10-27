std_msgs/Header     header
float64             battery             # バッテリー電圧
float64             distance            # 総移動距離
float64             rotation            # 総旋回量
float64             temperature         # ロボット内部温度
int64[]             encoder             # ロータリーエンコーダパルス計数値
float64[]           motor_pwm           # モータPWM
float64[]           motor_rpm           # モータ回転数
float64[]           motor_voltage       # モータ電圧
float64[]           motor_current       # モータ電流値
float64[]           motor_temperature   # モータ温度
nav_msgs/Odometry   odometry            # オドメトリ
sensor_msgs/Imu     imu                 # 慣性計測器
bool                is_emergency        # ロボット動作安定状態
