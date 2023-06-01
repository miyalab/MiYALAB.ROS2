std_msgs/Header     header
float64             battery         # バッテリー電圧
float64             distance        # 移動距離
float64             temperature     # ロボット内部温度
float64             cpu_temperature # CPU温度
int32[]             encoder         # ロータリーエンコーダパルス計数値
float64[]           pwm             # モータPWM
float64[]           rpm             # モータ回転数
float64[]           current         # モータ電流値
nav_msgs/Odometry   odometry        # オドメトリ
sensor_msgs/Imu     imu             # 慣性計測器
bool                is_emergency    # ロボット動作安定状態
