"""Drone system configuration parameters."""

# 無人機的基本安全參數
SAFE_DISTANCE = 1.5  # 最小無人機間距（米）
SPACING_FACTOR = 1.5  # 間距係數，用於計算實際間距
MIN_HEIGHT = 4.0  # 最低飛行高度（米）
LAYER_HEIGHT = 2.0  # 層間高度（米）

# 無人機的物理限制
MAX_VELOCITY = 5.0  # 最大速度（米/秒）
MAX_ACCELERATION = 2.0  # 最大加速度（米/秒²）
MAX_ANGULAR_VELOCITY = 45.0  # 最大角速度（度/秒）

# 無人機的通信參數
COMMUNICATION_RANGE = 50.0  # 通信範圍（米）
UPDATE_RATE = 10  # 位置更新頻率（赫茲）

# 編隊控制參數
CONVERGENCE_THRESHOLD = 0.15  # 編隊收斂閾值（米）
FORMATION_UPDATE_RATE = 30  # 編隊更新頻率（赫茲）
