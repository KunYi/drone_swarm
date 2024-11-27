"""Ground formation calculation module."""
import numpy as np
from core import (
    SAFE_DISTANCE,
    SPACING_FACTOR,
    MIN_HEIGHT
)

def calculate_ground_formation(num_drones, center_x, center_y, spacing=None):
    """Calculate positions for ground grid formation

    Args:
        num_drones: 無人機數量
        center_x: 中心x座標
        center_y: 中心y座標
        spacing: 間距（如果為None則使用安全間距）
    """
    positions = []

    # 如果沒有指定間距，使用安全間距
    if spacing is None:
        spacing = SAFE_DISTANCE * SPACING_FACTOR

    # 計算網格的邊長（使用黃金比例使形狀更接近正方形）
    phi = (1 + np.sqrt(5)) / 2  # 黃金比例
    cols = int(np.sqrt(num_drones * phi))
    rows = (num_drones + cols - 1) // cols  # 向上取整

    # 計算起始位置（使整體居中）
    start_x = center_x - (cols - 1) * spacing / 2
    start_y = center_y - (rows - 1) * spacing / 2

    # 生成網格位置
    count = 0
    for row in range(rows):
        for col in range(cols):
            if count >= num_drones:
                break
            x = start_x + col * spacing
            y = start_y + row * spacing
            z = MIN_HEIGHT  # 使用最低飛行高度
            positions.append((x, y, z))
            count += 1

    return positions
