"""Pyramid formation calculation module."""
import numpy as np
from typing import List, Tuple
from core import (
    SAFE_DISTANCE,
    SPACING_FACTOR,
    MIN_HEIGHT,
    LAYER_HEIGHT
)

# 無人機的安全參數
# SAFE_DISTANCE = 2.0  # 安全距離
# SPACING_FACTOR = 2.0  # 間距係數
# MIN_HEIGHT = 4.0  # 最低飛行高度
# LAYER_HEIGHT = 2.0  # 固定層高

class PyramidGeometry:
    def __init__(self, base_size: float, height: float, num_layers: int):
        self.base_size = base_size
        self.height = height
        self.num_layers = num_layers
        
    def get_layer_size(self, layer_idx: int) -> float:
        """計算特定層的邊長"""
        ratio = (self.num_layers - layer_idx) / self.num_layers
        return self.base_size * ratio
        
    def get_layer_height(self, layer_idx: int) -> float:
        """計算特定層的高度"""
        return MIN_HEIGHT + layer_idx * LAYER_HEIGHT  # 修改這裡，讓較低層的高度較低

def generate_edge_points(layer_size: float, center_x: float, center_y: float, 
                        height: float, safe_distance: float) -> List[Tuple[float, float, float]]:
    """在層的邊緣生成點"""
    points = []
    
    # 計算每條邊可以放置的點數
    points_per_edge = max(2, int(layer_size / (safe_distance * SPACING_FACTOR)))
    
    # 實際間距
    spacing = layer_size / (points_per_edge - 1) if points_per_edge > 1 else layer_size
    
    half_size = layer_size / 2
    
    # 生成四條邊的點
    for i in range(points_per_edge):
        # 計算相對位置 (-1 到 1 的範圍)
        t = -1 + 2 * i / (points_per_edge - 1) if points_per_edge > 1 else 0
        
        # 底邊
        points.append((center_x + t * half_size, center_y - half_size, height))
        # 頂邊
        points.append((center_x + t * half_size, center_y + half_size, height))
        # 左邊
        points.append((center_x - half_size, center_y + t * half_size, height))
        # 右邊
        points.append((center_x + half_size, center_y + t * half_size, height))
    
    # 移除重複的角點
    unique_points = []
    for p in points:
        if not any(np.allclose(p, up) for up in unique_points):
            unique_points.append(p)
            
    return unique_points

def generate_inner_points(layer_size: float, center_x: float, center_y: float,
                         height: float, safe_distance: float, edge_points: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
    """在層的內部生成點，確保與邊緣點保持安全距離"""
    points = []
    
    # 計算內部網格的大小
    inner_size = layer_size - 2 * safe_distance * SPACING_FACTOR
    if inner_size <= 0:
        return points
        
    # 計算內部網格的點數
    grid_points = max(1, int(inner_size / (safe_distance * SPACING_FACTOR)))
    
    if grid_points <= 1:
        # 如果只能放一個點，就放在中心
        points.append((center_x, center_y, height))
        return points
        
    # 生成內部網格點
    spacing = inner_size / (grid_points - 1)
    start_x = center_x - inner_size/2
    start_y = center_y - inner_size/2
    
    for i in range(grid_points):
        for j in range(grid_points):
            x = start_x + i * spacing
            y = start_y + j * spacing
            
            # 檢查與所有現有點的距離
            too_close = False
            for ep in edge_points:
                if np.sqrt((x - ep[0])**2 + (y - ep[1])**2) < safe_distance * SPACING_FACTOR:
                    too_close = True
                    break
                    
            if not too_close:
                points.append((x, y, height))
                
    return points

def calculate_layer_points(num_drones: int, layer_size: float, height: float,
                         center_x: float, center_y: float) -> List[Tuple[float, float, float]]:
    """計算單層的所有點位置"""
    # 首先生成邊緣點
    edge_points = generate_edge_points(layer_size, center_x, center_y, height, SAFE_DISTANCE)
    
    # 如果需要的點數小於等於邊緣點數，直接返回需要數量的邊緣點
    if num_drones <= len(edge_points):
        return edge_points[:num_drones]
    
    # 生成內部點
    inner_points = generate_inner_points(layer_size, center_x, center_y, height, 
                                       SAFE_DISTANCE, edge_points)
    
    # 合併所有點
    all_points = edge_points + inner_points
    
    # 如果點數過多，優先保留邊緣點和離中心較遠的點
    if len(all_points) > num_drones:
        # 計算每個點到中心的距離
        distances = [(p, np.sqrt((p[0]-center_x)**2 + (p[1]-center_y)**2)) 
                    for p in all_points[len(edge_points):]]  # 只對內部點計算距離
        
        # 按距離排序
        distances.sort(key=lambda x: -x[1])  # 降序排序，使外圍點優先
        
        # 選擇需要的點數
        selected_inner = [d[0] for d in distances[:num_drones-len(edge_points)]]
        return edge_points + selected_inner
    
    return all_points[:num_drones]

def calculate_pyramid_formation(num_drones: int, center_x: float, center_y: float) -> List[Tuple[float, float, float]]:
    """Calculate positions for pyramid formation"""
    # 定義金字塔的基本參數
    NUM_LAYERS = 6
    BASE_SIZE = SAFE_DISTANCE * SPACING_FACTOR * 12  # 基礎大小
    TOTAL_HEIGHT = LAYER_HEIGHT * (NUM_LAYERS - 1) + MIN_HEIGHT
    
    # 創建金字塔幾何對象
    pyramid = PyramidGeometry(BASE_SIZE, TOTAL_HEIGHT, NUM_LAYERS)
    
    # 計算每層大概需要的無人機數量（根據面積比例）
    layer_areas = [pyramid.get_layer_size(i)**2 for i in range(NUM_LAYERS)]
    total_area = sum(layer_areas)
    layer_ratios = [area/total_area for area in layer_areas]
    
    # 初步分配無人機到每層
    layer_counts = [max(1, int(num_drones * ratio)) for ratio in layer_ratios]
    
    # 調整分配以確保總數正確
    total_assigned = sum(layer_counts)
    if total_assigned < num_drones:
        # 將剩餘的無人機分配給底層
        layer_counts[-1] += num_drones - total_assigned
    elif total_assigned > num_drones:
        # 從上層開始減少無人機數量
        for i in range(NUM_LAYERS-1):
            if layer_counts[i] > 1:
                excess = min(layer_counts[i]-1, total_assigned-num_drones)
                layer_counts[i] -= excess
                total_assigned -= excess
                if total_assigned == num_drones:
                    break
    
    # 生成每層的點位置
    positions = []
    for layer in range(NUM_LAYERS):
        if layer_counts[layer] > 0:
            layer_size = pyramid.get_layer_size(layer)
            height = pyramid.get_layer_height(layer)
            
            layer_positions = calculate_layer_points(
                layer_counts[layer],
                layer_size,
                height,
                center_x,
                center_y
            )
            positions.extend(layer_positions)
    
    return positions[:num_drones]
