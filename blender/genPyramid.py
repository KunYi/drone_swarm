# use for Blender v4.x
import bpy
import math
import json
import os


# 金字塔参数
base_size = 20  # 底面边长
height = 18  # 金字塔高度
layers = 6  # 层数
min_distance = 2.20  # 点位的最小间距
output_path = "/tmp/points.json"  # 输出的 JSON 文件路径

# 删除现有物体
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# 创建金字塔的函数
def create_pyramid(base_size, height, layers):
    vertices = []
    faces = []
    step_height = height / layers
    step_shrink = base_size / (2 * layers)
    
    # 创建每一层
    for i in range(layers + 1):
        current_size = base_size - 2 * i * step_shrink
        current_height = i * step_height
        if current_size <= 0:
            break
        
        # 创建当前层的四个顶点
        half = current_size / 2
        vertices.extend([
            (-half, -half, current_height),
            ( half, -half, current_height),
            ( half,  half, current_height),
            (-half,  half, current_height),
        ])
        
        # 创建面（从第二层开始连接到上一层）
        if i > 0:
            base = 4 * (i - 1)
            current = 4 * i
            faces.extend([
                [base, base + 1, current + 1, current],
                [base + 1, base + 2, current + 2, current + 1],
                [base + 2, base + 3, current + 3, current + 2],
                [base + 3, base, current, current + 3],
            ])
    
    # 添加金字塔顶点
    vertices.append((0, 0, height))
    apex_index = len(vertices) - 1
    base = 4 * (layers - 1)
    faces.extend([
        [base, base + 1, apex_index],
        [base + 1, base + 2, apex_index],
        [base + 2, base + 3, apex_index],
        [base + 3, base, apex_index],
    ])
    
    return vertices, faces

# 插入顶点和边上点的函数
def create_points(vertices, layers, min_distance):
    points = []
    original_points = []  # 用来存储原始顶点
    # 每层根据顶点生成点
    for i in range(layers):
        layer_vertices = vertices[i * 4:(i + 1) * 4]
        for v1, v2 in zip(layer_vertices, layer_vertices[1:] + layer_vertices[:1]):
            # 加入原始顶点
            original_points.append(v1)
            # 均匀插入点
            length = math.dist(v1[:2], v2[:2])
            num_points = int(length / min_distance)
            for j in range(1, num_points):
                t = j / num_points
                x = (1 - t) * v1[0] + t * v2[0]
                y = (1 - t) * v1[1] + t * v2[1]
                z = v1[2]
                points.append((x, y, z))
        # 加入最后一个顶点
        original_points.append(layer_vertices[-1])
    
    # 添加金字塔顶点
    original_points.append((0, 0, height))  # 顶点的坐标
    return points, original_points

# 导出点位到 JSON 的函数
def export_points_to_json(original_points, edge_points, output_path):
    data = {
        "original_points": [{"x": p[0], "y": p[1], "z": p[2]} for p in original_points],
        "edge_points": [{"x": p[0], "y": p[1], "z": p[2]} for p in edge_points],
    }
    with open(output_path, "w") as f:
        json.dump(data, f, indent=4)
    print(f"点位数据已保存到 {output_path}")
    # 输出总点数
    total_points = len(original_points) + len(edge_points)
    print(f"总共生成了 {total_points} 个点位")

# 创建金字塔
vertices, faces = create_pyramid(base_size, height, layers)

# 创建金字塔模型
mesh = bpy.data.meshes.new("Pyramid")
mesh.from_pydata(vertices, [], faces)
obj = bpy.data.objects.new("Pyramid", mesh)
bpy.context.collection.objects.link(obj)

# 插入点位
edge_points, original_points = create_points(vertices, layers, min_distance)

# 显示原始顶点
for p in original_points:
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.2, location=p)  # 用较大的球体表示顶点

# 显示边上的点
for p in edge_points:
    bpy.ops.object.empty_add(type='SPHERE', radius=0.1, location=p)  # 用小球表示边上的点

# 导出点位数据到 JSON
export_points_to_json(original_points, edge_points, output_path)
