from dataclasses import dataclass
import yaml
from typing import List, Tuple
import numpy as np
import os

@dataclass
class VehicleParameters:
    name: str
    width: float
    length: float
    height: float

    wheel_radius: float
    wheel_width: float
    wheel_base: float
    wheel_tread: float

    front_overhang: float
    rear_overhang: float
    left_overhang: float
    right_overhang: float

    ground_offset: float

    front_bumper_height: float
    a_pillar_height: float
    front_roof_height: float
    rear_roof_height: float
    d_pillar_height: float
    rear_bumper_height: float

    front_bumper_length: float
    a_pillar_length: float
    d_pillar_length: float
    rear_bumper_length: float

    front_roof_width: float
    rear_roof_width: float

    up_front_chamfer_width: float
    up_front_chamfer_height: float
    down_front_chamfer_width: float
    down_front_chamfer_height: float

    up_side_chamfer_width: float
    up_side_chamfer_height: float
    down_side_chamfer_width: float
    down_side_chamfer_height: float

    up_rear_chamfer_width: float
    up_rear_chamfer_height: float
    down_rear_chamfer_width: float
    down_rear_chamfer_height: float

    body_color_r: float
    body_color_g: float
    body_color_b: float

    window_color_r: float
    window_color_g: float
    window_color_b: float

    @property
    def body_color(self) -> Tuple[float, float, float]:
        """차체 색상 반환"""
        return (self.body_color_r, self.body_color_g, self.body_color_b)

    @property
    def window_color(self) -> Tuple[float, float, float]:
        """창문 색상 반환"""
        return (self.window_color_r, self.window_color_g, self.window_color_b)

def polygon_to_triangles(polygon):
    triangles = []
    n = len(polygon)
    for i in range(n - 2):
        triangles.append([polygon[0], polygon[i + 1], polygon[i + 2]])
        
    return triangles

def create_vehicle_faces():
    faces = []

    FB_UP_UP_LEFT = 0
    FB_UP_SIDE_LEFT = 1
    FB_DOWN_SIDE_LEFT = 2
    FB_DOWN_DOWN_LEFT = 3
    FB_DOWN_DOWN_RIGHT = 4
    FB_DOWN_SIDE_RIGHT = 5
    FB_UP_SIDE_RIGHT = 6
    FB_UP_UP_RIGHT = 7

    faces.extend(polygon_to_triangles([
        FB_UP_UP_LEFT,
        FB_UP_SIDE_LEFT,
        FB_DOWN_SIDE_LEFT,
        FB_DOWN_DOWN_LEFT,
        FB_DOWN_DOWN_RIGHT,
        FB_DOWN_SIDE_RIGHT,
        FB_UP_SIDE_RIGHT,
        FB_UP_UP_RIGHT
    ]))

    AP_UP_UP_LEFT = 8
    AP_UP_SIDE_LEFT = 9
    AP_DOWN_SIDE_LEFT = 10
    AP_DOWN_DOWN_LEFT = 11
    AP_DOWN_DOWN_RIGHT = 12
    AP_DOWN_SIDE_RIGHT = 13
    AP_UP_SIDE_RIGHT = 14
    AP_UP_UP_RIGHT = 15

    faces.extend(polygon_to_triangles([
        FB_UP_UP_LEFT,
        FB_UP_SIDE_LEFT,
        AP_UP_SIDE_LEFT,
        AP_UP_UP_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_UP_SIDE_LEFT,
        FB_DOWN_SIDE_LEFT,
        AP_DOWN_SIDE_LEFT,
        AP_UP_SIDE_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_DOWN_SIDE_LEFT,
        FB_DOWN_DOWN_LEFT,
        AP_DOWN_DOWN_LEFT,
        AP_DOWN_SIDE_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_DOWN_DOWN_LEFT,
        FB_DOWN_DOWN_RIGHT,
        AP_DOWN_DOWN_RIGHT,
        AP_DOWN_DOWN_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_DOWN_DOWN_RIGHT,
        FB_DOWN_SIDE_RIGHT,
        AP_DOWN_SIDE_RIGHT,
        AP_DOWN_DOWN_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_DOWN_SIDE_RIGHT,
        FB_UP_SIDE_RIGHT,
        AP_UP_SIDE_RIGHT,
        AP_DOWN_SIDE_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_UP_SIDE_RIGHT,
        FB_UP_UP_RIGHT,
        AP_UP_UP_RIGHT,
        AP_UP_SIDE_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        FB_UP_UP_RIGHT,
        FB_UP_UP_LEFT,
        AP_UP_UP_LEFT,
        AP_UP_UP_RIGHT,
    ]))

    FR_LEFT = 16
    FR_RIGHT = 17
    RR_LEFT = 18
    RR_RIGHT = 19

    DP_UP_UP_LEFT = 20
    DP_UP_SIDE_LEFT = 21
    DP_DOWN_SIDE_LEFT = 22
    DP_DOWN_DOWN_LEFT = 23
    DP_DOWN_DOWN_RIGHT = 24
    DP_DOWN_SIDE_RIGHT = 25
    DP_UP_SIDE_RIGHT = 26
    DP_UP_UP_RIGHT = 27

    # front window
    faces.extend(polygon_to_triangles([
        AP_UP_UP_LEFT,
        AP_UP_UP_RIGHT,
        FR_RIGHT,
        FR_LEFT,
    ]))
    # left window
    faces.extend(polygon_to_triangles([
        AP_UP_UP_LEFT,
        FR_LEFT,
        RR_LEFT,
        DP_UP_UP_LEFT,
    ]))
    # right window
    faces.extend(polygon_to_triangles([
        AP_UP_UP_RIGHT,
        FR_RIGHT,
        RR_RIGHT,
        DP_UP_UP_RIGHT,
    ]))
    # rear window
    faces.extend(polygon_to_triangles([
        DP_UP_UP_LEFT,
        RR_LEFT,
        RR_RIGHT,
        DP_UP_UP_RIGHT,
    ]))
    # roof
    faces.extend(polygon_to_triangles([
        FR_LEFT,
        FR_RIGHT,
        RR_RIGHT,
        RR_LEFT,
    ]))

    faces.extend(polygon_to_triangles([
        AP_UP_UP_LEFT,
        AP_UP_SIDE_LEFT,
        DP_UP_SIDE_LEFT,
        DP_UP_UP_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        AP_UP_SIDE_LEFT,
        AP_DOWN_SIDE_LEFT,
        DP_DOWN_SIDE_LEFT,
        DP_UP_SIDE_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        AP_DOWN_SIDE_LEFT,
        AP_DOWN_DOWN_LEFT,
        DP_DOWN_DOWN_LEFT,
        DP_DOWN_SIDE_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        AP_DOWN_DOWN_LEFT,
        AP_DOWN_DOWN_RIGHT,
        DP_DOWN_DOWN_RIGHT,
        DP_DOWN_DOWN_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        AP_DOWN_DOWN_RIGHT,
        AP_DOWN_SIDE_RIGHT,
        DP_DOWN_SIDE_RIGHT,
        DP_DOWN_DOWN_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        AP_DOWN_SIDE_RIGHT,
        AP_UP_SIDE_RIGHT,
        DP_UP_SIDE_RIGHT,
        DP_DOWN_SIDE_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        AP_UP_SIDE_RIGHT,
        AP_UP_UP_RIGHT,
        DP_UP_UP_RIGHT,
        DP_UP_SIDE_RIGHT,
    ]))

    RB_UP_UP_LEFT = 28
    RB_UP_SIDE_LEFT = 29
    RB_DOWN_SIDE_LEFT = 30
    RB_DOWN_DOWN_LEFT = 31
    RB_DOWN_DOWN_RIGHT = 32
    RB_DOWN_SIDE_RIGHT = 33
    RB_UP_SIDE_RIGHT = 34
    RB_UP_UP_RIGHT = 35

    faces.extend(polygon_to_triangles([
        DP_UP_UP_LEFT,
        DP_UP_SIDE_LEFT,
        RB_UP_SIDE_LEFT,
        RB_UP_UP_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_UP_SIDE_LEFT,
        DP_DOWN_SIDE_LEFT,
        RB_DOWN_SIDE_LEFT,
        RB_UP_SIDE_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_DOWN_SIDE_LEFT,
        DP_DOWN_DOWN_LEFT,
        RB_DOWN_DOWN_LEFT,
        RB_DOWN_SIDE_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_DOWN_DOWN_LEFT,
        DP_DOWN_DOWN_RIGHT,
        RB_DOWN_DOWN_RIGHT,
        RB_DOWN_DOWN_LEFT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_DOWN_DOWN_RIGHT,
        DP_DOWN_SIDE_RIGHT,
        RB_DOWN_SIDE_RIGHT,
        RB_DOWN_DOWN_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_DOWN_SIDE_RIGHT,
        DP_UP_SIDE_RIGHT,
        RB_UP_SIDE_RIGHT,
        RB_DOWN_SIDE_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_UP_SIDE_RIGHT,
        DP_UP_UP_RIGHT,
        RB_UP_UP_RIGHT,
        RB_UP_SIDE_RIGHT,
    ]))
    faces.extend(polygon_to_triangles([
        DP_UP_UP_RIGHT,
        DP_UP_UP_LEFT,
        RB_UP_UP_LEFT,
        RB_UP_UP_RIGHT,
    ]))

    faces.extend(polygon_to_triangles([
        RB_UP_UP_LEFT,
        RB_UP_SIDE_LEFT,
        RB_DOWN_SIDE_LEFT,
        RB_DOWN_DOWN_LEFT,
        RB_DOWN_DOWN_RIGHT,
        RB_DOWN_SIDE_RIGHT,
        RB_UP_SIDE_RIGHT,
        RB_UP_UP_RIGHT,
    ]))

    return faces

def create_vehicle_vertices(params: VehicleParameters):
    vertices = []

    wheel_radius = params.wheel_radius
    wheel_width = params.wheel_width
    wheel_base = params.wheel_base
    wheel_tread = params.wheel_tread

    front_overhang = params.front_overhang
    rear_overhang = params.rear_overhang
    left_overhang = params.left_overhang
    right_overhang = params.right_overhang

    ground_offset = params.ground_offset

    front_bumper_height = params.front_bumper_height
    a_pillar_height = params.a_pillar_height
    front_roof_height = params.front_roof_height
    rear_roof_height = params.rear_roof_height
    d_pillar_height = params.d_pillar_height
    rear_bumper_height = params.rear_bumper_height

    front_bumper_length = params.front_bumper_length
    a_pillar_length = params.a_pillar_length
    d_pillar_length = params.d_pillar_length
    rear_bumper_length = params.rear_bumper_length

    front_roof_width = params.front_roof_width
    rear_roof_width = params.rear_roof_width

    up_side_chamfer_width = params.up_side_chamfer_width
    up_side_chamfer_height = params.up_side_chamfer_height
    down_side_chamfer_width = params.down_side_chamfer_width
    down_side_chamfer_height = params.down_side_chamfer_height

    up_front_chamfer_width = params.up_front_chamfer_width
    up_front_chamfer_height = params.up_front_chamfer_height
    down_front_chamfer_width = params.down_front_chamfer_width
    down_front_chamfer_height = params.down_front_chamfer_height

    up_rear_chamfer_width = params.up_rear_chamfer_width
    up_rear_chamfer_height = params.up_rear_chamfer_height
    down_rear_chamfer_width = params.down_rear_chamfer_width
    down_rear_chamfer_height = params.down_rear_chamfer_height

    x_left = -wheel_tread / 2 - left_overhang
    x_right = wheel_tread / 2 + right_overhang
    z = wheel_base / 2 + front_overhang
    FB_UP_UP_LEFT = [
        x_left + up_front_chamfer_width,
        front_bumper_height,
        z
        ]
    FB_UP_SIDE_LEFT = [
        x_left,
        front_bumper_height - up_side_chamfer_height,
        z
        ]
    FB_DOWN_SIDE_LEFT = [
        x_left,
        ground_offset + down_front_chamfer_height,
        z
        ]
    FB_DOWN_DOWN_LEFT = [
        x_left + down_front_chamfer_width,
        ground_offset,
        z
        ]
    FB_DOWN_DOWN_RIGHT = [
        x_right - down_front_chamfer_width,
        ground_offset,
        z
        ]
    FB_DOWN_SIDE_RIGHT = [
        x_right,
        ground_offset + down_front_chamfer_height,
        z
        ]
    FB_UP_SIDE_RIGHT = [
        x_right,
        front_bumper_height - up_side_chamfer_height,
        z
        ]
    FB_UP_UP_RIGHT = [
        x_right - up_front_chamfer_width,
        front_bumper_height,
        z
        ]

    z -= front_bumper_length
    AP_UP_UP_LEFT = [
        x_left + up_side_chamfer_width,
        a_pillar_height,
        z
        ]
    AP_UP_SIDE_LEFT = [
        x_left,
        a_pillar_height - up_side_chamfer_height,
        z
        ]
    AP_DOWN_SIDE_LEFT = [
        x_left,
        ground_offset + down_side_chamfer_height,
        z
        ]
    AP_DOWN_DOWN_LEFT = [
        x_left + down_side_chamfer_width,
        ground_offset,
        z
        ]
    AP_DOWN_DOWN_RIGHT = [
        x_right - down_side_chamfer_width,
        ground_offset,
        z
        ]
    AP_DOWN_SIDE_RIGHT = [
        x_right,
        ground_offset + down_side_chamfer_height,
        z
        ]
    AP_UP_SIDE_RIGHT = [
        x_right,
        a_pillar_height - up_side_chamfer_height,
        z
        ]
    AP_UP_UP_RIGHT = [
        x_right - up_side_chamfer_width,
        a_pillar_height,
        z
        ]

    z -= a_pillar_length
    FR_LEFT = [
        x_left + up_side_chamfer_width + front_roof_width,
        front_roof_height,
        z
        ]
    FR_RIGHT = [
        x_right - up_side_chamfer_width - front_roof_width,
        front_roof_height,
        z
        ]
    
    z = (-wheel_base / 2 - rear_overhang) + rear_bumper_length + d_pillar_length
    RR_LEFT = [
        x_left + up_side_chamfer_width + rear_roof_width,
        rear_roof_height,
        z
        ]
    RR_RIGHT = [
        x_right - up_side_chamfer_width - rear_roof_width,
        rear_roof_height,
        z
        ]

    z -= d_pillar_length
    DP_UP_UP_LEFT = [
        x_left + up_side_chamfer_width,
        d_pillar_height,
        z
        ]
    DP_UP_SIDE_LEFT = [
        x_left,
        d_pillar_height - up_side_chamfer_height,
        z
        ]
    DP_DOWN_SIDE_LEFT = [
        x_left,
        ground_offset + down_side_chamfer_height,
        z
        ]
    DP_DOWN_DOWN_LEFT = [
        x_left + down_side_chamfer_width,
        ground_offset,
        z
        ]
    DP_DOWN_DOWN_RIGHT = [
        x_right - down_side_chamfer_width,
        ground_offset,
        z
        ]
    DP_DOWN_SIDE_RIGHT = [
        x_right,
        ground_offset + down_side_chamfer_height,
        z
        ]
    DP_UP_SIDE_RIGHT = [
        x_right,
        d_pillar_height - up_side_chamfer_height,
        z
        ]
    DP_UP_UP_RIGHT = [
        x_right - up_side_chamfer_width,
        d_pillar_height,
        z
        ]

    z -= rear_bumper_length
    RB_UP_UP_LEFT = [
        x_left + up_side_chamfer_width,
        rear_bumper_height,
        z
        ]
    RB_UP_SIDE_LEFT = [
        x_left,
        rear_bumper_height - up_side_chamfer_height,
        z
        ]
    RB_DOWN_SIDE_LEFT = [
        x_left,
        ground_offset + down_side_chamfer_height,
        z
        ]
    RB_DOWN_DOWN_LEFT = [
        x_left + down_side_chamfer_width,
        ground_offset,
        z
        ]
    RB_DOWN_DOWN_RIGHT = [
        x_right - down_side_chamfer_width,
        ground_offset,
        z
        ]
    RB_DOWN_SIDE_RIGHT = [
        x_right,
        ground_offset + down_side_chamfer_height,
        z
        ]
    RB_UP_SIDE_RIGHT = [
        x_right,
        rear_bumper_height - up_side_chamfer_height,
        z
        ]
    RB_UP_UP_RIGHT = [
        x_right - up_side_chamfer_width,
        rear_bumper_height,
        z
        ]

    vertices.extend([
        FB_UP_UP_LEFT,
        FB_UP_SIDE_LEFT,
        FB_DOWN_SIDE_LEFT,
        FB_DOWN_DOWN_LEFT,
        FB_DOWN_DOWN_RIGHT,
        FB_DOWN_SIDE_RIGHT,
        FB_UP_SIDE_RIGHT,
        FB_UP_UP_RIGHT,
        AP_UP_UP_LEFT,
        AP_UP_SIDE_LEFT,
        AP_DOWN_SIDE_LEFT,
        AP_DOWN_DOWN_LEFT,
        AP_DOWN_DOWN_RIGHT,
        AP_DOWN_SIDE_RIGHT,
        AP_UP_SIDE_RIGHT,
        AP_UP_UP_RIGHT,
        FR_LEFT,
        FR_RIGHT,
        RR_LEFT,
        RR_RIGHT,
        DP_UP_UP_LEFT,
        DP_UP_SIDE_LEFT,
        DP_DOWN_SIDE_LEFT,
        DP_DOWN_DOWN_LEFT,
        DP_DOWN_DOWN_RIGHT,
        DP_DOWN_SIDE_RIGHT,
        DP_UP_SIDE_RIGHT,
        DP_UP_UP_RIGHT,
        RB_UP_UP_LEFT,
        RB_UP_SIDE_LEFT,
        RB_DOWN_SIDE_LEFT,
        RB_DOWN_DOWN_LEFT,
        RB_DOWN_DOWN_RIGHT,
        RB_DOWN_SIDE_RIGHT,
        RB_UP_SIDE_RIGHT,
        RB_UP_UP_RIGHT,
    ])

    return vertices

def export_to_obj(
    params: VehicleParameters,
    filename: str = "vehicle.obj",
    body_color: Tuple[float, float, float] = None,  # 기본값 None으로 변경
    window_color: Tuple[float, float, float] = None  # 기본값 None으로 변경
):
    """차량 모델을 OBJ 파일로 변환 (MTL 파일 포함)"""
    
    # 색상 파라미터가 제공되지 않으면 VehicleParameters에서 가져옴
    if body_color is None:
        body_color = params.body_color
    if window_color is None:
        window_color = params.window_color
    
    # 버텍스와 면 생성
    vertices = create_vehicle_vertices(params)
    faces = create_vehicle_faces()
    
    # 윈도우 면 인덱스 찾기 (프론트, 좌측, 우측, 리어 윈도우)
    window_face_indices = [15, 16, 17, 18]
    
    # OBJ 파일 작성
    with open(filename, 'w') as f:
        f.write(f"# Vehicle Model\n")
        f.write(f"# Generated from VehicleParameters\n")
        f.write(f"mtllib {filename.replace('.obj', '.mtl')}\n\n")
        
        # 버텍스 작성
        f.write("# Vertices\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        
        # 노말 작성 (단순화를 위해 모두 (0, 0, 1)로 설정)
        f.write("\n# Normals (simplified)\n")
        for _ in range(len(vertices)):
            f.write(f"vn 0.0 0.0 1.0\n")
        
        # MTL 재질 지정
        f.write("\n# Material assignment\n")
        f.write("usemtl Body\n")
        
        # 면 작성 (윈도우 면을 제외한 나머지)
        f.write("\n# Body faces\n")
        for i, face in enumerate(faces):
            if i not in window_face_indices:
                f.write(f"f {face[0]+1}//{face[0]+1} {face[1]+1}//{face[1]+1} {face[2]+1}//{face[2]+1}\n")
        
        # 윈도우 면 작성 (별도 재질)
        f.write("\n# Window faces\n")
        f.write("usemtl Windows\n")
        for i, face in enumerate(faces):
            if i in window_face_indices:
                f.write(f"f {face[0]+1}//{face[0]+1} {face[1]+1}//{face[1]+1} {face[2]+1}//{face[2]+1}\n")
    
    # MTL 파일 작성
    mtl_filename = filename.replace('.obj', '.mtl')
    with open(mtl_filename, 'w') as f:
        f.write("# Material definitions\n\n")
        
        # 차체 재질
        f.write("newmtl Body\n")
        f.write("Ka 0.2 0.2 0.2  # Ambient\n")
        f.write(f"Kd {body_color[0]:.3f} {body_color[1]:.3f} {body_color[2]:.3f}  # Diffuse\n")
        f.write("Ks 0.5 0.5 0.5  # Specular\n")
        f.write("Ns 32.0        # Shininess\n")
        f.write("d 1.0          # Transparency\n\n")
        
        # 윈도우 재질
        f.write("newmtl Windows\n")
        f.write("Ka 0.1 0.1 0.1  # Ambient\n")
        f.write(f"Kd {window_color[0]:.3f} {window_color[1]:.3f} {window_color[2]:.3f}  # Diffuse\n")
        f.write("Ks 0.8 0.8 0.8  # Specular\n")
        f.write("Ns 96.0        # Shininess (더 반짝임)\n")
        f.write("d 0.7          # Transparency (반투명)\n")


def export_to_obj_simple(
    params: VehicleParameters,
    filename: str = "vehicle_simple.obj",
    body_color: Tuple[float, float, float] = None,  # 기본값 None으로 변경
    window_color: Tuple[float, float, float] = None  # 기본값 None으로 변경
):
    """간단한 OBJ 파일 형식 (노말 없음, 윈도우 색상 포함)"""
    
    # 색상 파라미터가 제공되지 않으면 VehicleParameters에서 가져옴
    if body_color is None:
        body_color = params.body_color
    if window_color is None:
        window_color = params.window_color
    
    vertices = create_vehicle_vertices(params)
    faces = create_vehicle_faces()
    
    # 윈도우 면 인덱스 (동일하게 설정)
    window_face_indices = [15, 16, 17, 18]
    
    # OBJ 파일 작성
    with open(filename, 'w') as f:
        f.write(f"# Vehicle Model (Simple)\n")
        f.write(f"# Generated from VehicleParameters\n")
        f.write(f"mtllib {filename.replace('.obj', '.mtl')}\n\n")
        
        # 버텍스 작성
        f.write("# Vertices\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        
        # MTL 재질 지정
        f.write("\n# Material assignment\n")
        f.write("usemtl Body\n")
        
        # 차체 면 작성
        f.write("\n# Body faces\n")
        for i, face in enumerate(faces):
            if i not in window_face_indices:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
        
        # 윈도우 면 작성
        f.write("\n# Window faces\n")
        f.write("usemtl Windows\n")
        for i, face in enumerate(faces):
            if i in window_face_indices:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    
    # MTL 파일 작성
    mtl_filename = filename.replace('.obj', '.mtl')
    with open(mtl_filename, 'w') as f:
        f.write("# Material definitions\n\n")
        
        # 차체 재질
        f.write("newmtl Body\n")
        f.write(f"Kd {body_color[0]:.3f} {body_color[1]:.3f} {body_color[2]:.3f}\n")
        f.write("Ka 0.2 0.2 0.2\n\n")
        
        # 윈도우 재질
        f.write("newmtl Windows\n")
        f.write(f"Kd {window_color[0]:.3f} {window_color[1]:.3f} {window_color[2]:.3f}\n")
        f.write("Ka 0.1 0.1 0.1\n")


def export_vertex_face_list(
    params: VehicleParameters,
    return_lists: bool = False
):
    """디버깅용: 버텍스와 면 리스트 반환"""
    
    vertices = create_vehicle_vertices(params)
    faces = create_vehicle_faces()
    
    # 윈도우 면 인덱스 및 정보
    window_face_indices = [15, 16, 17, 18]
    window_names = ["Front Window", "Left Window", "Right Window", "Rear Window"]
    
    print(f"Total vertices: {len(vertices)}")
    print(f"Total faces: {len(faces)}")
    print(f"Window faces: {len(window_face_indices)}")
    
    for i, idx in enumerate(window_face_indices):
        print(f"  {window_names[i]}: Face index {idx}, Vertex indices: {faces[idx]}")
    
    if return_lists:
        return vertices, faces, window_face_indices