#!/usr/bin/env python3
"""
STL文件简化脚本
使用trimesh库减少STL文件的面数，加快加载速度
"""
import os
import sys
import subprocess

# 检查并安装所需库
try:
    import trimesh
    import numpy as np
except ImportError:
    print("正在安装所需库...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "trimesh[easy]", "numpy"])
    import trimesh
    import numpy as np

# 尝试安装简化库
try:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pyfqmr", "-q"], 
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
except:
    pass

def simplify_stl(input_path, output_path, target_reduction=0.1):
    """
    简化STL文件
    
    Args:
        input_path: 输入STL文件路径
        output_path: 输出STL文件路径
        target_reduction: 目标保留的面数比例 (0.1 = 保留10%的面)
    """
    print(f"处理: {os.path.basename(input_path)}")
    
    # 加载mesh
    mesh = trimesh.load(input_path)
    original_faces = len(mesh.faces)
    
    # 计算目标面数
    target_faces = int(original_faces * target_reduction)
    if target_faces < 100:
        target_faces = 100  # 至少保留100个面
    
    print(f"  原始面数: {original_faces}")
    print(f"  目标面数: {target_faces}")
    
    # 尝试使用不同的简化方法
    simplified = None
    try:
        # 方法1: 使用pyfqmr (最快最好)
        import pyfqmr
        mesh_simplifier = pyfqmr.Simplify()
        mesh_simplifier.setMesh(mesh.vertices, mesh.faces)
        mesh_simplifier.simplify_mesh(target_count=target_faces, aggressiveness=7, preserve_border=True, verbose=False)
        vertices, faces, normals = mesh_simplifier.getMesh()
        simplified = trimesh.Trimesh(vertices=vertices, faces=faces)
        print(f"  使用方法: pyfqmr")
    except:
        try:
            # 方法2: quadric decimation
            simplified = mesh.simplify_quadric_decimation(target_faces)
            print(f"  使用方法: quadric_decimation")
        except:
            # 方法3: 均匀采样
            ratio = target_faces / original_faces
            simplified = mesh.simplify_quadratic_decimation(face_count=target_faces)
            print(f"  使用方法: quadratic_decimation")
    
    if simplified is None:
        print(f"  警告: 无法简化，复制原文件")
        import shutil
        shutil.copy2(input_path, output_path)
        return False
    
    print(f"  简化后面数: {len(simplified.faces)}")
    print(f"  减少: {100 * (1 - len(simplified.faces) / original_faces):.1f}%")
    
    # 保存
    simplified.export(output_path)
    
    # 显示文件大小变化
    original_size = os.path.getsize(input_path) / (1024 * 1024)
    new_size = os.path.getsize(output_path) / (1024 * 1024)
    print(f"  文件大小: {original_size:.1f}MB -> {new_size:.1f}MB (减少 {100 * (1 - new_size / original_size):.1f}%)")
    print()
    return True

def process_directory(input_dir, output_dir, reduction=0.1):
    """
    处理整个目录的STL文件
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    stl_files = []
    for root, dirs, files in os.walk(input_dir):
        for file in files:
            if file.upper().endswith('.STL'):
                stl_files.append(os.path.join(root, file))
    
    print(f"找到 {len(stl_files)} 个STL文件\n")
    
    total_original = 0
    total_new = 0
    success_count = 0
    
    for stl_file in stl_files:
        # 保持相同的目录结构
        rel_path = os.path.relpath(stl_file, input_dir)
        output_path = os.path.join(output_dir, rel_path)
        
        # 创建输出目录
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        try:
            original_size = os.path.getsize(stl_file)
            if simplify_stl(stl_file, output_path, reduction):
                new_size = os.path.getsize(output_path)
                total_original += original_size
                total_new += new_size
                success_count += 1
        except Exception as e:
            print(f"  错误: {e}\n")
    
    print("=" * 60)
    if total_original > 0:
        print(f"成功处理: {success_count}/{len(stl_files)} 个文件")
        print(f"总文件大小: {total_original / (1024 * 1024):.1f}MB -> {total_new / (1024 * 1024):.1f}MB")
        print(f"总共减少: {100 * (1 - total_new / total_original):.1f}%")
    else:
        print("没有成功处理任何文件")
    print("=" * 60)

if __name__ == "__main__":
    # 配置路径
    mesh_dir = "/home/lanchong/bw_motion_ws/src/mantis_description/meshes"
    output_dir = "/home/lanchong/bw_motion_ws/src/mantis_description/meshes_simplified"
    
    # 简化比例：0.1 表示保留10%的面数，0.2 表示保留20%
    # 对于视觉效果，建议visual使用0.15-0.2，collision使用0.05-0.1
    reduction_ratio = 0.15  # 可以调整这个值
    
    print("=" * 60)
    print("STL文件简化工具")
    print("=" * 60)
    print(f"输入目录: {mesh_dir}")
    print(f"输出目录: {output_dir}")
    print(f"简化比例: 保留 {reduction_ratio * 100}% 的面数")
    print("=" * 60)
    print()
    
    response = input("是否继续? (y/n): ")
    if response.lower() != 'y':
        print("已取消")
        sys.exit(0)
    
    print()
    process_directory(mesh_dir, output_dir, reduction_ratio)
    
    print("\n完成！")
    print(f"简化后的文件保存在: {output_dir}")
    print("\n下一步:")
    print("1. 检查简化后的模型是否满意")
    print("2. 如果满意，备份原文件后替换:")
    print(f"   mv {mesh_dir} {mesh_dir}_backup")
    print(f"   mv {output_dir} {mesh_dir}")
    print("3. 重新编译: colcon build --packages-select mantis_description")
