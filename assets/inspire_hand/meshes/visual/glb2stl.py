import bpy
import os

# 设置输入和输出文件夹路径
input_folder = "/home/rocos/Jian/dex-retargeting/assets/robots/hands/inspire_hand/meshes/visual"  # 替换为你的文件夹路径
output_folder = "/home/rocos/Documents/GitHub/Inspire_Hand/src/inspire_hand_description/meshes/visual"  # 替换为输出文件夹路径

# 确保 OBJ 导出插件已启用
if not bpy.ops.export_scene.obj.poll():
    print("OBJ 导出操作符不可用，请确保插件已启用。")
    exit()

# 获取文件夹中的所有 .glb 文件
glb_files = [f for f in os.listdir(input_folder) if f.endswith(".glb")]

# 批量导入并导出
for glb_file in glb_files:
    # 构造完整路径
    input_path = os.path.join(input_folder, glb_file)
    
    # 导入 GLB 文件
    bpy.ops.import_scene.gltf(filepath=input_path)
    
    # 构造输出路径，文件名不变但扩展名改为 .obj
    output_path = os.path.join(output_folder, os.path.splitext(glb_file)[0] + ".obj")
    
    # 导出为 OBJ 文件
    bpy.ops.export_scene.obj(filepath=output_path, use_selection=False, axis_forward='Y', axis_up='Z')
    
    # 删除当前导入的对象，为下一次导入做准备
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

print("批量转换完成！")