# import mmap
# import os

# def read_image_from_shared_memory(shared_memory_name, image_size):
#     # 打开共享内存
#     with open(shared_memory_name, "r+b") as f:
#         # 创建内存映射
#         with mmap.mmap(f.fileno(), length=image_size, access=mmap.ACCESS_READ) as mm:
#             # 读取内存映射中的图像数据
#             image_data = mm.read()

#     return image_data

# # 示例用法
# shared_memory_name = "rgbimage"
# image_size = 188942 #1024 * 1024 * 3  # 假设图像大小为 3 MB
# image_data = read_image_from_shared_memory(shared_memory_name, image_size)

# # 将图像数据保存到文件
# with open("output.jpg", "wb") as f:
#     f.write(image_data)


import mmap
import struct
from PIL import Image
import io

def read_image_from_shared_memory(shared_memory_name, image_size):
    # 打开共享内存对象
    h_mapping = mmap.mmap(0, image_size, tagname=shared_memory_name, access=mmap.ACCESS_READ)
    
    # 读取图像数据
    image_data = h_mapping.read(image_size)
    h_mapping.close()

    # 使用Pillow将字节数据转换为图像
    image = Image.open(io.BytesIO(image_data))
    return image

# 示例用法
shared_memory_name = 'rgbimage'
image_size = 188942 #1024 * 1024  # 假设图像大小为1MB，可以根据实际情况调整
image = read_image_from_shared_memory(shared_memory_name, image_size)

# 显示图像
image.show()
