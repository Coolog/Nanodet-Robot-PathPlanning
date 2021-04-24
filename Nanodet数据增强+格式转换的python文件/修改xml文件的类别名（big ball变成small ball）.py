#!/usr/bin/env python2
# -*- coding: utf-8 -*-
 
import os
import xml.etree.ElementTree as ET
 
origin_ann_dir = './第二次标注/'# 设置原始标签路径为 Annos
new_ann_dir = './数据集准备/xml/'# 设置新标签路径 Annotations
for dirpaths, dirnames, filenames in os.walk(origin_ann_dir):   # os.walk游走遍历目录名
  for filename in filenames:
    print("process...")
    if os.path.isfile(r'%s%s' %(origin_ann_dir, filename)):   # 获取原始xml文件绝对路径，isfile()检测是否为文件 isdir检测是否为目录
      origin_ann_path = os.path.join(r'%s%s' %(origin_ann_dir, filename))   # 如果是，获取绝对路径（重复代码）
      new_ann_path = os.path.join(r'%s%s' %(new_ann_dir, filename))
      tree = ET.parse(origin_ann_path)  # ET是一个xml文件解析库，ET.parse（）打开xml文件。parse--"解析"
      root = tree.getroot()   # 获取根节点
      for object in root.findall('object'):   # 找到根节点下所有“object”节点
        name = str(object.find('name').text)  # 找到object节点下name子节点的值（字符串）
    # 如果name等于str，则删除该节点
# =============================================================================
#         if (name in ["car_head"]):
#           root.remove(object)
# =============================================================================
 
    # 如果name等于str，则修改name
        if(name in ["small ball","big ball"]):
          object.find('name').text = "ball"
 
    # 检查是否存在labelmap中没有的类别
      for object in root.findall('object'):
        name = str(object.find('name').text)
        if not (name in ["ball"]):
            print(filename + "------------->label is error--->" + name)
      tree.write(new_ann_path)#tree为文件，write写入新的文件中。