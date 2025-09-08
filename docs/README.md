# ORB-SLAM2 SSD语义SLAM系统文档

本文档目录提供了ORB-SLAM2 SSD语义增强SLAM系统的全面技术文档，包括系统架构、模块详解、实现原理等。

## 文档结构

- [architecture/](architecture/) - 系统架构文档
  - [system-overview.md](architecture/system-overview.md) - 系统总体架构
  - [class-diagrams.md](architecture/class-diagrams.md) - 类图设计
  - [sequence-diagrams.md](architecture/sequence-diagrams.md) - 序列图
  - [interaction-diagrams.md](architecture/interaction-diagrams.md) - 交互图

- [modules/](modules/) - 各模块详细文档
  - [core-slam/](modules/core-slam/) - ORB-SLAM2核心模块
  - [semantic-detection/](modules/semantic-detection/) - 语义检测模块  
  - [pointcloud-mapping/](modules/pointcloud-mapping/) - 点云建图模块
  - [visualization/](modules/visualization/) - 可视化模块

- [implementation/](implementation/) - 实现原理文档
  - [semantic-slam-principles.md](implementation/semantic-slam-principles.md) - 语义SLAM实现原理
  - [dynamic-environment-handling.md](implementation/dynamic-environment-handling.md) - 动态环境处理
  - [data-association.md](implementation/data-association.md) - 数据关联算法

- [apis/](apis/) - API接口文档
  - [system-api.md](apis/system-api.md) - 系统主要接口
  - [mapping-api.md](apis/mapping-api.md) - 建图接口
  - [detection-api.md](apis/detection-api.md) - 检测接口

## 快速开始

1. 阅读[系统总体架构](architecture/system-overview.md)了解整体设计
2. 查看[类图设计](architecture/class-diagrams.md)理解组件关系
3. 参考[序列图](architecture/sequence-diagrams.md)了解工作流程
4. 深入[模块文档](modules/)学习具体实现

## 文档更新

本文档与代码同步更新，确保准确反映当前系统实现。