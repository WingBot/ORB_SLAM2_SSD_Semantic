# ORB_SLAM2_SSD_Semantic 编译运行指南

## 📋 项目概述

ORB_SLAM2_SSD_Semantic 是一个基于ORB_SLAM2的语义SLAM系统，集成了：
- 目标检测（SSD）
- 动态环境处理
- 语义地图构建
- OctoMap八叉树地图
- 点云处理

## 🚀 快速开始（推荐方式）

### 方法一：使用交互式管理脚本（最简单）

```bash
# 1. 进入项目目录
cd /home/slam/Project_ws/ORB_SLAM2_SSD_Semantic

# 2. 启动交互式管理界面
./start.sh

# 3. 在菜单中选择：
#    - 选择 "4) 一键构建所有镜像" (首次使用)
#    - 等待构建完成（30-50分钟）
#    - 选择 "5) 启动开发环境"
#    - 在容器内运行构建命令
```

### 方法二：使用命令行脚本

```bash
# 1. 一键构建Docker环境
./docker/scripts/build-all.sh

# 2. 启动开发环境
./docker/scripts/run-dev.sh

# 3. 在容器内构建项目
./docker/scripts/build_project.sh

# 4. 运行测试
./docker/scripts/run_tests.sh
```

### 方法三：使用Docker Compose

```bash
# 开发环境
docker-compose -f docker/docker-compose.dev.yml up orb-slam-dev

# 生产环境
docker-compose up orb-slam-analysis
```

## 🏗️ 详细构建步骤

### 第一步：环境准备

#### 系统要求
- **操作系统**: Ubuntu 18.04+ (推荐20.04)
- **内存**: 至少8GB RAM
- **磁盘空间**: 至少10GB可用空间
- **Docker**: 已安装并运行

#### 检查环境
```bash
# 检查Docker状态
docker --version
docker info

# 检查系统资源
free -h
df -h

# 检查GPU支持（可选）
nvidia-smi
```

### 第二步：Docker镜像构建

项目采用分层构建策略，包含三个镜像：

#### 1. 系统基础镜像 (5-10分钟)
```bash
./docker/scripts/build-system.sh
```
- Ubuntu 20.04基础系统
- 编译工具链（gcc, cmake, git等）
- 基础开发库

#### 2. 依赖库镜像 (20-30分钟)
```bash
./docker/scripts/build-deps.sh
```
- Eigen3 (矩阵运算库)
- OpenCV (计算机视觉库)
- Pangolin (3D可视化库)
- PCL (点云处理库)
- OctoMap (八叉树地图库)
- NCNN (神经网络推理库)
- DBoW2 (词袋模型库)
- g2o (图优化库)

#### 3. 最终开发镜像 (3-5分钟)
```bash
./docker/scripts/build-final.sh
```
- 项目源码
- 构建脚本
- 开发工具

### 第三步：项目编译

#### 在Docker容器中编译
```bash
# 启动开发环境
./docker/scripts/run-dev.sh

# 在容器内执行
cd /home/slam/workspace
./docker/scripts/build_project.sh
```

#### 编译的子模块
1. **主项目 (src/)**: 核心SLAM功能
2. **完善版本 (perfect/)**: 改进的SLAM实现
3. **实时检测定位 (realtime_dect_loc/)**: 目标检测和定位

### 第四步：验证构建结果

```bash
# 检查生成的库文件
ls -la lib/
# 应该看到: libORB_SLAM2_pc.so

# 检查可执行文件
ls -la bin/
# 应该看到: ty_rgbd, 3d_object_dect

# 运行测试
./docker/scripts/run_tests.sh
```

## 🎮 运行程序

### 1. RGB-D SLAM运行

```bash
# 在容器内运行
./bin/ty_rgbd Vocabulary/ORBvoc.txt config/my_rgbd_ty_api_adj.yaml data/
```

### 2. 3D目标检测运行

```bash
# 在容器内运行
./bin/3d_object_dect
```

### 3. 使用TUM数据集测试

```bash
# 下载TUM数据集
wget https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_walking.tgz

# 解压数据集
tar -xzf rgbd_dataset_freiburg3_walking.tgz

# 运行RGB-D SLAM
./bin/ty_rgbd Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml rgbd_dataset_freiburg3_walking/ rgbd_dataset_freiburg3_walking/associate.txt
```

## 📊 项目结构

```
ORB_SLAM2_SSD_Semantic/
├── src/                    # 主要源码
├── perfect/               # 完善版本
├── realtime_dect_loc/     # 实时检测定位
├── include/               # 头文件
├── docker/                # Docker配置
│   ├── Dockerfile.*       # 各层Dockerfile
│   └── scripts/          # 构建脚本
├── build/                 # 构建目录
├── lib/                   # 生成的库文件
├── bin/                   # 可执行文件
├── Vocabulary/            # ORB词汇表
├── Examples/              # 示例配置
└── Thirdparty/            # 第三方库
```

## 🔧 配置说明

### 主要配置文件

1. **CMakeLists.txt**: 项目构建配置
2. **Examples/RGB-D/TUM3.yaml**: RGB-D相机参数
3. **config/my_rgbd_ty_api_adj.yaml**: 自定义相机配置

### 环境变量设置

```bash
export LD_LIBRARY_PATH=/home/slam/workspace/lib:/usr/local/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```

## 🧪 测试验证

### 1. 依赖库测试
```bash
# 在容器内运行
/usr/local/bin/verify-deps.sh
```

### 2. 编译产物测试
```bash
# 检查库文件
ldd lib/libORB_SLAM2_pc.so

# 检查可执行文件
ldd bin/ty_rgbd
```

### 3. 功能测试
```bash
# 运行完整测试套件
./docker/scripts/run_tests.sh
```

## 🐛 故障排除

### 常见问题及解决方案

#### 1. Docker构建失败
```bash
# 检查磁盘空间
df -h

# 清理Docker缓存
docker system prune -a

# 检查网络连接
ping -c 3 archive.ubuntu.com
```

#### 2. 编译错误
```bash
# 检查依赖库
ldconfig -p | grep -E "(opencv|eigen|pangolin|pcl)"

# 重新构建依赖镜像
./docker/scripts/build-deps.sh
```

#### 3. GUI显示问题
```bash
# 设置X11权限
xhost +local:docker

# 检查DISPLAY变量
echo $DISPLAY
```

#### 4. 权限问题
```bash
# 确保脚本可执行
chmod +x docker/scripts/*.sh

# 检查Docker权限
docker info
```

### 调试技巧

```bash
# 查看构建日志
docker logs <container_id>

# 进入容器调试
docker run -it --rm orb_slam2_ssd_semantic:latest /bin/bash

# 检查镜像层
docker history orb_slam2_ssd_semantic:latest
```

## 📈 性能优化

### 构建优化
- 使用多核编译: `make -j$(nproc)`
- 启用优化编译: `-DCMAKE_BUILD_TYPE=Release`
- 使用本地架构优化: `-march=native`

### 运行优化
- 设置线程数: `export OMP_NUM_THREADS=4`
- 使用GPU加速（如果支持）
- 调整内存映射

## 🔄 开发工作流

### 1. 日常开发
```bash
# 启动开发环境
./docker/scripts/run-dev.sh

# 修改代码后重新编译
./docker/scripts/build_project.sh

# 运行测试
./docker/scripts/run_tests.sh
```

### 2. 代码同步
- 容器内代码修改会自动同步到主机
- 主机代码修改会自动同步到容器

### 3. 调试模式
```bash
# 使用调试构建
docker run -it --rm \
  -v $(pwd):/home/slam/workspace \
  orb_slam2_ssd_semantic:latest \
  bash -c "cd /home/slam/workspace && cmake -DCMAKE_BUILD_TYPE=Debug . && make"
```

## 📚 参考资料

- [ORB_SLAM2原论文](https://arxiv.org/pdf/1610.06475.pdf)
- [Docker最佳实践](https://docs.docker.com/develop/dev-best-practices/)
- [OpenCV文档](https://docs.opencv.org/)
- [PCL文档](https://pointclouds.org/documentation/)

## 🤝 贡献指南

1. Fork本项目
2. 创建特性分支: `git checkout -b feature/your-feature`
3. 在Docker环境中开发和测试
4. 提交更改: `git commit -am 'Add some feature'`
5. 推送分支: `git push origin feature/your-feature`
6. 提交Pull Request

## 📄 许可证

本项目继承原ORB_SLAM2的GPLv3许可证。

---

**🎉 恭喜！您已成功编译和运行ORB_SLAM2_SSD_Semantic！**

如有问题，请查看故障排除部分或提交Issue。