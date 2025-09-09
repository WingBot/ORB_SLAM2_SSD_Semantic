# ORB_SLAM2_SSD_Semantic Docker环境

本项目现已提供完整的Docker容器化开发环境，支持一键构建和部署。

## 🚀 快速开始

### 方法一：使用交互式管理脚本（推荐）

```bash
# 启动交互式管理界面
./start.sh
```

这将打开一个菜单式界面，您可以：
- 一键构建所有Docker镜像
- 启动开发环境
- 运行项目构建和测试
- 管理Docker资源

### 方法二：使用命令行脚本

```bash
# 1. 一键构建所有镜像（首次使用）
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

## 🏗️ Docker架构设计

本项目采用**分层构建策略**，优化构建效率和错误排查：

### 镜像层次结构

```
orb_slam2_ssd_semantic:system  (系统基础镜像)
    ↓
orb_slam2_ssd_semantic:deps    (依赖库镜像)
    ↓
orb_slam2_ssd_semantic:latest  (最终开发镜像)
```

### 各层说明

1. **系统镜像** (`Dockerfile.system`)
   - Ubuntu 20.04基础系统
   - 编译工具链（gcc, cmake, git等）
   - 基础开发库
   - 构建时间：5-10分钟

2. **依赖镜像** (`Dockerfile.deps`)
   - 所有第三方库编译安装
   - 包含：Eigen3, OpenCV, Pangolin, PCL, OctoMap, NCNN, DBoW2, g2o
   - 构建时间：20-30分钟（最耗时）

3. **最终镜像** (`Dockerfile.final`)
   - 项目源码
   - 构建脚本
   - 开发工具
   - 构建时间：3-5分钟

## 📦 主要组件

### 核心依赖库
- **Eigen3**: 线性代数运算库
- **OpenCV**: 计算机视觉库
- **Pangolin**: 3D可视化库
- **PCL**: 点云处理库
- **OctoMap**: 八叉树地图库
- **NCNN**: 神经网络推理库
- **DBoW2**: 词袋模型库
- **g2o**: 图优化库

### 构建脚本
- `build-system.sh`: 构建系统镜像
- `build-deps.sh`: 构建依赖镜像
- `build-final.sh`: 构建最终镜像
- `build-all.sh`: 一键构建所有镜像
- `run-dev.sh`: 启动开发环境
- `docker_manager.sh`: Docker管理工具

## 🛠️ 开发工作流

### 1. 环境准备
```bash
# 检查系统要求
./start.sh  # 选择选项12进行环境诊断

# 一键构建（首次使用）
./start.sh  # 选择选项4
```

### 2. 开发过程
```bash
# 启动开发环境
./start.sh  # 选择选项5

# 在容器内：
cd /home/slam/workspace
./docker/scripts/build_project.sh  # 构建项目
./docker/scripts/run_tests.sh      # 运行测试
```

### 3. 项目构建
```bash
# 自动构建所有子模块
./docker/scripts/build_project.sh

# 检查构建结果
ls -la lib/    # 生成的库文件
ls -la bin/    # 可执行文件
```

## 🧪 测试验证

项目包含完整的测试套件：

```bash
# 在容器内运行
./docker/scripts/run_tests.sh
```

测试内容包括：
- 依赖库安装验证
- 编译产物检查
- 库文件链接测试
- 基础功能验证

## 🔧 故障排除

### 常见问题

1. **构建失败**
   ```bash
   # 检查磁盘空间（需要至少10GB）
   df -h
   
   # 检查内存（建议8GB以上）
   free -h
   
   # 清理Docker缓存
   docker system prune -a
   ```

2. **GUI显示问题**
   ```bash
   # 设置X11权限
   xhost +local:docker
   
   # 检查DISPLAY变量
   echo $DISPLAY
   ```

3. **权限问题**
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

## 📊 性能优化

### 构建优化
- 使用多阶段构建减少最终镜像大小
- 分层缓存机制避免重复编译
- 并行编译利用多核处理器

### 运行优化
- 支持GPU加速（NVIDIA Docker）
- 内存映射优化数据访问
- 容器资源限制避免系统过载

## 🔄 CI/CD集成

项目支持持续集成：

```yaml
# GitHub Actions示例
- name: Build Docker Images
  run: |
    ./docker/scripts/build-all.sh --no-test
    
- name: Run Tests
  run: |
    docker run --rm orb_slam2_ssd_semantic:latest ./docker/scripts/run_tests.sh
```

## 📈 监控和日志

### 容器监控
```bash
# 查看资源使用
docker stats

# 查看容器状态
docker ps -a

# 查看镜像信息
docker images | grep orb_slam2
```

### 日志管理
```bash
# 查看构建日志
docker logs -f <container_name>

# 导出日志
docker logs <container_name> > build.log 2>&1
```

## 🤝 贡献指南

1. Fork本项目
2. 创建特性分支：`git checkout -b feature/your-feature`
3. 在Docker环境中开发和测试
4. 提交更改：`git commit -am 'Add some feature'`
5. 推送分支：`git push origin feature/your-feature`
6. 提交Pull Request

## 📄 许可证

本项目继承原ORB_SLAM2的GPLv3许可证。

## 🙏 致谢

- 感谢ORB_SLAM2原作者团队
- 感谢所有开源依赖库的贡献者
- 参考Tennis Analysis项目的Docker最佳实践
