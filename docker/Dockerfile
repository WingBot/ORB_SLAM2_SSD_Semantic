# ORB_SLAM2_SSD_Semantic 开发镜像
# 基于基础依赖镜像构建项目

FROM orb_slam2_ssd_semantic:base

USER root

# 安装额外的运行时依赖
RUN apt-get update && apt-get install -y \
    libqt5opengl5-dev \
    qt5-default \
    libfreeimage3 \
    libfreeimage-dev \
    && rm -rf /var/lib/apt/lists/*

# 切换到用户目录
USER slam
WORKDIR /home/slam

# 创建项目目录
RUN mkdir -p /home/slam/workspace

# 设置工作目录
WORKDIR /home/slam/workspace

# 复制项目源码（通过docker构建时复制）
COPY --chown=slam:slam . .

# 复制构建脚本并设置权限
COPY --chown=slam:slam docker/scripts/ ./scripts/
RUN chmod +x ./scripts/*.sh

# 创建Thirdparty目录结构
RUN mkdir -p Thirdparty/DBoW2/lib Thirdparty/g2o/lib

# 创建软链接到系统安装的库
RUN ln -s /usr/local/lib/libDBoW2.so Thirdparty/DBoW2/lib/libDBoW2.so || \
    ln -s /usr/local/lib/libDBoW2.a Thirdparty/DBoW2/lib/libDBoW2.so || \
    echo "DBoW2 library not found in expected location"

RUN ln -s /usr/local/lib/libg2o_*.so Thirdparty/g2o/lib/ 2>/dev/null || \
    echo "g2o libraries not found in expected location"

# 设置环境变量
ENV LD_LIBRARY_PATH=/usr/local/lib:/home/slam/workspace/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

# 暴露可能需要的端口（用于可视化等）
EXPOSE 8080 9090

# 默认启动命令
CMD ["/bin/bash"]
