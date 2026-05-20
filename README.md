# 在docker22.04下使用mid70测量距离

## 安装

```bash
git clone https://github.com/xu0829/mid70_later.git
docker pull xuputishu/mid70_lidar:latest
```

## 运行容器

**1. 开放图形界面显示权限给 Docker**

```bash
xhost +local: 
xhost +localhost
```

**2. 启动容器**

```bash
docker run -it \
--name mid70_lidar3 \
--privileged \
--network host \
--ipc=host \
--shm-size=11g \
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /dev:/dev \
-v $(pwd):/workspace \
--device /dev/dri \
xuputishu/mid70_lidar:latest \
/bin/bash
```

**3. 进入工作区并编译代码**

```bash
docker exec -it mid70_lidar3 /bin/bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```