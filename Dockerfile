FROM osrf/ros:foxy-desktop
SHELL ["/bin/bash", "-c"]
# Dockerfile info
LABEL maintainer="sesantam@pa.uc3m.es"
LABEL version="0.1"
LABEL description="Docker image to run ros2 publisher."

ARG PYTHON_VERSION=3.8
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update && apt-get install -y \
    python3-pip
COPY . /home/dev_ws/
WORKDIR /home/dev_ws/ros2

RUN pip install --no-cache-dir -r ../requirements.txt

RUN source /opt/ros/foxy/setup.bash && colcon build && source /home/dev_ws/ros2/install/setup.bash
RUN chmod  +x "../ros_entrypoint.sh"
CMD ["../ros_entrypoint.sh"]