FROM osrf/ros:humble-desktop-full

# Install VNC and display dependencies
RUN apt-get update && apt-get install -y \
    xvfb \
    x11vnc \
    fluxbox \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Set display environment variable
ENV DISPLAY=:55
ENV ROS_LOCALHOST_ONLY=1

# Add ROS sourcing to bashrc
RUN echo "source /opt/ros/humble/setup.sh" >> /root/.bashrc

# Create startup script
RUN echo '#!/bin/bash\n\
Xvfb :55 -screen 0 1920x1080x24 &\n\
sleep 2\n\
fluxbox &\n\
x11vnc -display :55 -forever -shared -nopw &\n\
echo "VNC server started on port 5900"\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]