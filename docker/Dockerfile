FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3 python3-pip git wget curl \
    libgl1-mesa-glx libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Webots
RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | apt-key add -
RUN echo "deb https://cyberbotics.com/repository master main" > /etc/apt/sources.list.d/cyberbotics.list
RUN apt-get update && apt-get install -y webots

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace
COPY . /workspace/

ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONPATH=/workspace:$PYTHONPATH

CMD ["python3", "analysis/metrics_dashboard.py"]
