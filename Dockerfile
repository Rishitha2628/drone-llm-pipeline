FROM jonasvautherin/px4-gazebo-headless:1.14.3
WORKDIR /app
RUN apt-get update || true; apt-get install -y python3 python3-pip || pip3 --version
COPY requirements.txt .
RUN pip3 install -r requirements.txt
COPY . .
ENTRYPOINT ["python3", "-m", "pipeline.main"]
