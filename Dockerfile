FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY pipeline/ pipeline/
COPY missions/ missions/
ENTRYPOINT ["python", "-m", "pipeline.main"]
