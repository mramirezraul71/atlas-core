# ATLAS NEXUS - API en puerto 8000
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
RUN mkdir -p logs config

ENV PYTHONPATH=/app

EXPOSE 8000

CMD ["python", "nexus.py", "--mode", "api", "--env", "production"]
