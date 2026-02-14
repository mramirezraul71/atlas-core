# PUSH (ATLAS Adapter) - puerto 8791
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
ENV PYTHONPATH=/app
ENV SERVICE_PORT=8791

EXPOSE 8791

CMD ["python", "-m", "uvicorn", "atlas_adapter.atlas_http_api:app", "--host", "0.0.0.0", "--port", "8791"]
