# ATLAS NEXUS Robot - API en puerto 8002
FROM python:3.11-slim

WORKDIR /app

# Backend está en backend/
COPY backend/requirements.txt ./requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ .

EXPOSE 8002

CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8002"]
