FROM python:3.11-slim

WORKDIR /app

# Install dependencies (psycopg2, pydantic, etc.)
RUN apt-get update && apt-get install -y \
    build-essential \
    libpq-dev \
    --no-install-recommends && \
    rm -rf /var/lib/apt/lists/*

# Copy project
COPY . /app

# Install Python libs
RUN pip install --no-cache-dir -r requirements.txt

# HuggingFace requires port 7860 (NOT 8000)
EXPOSE 7860

# Run FastAPI with uvicorn (the ONLY supported production method on HF)
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
