FROM python:3.10-slim-bookworm
RUN apt-get update && apt-get install -y --no-install-recommends \
    libosmesa6 libglfw3 libgl1 libegl1 libx11-6 libxrandr2 libxi6 libxcursor1 libxinerama1 \
    fonts-dejavu-core ca-certificates && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY tools/fetch_assets.py tools/fetch_assets.py
COPY assets assets
RUN python tools/fetch_assets.py
COPY manipulation manipulation
COPY simulate_grasp.py .
COPY licenses licenses
COPY THIRD_PARTY.md .
ENV MUJOCO_GL=osmesa OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MPLCONFIGDIR=/tmp/matplotlib
ENTRYPOINT ["python", "simulate_grasp.py"]
CMD ["--render", "--language", "en"]
