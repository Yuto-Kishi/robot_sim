FROM python:3.11-slim

# --- 必要パッケージのインストール ---
RUN apt update && apt install -y \
    libgl1-mesa-glx libglfw3 libglew-dev libosmesa6-dev \
    patchelf ffmpeg xvfb x11vnc fluxbox wget curl unzip \
    && rm -rf /var/lib/apt/lists/*

# --- Pythonパッケージインストール ---
COPY requirements.txt .
RUN pip install --upgrade pip && pip install -r requirements.txt

# --- 作業ディレクトリとコードコピー ---
WORKDIR /workspace
COPY . /workspace

# --- Xvfb + VNC起動用エントリポイント ---
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
