# Build stage
FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy source code
COPY . .

# Clone whisper.cpp if not exists
RUN if [ ! -d "third_party/whisper.cpp" ]; then \
    git clone https://github.com/ggerganov/whisper.cpp.git third_party/whisper.cpp; \
    fi

# Build
RUN mkdir -p build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# Download Whisper model (base - good balance of speed/accuracy)
RUN mkdir -p models && \
    wget -q https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.bin -O models/ggml-base.bin

# Runtime stage
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy binary and models
COPY --from=builder /app/build/audio-engine .
COPY --from=builder /app/models ./models

# Expose gRPC port
EXPOSE 50051

# Run
CMD ["./audio-engine", "--model", "models/ggml-base.bin", "--port", "50051"]
