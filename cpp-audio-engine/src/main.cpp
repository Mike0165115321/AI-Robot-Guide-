#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "audio_processor.h"

#ifdef WHISPER_ENABLED
#include "whisper_service.h"
#endif

std::atomic<bool> g_running{true};

void signalHandler(int signal) {
  std::cout << "\n⏳ Received signal " << signal << ", shutting down..."
            << std::endl;
  g_running = false;
}

void printBanner() {
  std::cout << R"(
╔═══════════════════════════════════════════════════════╗
║         C++ Audio Engine for AI Robot Guide           ║
║                    Version 1.0.0                      ║
╚═══════════════════════════════════════════════════════╝
)" << std::endl;
}

int main(int argc, char *argv[]) {
  printBanner();

  // Setup signal handlers
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // Parse arguments
  std::string model_path = "models/ggml-base.bin";
  int port = 50051;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--model" && i + 1 < argc) {
      model_path = argv[++i];
    } else if (arg == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  --model <path>  Path to Whisper model (default: "
                   "models/ggml-base.bin)\n"
                << "  --port <port>   gRPC server port (default: 50051)\n"
                << "  --help, -h      Show this help message\n";
      return 0;
    }
  }

  // Initialize audio processor
  audio::AudioProcessor processor;
  audio::AudioConfig config;
  config.sample_rate = 16000;
  config.channels = 1;
  config.bit_depth = 16;

  if (!processor.initialize(config)) {
    std::cerr << "❌ Failed to initialize audio processor" << std::endl;
    return 1;
  }
  std::cout << "✅ Audio processor initialized (16kHz, mono, 16-bit)"
            << std::endl;

#ifdef WHISPER_ENABLED
  // Initialize Whisper service
  audio::WhisperService whisper;
  if (whisper.loadModel(model_path)) {
    std::cout << "✅ Whisper model loaded: " << model_path << std::endl;
  } else {
    std::cout << "⚠️  Whisper model not loaded (STT will be unavailable)"
              << std::endl;
  }
#else
  std::cout << "⚠️  Compiled without Whisper support" << std::endl;
#endif

  std::cout << "\n🚀 C++ Audio Engine ready on port " << port << std::endl;
  std::cout << "📡 Waiting for gRPC connections..." << std::endl;

  // TODO: Start gRPC server here
  // For now, just wait
  while (g_running) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "✅ Audio engine shutdown complete" << std::endl;
  return 0;
}
