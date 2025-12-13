#include "whisper_service.h"
#include <chrono>
#include <iostream>

#ifdef WHISPER_ENABLED
#include "whisper.h"
#endif

namespace audio {

struct WhisperService::Impl {
#ifdef WHISPER_ENABLED
  whisper_context *ctx = nullptr;
  whisper_full_params params;
#endif
  bool model_loaded = false;
  std::string model_path;
};

WhisperService::WhisperService() : impl_(std::make_unique<Impl>()) {
#ifdef WHISPER_ENABLED
  impl_->params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  impl_->params.print_progress = false;
  impl_->params.print_special = false;
  impl_->params.print_realtime = false;
  impl_->params.print_timestamps = false;
  impl_->params.language = "th"; // Thai language
  impl_->params.translate = false;
#endif
}

WhisperService::~WhisperService() {
#ifdef WHISPER_ENABLED
  if (impl_->ctx) {
    whisper_free(impl_->ctx);
  }
#endif
}

bool WhisperService::loadModel(const std::string &model_path) {
#ifdef WHISPER_ENABLED
  std::cout << "[Whisper] Loading model: " << model_path << std::endl;

  impl_->ctx = whisper_init_from_file(model_path.c_str());
  if (!impl_->ctx) {
    std::cerr << "[Whisper] Failed to load model" << std::endl;
    return false;
  }

  impl_->model_path = model_path;
  impl_->model_loaded = true;
  std::cout << "[Whisper] Model loaded successfully" << std::endl;
  return true;
#else
  std::cerr << "[Whisper] Not compiled with Whisper support" << std::endl;
  return false;
#endif
}

TranscriptionResult WhisperService::transcribe(const AudioBuffer &audio) {
  TranscriptionResult result;
  result.confidence = 0.0f;
  result.duration_ms = 0;

#ifdef WHISPER_ENABLED
  if (!impl_->model_loaded || !impl_->ctx) {
    result.text = "[Error: Model not loaded]";
    return result;
  }

  auto start = std::chrono::high_resolution_clock::now();

  // Run inference
  if (whisper_full(impl_->ctx, impl_->params, audio.data(), audio.size()) !=
      0) {
    result.text = "[Error: Transcription failed]";
    return result;
  }

  auto end = std::chrono::high_resolution_clock::now();
  result.duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  // Get transcription
  const int n_segments = whisper_full_n_segments(impl_->ctx);
  for (int i = 0; i < n_segments; ++i) {
    const char *text = whisper_full_get_segment_text(impl_->ctx, i);
    if (text) {
      result.text += text;
    }
  }

  // Estimate confidence (placeholder - Whisper doesn't directly provide this)
  result.confidence = n_segments > 0 ? 0.85f : 0.0f;

#else
  result.text = "[Whisper not enabled - placeholder transcription]";
  result.confidence = 0.0f;
#endif

  return result;
}

bool WhisperService::isReady() const { return impl_->model_loaded; }

std::string WhisperService::getModelInfo() const {
#ifdef WHISPER_ENABLED
  if (impl_->ctx) {
    return "Whisper model: " + impl_->model_path;
  }
#endif
  return "No model loaded";
}

} // namespace audio
