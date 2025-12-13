#pragma once

#include "audio_types.h"
#include <memory>
#include <string>

namespace audio {

class WhisperService {
public:
  WhisperService();
  ~WhisperService();

  // Load Whisper model
  bool loadModel(const std::string &model_path);

  // Transcribe audio buffer
  TranscriptionResult transcribe(const AudioBuffer &audio);

  // Check if model is loaded
  bool isReady() const;

  // Get model info
  std::string getModelInfo() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace audio
