#pragma once

#include "audio_types.h"
#include <memory>
#include <string>

namespace audio {

class AudioProcessor {
public:
    AudioProcessor();
    ~AudioProcessor();
    
    // Initialize with configuration
    bool initialize(const AudioConfig& config);
    
    // Process raw audio bytes to float samples
    bool processRawAudio(const uint8_t* data, size_t size, AudioBuffer& output);
    
    // Normalize audio levels
    void normalize(AudioBuffer& buffer, float target_level = 0.9f);
    
    // Apply noise reduction (simple high-pass filter)
    void reduceNoise(AudioBuffer& buffer, float cutoff_freq = 80.0f);
    
    // Resample audio to target sample rate
    bool resample(const AudioBuffer& input, AudioBuffer& output, 
                  int src_rate, int dst_rate);
    
    // Check if silence (for VAD)
    bool isSilence(const AudioBuffer& buffer, float threshold = 0.01f) const;
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    AudioConfig config_;
};

} // namespace audio
