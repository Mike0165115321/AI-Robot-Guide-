#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace audio {

struct AudioConfig {
    int sample_rate = 16000;
    int channels = 1;
    int bit_depth = 16;
};

struct TranscriptionResult {
    std::string text;
    float confidence;
    int64_t duration_ms;
};

// Audio buffer for processing
class AudioBuffer {
public:
    AudioBuffer() = default;
    explicit AudioBuffer(size_t size) : data_(size) {}
    
    void resize(size_t size) { data_.resize(size); }
    size_t size() const { return data_.size(); }
    bool empty() const { return data_.empty(); }
    
    float* data() { return data_.data(); }
    const float* data() const { return data_.data(); }
    
    void clear() { data_.clear(); }
    
private:
    std::vector<float> data_;
};

} // namespace audio
