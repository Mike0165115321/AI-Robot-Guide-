#include "audio_processor.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace audio {

struct AudioProcessor::Impl {
  bool initialized = false;
};

AudioProcessor::AudioProcessor() : impl_(std::make_unique<Impl>()) {}

AudioProcessor::~AudioProcessor() = default;

bool AudioProcessor::initialize(const AudioConfig &config) {
  config_ = config;
  impl_->initialized = true;
  return true;
}

bool AudioProcessor::processRawAudio(const uint8_t *data, size_t size,
                                     AudioBuffer &output) {
  if (!impl_->initialized)
    return false;

  // Assume 16-bit PCM audio
  size_t num_samples = size / 2;
  output.resize(num_samples);

  const int16_t *samples = reinterpret_cast<const int16_t *>(data);
  float *out_data = output.data();

  // Convert to float [-1.0, 1.0]
  for (size_t i = 0; i < num_samples; ++i) {
    out_data[i] = static_cast<float>(samples[i]) / 32768.0f;
  }

  return true;
}

void AudioProcessor::normalize(AudioBuffer &buffer, float target_level) {
  if (buffer.empty())
    return;

  float *data = buffer.data();
  size_t size = buffer.size();

  // Find max absolute value
  float max_val = 0.0f;
  for (size_t i = 0; i < size; ++i) {
    max_val = std::max(max_val, std::abs(data[i]));
  }

  if (max_val < 0.001f)
    return; // Silence

  // Scale to target level
  float scale = target_level / max_val;
  for (size_t i = 0; i < size; ++i) {
    data[i] *= scale;
  }
}

void AudioProcessor::reduceNoise(AudioBuffer &buffer, float cutoff_freq) {
  if (buffer.empty())
    return;

  // Simple first-order high-pass filter
  float *data = buffer.data();
  size_t size = buffer.size();

  float rc = 1.0f / (2.0f * 3.14159f * cutoff_freq);
  float dt = 1.0f / static_cast<float>(config_.sample_rate);
  float alpha = rc / (rc + dt);

  float prev_input = data[0];
  float prev_output = data[0];

  for (size_t i = 1; i < size; ++i) {
    float output = alpha * (prev_output + data[i] - prev_input);
    prev_input = data[i];
    prev_output = output;
    data[i] = output;
  }
}

bool AudioProcessor::resample(const AudioBuffer &input, AudioBuffer &output,
                              int src_rate, int dst_rate) {
  if (input.empty() || src_rate <= 0 || dst_rate <= 0)
    return false;

  // Simple linear interpolation resampling
  double ratio = static_cast<double>(dst_rate) / static_cast<double>(src_rate);
  size_t out_size = static_cast<size_t>(input.size() * ratio);
  output.resize(out_size);

  const float *in_data = input.data();
  float *out_data = output.data();

  for (size_t i = 0; i < out_size; ++i) {
    double src_idx = static_cast<double>(i) / ratio;
    size_t idx0 = static_cast<size_t>(src_idx);
    size_t idx1 = std::min(idx0 + 1, input.size() - 1);
    double frac = src_idx - static_cast<double>(idx0);

    out_data[i] =
        static_cast<float>(in_data[idx0] * (1.0 - frac) + in_data[idx1] * frac);
  }

  return true;
}

bool AudioProcessor::isSilence(const AudioBuffer &buffer,
                               float threshold) const {
  if (buffer.empty())
    return true;

  const float *data = buffer.data();
  size_t size = buffer.size();

  // Calculate RMS
  double sum = 0.0;
  for (size_t i = 0; i < size; ++i) {
    sum += data[i] * data[i];
  }
  double rms = std::sqrt(sum / static_cast<double>(size));

  return rms < threshold;
}

} // namespace audio
