#include "simulator_paths.h"

#include "utils/IIRFilter.h"
#include "utils/LPButterworthCoeff.h"
#include "utils/PeakingBiquadCoeff.h"

#include <algorithm>

namespace dsp::sim {
namespace {

IRBlock buildPathImpulseResponse(const PathConfig &config) {
  IRBlock impulseResponse = IRBlock::Zero();

  IIRFilter lowpassFilter;
  if (config.lowpassHz > 0.0f) {
    LPButterworthCoeff lowpassCoeff(config.lowpassHz,
                                    static_cast<float>(SAMPLE_RATE));
    lowpassFilter.setCoefficients(lowpassCoeff.getCoefficients());
  }

  IIRFilter resonanceFilter;
  if (config.resonance.enabled) {
    PeakingBiquadCoeff resonanceCoeff(config.resonance.frequencyHz,
                                      config.resonance.q,
                                      config.resonance.gainDb,
                                      static_cast<float>(SAMPLE_RATE));
    resonanceFilter.setCoefficients(resonanceCoeff.getCoefficients());
  }

  const int delaySamples = std::clamp(config.delaySamples, 0, IR_SIZE - 1);

  for (int sampleIndex = 0; sampleIndex < IR_SIZE; ++sampleIndex) {
    float sample = sampleIndex == delaySamples ? 1.0f : 0.0f;
    sample = lowpassFilter.filterSample(sample);
    if (config.resonance.enabled) {
      sample = resonanceFilter.filterSample(sample);
    }
    impulseResponse(sampleIndex) = config.gain * sample;
  }

  return impulseResponse;
}

IRBlock buildIdentitySpeakerPath() {
  IRBlock speaker = IRBlock::Zero();
  speaker(0) = 1.0f;
  return speaker;
}

} // namespace

PathSet buildPathSet(const ModelConfig &config) {
  PathSet pathSet;
  pathSet.reference = buildPathImpulseResponse(config.reference);
  pathSet.primary = buildPathImpulseResponse(config.primary);
  pathSet.secondary = buildPathImpulseResponse(config.secondary);
  pathSet.feedback = config.enableReferenceFeedback
                         ? buildPathImpulseResponse(config.feedback)
                         : IRBlock::Zero();
  pathSet.speaker = buildIdentitySpeakerPath();
  return pathSet;
}

} // namespace dsp::sim
