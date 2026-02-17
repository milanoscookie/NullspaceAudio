#pragma once

#include "IIRFilter.h"

#include <cmath>
#include <stdexcept>

class LPButterworthCoeff {
public:
    using FilterCoeff = IIRFilter::FilterCoeff;

    LPButterworthCoeff() = default;
    ~LPButterworthCoeff() = default;

    LPButterworthCoeff(float cutoffFrequency, float samplingRate) :
    cutoffFrequency_(cutoffFrequency), samplingRate_(samplingRate) {
        calculateCoefficients();
    }

    void setCutoffFrequency(float cutoffFrequency) {
        cutoffFrequency_ = cutoffFrequency;
        calculateCoefficients();
    }

    float getCutoffFrequency() const { return cutoffFrequency_; }

    void setSamplingRate(float samplingRate) {
        samplingRate_ = samplingRate;
        calculateCoefficients();
    }

    float getSamplingRate() const { return samplingRate_; }

    void calculateCoefficients(){
                                                        


        constexpr float PI = 3.14159265358979323846f;

        const float w0 = 2.0 * PI * cutoffFrequency_ / samplingRate_;
        const float cosw0 = std::cos(w0);
        const float sinw0 = std::sin(w0);

        constexpr float q = 0.7071067811865476;
        const float alpha = sinw0 / (2.0f * q);

        float b0 = (1.0f - cosw0) * 0.5f;
        float b1 = 1.0f - cosw0;
        float b2 = (1.0f - cosw0) * 0.5f;
        const float a0 = 1.0f + alpha;
        float a1 = -2.0f * cosw0;
        float a2 = 1.0f - alpha;

        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;

        coeffs_ << b0, b1, b2, a1, a2;
    }

    const FilterCoeff &getCoefficients() const { return coeffs_; }

private:
    FilterCoeff coeffs_ = IIRFilter::identityCoeffs();
    float cutoffFrequency_ = 1.0f;
    float samplingRate_ = 1.0f;
};