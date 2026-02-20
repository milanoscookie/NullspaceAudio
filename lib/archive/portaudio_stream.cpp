#include "portaudio_stream.h"
#include <cstring>
#include <sstream>

PortAudioStream::PortAudioStream() {
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        throw std::runtime_error(std::string("PortAudio init failed: ") + Pa_GetErrorText(err));
    }
    initialized_ = true;
}

PortAudioStream::~PortAudioStream() {
    close();
    if (initialized_) {
        Pa_Terminate();
    }
}

void PortAudioStream::open(const Config& config, AudioCallback callback) {
    if (stream_) {
        close();
    }

    callback_ = std::move(callback);

    PaStreamParameters inputParams{};
    PaStreamParameters outputParams{};
    PaStreamParameters* pInputParams = nullptr;
    PaStreamParameters* pOutputParams = nullptr;

    // Configure input
    if (config.inputChannels > 0) {
        inputParams.device = (config.inputDevice < 0) 
            ? Pa_GetDefaultInputDevice() 
            : config.inputDevice;
        
        if (inputParams.device == paNoDevice) {
            throw std::runtime_error("No default input device available");
        }

        inputParams.channelCount = config.inputChannels;
        inputParams.sampleFormat = paFloat32;
        inputParams.suggestedLatency = 
            Pa_GetDeviceInfo(inputParams.device)->defaultLowInputLatency;
        inputParams.hostApiSpecificStreamInfo = nullptr;
        pInputParams = &inputParams;
    }

    // Configure output
    if (config.outputChannels > 0) {
        outputParams.device = (config.outputDevice < 0) 
            ? Pa_GetDefaultOutputDevice() 
            : config.outputDevice;
        
        if (outputParams.device == paNoDevice) {
            throw std::runtime_error("No default output device available");
        }

        outputParams.channelCount = config.outputChannels;
        outputParams.sampleFormat = paFloat32;
        outputParams.suggestedLatency = 
            Pa_GetDeviceInfo(outputParams.device)->defaultLowOutputLatency;
        outputParams.hostApiSpecificStreamInfo = nullptr;
        pOutputParams = &outputParams;
    }

    PaError err = Pa_OpenStream(
        &stream_,
        pInputParams,
        pOutputParams,
        config.sampleRate,
        dsp::BLOCK_SIZE,  // frames per buffer = latency block size
        paClipOff,   // don't clip
        paCallback,
        this
    );

    if (err != paNoError) {
        throw std::runtime_error(std::string("Failed to open stream: ") + Pa_GetErrorText(err));
    }
}

void PortAudioStream::start() {
    if (!stream_) {
        throw std::runtime_error("Stream not opened");
    }

    PaError err = Pa_StartStream(stream_);
    if (err != paNoError) {
        throw std::runtime_error(std::string("Failed to start stream: ") + Pa_GetErrorText(err));
    }
    running_.store(true);
}

void PortAudioStream::stop() {
    if (stream_ && running_.load()) {
        Pa_StopStream(stream_);
        running_.store(false);
    }
}

void PortAudioStream::close() {
    stop();
    if (stream_) {
        Pa_CloseStream(stream_);
        stream_ = nullptr;
    }
}

int PortAudioStream::getInputLatencySamples() const {
    if (!stream_) return 0;
    const PaStreamInfo* info = Pa_GetStreamInfo(stream_);
    return static_cast<int>(info->inputLatency * dsp::SAMPLE_RATE);
}

int PortAudioStream::getOutputLatencySamples() const {
    if (!stream_) return 0;
    const PaStreamInfo* info = Pa_GetStreamInfo(stream_);
    return static_cast<int>(info->outputLatency * dsp::SAMPLE_RATE);
}

std::string PortAudioStream::listDevices() {
    std::ostringstream oss;
    
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) {
        return "Error getting device count";
    }

    oss << "Available audio devices:\n";
    oss << "========================\n";

    for (int i = 0; i < numDevices; ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        oss << "[" << i << "] " << info->name << "\n";
        oss << "    In: " << info->maxInputChannels 
            << ", Out: " << info->maxOutputChannels
            << ", Sample Rate: " << info->defaultSampleRate << "\n";
    }

    int defaultIn = Pa_GetDefaultInputDevice();
    int defaultOut = Pa_GetDefaultOutputDevice();
    oss << "\nDefault Input: " << defaultIn << "\n";
    oss << "Default Output: " << defaultOut << "\n";

    return oss.str();
}

int PortAudioStream::paCallback(
    const void* inputBuffer, 
    void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* /*timeInfo*/,
    PaStreamCallbackFlags /*statusFlags*/,
    void* userData
) {
    auto* self = static_cast<PortAudioStream*>(userData);
    
    const float* in = static_cast<const float*>(inputBuffer);
    float* out = static_cast<float*>(outputBuffer);

    // Copy input to Eigen block (handles nullptr for output-only streams)
    if (in) {
        for (size_t i = 0; i < framesPerBuffer && i < dsp::BLOCK_SIZE; ++i) {
            self->inputBlock_(i) = in[i];
        }
    } else {
        self->inputBlock_.setZero();
    }

    // Initialize output block
    self->outputBlock_.setZero();

    // Call user callback
    if (self->callback_) {
        self->callback_(self->inputBlock_, self->outputBlock_);
    }

    // Copy Eigen block to output (handles nullptr for input-only streams)
    if (out) {
        for (size_t i = 0; i < framesPerBuffer && i < dsp::BLOCK_SIZE; ++i) {
            out[i] = self->outputBlock_(i);
        }
    }

    return paContinue;
}

