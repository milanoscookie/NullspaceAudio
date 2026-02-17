#include "audio_source.h"
#include "portaudio_stream.h"
#include "wav_file_source.h"

// PortAudio adapter to AudioSource interface
class PortAudioSource : public AudioSource {
public:
    struct Config {
        int inputDevice = -1;
        int outputDevice = -1;
        int inputChannels = 1;
        int outputChannels = 1;
    };

    explicit PortAudioSource(const Config& config) : config_(config) {}
    
    void open(AudioCallback callback) override {
        PortAudioStream::Config paConfig;
        paConfig.inputDevice = config_.inputDevice;
        paConfig.outputDevice = config_.outputDevice;
        paConfig.inputChannels = config_.inputChannels;
        paConfig.outputChannels = config_.outputChannels;
        paConfig.sampleRate = static_cast<double>(dsp::SAMPLE_RATE);
        
        stream_.open(paConfig, std::move(callback));
    }

    void start() override { stream_.start(); }
    void stop() override { stream_.stop(); }
    void close() override { stream_.close(); }
    bool isRunning() const override { return stream_.isRunning(); }
    int getSampleRate() const override { return dsp::SAMPLE_RATE; }

private:
    Config config_;
    PortAudioStream stream_;
};

std::unique_ptr<AudioSource> AudioSourceFactory::create(const Config& config) {
    switch (config.type) {
        case Type::PortAudio: {
            PortAudioSource::Config paConfig;
            paConfig.inputDevice = config.inputDevice;
            paConfig.outputDevice = config.outputDevice;
            paConfig.inputChannels = config.inputChannels;
            paConfig.outputChannels = config.outputChannels;
            return std::make_unique<PortAudioSource>(paConfig);
        }
        
        case Type::WavFile: {
            WavFileSource::Config wavConfig;
            wavConfig.inputPath = config.inputWavPath;
            wavConfig.outputPath = config.outputWavPath;
            wavConfig.loop = config.loop;
            return std::make_unique<WavFileSource>(wavConfig);
        }
    }
    
    return nullptr;
}
