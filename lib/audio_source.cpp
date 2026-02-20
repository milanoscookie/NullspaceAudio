#include "audio_source.h"
#include "wav_file_source.h"

std::unique_ptr<AudioSource> AudioSourceFactory::create(const Config& config) {
    switch (config.type) {
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
