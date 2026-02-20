#include "dsp_interface.h"
#include "wav_writer.h"
#include "anc.h"
#include <iostream>
#include <chrono>
#include <string>
#include <print>
int main(int argc, char *argv[]) {
    try {
        // Help message
        if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
            std::cout << "Usage: " << argv[0] << " [input.wav] [output_prefix]" << std::endl;
            std::cout << "  input.wav      : Input WAV file (default: input.wav)" << std::endl;
            std::cout << "  output_prefix  : Prefix for output files (default: output)" << std::endl;
            std::cout << "Output files: <prefix>_outside_mic.wav, <prefix>_inear_mic.wav" << std::endl;
            return 0;
        }

        // Parse command-line arguments
        std::string inputWavFile = "input.wav";  // Default input file
        std::string outputPrefix = "output";     // Default output prefix

        if (argc > 1) {
            inputWavFile = argv[1];
        }
        if (argc > 2) {
            outputPrefix = argv[2];
        }

        std::cout << "Using input WAV file: " << inputWavFile << std::endl;
        std::cout << "Output file prefix: " << outputPrefix << std::endl;

        // Initialize DSP parameters
        Params params;

        // Set input WAV file path
        params.audioConfig.inputWavPath = inputWavFile;
        // Set reasonable impulse response parameters
        // H: noise -> outside mic (realistic microphone coupling)
        {
            auto &H = params.paths.H;
            H(0) = 1.0f;      // Direct path
            H(1) = 0.5f;      // Early reflection
            H(2) = 0.25f;     // Second reflection
            H(3) = 0.15f;     // Decay
            // Exponential decay of remaining samples
            for (size_t i = 4; i < dsp::IR_SIZE; ++i) {
                H(i) = 0.15f * std::exp(-0.01f * (i - 3));
            }
        }
        
        // P: noise -> in-ear mic (closer to source, slightly different path)
        {
            auto &P = params.paths.P;
            P(0) = 0.9f;      // Slightly less direct than H
            P(1) = 0.4f;      // Weaker early reflections
            P(2) = 0.2f;
            P(3) = 0.1f;
            // Exponential decay
            for (size_t i = 4; i < dsp::IR_SIZE; ++i) {
                P(i) = 0.1f * std::exp(-0.012f * (i - 3));
            }
        }
        
        // C: speaker -> outside mic (feedback path, secondary path)
        {
            auto &C = params.paths.C;
            C(0) = 0.7f;      // Direct speaker coupling
            C(1) = 0.35f;
            C(2) = 0.15f;
            C(3) = 0.08f;
            // Exponential decay with longer tail
            for (size_t i = 4; i < dsp::IR_SIZE; ++i) {
                C(i) = 0.08f * std::exp(-0.008f * (i - 3));
            }
        }
        
        // Speaker: non-flat speaker response
        {
            auto &speaker = params.paths.speaker;
            speaker(0) = 0.95f;  // Slightly attenuated direct response
            speaker(1) = 0.1f;   // Some decay
            speaker(2) = 0.05f;
            // Quick decay after initial transient
            for (size_t i = 3; i < dsp::IR_SIZE; ++i) {
                speaker(i) = 0.05f * std::exp(-0.02f * (i - 2));
            }
        }
        
        // Set reasonable noise and dynamics parameters
        params.noise.outside_mic_stddev = 0.001f;  // Small ambient noise
        params.noise.inear_mic_stddev = 0.5f;   // Even less in-ear noise
        params.dynamics.noise_gain = 0.001f;
        
        // Create DSP interface with n block of system latency
        DSPInterface dspInterface(params, anc::systemLatencyBlocks);
        

        // Create WAV writers for outside and in-ear microphones
        std::string outsideFile = outputPrefix + "_outside_mic.wav";
        std::string inearFile = outputPrefix + "_inear_mic.wav";
        WavWriter wavWriterOutside(outsideFile);
        WavWriter wavWriterInear(inearFile);

        // Open WAV files
        if (!wavWriterOutside.open() || !wavWriterInear.open()) {
            std::cerr << "Failed to open WAV files for writing" << std::endl;
            return 1;
        }
        
        // Set up the microphone processing function
        dspInterface.setProcessMics([&](const MicBlock &micBlock, Block &control) {
            // Fill control with zeros (no active control signal)
            anc::step(micBlock, control);
            
            // Write both outside and in-ear microphone samples to respective WAV files
            wavWriterOutside.writeBlock(micBlock.outside);
            wavWriterInear.writeBlock(micBlock.inear);
        });
        
        // Process microphone data until the WAV file is complete
        int blockCount = 0;
        auto startTime = std::chrono::steady_clock::now();
        auto lastReportTime = startTime;
        
        std::cout << "Starting audio processing..." << std::endl;
        
        // Continue processing while audio source is running or we still have buffered data
        while (dspInterface.isAudioSourceRunning() || blockCount < 10) {
            // Get the next block of microphone data
            auto micBlock = dspInterface.getMics();
            
            if (micBlock) {
                blockCount++;
                
                // Report progress based on actual elapsed time
                auto now = std::chrono::steady_clock::now();
                auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReportTime).count();
                
                // Report every ~1000ms of actual elapsed time
                if (elapsedMs >= 1000) {
                    auto totalElapsedSecs = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
                    std::cout << "Processed " << totalElapsedSecs << " second(s) - " << blockCount << " blocks" << std::endl;
                    lastReportTime = now;
                }
            } else {
                // No data available yet, wait a bit and try again
                if (dspInterface.isAudioSourceRunning()) {
                    std::this_thread::sleep_for(std::chrono::microseconds(10));  // Very short sleep to avoid blocking
                } else {
                    // Audio source finished but we might have buffered data
                    break;
                }
            }
        }
        
        auto endTime = std::chrono::steady_clock::now();
        auto totalElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        std::cout << "Audio processing complete. Total blocks processed: " << blockCount 
                  << " in " << totalElapsedMs << "ms" << std::endl;
        
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
