//add a windowing function

#include <Arduino.h>
#include <Control_Surface.h>
#include <dsps_fft2r.h>   
#include <dsps_wind_hann_f32.h> // Header for windowing function
#include <driver/i2s.h>

// --- Configuration ---
#define SAMPLES 1024             
#define HOP_SIZE 256             
#define SAMPLING_FREQUENCY 44100 

// --- Global Buffers ---
float fft_buffer[SAMPLES * 2] __attribute__((aligned(16))); 
float window_coefficients[SAMPLES]; // Buffer to store pre-calculated Hann window
float prevAnalysisPhase[SAMPLES / 2];
float prevSynthesisPhase[SAMPLES / 2];
volatile float pitchShiftFactor = 1.0; 

// --- MIDI Objects ---
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<2> pipes;
pin_t pinPB = A0; 
Bank<16> bankChannel;
Bankable::PBPotentiometer potPB {{bankChannel, BankType::ChangeChannel}, pinPB, Channel_1};

// --- CORE 1: PHASE-LOCKED DSP WITH WINDOWING ---
void AudioDSPTask(void * pvParameters) {
    // 1. Initialize DSP Tables
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) { vTaskDelete(NULL); }

    // 2. Pre-calculate Hann Window Coefficients
    dsps_wind_hann_f32(window_coefficients, SAMPLES);

    for(;;) {
        // 3. Apply Windowing to the real part of the complex buffer
        for (int i = 0; i < SAMPLES; i++) {
            fft_buffer[i * 2] *= window_coefficients[i]; 
            fft_buffer[i * 2 + 1] = 0; // Clear imaginary part for new input
        }

        // 4. Analysis: Forward FFT
        dsps_fft2r_fc32(fft_buffer, SAMPLES);
        dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

        float newMag[SAMPLES / 2] = {0};
        float newPhase[SAMPLES / 2] = {0};

        // 5. Phase-Locked Vocoder Logic
        int lastPeak = 0;
        for (int i = 1; i < (SAMPLES / 2) - 1; i++) {
            float re = fft_buffer[i * 2];
            float im = fft_buffer[i * 2 + 1];
            float mag = sqrtf(re*re + im*im);
            float phase = atan2f(im, re);

            float prevM = sqrtf(powf(fft_buffer[(i-1)*2],2) + powf(fft_buffer[(i-1)*2+1],2));
            float nextM = sqrtf(powf(fft_buffer[(i+1)*2],2) + powf(fft_buffer[(i+1)*2+1],2));
            if (mag > prevM && mag > nextM) lastPeak = i;

            float binFreq = 2.0f * PI * (float)lastPeak / (float)SAMPLES;
            float deltaPhase = (phase - prevAnalysisPhase[i]) - (binFreq * HOP_SIZE);
            
            int targetBin = roundf(i * pitchShiftFactor);
            if (targetBin < SAMPLES / 2) {
                newMag[targetBin] += mag;
                float shiftedFreq = (binFreq + deltaPhase / HOP_SIZE) * pitchShiftFactor;
                newPhase[targetBin] = prevSynthesisPhase[targetBin] + (shiftedFreq * HOP_SIZE);
                prevAnalysisPhase[i] = phase;
                prevSynthesisPhase[targetBin] = newPhase[targetBin];
            }
        }

        // 6. Synthesis: Inverse FFT
        for (int i = 0; i < SAMPLES / 2; i++) {
            fft_buffer[i * 2] = newMag[i] * cosf(newPhase[i]);
            fft_buffer[i * 2 + 1] = newMag[i] * sinf(newPhase[i]);
        }
        dsps_fft2r_fc32(fft_buffer, SAMPLES);
        dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

        // 7. Output to I2S DAC
        size_t bytes_written;
        int16_t out = (int16_t)(fft_buffer[0] * 32767);
        i2s_write(I2S_NUM_0, &out, sizeof(out), &bytes_written, portMAX_DELAY);
    }
}

// --- CORE 0: MIDI CONTROL TASK ---
void MidiTask(void * pvParameters) {
    for(;;) {
        Control_Surface.loop(); //
        uint32_t raw = potPB.getRawValue(); 
        pitchShiftFactor = powf(2.0f, (float(raw) - 8192.0f) / 8192.0f); //
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);
    Control_Surface >> pipes >> btmidi;
    Control_Surface >> pipes >> usbmidi;
    Control_Surface.begin();

    // Configure I2S for T-Display S3 Pins
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLING_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 4,
        .dma_buf_len = 64
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    // Pin Tasks to Cores
    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, 2, NULL, 1);
}

void loop() {}