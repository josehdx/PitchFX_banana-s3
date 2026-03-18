#include <Arduino.h>
#include <Control_Surface.h>
#include <dsps_fft2r.h>   
#include <dsps_wind.h>
#include <driver/i2s.h>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "dsps_mul.h"

// --- GitHub Configuration & Constants ---
#define SAMPLES 1024             
#define HOP_SIZE 256             
#define SAMPLING_FREQUENCY 44100 
#define SERIAL_BAUDRATE 115200
#define FORCE_CENTER_UPDATE_DELAY 250
#define STREAM_BUFFER_SIZE (SAMPLES * 4 * sizeof(int16_t)) 

// --- Global DSP Buffers (Core 1) ---
float fft_buffer[SAMPLES * 2] __attribute__((aligned(16))); 
float window_coefficients[SAMPLES];
float prevAnalysisPhase[SAMPLES / 2];
float prevSynthesisPhase[SAMPLES / 2];
volatile float pitchShiftFactor = 1.0; 

StreamBufferHandle_t i2s_out_buffer;

// --- MIDI & Full Calibration Suite (Core 0) ---
pin_t pinPB = A13; 
BluetoothMIDI_Interface btmidi;
USBMIDI_Interface usbmidi;
MIDI_PipeFactory<2> pipes;
Bank<16> bankChannel;
Bankable::PBPotentiometer potPB {{bankChannel, BankType::ChangeChannel}, pinPB, Channel_1};
FilteredAnalog<12, 14, uint32_t, uint32_t> filterPB = pinPB;

// Calibration Variables
double PBdeadzoneMultiplier = 2.50;
double PBdeadzoneMinimum = 387;
double PBdeadzoneMaximum = 539;
analog_t PBcenter = 8192;
analog_t PBdeadzone = PBdeadzoneMinimum;
analog_t PBminimumValue = 0;
analog_t PBmaximumValue = 16383;
bool PBwasOffCenter = false;
long PBlastCenteredOn = millis();
static bool lockedAtMin = false;
static bool lockedAtMax = false;

// --- Full Mapping Logic ---
analog_t map_PB_Full(analog_t raw) {
    raw = constrain(raw, PBminimumValue, PBmaximumValue);
    
    if (raw <= PBcenter - PBdeadzone) {
        PBwasOffCenter = true;
        return map(raw, PBminimumValue, PBcenter - PBdeadzone, 0, 8191);
    }
    else if (raw >= PBcenter + PBdeadzone) {
        PBwasOffCenter = true;
        return map(raw, PBcenter + PBdeadzone, PBmaximumValue, 8191, 16383);
    }
    else {
        return 8192;
    }
}

// --- GitHub Calibration Routine (All Serial Lines Restored) ---
void calibrateCenterAndDeadzone() {
  Serial.println("Calibrating Center and Deadzones...");
  Serial.println("Please Wait...Do Not Touch Stick!");
  
  int iNumberOfSamples = 750;
  analog_t calibPBLow = 4095; 
  analog_t calibPBHigh = 0; 

  Serial.print("Sampling center. Number of samples: "); Serial.println(iNumberOfSamples);
  
  long lSampleSumPB = 0;
  for (int iSample = 1; iSample <= iNumberOfSamples; iSample++) {
    analog_t calibPB = analogRead(pinPB); delay(1);
    lSampleSumPB += calibPB;
    if (calibPB < calibPBLow) { calibPBLow = calibPBLow; } 
    if (calibPB > calibPBHigh) { calibPBHigh = calibPB; } 
  }
  
  PBcenter = map((analog_t(lSampleSumPB / iNumberOfSamples)), 0, 4095, 0, 16383);
  Serial.print("PB Center: "); Serial.println(PBcenter);

  analog_t calibPBLowMidi = map(calibPBLow, 0, 4095, 0, 16383);
  analog_t calibPBHighMidi = map(calibPBHigh, 0, 4095, 0, 16383);
  Serial.print("PB Low MIDI: "); Serial.println(calibPBLowMidi);
  Serial.print("PB High MIDI: "); Serial.println(calibPBHighMidi);
 
  PBdeadzone = (analog_t)constrain(((calibPBHighMidi - calibPBLowMidi) * PBdeadzoneMultiplier), PBdeadzoneMinimum, PBdeadzoneMaximum);
  Serial.print("PB Deadzone (Constrained/Value Used): "); Serial.println(PBdeadzone);
}

// --- GitHub debugPrint() - Fully Restored ---
void debugPrint() {
    //Serial.print("Factor: "); Serial.print(pitchShiftFactor); Serial.print("\t");
  /* Serial.print("AR: ");
  Serial.print(analogRead(pinPB)); Serial.print("\t");
  Serial.print("PB Min/Cen/Max/Range/DZ: ");
  Serial.print(PBminimumValue); Serial.print(" ");
  Serial.print(PBcenter); Serial.print(" ");
  Serial.print(PBmaximumValue); Serial.print(" ");
  Serial.print(PBmaximumValue-PBminimumValue); Serial.print(" ");
  Serial.print(PBdeadzone); Serial.print(" ");
  Serial.print("PB OffCenter/Raw(14bit)/Val(12bit): ");
  Serial.print(PBwasOffCenter); Serial.print("\t");
  Serial.print(potPB.getRawValue());  Serial.print("\t");
  Serial.print(potPB.getValue()); Serial.print("\t");
  */
  
  // Custom addition to monitor the new Ring Buffer health
  // Serial.print("Free Buffer: "); Serial.print(xStreamBufferSpacesAvailable(i2s_out_buffer));
  
  Serial.println();
}

// --- CORE 1: PHASE-LOCKED DSP TASK ---
void AudioDSPTask(void * pvParameters) {
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) { vTaskDelete(NULL); }
    
    dsps_wind_hann_f32(window_coefficients, SAMPLES);

    for(;;) {
        // VECTORIZED WINDOWING
        dsps_mul_f32(fft_buffer, window_coefficients, fft_buffer, SAMPLES, 2, 1, 2);
        for (int i = 0; i < SAMPLES; i++) { fft_buffer[i * 2 + 1] = 0; }

        dsps_fft2r_fc32(fft_buffer, SAMPLES);
        dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

        float newMag[SAMPLES / 2] = {0}, newPhase[SAMPLES / 2] = {0};
        int lastPeak = 0;

        // FPU Math Loop for Phase-Locking
        for (int i = 1; i < (SAMPLES / 2) - 1; i++) {
            float re = fft_buffer[i * 2], im = fft_buffer[i * 2 + 1];
            float mag = sqrtf(re*re + im*im), phase = atan2f(im, re);

            float pM = sqrtf(powf(fft_buffer[(i-1)*2],2) + powf(fft_buffer[(i-1)*2+1],2));
            float nM = sqrtf(powf(fft_buffer[(i+1)*2],2) + powf(fft_buffer[(i+1)*2+1],2));
            if (mag > pM && mag > nM) lastPeak = i;

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

        for (int i = 0; i < SAMPLES / 2; i++) {
            fft_buffer[i * 2] = newMag[i] * cosf(newPhase[i]);
            fft_buffer[i * 2 + 1] = newMag[i] * sinf(newPhase[i]);
        }
        dsps_fft2r_fc32(fft_buffer, SAMPLES);
        dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

        // Circular Buffer Write
        int16_t outSample = (int16_t)(fft_buffer[0] * 32767);
        xStreamBufferSend(i2s_out_buffer, &outSample, sizeof(int16_t), 0);

        // I2S Hardware Output
        size_t bw;
        int16_t playSample;
        if (xStreamBufferReceive(i2s_out_buffer, &playSample, sizeof(int16_t), 0) > 0) {
            i2s_write(I2S_NUM_0, &playSample, sizeof(int16_t), &bw, portMAX_DELAY);
        }
    }
}

// --- CORE 0: MIDI & ADVANCED CONTROL ---
void MidiTask(void * pvParameters) {
    for(;;) {
        Control_Surface.loop();
        
        uint32_t pb12Val = potPB.getValue(); 
        analog_t calibratedMidi = map_PB_Full(potPB.getRawValue()); 

        if (pb12Val == 0) {
            if (!lockedAtMin) {
                Control_Surface.sendPitchBend(Channel_1, (uint16_t)0);
                lockedAtMin = true;
                Serial.println("[LIMIT: MIN]");
            }
        } else { lockedAtMin = false; }

        if (calibratedMidi == 8192 && PBwasOffCenter) {
            if ((millis() - PBlastCenteredOn) > FORCE_CENTER_UPDATE_DELAY) {
                Control_Surface.sendPitchBend(Channel_1, 8192);
                PBwasOffCenter = false;
                PBlastCenteredOn = millis();
                Serial.println("[FORCE MIDI CENTER]");
            }
        }

        if (pb12Val == 4095) { 
            if (!lockedAtMax) {
                Control_Surface.sendPitchBend(Channel_1, (uint16_t)16383);
                lockedAtMax = true;
                Serial.println("[LIMIT: MAX]");
            }
        } else { lockedAtMax = false; }

        pitchShiftFactor = powf(2.0f, (float(calibratedMidi) - 8192.0f) / 8192.0f);
        
        debugPrint();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    btmidi.setName("Whammy_S3");
    
    FilteredAnalog<>::setupADC();
    potPB.map(map_PB_Full);
    Control_Surface >> pipes >> btmidi;
    Control_Surface >> pipes >> usbmidi;
    Control_Surface.begin();
    
    calibrateCenterAndDeadzone();

    i2s_out_buffer = xStreamBufferCreate(STREAM_BUFFER_SIZE, sizeof(int16_t));

    i2s_config_t i2c = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLING_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 4,
        .dma_buf_len = 64
    };
    i2s_driver_install(I2S_NUM_0, &i2c, 0, NULL);

    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, 2, NULL, 1);
}

void loop() {}