#pragma GCC optimize ("O3")
#include <Arduino.h>
#include <Control_Surface.h>
#include <dsps_fft2r.h>   
#include <dsps_wind.h>
#include <driver/i2s.h>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "dsps_mul.h"


// --- GitHub Configuration & Constants ---
#define SAMPLES 512             
#define HOP_SIZE 128             
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
volatile bool isFrozen = false;
float frozenMag[SAMPLES / 2] = {0};


// --- MIDI & Full Calibration Suite (Core 0) ---
pin_t pinPB = A0;
const int FREEZE_BUTTON_PIN = 0; // Set this to your desired GPIO (e.g., 0, 4, or 5)
const int INTERVAL_BUTTON_PIN = 4;
const int FEEDBACK_BUTTON_PIN = 5;

bool lastButtonState = HIGH;
bool lastIntervalButtonState = HIGH;
unsigned long intervalButtonPressedTime = 0;
float maxSemitones = 0.0f;

const int HARMONY_BUTTON_PIN = 18; 
bool lastHarmonyButtonState = HIGH;
volatile bool isHarmonizerMode = false; 

volatile float feedbackRamp = 0.0f;

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
  
  Serial.println();
}

inline float IRAM_ATTR fast_mag(float re, float im) {
    float abs_re = fabsf(re);
    float abs_im = fabsf(im);
    float max_val = (abs_re > abs_im) ? abs_re : abs_im;
    float min_val = (abs_re < abs_im) ? abs_re : abs_im;
    return max_val + 0.337f * min_val;
}


// --- CORE 1: PHASE-LOCKED DSP TASK ---
    void IRAM_ATTR AudioDSPTask(void * pvParameters) { 
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) { vTaskDelete(NULL); }
    
    dsps_wind_hann_f32(window_coefficients, SAMPLES);

    for(;;) {

        // --- . AUDIO INPUT (I2S READ) ---
        // Shift the old data down to make room for the new chunk
        memmove(fft_buffer, &fft_buffer[HOP_SIZE * 2], (SAMPLES - HOP_SIZE) * 2 * sizeof(float));

        // Read new audio from the hardware 
        int16_t in_block[HOP_SIZE];
        size_t bytes_read;
        i2s_read(I2S_NUM_0, in_block, sizeof(in_block), &bytes_read, portMAX_DELAY);

        // Convert hardware audio to float for the DSP math
        for (int i = 0; i < HOP_SIZE; i++) {
            fft_buffer[(SAMPLES - HOP_SIZE + i) * 2] = (float)in_block[i] / 32768.0f; 
            fft_buffer[(SAMPLES - HOP_SIZE + i) * 2 + 1] = 0.0f; 
        }

        // VECTORIZED WINDOWING
        dsps_mul_f32(fft_buffer, window_coefficients, fft_buffer, SAMPLES, 2, 1, 2);
        for (int i = 0; i < SAMPLES; i++) { fft_buffer[i * 2 + 1] = 0; }

        dsps_fft2r_fc32(fft_buffer, SAMPLES);
        dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

                float newMag[SAMPLES / 2] = {0}, newPhase[SAMPLES / 2] = {0};
                int lastPeak = 0; // Declared OUTSIDE the loop

        // FPU Math Loop for Phase-Locking (With Freeze Support)
        // FPU Math Loop for Phase-Locking (With Multi-Path Synthesis)
        for (int i = 1; i < (SAMPLES / 2) - 1; i++) {
            float re = fft_buffer[i * 2], im = fft_buffer[i * 2 + 1];
            
            float liveMag = fast_mag(re, im);
            
            // Capture snapshot only if neither effect is actively holding it
            if (!isFrozen && feedbackRamp == 0.0f) {
                frozenMag[i] = liveMag;
            }

            // Peak Detection (tracks live audio unless fully frozen)
            float pM = isFrozen ? frozenMag[i-1] : fast_mag(fft_buffer[(i-1)*2], fft_buffer[(i-1)*2+1]);
            float nM = isFrozen ? frozenMag[i+1] : fast_mag(fft_buffer[(i+1)*2], fft_buffer[(i+1)*2+1]);
            float currentMagForPeak = isFrozen ? frozenMag[i] : liveMag;
            if (currentMagForPeak > pM && currentMagForPeak > nM) lastPeak = i;

            // Phase Calculation
            float phase;
            float deltaPhase = 0.0f;
            float binFreq = 2.0f * PI * (float)lastPeak * (1.0f / (float)SAMPLES);

            if (!isFrozen) {
                // --- LIVE MODE: Phase Tracking ---
                float abs_re = fabsf(re);
                float abs_im = fabsf(im);
                float abs_re_plus = abs_re + 1e-7f; 
                if (abs_re >= abs_im) {
                    float r = abs_im / abs_re_plus;
                    phase = r * (0.9724f - 0.1919f * r * r); 
                } else {
                    float r = abs_re / (abs_im + 1e-7f);
                    phase = PI / 2.0f - r * (0.9724f - 0.1919f * r * r);
                }
                if (re < 0) phase = PI - phase;
                if (im < 0) phase = -phase;

                deltaPhase = (phase - prevAnalysisPhase[i]) - (binFreq * HOP_SIZE);
                prevAnalysisPhase[i] = phase; 
            } else {
                // --- FREEZE MODE ---
                phase = prevAnalysisPhase[i] + (binFreq * HOP_SIZE);
                prevAnalysisPhase[i] = phase;
            }
            
            // --- SYNTHESIS PATH 1: Main Audio (Dry or Shifted) ---
            // If Harmonizer is ON, lock the main audio to 1.0 (Dry). Otherwise, use the expression pedal.
            float mainFactor = isHarmonizerMode ? 1.0f : pitchShiftFactor;
            int targetBin = roundf(i * mainFactor);
            
            if (targetBin < SAMPLES / 2) {
                newMag[targetBin] += isFrozen ? frozenMag[i] : liveMag;
                float shiftedFreq = (binFreq + deltaPhase / HOP_SIZE) * mainFactor;
                newPhase[targetBin] = prevSynthesisPhase[targetBin] + (shiftedFreq * HOP_SIZE);
                prevSynthesisPhase[targetBin] = newPhase[targetBin];
            }

            // --- SYNTHESIS PATH 2: Auto-Feedbacker ---
            if (feedbackRamp > 0.0f) {
                float fbFactor = pitchShiftFactor * (1.0f + feedbackRamp); 
                int fbTargetBin = roundf(i * fbFactor);
                
                if (fbTargetBin < SAMPLES / 2) {
                    newMag[fbTargetBin] += (frozenMag[i] * feedbackRamp); 
                    float fbShiftedFreq = (binFreq * fbFactor);
                    newPhase[fbTargetBin] = prevSynthesisPhase[fbTargetBin] + (fbShiftedFreq * HOP_SIZE);
                    prevSynthesisPhase[fbTargetBin] = newPhase[fbTargetBin];
                }
            }

          // --- SYNTHESIS PATH 3: Polyphonic Harmonizer ---
            // Runs if Harmony mode is ON and the pedal is moved from center (UP or DOWN)
            if (isHarmonizerMode && fabsf(pitchShiftFactor - 1.0f) > 0.001f) {
                int harmTargetBin = roundf(i * pitchShiftFactor);
                
                if (harmTargetBin < SAMPLES / 2) {
                    // Mix the harmony voice in at 100% volume
                    newMag[harmTargetBin] += isFrozen ? frozenMag[i] : liveMag; 
                    
                    float harmShiftedFreq = (binFreq + deltaPhase / HOP_SIZE) * pitchShiftFactor;
                    newPhase[harmTargetBin] = prevSynthesisPhase[harmTargetBin] + (harmShiftedFreq * HOP_SIZE);
                    prevSynthesisPhase[harmTargetBin] = newPhase[harmTargetBin];
                }
            }
        }

        for (int i = 0; i < SAMPLES / 2; i++) {
            fft_buffer[i * 2] = newMag[i] * cosf(newPhase[i]);
            fft_buffer[i * 2 + 1] = newMag[i] * sinf(newPhase[i]);
        }
        dsps_fft2r_fc32(fft_buffer, SAMPLES);
        dsps_bit_rev2r_fc32(fft_buffer, SAMPLES);

        // --- OVERLAP-ADD (OLA) SYNTHESIS ---
        // Create a static buffer to hold the overlapping audio frames
        static float ola_buffer[SAMPLES] = {0};

        // 1. Add the newly processed frame into the OLA buffer
        for (int i = 0; i < SAMPLES; i++) {
            ola_buffer[i] += fft_buffer[i * 2]; // Extract Real part from complex IFFT output
        }

        // 2. Extract the finished HOP_SIZE block to send to the DAC
        int16_t out_block[HOP_SIZE];
        for (int i = 0; i < HOP_SIZE; i++) {
            // Scale to 16-bit audio and constrain to prevent digital clipping
            float scaled_sample = ola_buffer[i] * 32767.0f;
            if (scaled_sample > 32767.0f) scaled_sample = 32767.0f;
            if (scaled_sample < -32768.0f) scaled_sample = -32768.0f;
            out_block[i] = (int16_t)scaled_sample;
        }

        // 3. Shift the OLA buffer down by HOP_SIZE to prepare for the next frame
        memmove(ola_buffer, &ola_buffer[HOP_SIZE], (SAMPLES - HOP_SIZE) * sizeof(float));
        
        // 4. Clear the tail end of the OLA buffer
        memset(&ola_buffer[SAMPLES - HOP_SIZE], 0, HOP_SIZE * sizeof(float));

    

        // --- I2S HARDWARE OUTPUT ---
        // Write the finished audio block directly to the DAC
        size_t bw;
        i2s_write(I2S_NUM_0, out_block, sizeof(out_block), &bw, portMAX_DELAY);
    }
    
}

// --- CORE 0: MIDI & ADVANCED CONTROL ---
void MidiTask(void * pvParameters) {

    for(;;) {
        Control_Surface.loop();
        
        // --- FREEZE BUTTON LOGIC ---
        bool currentButtonState = digitalRead(FREEZE_BUTTON_PIN);
        
        if (currentButtonState == LOW && lastButtonState == HIGH) {
            isFrozen = !isFrozen;
            if (isFrozen) {
                Serial.println("FREEZE ON - Snapshot Captured!");
            } else {
                Serial.println("FREEZE OFF - Back to Live Audio");
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Simple debounce delay
        }
        lastButtonState = currentButtonState;


        // --- HARMONY MODE TOGGLE ---
        bool currentHarmonyState = digitalRead(HARMONY_BUTTON_PIN);
        if (currentHarmonyState == LOW && lastHarmonyButtonState == HIGH) {
            isHarmonizerMode = !isHarmonizerMode;
            if (isHarmonizerMode) {
                Serial.println("HARMONY MODE ON: Dry Signal + Sweeping Harmony");
            } else {
                Serial.println("WHAMMY MODE ON: Pitch Bend Only");
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        lastHarmonyButtonState = currentHarmonyState;


        // --- 2. INTERVAL BUTTON LOGIC (Short vs Long Press) ---
        bool currentIntervalButtonState = digitalRead(INTERVAL_BUTTON_PIN);
        
        if (currentIntervalButtonState == LOW && lastIntervalButtonState == HIGH) {
            // Button was just pressed, start the timer
            intervalButtonPressedTime = millis();
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        } 
        else if (currentIntervalButtonState == HIGH && lastIntervalButtonState == LOW) {
            // Button was just released, check how long it was held
            unsigned long pressDuration = millis() - intervalButtonPressedTime;
            
            if (pressDuration < 1000) {
                // Short Press: Increment by 0.5 semitones
                maxSemitones += 0.5f;
                if (maxSemitones > 24.0f) maxSemitones = 24.0f; // Max +/- 24 semitones
                Serial.print("Interval Up: +/- "); Serial.println(maxSemitones);
            } else {
                // Long Press (>= 1 second): Decrement by 0.5 semitones
                maxSemitones -= 0.5f;
                if (maxSemitones < 0.0f) maxSemitones = 0.0f; // Min 0 (No shift)
                Serial.print("Interval Down: +/- "); Serial.println(maxSemitones);
            }
            vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
        }
        
        lastIntervalButtonState = currentIntervalButtonState;

        // --- 3. AUTO-FEEDBACKER ENVELOPE ---
        // Momentary action: only active while your foot is holding it down
        bool currentFbState = digitalRead(FEEDBACK_BUTTON_PIN);
        
        if (currentFbState == LOW) { 
            feedbackRamp += 0.01f; // Swell attack (approx 500ms fade-in)
            if (feedbackRamp > 1.0f) feedbackRamp = 1.0f;
        } else { 
            feedbackRamp -= 0.02f; // Fade out release (approx 250ms decay)
            if (feedbackRamp < 0.0f) feedbackRamp = 0.0f;
        }
    


        // --- 4. WHAMMY POTENTIOMETER LOGIC ---
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


        // --- 5. DYNAMIC PITCH SHIFT MATH ---
        // Normalized throw goes from -1.0 (all the way down) to +1.0 (all the way up)
        float normalizedThrow = (float(calibratedMidi) - 8192.0f) / 8192.0f;
        
        // Convert the maxSemitones into octaves (e.g., 24 semitones = 2.0 octaves)
        float targetOctaves = maxSemitones / 12.0f;
        
        // Calculate the final pitch factor
        pitchShiftFactor = powf(2.0f, normalizedThrow * targetOctaves);
        
        debugPrint();
        vTaskDelay(pdMS_TO_TICKS(5)); // Yield to RTOS scheduler
    }
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    btmidi.setName("Whammy_S3");
    pinMode(FREEZE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(INTERVAL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FEEDBACK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(HARMONY_BUTTON_PIN, INPUT_PULLUP);
    FilteredAnalog<>::setupADC();
    potPB.map(map_PB_Full);
    Control_Surface >> pipes >> btmidi;
    Control_Surface >> pipes >> usbmidi;
    Control_Surface.begin();
    
    calibrateCenterAndDeadzone();


    i2s_config_t i2c = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
        .sample_rate = SAMPLING_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 4,
        .dma_buf_len = 64
    };
    i2s_driver_install(I2S_NUM_0, &i2c, 0, NULL);
    i2s_pin_config_t pin_config = {
        .bck_io_num = 14,   // Replace with your BCLK pin
        .ws_io_num = 15,    // Replace with your LRCK/WS pin
        .data_out_num = 16, // Replace with your DAC Data pin
        .data_in_num = 17   // Replace with your ADC Data pin
    };
    i2s_set_pin(I2S_NUM_0, &pin_config);

    xTaskCreatePinnedToCore(MidiTask, "Midi", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(AudioDSPTask, "DSP", 16384, NULL, 2, NULL, 1);
}

void loop() {}