#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

// I2S Configuration
#define I2S_NUM          I2S_NUM_0
#define I2S_WS           25  // Word Select (L/R clock)
#define I2S_SCK          26  // Serial Clock (Bit clock)
#define I2S_SD           32  // Serial Data (Data input from INMP441)
#define I2S_BUFFER_SIZE  1024

// FFT Configuration
#define SAMPLE_RATE      44100 // Sampling frequency
#define SAMPLES          512   // Must be a power of 2
#define FFT_THRESHOLD    50    // Amplitude threshold for noise filtering

// Global variables for FFT
float vReal[SAMPLES]; // Real part of FFT (changed to float)
float vImag[SAMPLES]; // Imaginary part of FFT (changed to float)

// FFT object
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_RATE, false);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // I2S configuration for the microphone (INMP441)
  i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),   // Master, Receive mode
      .sample_rate = SAMPLE_RATE,                           // Sampling rate
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,         // 16-bit samples
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // Mono (only left channel)
      .communication_format = I2S_COMM_FORMAT_I2S,         // I2S protocol
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // Interrupt flag
      .dma_buf_count = 4,                                   // Number of DMA buffers
      .dma_buf_len = I2S_BUFFER_SIZE,                       // Length of each buffer
      .use_apll = false,                                    // No APLL
      .tx_desc_auto_clear = false,                          // Disable automatic tx descriptor clearing
      .fixed_mclk = 0                                       // No fixed MCLK
  };

  // I2S pin configuration
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_SCK,      // Bit clock
      .ws_io_num = I2S_WS,        // Word Select (L/R clock)
      .data_out_num = I2S_PIN_NO_CHANGE, // No data output
      .data_in_num = I2S_SD       // Data input from INMP441
  };

  // Install and start the I2S driver
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_set_clk(I2S_NUM, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO); // Mono audio

  Serial.println("FFT Inverted Absolute Amplitude Initialized");
}

void loop() {
  int16_t i2s_buffer[SAMPLES];
  size_t bytes_read;

  // Read audio data from the INMP441 microphone via I2S
  i2s_read(I2S_NUM, (void *)i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);

  // Fill FFT input arrays (convert to float)
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (float)i2s_buffer[i]; // Convert to float
    vImag[i] = 0.0f;                 // Imaginary part is 0
  }

  // Perform FFT (Fast Fourier Transform)
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Process FFT results
  for (int i = 0; i < SAMPLES / 2; i++) {
    float magnitude = vReal[i];      // Current magnitude value
    float invertedNoise = 0.0;       // Placeholder for inverted noise

    // Check if the magnitude is below the noise threshold
    if (magnitude < FFT_THRESHOLD) {
      invertedNoise = -magnitude;    // Invert the noise
    } else {
      invertedNoise = 0.0;           // Ignore non-noise frequencies
    }

    // Output both the original signal and inverted noise
    Serial.print("Original:");       // Label for original signal
    Serial.print(magnitude);
    Serial.print(" InvertedNoise:"); // Label for inverted noise
    Serial.println(invertedNoise);
  }

  delay(100); // Delay to control the frequency of output (adjust as needed)
}