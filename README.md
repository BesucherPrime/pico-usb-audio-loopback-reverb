# USB Audio Loopback FX Reverb for RP2350 with parallel I2S 32b out

A reverb effect for RP2350 that runs on a USB audio loopback device.

## Overview

This project turns a Raspberry Pi Pico 2 (RP2350) into a USB audio device that inserts a reverb effect into the loopback audio path between a host PC, an external Speaker, and the Pico.

It handles 32-bit, 48kHz, 2-channel PCM audio data and applies a Freeverb-style reverb using Q15 fixed-point arithmetic. The effect runs with low latency and fits within the limited resources of the RP2350.

_What is USB audio loopback?_  
In this context, the Pico appears to the host PC as a virtual sound card. It receives audio that is playing on the PC—such as system sounds, music, or game audio—processes it, and sends it back to the PC as if it were microphone input. This technique is commonly used to capture or process system audio without additional software.

The audio flow works like this:

1. **PC audio output:**  
   Music, video, or game audio played on the PC is sent as digital data to the Pico device.

2. **Audio processing on Pico:**  
   The Pico applies the configured effect (reverb) in real-time. This adds spatial depth and ambience to the original sound.

3. **Audio input back to PC:**  
   The processed audio is returned to the PC as an input device, such as a virtual microphone.

## Requirements

- Raspberry Pi Pico or any RP2350-based board  
- Host PC with USB connection (tested on Windows 11)

## Visual Code Build Instructions

```bash
git clone https://github.com/oyama/pico-usb-audio-loopback-reverb.git
cd pico-usb-audio-loopback-reverb
git submodule update --init
```
'Import' Project in Visual Code from the Pico extension as a SDK 2.1.1 project

Ignore and don't fix these warnings:
warning: "CFG_TUSB_DEBUG" redefined
[build]    11 | #define CFG_TUSB_DEBUG 

Once built, write the generated `pico-usb-audio-loopback-reverb.uf2` file to the Pico.  
YOU WILL HAVE TO DO THIS MANUALLY.
PRESSING RUN WILL NOT FLASH YOUR PICO. DRAG AND DROP.
It will then appear as a USB audio device to your computer.

## Usage

### Modes of Operation

- **Passthrough:**  
  By default, the Pico simply forwards incoming audio to the output without modification.

- **Effect Enabled:**  
  When the BOOTSEL button is held down, the reverb effect is applied.

### Windows 11 Instructions

1. Connect the Pico to your PC via USB.  
2. Open the Settings app and go to “System” > “Sound”.  
3. Under “Output”, select _Speaker: Pico USB Audio Loopback Reverb_.  
   Under “Input”, select _Microphone: Pico USB Audio Loopback Reverb_.  
4. Click on “More sound settings” > “Recording” tab.  
   Open the properties for _Pico USB Audio Loopback Reverb_, go to the “Listen” tab, and check “Listen to this device”.  
   Set the playback device (e.g., your PC speakers), then click “Apply”.  
5. Start playback from your preferred source (YouTube, Spotify, etc.) and press the Pico’s BOOTSEL button to enable the reverb effect.

### macOS (GarageBand) Instructions

1. Connect the Pico to your Mac via USB.  
2. Launch GarageBand. In the menu, go to “GarageBand” > “Settings” > “Audio/MIDI”.  
3. Set the input device to _Pico USB Audio Loopback Reverb_ and the output device to your _speakers_.  
4. Create an audio track with input set to _Input 1+2_, and enable the “Monitor” option.  
5. In macOS System Settings, set the sound output device to _Pico USB Audio Loopback Reverb_.  
6. Start playback from your preferred source (YouTube, Spotify, etc.) and press the Pico’s BOOTSEL button to enable the reverb effect.

## License

This project is licensed under the 3-Clause BSD License. For details, see the [LICENSE](LICENSE.md) file.
