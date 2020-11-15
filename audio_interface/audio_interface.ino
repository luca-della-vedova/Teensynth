/* Receive Incoming USB Host MIDI using functions.  As usbMIDI
   reads incoming messages, handler functions are run.
   See the InputRead example for the non-function alterative.

   This very long example demonstrates all possible handler
   functions.  Most applications need only some of these.
   This example is meant to allow easy copy-and-paste of the
   desired functions.

   Use the Arduino Serial Monitor to view the messages
   as Teensy receives them by USB MIDI

   You must select MIDI from the "Tools > USB Type" menu

   This example code is in the public domain.
*/

#include <USBHost_t36.h>
#include <Metro.h>
#include <Adafruit_VS1053.h>
#include "vs1053_plugins.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);

// Begin audio generated stuff
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
//AudioInputI2S            i2s1;           //xy=55.00000762939453,174.00000762939453
//AudioMixer4              mixer2;         //xy=225,229
//AudioMixer4              mixer1;         //xy=236,140
//AudioOutputUSB           usb1;           //xy=445,142
//AudioConnection          patchCord1(i2s1, 0, mixer1, 0);
//AudioConnection          patchCord2(i2s1, 1, mixer2, 0);
//AudioConnection          patchCord3(mixer2, 0, usb1, 1);
//AudioConnection          patchCord4(mixer1, 0, usb1, 0);
// GUItool: end automatically generated code

// Defines for VS1053B board
#define VS1053_RESET  9      // VS1053 reset pin (output)
#define VS1053_CS     10     // VS1053 chip select pin (output)
#define VS1053_DCS    8      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define VS1053_CARDCS 24     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define VS1053_DREQ 25       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053 musicPlayer =
  // create breakout-example object!
  Adafruit_VS1053(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ);

#define VS1053_MIDI Serial3
#define VS1053_BANK_MELODY 0x79
#define VS1053_GM1_OCARINA 80

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_PROGRAM | chan);  
  VS1053_MIDI.write(inst);
}

void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write(MIDI_CHAN_VOLUME);
  VS1053_MIDI.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write((uint8_t)MIDI_CHAN_BANK);
  VS1053_MIDI.write(bank);
}

void VS1053_init()
{
  delay(100);
  if (! musicPlayer.begin()) { // initialise the music player
      Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
  }
  Serial.println("VS1053 found");

  // Magic stuff to enable I2S based on datasheet
  musicPlayer.sciWrite(0x7, 0xC017);
  musicPlayer.sciWrite(0x6, 0x00F0);

  // Enable I2S, start with WRAMADDR
  musicPlayer.sciWrite(0x7, 0xC040);
  // Then WRAM
  musicPlayer.sciWrite(0x6, 0x000E);
  musicPlayer.applyPatch(patches_plugin, PATCHES_PLUGIN_SIZE);// Add here
  // Start the patches
  while(digitalRead(VS1053_DREQ) == 0);
  musicPlayer.applyPatch(midistart_plugin, MIDISTART_PLUGIN_SIZE);// Add here
//  // Start midi plugin
//  musicPlayer.sciWrite(0xA, 0x50);
  delay(50);
  
  musicPlayer.applyPatch(admix_plugin, ADMIX_PLUGIN_SIZE);// Add here

  // Start the ADMIX plugin
  // Lower gain
  musicPlayer.sciWrite(0xC, 0xFFFD);
  musicPlayer.sciWrite(0xA, 0x0F00);
  while(digitalRead(VS1053_DREQ) == 0)
  {
    Serial.println("Waiting for ADMIX start");
  }
  delay(100);
  
  VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate'
  midiSetChannelBank(0, VS1053_BANK_MELODY);
  midiSetInstrument(0, VS1053_GM1_OCARINA);
  midiSetChannelVolume(0, 127);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_ON | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(VS1053_DREQ, INPUT);
  // Initialize VS1052 chip
  VS1053_init();

  // Wait 1.5 seconds before turning on USB Host.  If connected USB devices
  // use too much power, Teensy at least completes USB enumeration, which
  // makes isolating the power issue easier.
  delay(1000);
  Serial.println("USB Host InputFunctions example");
  delay(10);
  myusb.begin();

  midi1.setHandleNoteOn(myNoteOn);
  midi1.setHandleNoteOff(myNoteOff);
  midi1.setHandleControlChange(myControlChange);
  midi1.setHandleProgramChange(myProgramChange);
}

void test_midi_send()
{
  static constexpr int NOTE_DT = 500;
  static uint32_t last_t = 0;
  static bool note_status = 0;
  uint32_t current_t = millis();
  if (current_t - last_t > NOTE_DT)
  {
    if (note_status == 0)
    {
      // Turn on
      midiNoteOn(0, 60, 127);
//      midi1.sendNoteOn(60, 99, 0); // TODO check channel
    }
    else
    {
      // Turn off
      midiNoteOff(0, 60, 127);
//      midi1.sendNoteOff(60, 99, 0);
    }
    note_status = !note_status;
    last_t = current_t;
  }
//  midi1.sendPitchBend((current_t - last_t) * 10, 0);
}

// 20 Hz sampling
Metro pitch_bend_sampler = Metro(50);

void samplePitchBend()
{
  // Basic calibration, get the first reading as a reference
  static int init_bend_val = analogRead(0);
  static int last_bend_val = init_bend_val;
  int bend_val = analogRead(0); // Pin 14 on the board
  // Rescale from 0 to 16384
  // Only send if there has been a change
  if (bend_val != last_bend_val)
  {
    bend_val <<= 4;
    // And subtract the mid
    bend_val -= 8192;
    midi1.sendPitchBend(bend_val, 1);
    last_bend_val = bend_val;
  }
  test_midi_send();
}

void loop() {
  // The handler functions are called when midi1 reads data.  They
  // will not be called automatically.  You must call midi1.read()
  // regularly from loop() for midi1 to actually read incoming
  // data and run the handler functions as messages arrive.
  myusb.Task();
  midi1.read();
  // Check if it's time to send the pitch bend value
  if (pitch_bend_sampler.check())
    samplePitchBend();
}


void myNoteOn(byte channel, byte note, byte velocity) {
  // When a USB device with multiple virtual cables is used,
  // midi1.getCable() can be used to read which of the virtual
  // MIDI cables received this message.
  usbMIDI.sendNoteOn(channel, note, velocity);
}

void myNoteOff(byte channel, byte note, byte velocity) {
  usbMIDI.sendNoteOff(channel, note, velocity);
}

void myControlChange(byte channel, byte control, byte value) {
  usbMIDI.sendControlChange(channel, control, value);
}

void myProgramChange(byte channel, byte program) {
  usbMIDI.sendProgramChange(channel, program);
}
