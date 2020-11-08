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

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);

// Defines for VS1053B board
#define VS1053_RESET  9      // VS1053 reset pin (output)
#define VS1053_CS     10     // VS1053 chip select pin (output)
#define VS1053_DCS    8      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define VS1053_CARDCS 24     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define VS1053_DREQ 25       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer =
  // create breakout-example object!
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, VS1053_CARDCS);

void VS1053_init()
{
  if (! musicPlayer.begin()) { // initialise the music player
      Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
  }
  Serial.println(F("VS1053 found"));

  SD.begin(VS1053_CARDCS);    // initialise the SD card

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(20, 20);

  // If DREQ is on an interrupt pin (on uno, #2 or #3) we can do background
  // audio playing
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
//  musicPlayer.startPlayingFile("track001.mp3");

  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
}

void setup() {
  Serial.begin(115200);

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
      midi1.sendNoteOn(60, 99, 0); // TODO check channel
    }
    else
    {
      // Turn off
      midi1.sendNoteOff(60, 99, 0);
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
