#define PREFER_SDFAT_LIBRARY
#define FILES_PER_PAGE 10
#include <array>

#include <SdFat.h>

#include <USBHost_t36.h>
#include <Metro.h>
#include <Adafruit_VS1053.h>
#include "hw_defs.h"
#include "vs1053_plugins.h"
#include "lcd_driver.h"

#include <MD_MIDIHelper.h>
#include <MD_MIDIFile.h>

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
MIDIDevice midi1(myusb);

Adafruit_VS1053 vs_codec =
  Adafruit_VS1053(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ);


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

SdFat  SD;
MD_MIDIFile SMF;

// The files in the tune list should be located on the SD card 
// or an error will occur opening the file and the next in the 
// list will be opened (skips errors).
const char *tuneList[] = 
{
  "sandstorm.mid",  // simplest and shortest file
  "pirates.mid",
};
const uint16_t WAIT_DELAY = 2000; // ms

void midiCallback(midi_event *pev)
// Called by the MIDIFile library when a file event needs to be processed
// thru the midi communications interface.
// This callback is set up in the setup() function.
{
  if ((pev->data[0] >= 0x80) && (pev->data[0] <= 0xe0))
  {
    VS1053_MIDI.write(pev->data[0] | pev->channel);
    VS1053_MIDI.write(&pev->data[1], pev->size-1);
  }
  else
    VS1053_MIDI.write(pev->data, pev->size);
}

void midiSilence(void)
// Turn everything off on every channel.
// Some midi files are badly behaved and leave notes hanging, so between songs turn
// off all the notes and sound
{
  midi_event ev;

  // All sound off
  // When All Sound Off is received all oscillators will turn off, and their volume
  // envelopes are set to zero as soon as possible.
  ev.size = 0;
  ev.data[ev.size++] = 0xb0;
  ev.data[ev.size++] = 120;
  ev.data[ev.size++] = 0;

  for (ev.channel = 0; ev.channel < 16; ev.channel++)
    midiCallback(&ev);
}

void midifile_setup(void)
{
  // Set up LED pins
  pinMode(READY_LED, OUTPUT);
  pinMode(SD_ERROR_LED, OUTPUT);
  pinMode(SMF_ERROR_LED, OUTPUT);
  pinMode(BEAT_LED, OUTPUT);

  // reset LEDs
  digitalWrite(READY_LED, LOW);
  digitalWrite(SD_ERROR_LED, LOW);
  digitalWrite(SMF_ERROR_LED, LOW);
  digitalWrite(BEAT_LED, LOW);

  SdioConfig sd_config;
  // Initialize SD
  if (!SD.begin(sd_config))
  {
    digitalWrite(SD_ERROR_LED, HIGH);
    while (true)
      Serial.println("SD init failed");
  }

  // Initialize MIDIFile
  SMF.begin(&SD);
  SMF.setMidiHandler(midiCallback);
  
  digitalWrite(READY_LED, HIGH);
}

void tickMetronome(void)
// flash a LED to the beat
{
  static uint32_t lastBeatTime = 0;
  static boolean  inBeat = false;
  uint16_t  beatTime;

  beatTime = 60000/SMF.getTempo();    // msec/beat = ((60sec/min)*(1000 ms/sec))/(beats/min)
  if (!inBeat)
  {
    if ((millis() - lastBeatTime) >= beatTime)
    {
      lastBeatTime = millis();
      digitalWrite(BEAT_LED, HIGH);
      inBeat = true;
    }
  }
  else
  {
    if ((millis() - lastBeatTime) >= 100) // keep the flash on for 100ms only
    {
      digitalWrite(BEAT_LED, LOW);
      inBeat = false;
    }
  }
}

void midifile_loop(void)
{
  static enum { S_IDLE, S_PLAYING, S_END, S_WAIT_BETWEEN } state = S_IDLE;
  static uint16_t currTune = ARRAY_SIZE(tuneList);
  static uint32_t timeStart;

  switch (state)
  {
  case S_IDLE:    // now idle, set up the next tune
    {
      int err;

      digitalWrite(READY_LED, LOW);
      digitalWrite(SMF_ERROR_LED, LOW);

      currTune++;
      if (currTune >= ARRAY_SIZE(tuneList))
        currTune = 0;

      // use the next file name and play it
      err = SMF.load(tuneList[currTune]);
      if (err != MD_MIDIFile::E_OK)
      {
        digitalWrite(SMF_ERROR_LED, HIGH);
        timeStart = millis();
        state = S_WAIT_BETWEEN;
      }
      else
      {
        state = S_PLAYING;
      }
    }
    break;

  case S_PLAYING: // play the file
    if (!SMF.isEOF())
    {
      if (SMF.getNextEvent())
        tickMetronome();
    }
    else
      state = S_END;
    break;

  case S_END:   // done with this one
    SMF.close();
    midiSilence();
    timeStart = millis();
    state = S_WAIT_BETWEEN;
    break;

  case S_WAIT_BETWEEN:    // signal finished with a dignified pause
    digitalWrite(READY_LED, HIGH);
    if (millis() - timeStart >= WAIT_DELAY)
      state = S_IDLE;
    break;

  default:
    state = S_IDLE;
    break;
  }
}

void VS1053_init()
{
  delay(100);
  if (! vs_codec.begin()) { // initialise the music player
      Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
  }
  Serial.println("VS1053 found");
  // Magic stuff to enable I2S based on datasheet
  vs_codec.sciWrite(0x7, 0xC017);
  vs_codec.sciWrite(0x6, 0x00F0);

  // Enable I2S, start with WRAMADDR
  vs_codec.sciWrite(0x7, 0xC040);
  // Then WRAM
  vs_codec.sciWrite(0x6, 0x000E); // PUT E FOR 192KHZ
  vs_codec.applyPatch(patches_plugin, PATCHES_PLUGIN_SIZE);// Add here
  // Start the patches
  while(digitalRead(VS1053_DREQ) == 0);
  vs_codec.applyPatch(midistart_plugin, MIDISTART_PLUGIN_SIZE);// Add here
//  // Start midi plugin
//  vs_codec.sciWrite(0xA, 0x50);
  delay(50);
  
  vs_codec.applyPatch(admix_plugin, ADMIX_PLUGIN_SIZE);// Add here

  // Start the ADMIX plugin
  // Lower gain
  vs_codec.sciWrite(0xC, 0xFFFD);
  vs_codec.sciWrite(0xA, 0x0F00);
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

AudioLCD lcd;

std::array<String, FILES_PER_PAGE> probe_sd_files()
{
  std::array<String, FILES_PER_PAGE> ret_arr;
  int file_count = 0;
  char name_buf[50];
  File file;
  File dir;
  // Open root directory
  dir.open("/");
  while (file.openNext(&dir, O_READ) && file_count < FILES_PER_PAGE)
  {
    file.getName(name_buf, sizeof(name_buf));
    size_t name_size = strlen(name_buf);
    if (name_size == sizeof(name_buf))
      Serial.println("WARNING: buffer overflow in name size");
    if (file.isDir())
    {
      Serial.println("WARNING: Found a folder, skipping");
    }
    else
    {
      // Check extension
      if (strstr(strlwr(name_buf + (name_size - 4)), ".mid"))
      {
        Serial.print("Found midi file: ");
        Serial.println(name_buf);
        ret_arr[file_count] = String(name_buf);
        ++file_count;
      }
    }
    file.close();
  }
  return ret_arr;
}

void setup() {
  Serial.begin(115200);
  midifile_setup();
  int tmp = 0;
  lcd.printMenu(tmp);
  delay(2000);
  pinMode(VS1053_DREQ, INPUT);
  // Initialize VS1053 chip
  VS1053_init();

  // Wait 1.5 seconds before turning on USB Host.  If connected USB devices
  // use too much power, Teensy at least completes USB enumeration, which
  // makes isolating the power issue easier.
  delay(1000);
  Serial.println("USB Host InputFunctions example");
  delay(10);
  myusb.begin();

  // Find all the files in the SD card
//  lcd.setS  dFiles(probe_sd_files());

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
  midifile_loop();
  lcd.update();
  // Check if it's time to send the pitch bend value
//  if (pitch_bend_sampler.check())
//    samplePitchBend();
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
