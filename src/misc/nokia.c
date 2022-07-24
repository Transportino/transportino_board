#include "misc/nokia.h"

#include <stdlib.h>

#include "pico/stdlib.h"

// Code adapted from a sketch made by Robson Couto
// https://github.com/robsoncouto/arduino-songs/blob/master/nokia/nokia.ino

const uint8_t tempo = 180;

const uint32_t melody[] = {
  NOTE_E5, 8, NOTE_D5, 8, NOTE_FS4, 4, NOTE_GS4, 4, 
  NOTE_CS5, 8, NOTE_B4, 8, NOTE_D4, 4, NOTE_E4, 4, 
  NOTE_B4, 8, NOTE_A4, 8, NOTE_CS4, 4, NOTE_E4, 4,
  NOTE_A4, 2,
};

const uint8_t notes = sizeof(melody) / (sizeof(melody[0]) * 2);

const uint32_t whole_note = ((60000 * 4)/tempo);

uint8_t divider = 0; 
uint32_t noteDuration = 0;

void nokia_play(buzzer* buzz) 
{
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
        // calculates the duration of each note
        divider = melody[thisNote + 1];
        if (divider > 0) {
        // regular note, just proceed
        noteDuration = (whole_note) / divider;
        } else if (divider < 0) {
        // dotted notes are represented with negative durations!!
        noteDuration = (whole_note) / abs(divider);
        noteDuration *= 1.5; // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        buzz_play(buzz, melody[thisNote]);

        // Wait for the specief duration before playing the next note.
        sleep_ms(noteDuration * 0.9);

        // stop the waveform generation before the next note.
        buzz_mute(buzz);

        sleep_ms(noteDuration * 0.1);
    }
}