//
//  PicoCalc Songs Library
//
//  This file contains popular songs arranged for the PicoCalc audio driver.
//  Songs are stored as arrays of notes with frequencies and durations.
//

#include <stdio.h>
#include <string.h>
#include "pico/time.h"
#include "drivers/audio.h"
#include "songs.h"

// "Baa Baa Black Sheep" - Traditional nursery rhyme with stereo harmony
const audio_note_t notes_baa_baa[] = {
    // "Baa baa black sheep, have you any wool?"
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_A4, PITCH_F4, NOTE_QUARTER},
    {PITCH_A4, PITCH_F4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_HALF},

    // "Yes sir, yes sir, three bags full"
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},

    // "One for the master, one for the dame"
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_HALF},

    // "And one for the little boy who lives down the lane"
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_HALF},

    // "Baa baa black sheep, have you any wool?" (repeat)
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_A4, PITCH_F4, NOTE_QUARTER},
    {PITCH_A4, PITCH_F4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_HALF},

    // "Yes sir, yes sir, three bags full" (repeat)
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Old MacDonald Had a Farm" - Popular children's song with stereo harmony
const audio_note_t notes_old_macdonald[] = {
    // "Old MacDonald had a farm, E-I-E-I-O"
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_A3, PITCH_F3, NOTE_QUARTER},
    {PITCH_A3, PITCH_F3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_HALF},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},

    // "And on his farm he had a cow, E-I-E-I-O"
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_HALF},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},

    // "With a moo-moo here, and a moo-moo there" with panning effect
    {PITCH_C4, SILENCE, NOTE_EIGHTH},
    {SILENCE, PITCH_G3, NOTE_EIGHTH},
    {PITCH_C4, SILENCE, NOTE_EIGHTH},
    {SILENCE, PITCH_G3, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {SILENCE, PITCH_C4, NOTE_EIGHTH},
    {PITCH_G3, SILENCE, NOTE_EIGHTH},
    {SILENCE, PITCH_C4, NOTE_EIGHTH},
    {PITCH_G3, SILENCE, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},

    // "Here a moo, there a moo, everywhere a moo-moo"
    {PITCH_C4, SILENCE, NOTE_EIGHTH},
    {SILENCE, PITCH_G3, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {SILENCE, PITCH_C4, NOTE_EIGHTH},
    {PITCH_G3, SILENCE, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {SILENCE, PITCH_C4, NOTE_EIGHTH},
    {PITCH_G3, SILENCE, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},

    // "Old MacDonald had a farm, E-I-E-I-O"
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_A3, PITCH_F3, NOTE_QUARTER},
    {PITCH_A3, PITCH_F3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_HALF},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_WHOLE},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Itsy Bitsy Spider" - Classic children's nursery rhyme with stereo harmony
const audio_note_t notes_itsy_bitsy_spider[] = {
    // "The itsy bitsy spider climbed up the water spout"
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},
    {SILENCE, SILENCE, NOTE_QUARTER},

    // "Down came the rain and washed the spider out"
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},
    {SILENCE, SILENCE, NOTE_QUARTER},

    // "Out came the sun and dried up all the rain" with panning effect
    {PITCH_G4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_G4, NOTE_QUARTER},
    {PITCH_E4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_E4, NOTE_QUARTER},
    {PITCH_D4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_D4, NOTE_QUARTER},
    {PITCH_C4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_E4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_HALF},
    {SILENCE, SILENCE, NOTE_QUARTER},

    // "And the itsy bitsy spider climbed up the spout again"
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G3, PITCH_E3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_WHOLE},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Mary Had a Little Lamb" - Stereo version with call and response
const audio_note_t notes_mary_lamb[] = {
    // Melody on left, harmony on right
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_HALF},

    // Switch channels - harmony on left, melody on right
    {PITCH_B3, PITCH_D4, NOTE_QUARTER},
    {PITCH_B3, PITCH_D4, NOTE_QUARTER},
    {PITCH_B3, PITCH_D4, NOTE_HALF},
    {PITCH_C4, PITCH_E4, NOTE_QUARTER},
    {PITCH_E4, PITCH_G4, NOTE_QUARTER},
    {PITCH_E4, PITCH_G4, NOTE_HALF},

    // Back to original - melody left, harmony right
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_WHOLE},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Happy Birthday" - Stereo version with harmonies
const audio_note_t notes_happy_birthday[] = {
    // First verse - melody left, harmony right
    {PITCH_C4, PITCH_A3, NOTE_DOTTED_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_SIXTEENTH},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_HALF},

    {PITCH_C4, PITCH_A3, NOTE_DOTTED_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_SIXTEENTH},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_HALF},

    // Switch channels for variety
    {PITCH_A3, PITCH_C4, NOTE_DOTTED_EIGHTH},
    {PITCH_A3, PITCH_C4, NOTE_SIXTEENTH},
    {PITCH_B3, PITCH_C5, NOTE_QUARTER},
    {PITCH_A3, PITCH_A4, NOTE_QUARTER},
    {PITCH_D4, PITCH_F4, NOTE_QUARTER},
    {PITCH_C4, PITCH_E4, NOTE_QUARTER},
    {PITCH_B3, PITCH_D4, NOTE_QUARTER},

    {PITCH_D4, PITCH_AS4, NOTE_DOTTED_EIGHTH},
    {PITCH_D4, PITCH_AS4, NOTE_SIXTEENTH},
    {PITCH_C4, PITCH_A4, NOTE_QUARTER},
    {PITCH_A3, PITCH_F4, NOTE_QUARTER},
    {PITCH_B3, PITCH_G4, NOTE_QUARTER},
    {PITCH_A3, PITCH_F4, NOTE_HALF},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Twinkle Twinkle Little Star" - Stereo version with panning effect
const audio_note_t notes_twinkle[] = {
    // Start left, move to right
    {PITCH_C4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_C4, NOTE_QUARTER},
    {PITCH_G4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_G4, NOTE_QUARTER},
    {PITCH_A4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_A4, NOTE_QUARTER},
    {PITCH_G4, PITCH_G4, NOTE_HALF}, // Both channels

    {SILENCE, PITCH_F4, NOTE_QUARTER},
    {PITCH_F4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_E4, NOTE_QUARTER},
    {PITCH_E4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_D4, NOTE_QUARTER},
    {PITCH_D4, SILENCE, NOTE_QUARTER},
    {PITCH_C4, PITCH_C4, NOTE_HALF}, // Both channels

    // Harmony section
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_HALF},

    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_G4, PITCH_E4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_HALF},

    // Final section with panning
    {PITCH_C4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_C4, NOTE_QUARTER},
    {PITCH_G4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_G4, NOTE_QUARTER},
    {PITCH_A4, SILENCE, NOTE_QUARTER},
    {SILENCE, PITCH_A4, NOTE_QUARTER},
    {PITCH_G4, PITCH_G4, NOTE_HALF},

    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_F4, PITCH_D4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_E4, PITCH_C4, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_D4, PITCH_B3, NOTE_QUARTER},
    {PITCH_C4, PITCH_A3, NOTE_HALF},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Canon in D" - Simplified stereo version with overlapping melodies
const audio_note_t notes_canon_in_d[] = {
    // Left channel starts first
    {PITCH_D4, SILENCE, NOTE_HALF},
    {PITCH_A3, SILENCE, NOTE_HALF},
    {PITCH_B3, SILENCE, NOTE_HALF},
    {PITCH_FS3, SILENCE, NOTE_HALF},

    // Right channel joins with the same melody delayed
    {PITCH_G3, PITCH_D4, NOTE_HALF},
    {PITCH_D3, PITCH_A3, NOTE_HALF},
    {PITCH_G3, PITCH_B3, NOTE_HALF},
    {PITCH_A3, PITCH_FS3, NOTE_HALF},

    // Both channels in harmony
    {PITCH_D4, PITCH_G3, NOTE_QUARTER},
    {PITCH_E4, PITCH_A3, NOTE_QUARTER},
    {PITCH_FS4, PITCH_B3, NOTE_QUARTER},
    {PITCH_G4, PITCH_C4, NOTE_QUARTER},
    {PITCH_A4, PITCH_D4, NOTE_QUARTER},
    {PITCH_G4, PITCH_C4, NOTE_QUARTER},
    {PITCH_FS4, PITCH_B3, NOTE_QUARTER},
    {PITCH_E4, PITCH_A3, NOTE_QUARTER},

    {PITCH_D4, PITCH_G3, NOTE_QUARTER},
    {PITCH_C4, PITCH_FS3, NOTE_QUARTER},
    {PITCH_B3, PITCH_E3, NOTE_QUARTER},
    {PITCH_A3, PITCH_D3, NOTE_QUARTER},
    {PITCH_B3, PITCH_E3, NOTE_QUARTER},
    {PITCH_C4, PITCH_FS3, NOTE_QUARTER},
    {PITCH_D4, PITCH_G3, NOTE_QUARTER},
    {PITCH_E4, PITCH_A3, NOTE_QUARTER},

    // Finale with both channels
    {PITCH_FS4, PITCH_D4, NOTE_HALF},
    {PITCH_G4, PITCH_E4, NOTE_HALF},
    {PITCH_A4, PITCH_FS4, NOTE_HALF},
    {PITCH_D5, PITCH_A4, NOTE_WHOLE},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Für Elise" - Simplified opening with stereo harmony
const audio_note_t notes_fur_elise[] = {
    // Famous opening melody with bass accompaniment
    {PITCH_E4, SILENCE, NOTE_EIGHTH},
    {PITCH_DS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_DS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, SILENCE, NOTE_EIGHTH},
    {PITCH_B3, PITCH_E3, NOTE_EIGHTH},
    {PITCH_D4, SILENCE, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_EIGHTH},

    {PITCH_A3, PITCH_C3, NOTE_QUARTER},
    {SILENCE, PITCH_E3, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_E3, SILENCE, NOTE_EIGHTH},
    {PITCH_A3, PITCH_C4, NOTE_EIGHTH},

    {PITCH_B3, PITCH_E3, NOTE_QUARTER},
    {SILENCE, PITCH_E3, NOTE_EIGHTH},
    {PITCH_E4, PITCH_GS3, NOTE_EIGHTH},
    {PITCH_GS3, SILENCE, NOTE_EIGHTH},
    {PITCH_B3, PITCH_E4, NOTE_EIGHTH},

    // Repeat with variation
    {PITCH_C4, PITCH_A3, NOTE_EIGHTH},
    {SILENCE, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_DS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_DS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_B3, PITCH_E3, NOTE_EIGHTH},

    {PITCH_D4, SILENCE, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_A3, PITCH_C3, NOTE_QUARTER},
    {SILENCE, PITCH_E3, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_E3, SILENCE, NOTE_EIGHTH},
    {PITCH_A3, PITCH_C4, NOTE_EIGHTH},

    {PITCH_B3, PITCH_E3, NOTE_QUARTER},
    {SILENCE, PITCH_E3, NOTE_EIGHTH},
    {PITCH_C4, PITCH_A3, NOTE_EIGHTH},
    {PITCH_B3, SILENCE, NOTE_EIGHTH},
    {PITCH_A3, PITCH_C4, NOTE_QUARTER},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Moonlight Sonata" - Simplified first movement opening
const audio_note_t notes_moonlight_sonata[] = {
    // Characteristic triplet accompaniment in left, melody in right
    {PITCH_GS3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_GS4, NOTE_EIGHTH},
    {PITCH_GS3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_GS4, NOTE_EIGHTH},

    {PITCH_A3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A4, NOTE_EIGHTH},
    {PITCH_A3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A4, NOTE_EIGHTH},

    {PITCH_FS3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_DS4, PITCH_FS4, NOTE_EIGHTH},
    {PITCH_FS3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_DS4, PITCH_FS4, NOTE_EIGHTH},

    {PITCH_GS3, SILENCE, NOTE_EIGHTH},
    {PITCH_B3, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_GS4, NOTE_EIGHTH},
    {PITCH_GS3, SILENCE, NOTE_EIGHTH},
    {PITCH_B3, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_GS4, NOTE_EIGHTH},

    // Melody becomes more prominent
    {PITCH_A3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_A4, NOTE_QUARTER},
    {PITCH_A3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_GS4, NOTE_QUARTER},

    {PITCH_FS3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_DS4, PITCH_FS4, NOTE_QUARTER},
    {PITCH_GS3, SILENCE, NOTE_EIGHTH},
    {PITCH_CS4, SILENCE, NOTE_EIGHTH},
    {PITCH_E4, PITCH_E4, NOTE_HALF},

    // End marker
    {SILENCE, SILENCE, 0}};

// "Ode to Joy" - Beethoven's 9th Symphony, arranged for stereo
const audio_note_t notes_ode_to_joy[] = {
    // Opening phrase - melody on right, bass on left
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_F4, NOTE_QUARTER},
    {PITCH_A3, PITCH_G4, NOTE_QUARTER},
    {PITCH_A3, PITCH_G4, NOTE_QUARTER},
    {PITCH_E3, PITCH_F4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},

    // Second phrase
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_DOTTED_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_EIGHTH},
    {PITCH_C3, PITCH_D4, NOTE_HALF},

    // Repeat of opening phrase
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_F4, NOTE_QUARTER},
    {PITCH_A3, PITCH_G4, NOTE_QUARTER},
    {PITCH_A3, PITCH_G4, NOTE_QUARTER},
    {PITCH_E3, PITCH_F4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},

    // Final phrase of first section
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_DOTTED_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_EIGHTH},
    {PITCH_C3, PITCH_C4, NOTE_HALF},

    // Bridge section with more complex harmonies
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_D3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_C4, NOTE_QUARTER},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},
    {PITCH_D3, PITCH_E4, NOTE_EIGHTH},
    {PITCH_E3, PITCH_C4, NOTE_EIGHTH},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},

    // Repeat bridge with variation
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_D3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_C4, NOTE_QUARTER},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},
    {PITCH_D3, PITCH_E4, NOTE_EIGHTH},
    {PITCH_E3, PITCH_C4, NOTE_EIGHTH},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},

    // Final triumphant section with both channels in harmony
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_F4, NOTE_QUARTER},
    {PITCH_A3, PITCH_G4, NOTE_QUARTER},
    {PITCH_A3, PITCH_G4, NOTE_QUARTER},
    {PITCH_E3, PITCH_F4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_E3, PITCH_D4, NOTE_QUARTER},

    // Grand finale
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_QUARTER},
    {PITCH_E3, PITCH_E4, NOTE_QUARTER},
    {PITCH_C3, PITCH_D4, NOTE_DOTTED_QUARTER},
    {PITCH_C3, PITCH_C4, NOTE_EIGHTH},
    {PITCH_C3, PITCH_C4, NOTE_WHOLE},

    // End marker
    {SILENCE, SILENCE, 0}};

// Song library for easy access
const audio_song_t songs[] = {
    {"baa", notes_baa_baa, "Baa Baa Black Sheep"},
    {"birthday", notes_happy_birthday, "Happy Birthday"},
    {"canon", notes_canon_in_d, "Canon in D"},
    {"elise", notes_fur_elise, "Fur Elise"},
    {"macdonald", notes_old_macdonald, "Old MacDonald Had a Farm"},
    {"mary", notes_mary_lamb, "Mary Had a Little Lamb"},
    {"moonlight", notes_moonlight_sonata, "Moonlight Sonata"},
    {"ode", notes_ode_to_joy, "Ode to Joy (Beethoven)"},
    {"spider", notes_itsy_bitsy_spider, "Itsy Bitsy Spider"},
    {"twinkle", notes_twinkle, "Twinkle Twinkle Little Star"},
    {NULL, NULL, NULL} // End marker
};


// Function to find a song by name
const audio_song_t *find_song(const char *song_name)
{
    for (int i = 0; songs[i].name != NULL; i++)
    {
        if (strcmp(songs[i].name, song_name) == 0)
        {
            return &songs[i];
        }
    }
    return NULL; // Song not found
}

// Function to list all available songs
void show_song_library(void)
{
    printf("\033[?25l\033[4mSong Library\033[0m\n\n");

    for (int i = 0; songs[i].name != NULL; i++)
    {
        printf("  \033[1m%s\033[0m - %s\n", songs[i].name, songs[i].description);
    }
    printf("\033[?25h\n");
}
