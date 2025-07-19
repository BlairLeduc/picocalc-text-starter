#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drivers/audio.h"
#include "drivers/fat32.h"
#include "tests.h"

extern volatile bool user_interrupt;

// Helper function names corrected
void play_stereo_melody_demo()
{
    printf("Playing stereo melody demo...\n");
    printf("Listen for the melody bouncing between\n");
    printf("left and right channels!\n\n");

    // Define a simple melody with stereo panning
    struct
    {
        uint16_t left_freq;
        uint16_t right_freq;
        uint32_t duration;
        const char *note_name;
    } stereo_melody[] = {
        // Twinkle Twinkle Little Star with stereo panning
        {PITCH_C4, SILENCE, NOTE_QUARTER, "C4 (Left)"},
        {SILENCE, PITCH_C4, NOTE_QUARTER, "C4 (Right)"},
        {PITCH_G4, SILENCE, NOTE_QUARTER, "G4 (Left)"},
        {SILENCE, PITCH_G4, NOTE_QUARTER, "G4 (Right)"},
        {PITCH_A4, SILENCE, NOTE_QUARTER, "A4 (Left)"},
        {SILENCE, PITCH_A4, NOTE_QUARTER, "A4 (Right)"},
        {PITCH_G4, PITCH_G4, NOTE_HALF, "G4 (Both)"}, // Both channels for emphasis

        // Second phrase
        {SILENCE, PITCH_F4, NOTE_QUARTER, "F4 (Right)"},
        {PITCH_F4, SILENCE, NOTE_QUARTER, "F4 (Left)"},
        {SILENCE, PITCH_E4, NOTE_QUARTER, "E4 (Right)"},
        {PITCH_E4, SILENCE, NOTE_QUARTER, "E4 (Left)"},
        {SILENCE, PITCH_D4, NOTE_QUARTER, "D4 (Right)"},
        {PITCH_D4, SILENCE, NOTE_QUARTER, "D4 (Left)"},
        {PITCH_C4, PITCH_C4, NOTE_HALF, "C4 (Both)"}, // Both channels for ending

        // End marker
        {SILENCE, SILENCE, 0, "End"}};

    int i = 0;
    while (stereo_melody[i].duration > 0)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            audio_stop();
            return;
        }

        printf("  %s\n", stereo_melody[i].note_name);
        audio_play_sound_blocking(
            stereo_melody[i].left_freq,
            stereo_melody[i].right_freq,
            stereo_melody[i].duration);
        sleep_ms(50); // Small gap between notes
        i++;
    }

    printf("\nStereo melody demo complete!\n");
}

void play_stereo_harmony_demo()
{
    printf("Playing stereo harmony demo...\n");
    printf("Listen for harmonious intervals played\n");
    printf("simultaneously in both channels!\n\n");

    // Define chord progression with harmony
    struct
    {
        uint16_t left_freq;
        uint16_t right_freq;
        uint32_t duration;
        const char *chord_name;
    } harmony_progression[] = {
        // C Major chord progression
        {PITCH_C4, PITCH_E4, NOTE_WHOLE, "C Major (C4-E4)"},
        {PITCH_F4, PITCH_A4, NOTE_WHOLE, "F Major (F4-A4)"},
        {PITCH_G4, PITCH_B4, NOTE_WHOLE, "G Major (G4-B4)"},
        {PITCH_C4, PITCH_E4, NOTE_WHOLE, "C Major (C4-E4)"},

        // With bass notes
        {PITCH_C3, PITCH_C4, NOTE_WHOLE, "C Octave (C3-C4)"},
        {PITCH_F3, PITCH_F4, NOTE_WHOLE, "F Octave (F3-F4)"},
        {PITCH_G3, PITCH_G4, NOTE_WHOLE, "G Octave (G3-G4)"},
        {PITCH_C3, PITCH_C4, NOTE_WHOLE, "C Octave (C3-C4)"},

        // End marker
        {SILENCE, SILENCE, 0, "End"}};

    int i = 0;
    while (harmony_progression[i].duration > 0)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            audio_stop();
            return;
        }

        printf("  %s\n", harmony_progression[i].chord_name);
        audio_play_sound_blocking(
            harmony_progression[i].left_freq,
            harmony_progression[i].right_freq,
            harmony_progression[i].duration);
        sleep_ms(200); // Pause between chords
        i++;
    }

    printf("\nStereo harmony demo complete!\n");
}

void audiotest()
{
    printf("Comprehensive Audio Driver Test\n");

    printf("\n1. Playing musical scale (C4 to C5):\n");

    // Play a simple C major scale
    uint16_t scale[] = {
        PITCH_C4, PITCH_D4, PITCH_E4, PITCH_F4,
        PITCH_G4, PITCH_A4, PITCH_B4, PITCH_C5};

    const char *note_names[] = {
        "C4", "D4", "E4", "F4", "G4", "A4", "B4", "C5"};

    for (int i = 0; i < 8; i++)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\nStopping audio test.\n");
            return;
        }

        printf("Playing %s (%d Hz)...\n", note_names[i], scale[i]);
        audio_play_sound_blocking(scale[i], scale[i], NOTE_HALF);
        sleep_ms(100); // Small gap between notes
    }

    printf("\n2. Testing stereo channel separation:\n");

    // Test left channel only
    printf("Left channel only (C4 - 262 Hz)...\n");
    audio_play_sound_blocking(PITCH_C4, SILENCE, NOTE_WHOLE);
    if (user_interrupt)
        return;
    sleep_ms(200);

    // Test right channel only
    printf("Right channel only (E4 - 330 Hz)...\n");
    audio_play_sound_blocking(SILENCE, PITCH_E4, NOTE_WHOLE);
    if (user_interrupt)
        return;
    sleep_ms(200);

    // Test both channels with different frequencies
    printf("Both channels (Left: G4, Right: C5)...\n");
    audio_play_sound_blocking(PITCH_G4, PITCH_C5, NOTE_WHOLE);
    if (user_interrupt)
        return;
    sleep_ms(200);

    printf("\n3. Harmony Test:\n");
    printf("Playing musical intervals...\n");

    // Musical intervals for stereo harmony
    struct
    {
        uint16_t left;
        uint16_t right;
        const char *interval;
        const char *description;
    } harmonies[] = {
        {PITCH_C4, PITCH_C4, "Unison", "Same note both channels"},
        {PITCH_C4, PITCH_E4, "Major 3rd", "C4 + E4"},
        {PITCH_C4, PITCH_G4, "Perfect 5th", "C4 + G4"},
        {PITCH_C4, PITCH_C5, "Octave", "C4 + C5"},
        {PITCH_F4, PITCH_A4, "Major 3rd", "F4 + A4"},
        {PITCH_G4, PITCH_D5, "Perfect 5th", "G4 + D5"},
        {PITCH_A3, PITCH_CS4, "Major 3rd", "A3 + C#4"},
        {PITCH_E4, PITCH_B4, "Perfect 5th", "E4 + B4"}};

    for (int i = 0; i < 8; i++)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            return;
        }

        printf("  %s: %s\n", harmonies[i].interval, harmonies[i].description);
        audio_play_sound_blocking(harmonies[i].left, harmonies[i].right, NOTE_HALF);
        sleep_ms(100);
    }

    printf("\n4. Beat Frequency Test:\n");
    printf("Creating beat effects with detuned\nfrequencies...\n");

    // Beat frequency tests - slightly detuned frequencies create beating effects
    struct
    {
        uint16_t left;
        uint16_t right;
        const char *description;
    } beats[] = {
        {440, 442, "A4 vs A4+2Hz (slow beat)"},
        {440, 444, "A4 vs A4+4Hz (medium beat)"},
        {440, 448, "A4 vs A4+8Hz (fast beat)"},
        {523, 527, "C5 vs C5+4Hz (medium beat)"}};

    for (int i = 0; i < 4; i++)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            return;
        }

        printf("  %s\n", beats[i].description);
        audio_play_sound_blocking(beats[i].left, beats[i].right, NOTE_WHOLE + NOTE_HALF);
        sleep_ms(300);
    }

    printf("\n5. Stereo Sweep Test:\n");
    printf("Frequency sweep in stereo...\n");

    // Sweep both channels with different patterns
    printf("  Parallel sweep (both channels rising)\n");
    for (int freq = 200; freq <= 1000; freq += 100)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            return;
        }

        audio_play_sound_blocking(freq, freq, NOTE_EIGHTH);
        sleep_ms(25);
    }

    printf("  Counter sweep (left up, right down)\n");
    for (int i = 0; i < 9; i++)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            return;
        }

        uint16_t left_freq = 200 + (i * 100);
        uint16_t right_freq = 1000 - (i * 100);
        audio_play_sound_blocking(left_freq, right_freq, NOTE_EIGHTH);
        sleep_ms(25);
    }

    // === FREQUENCY RANGE TESTS ===
    printf("\n6. Testing frequency range (stereo):\n");

    // Test low to high frequency sweep in stereo
    uint16_t test_freqs[] = {
        LOW_BEEP, PITCH_C3, PITCH_C4, PITCH_C5, PITCH_C6, HIGH_BEEP};

    const char *freq_names[] = {
        "Low Beep (100 Hz)", "C3 (131 Hz)", "C4 (262 Hz)",
        "C5 (523 Hz)", "C6 (1047 Hz)", "High Beep (2000 Hz)"};

    for (int i = 0; i < 6; i++)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\nStopping audio test.\n");
            return;
        }

        printf("Playing %s (stereo)...\n", freq_names[i]);
        audio_play_sound_blocking(test_freqs[i], test_freqs[i], NOTE_QUARTER);
        sleep_ms(300);
    }

    // Test async stereo playback
    printf("\n7. Testing async stereo playback:\n");
    printf("Playing continuous stereo harmony\n");
    printf("for 3 seconds (C4 left, E4 right):\n");

    audio_play_sound(PITCH_C4, PITCH_E4);

    for (int i = 3; i > 0; i--)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\nStopping audio test.\n");
            break;
        }
        printf("%d...\n", i);
        sleep_ms(1000);
    }

    audio_stop();
    printf("Audio stopped.\n");

    printf("\n8. Stereo Phase Test:\n");
    printf("Playing identical frequencies to test\nphase alignment...\n");

    // Test phase alignment with identical frequencies
    uint16_t test_tones[] = {PITCH_A3, PITCH_A4, PITCH_A5};
    const char *PITCH_names[] = {"A3 (220 Hz)", "A4 (440 Hz)", "A5 (880 Hz)"};

    for (int i = 0; i < 3; i++)
    {
        if (user_interrupt)
        {
            printf("\nUser interrupt detected.\n");
            return;
        }

        printf("  %s on both channels...\n", PITCH_names[i]);
        audio_play_sound_blocking(test_tones[i], test_tones[i], NOTE_WHOLE);
        sleep_ms(200);
    }

    printf("\nDemo 1: Stereo Melody\n");
    play_stereo_melody_demo();

    if (user_interrupt)
    {
        printf("Demo interrupted.\n");
        return;
    }

    printf("\nDemo 2: Stereo Harmony\n");
    play_stereo_harmony_demo();

    if (user_interrupt)
    {
        printf("Demo interrupted.\n");
        return;
    }

    printf("\nComprehensive audio test complete!\n");
    printf("Your stereo audio system is working\n");
    printf("properly if you heard distinct\n");
    printf("left/right separation, melodies\n");
    printf("bouncing between channels, and\n");
    printf("harmonious intervals.\n\n");
    printf("Press BREAK key anytime during audio\nplayback to interrupt.\n");
}

void displaytest()
{
    int row = 1;
    printf("\033[?25l"); // Hide cursor

    absolute_time_t start_time = get_absolute_time();

    while (!user_interrupt && row <= 2000)
    {
        int colour = 16 + (row % 215);
        printf("\033[38;5;%dmRow: %04d 01234567890ABCDEFGHIJKLMNOPQRS", colour, row++);
    }

    absolute_time_t end_time = get_absolute_time();
    uint64_t scrolling_elapsed_us = absolute_time_diff_us(start_time, end_time);
    float scrolling_elapsed_seconds = scrolling_elapsed_us / 1000000.0;
    float rows_per_second = (row - 1) / scrolling_elapsed_seconds;

    int chars = 0;
    printf("\033[m\033[2J\033[H"); // Reset colors, clear the screen, and move cursor to home position
    printf("Character stress test:\n\n");
    printf("\033(0");  // Select DEC Special Character Set for G0
    printf("lqqqk\n"); // Top border: ┌───┐
    printf("x   x\n"); // Sides:      │   │
    printf("mqqqj\n"); // Bottom:     └───┘

    char buffer[32];
    uint output_chars = 0;
    start_time = get_absolute_time(); // Reset the start time for the next display
    while (!user_interrupt && chars < 60000)
    {
        int colour = 16 + (chars % 215);

        sprintf(buffer, "\033[4;3H\033[38;5;%dm%c", colour, 'A' + (chars % 26));
        output_chars += strlen(buffer);
        printf("%s", buffer);
        chars++;
    }
    end_time = get_absolute_time();
    uint64_t cps_elapsed_us = absolute_time_diff_us(start_time, end_time);
    float cps_elapsed_seconds = cps_elapsed_us / 1000000.0;
    float chars_per_second = output_chars / cps_elapsed_seconds;
    float displayed_per_second = chars / cps_elapsed_seconds;

    printf("\n\n\n\033(B\033[m\033[?25h");
    printf("Display stress test complete.\n");
    printf("\nRows processed: %d\n", row - 1);
    printf("Rows time elapsed: %.2f seconds\n", scrolling_elapsed_seconds);
    printf("Average rows per second: %.2f\n", rows_per_second);
    printf("\nCharacters processed: %d\n", output_chars);
    printf("Characters time elapsed: %.2f seconds\n", cps_elapsed_seconds);
    printf("Average characters per second: %.0f\n", chars_per_second);
    printf("Characters displayed: %d\n", chars);
    printf("Average displayed cps: %.0f\n", displayed_per_second);
}

void keyboardtest()
{
    while (!user_interrupt)
    {
        char ch = getchar();
        printf("You pressed: '%c' - 0%o, %d, 0x%x\n", ch, ch, ch, ch);
    }
}

static bool fat32_test_setup()
{
    printf("Setting up FAT32 test environment...\n");

    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        fat32_file_t base_dir;

        // Create the tests directory if it doesn't exist
        if (fat32_dir_create(&base_dir, "/tests") != FAT32_OK)
        {
            printf("FAIL: Cannot create or open tests directory\n");
            return false;
        }
        fat32_close(&base_dir);
        fat32_set_current_dir("/tests");
    }

    printf("Test directory ready.\n");
    return true;
}

static bool fat32_test_cleanup()
{
    fat32_file_t dir;

    printf("Cleaning up test files...\n");

    fat32_set_current_dir("/");

    printf("Cleanup complete.\n");
    return true;
}

static bool fat32_test_basic_operations()
{
    fat32_file_t file;

    printf("\n=== Basic Operations Test ===\n");

    // Test directory creation
    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        printf("FAIL: Cannot change to tests directory\n");
        return false;
    }

    // Test file creation
    if (fat32_create(&file, "basic_test.txt") != FAT32_OK)
    {
        // File might exist, try to open it
        if (fat32_open(&file, "basic_test.txt") != FAT32_OK)
        {
            printf("FAIL: Cannot create or open basic_test.txt\n");
            return false;
        }
    }

    // Test basic write
    const char *test_data = "Hello FAT32!";
    size_t bytes_written;
    if (fat32_write(&file, test_data, strlen(test_data), &bytes_written) != FAT32_OK)
    {
        printf("FAIL: Cannot write to basic_test.txt\n");
        return false;
    }

    if (bytes_written != strlen(test_data))
    {
        printf("FAIL: Wrote %zu bytes, expected %zu\n", bytes_written, strlen(test_data));
        return false;
    }

    fat32_close(&file);

    // Test file read
    if (fat32_open(&file, "basic_test.txt") != FAT32_OK)
    {
        printf("FAIL: Cannot reopen basic_test.txt\n");
        return false;
    }

    char read_buffer[32];
    size_t bytes_read;
    if (fat32_read(&file, read_buffer, strlen(test_data), &bytes_read) != FAT32_OK)
    {
        printf("FAIL: Cannot read from basic_test.txt\n");
        return false;
    }

    if (bytes_read != strlen(test_data) || memcmp(read_buffer, test_data, bytes_read) != 0)
    {
        printf("FAIL: Read data doesn't match written data\n");
        return false;
    }

    fat32_close(&file);

    printf("PASS: Basic operations test\n");
    return true;
}

static bool fat32_test_sector_boundaries()
{
    fat32_file_t file;

    printf("\n=== Sector Boundary Test ===\n");

    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        printf("FAIL: Cannot change to tests directory\n");
        return false;
    }

    // Create a file that spans multiple sectors (512 bytes each)
    if (fat32_create(&file, "sector_test.bin") != FAT32_OK)
    {
        if (fat32_open(&file, "sector_test.bin") != FAT32_OK)
        {
            printf("FAIL: Cannot create sector_test.bin\n");
            return false;
        }
    }

    // Test writing exactly one sector
    char sector_data[512];
    for (int i = 0; i < 512; i++)
    {
        sector_data[i] = (char)(i % 256);
    }

    size_t bytes_written;
    if (fat32_write(&file, sector_data, 512, &bytes_written) != FAT32_OK)
    {
        printf("FAIL: Cannot write 512 bytes\n");
        return false;
    }

    if (bytes_written != 512)
    {
        printf("FAIL: Wrote %lu bytes, expected 512\n", bytes_written);
        return false;
    }

    // Test writing across sector boundary (512 + 256 = 768 bytes)
    char extra_data[256];
    for (int i = 0; i < 256; i++)
    {
        extra_data[i] = (char)((i + 128) % 256);
    }

    if (fat32_write(&file, extra_data, 256, &bytes_written) != FAT32_OK)
    {
        printf("FAIL: Cannot write additional 256 bytes\n");
        return false;
    }

    if (bytes_written != 256)
    {
        printf("FAIL: Wrote %lu bytes, expected 256\n", bytes_written);
        return false;
    }

    fat32_close(&file);

    // Verify the data
    if (fat32_open(&file, "sector_test.bin") != FAT32_OK)
    {
        printf("FAIL: Cannot reopen sector_test.bin\n");
        return false;
    }

    char verify_buffer[768];
    size_t bytes_read;
    if (fat32_read(&file, verify_buffer, 768, &bytes_read) != FAT32_OK)
    {
        printf("FAIL: Cannot read 768 bytes\n");
        return false;
    }

    if (bytes_read != 768)
    {
        printf("FAIL: Read %lu bytes, expected 768\n", bytes_read);
        return false;
    }

    // Verify first sector
    for (int i = 0; i < 512; i++)
    {
        if (verify_buffer[i] != (char)(i % 256))
        {
            printf("FAIL: Data mismatch at byte %d\n", i);
            return false;
        }
    }

    // Verify second part
    for (int i = 0; i < 256; i++)
    {
        if (verify_buffer[512 + i] != (char)((i + 128) % 256))
        {
            printf("FAIL: Data mismatch at byte %d\n", 512 + i);
            return false;
        }
    }

    fat32_close(&file);

    printf("PASS: Sector boundary test\n");
    return true;
}

static bool fat32_test_cluster_boundaries()
{
    fat32_file_t file;

    printf("\n=== Cluster Boundary Test ===\n");

    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        printf("FAIL: Cannot change to tests directory\n");
        return false;
    }

    // Create a file that spans multiple clusters (32 KiB each)
    if (fat32_create(&file, "cluster_test.bin") != FAT32_OK)
    {
        if (fat32_open(&file, "cluster_test.bin") != FAT32_OK)
        {
            printf("FAIL: Cannot create cluster_test.bin\n");
            return false;
        }
    }

    printf("Writing cluster boundary test data...\n");

    // Write exactly one cluster (32768 bytes) in 1KB chunks
    const uint32_t cluster_size = 32768;
    const uint32_t chunk_size = 1024;
    char chunk_data[chunk_size];

    for (uint32_t offset = 0; offset < cluster_size; offset += chunk_size)
    {
        // Fill chunk with pattern based on offset
        for (uint32_t i = 0; i < chunk_size; i++)
        {
            chunk_data[i] = (char)((offset + i) % 256);
        }

        size_t bytes_written;
        if (fat32_write(&file, chunk_data, chunk_size, &bytes_written) != FAT32_OK)
        {
            printf("FAIL: Cannot write chunk at offset %lu\n", offset);
            return false;
        }

        if (bytes_written != chunk_size)
        {
            printf("FAIL: Wrote %lu bytes, expected %lu\n", bytes_written, chunk_size);
            return false;
        }

        if (user_interrupt)
        {
            printf("\nTest interrupted by user\n");
            return false;
        }
    }

    // Write additional data to cross cluster boundary
    const char *boundary_data = "CLUSTER_BOUNDARY_MARKER";
    size_t bytes_written;
    if (fat32_write(&file, boundary_data, strlen(boundary_data), &bytes_written) != FAT32_OK)
    {
        printf("FAIL: Cannot write boundary marker\n");
        return false;
    }

    fat32_close(&file);

    printf("Verifying cluster boundary test data...\n");

    // Verify the data
    if (fat32_open(&file, "cluster_test.bin") != FAT32_OK)
    {
        printf("FAIL: Cannot reopen cluster_test.bin\n");
        return false;
    }

    // Verify first cluster in chunks
    for (uint32_t offset = 0; offset < cluster_size; offset += chunk_size)
    {
        size_t bytes_read;
        if (fat32_read(&file, chunk_data, chunk_size, &bytes_read) != FAT32_OK)
        {
            printf("FAIL: Cannot read chunk at offset %lu\n", offset);
            return false;
        }

        if (bytes_read != chunk_size)
        {
            printf("FAIL: Read %lu bytes, expected %lu\n", bytes_read, chunk_size);
            return false;
        }

        // Verify chunk data
        for (uint32_t i = 0; i < chunk_size; i++)
        {
            if (chunk_data[i] != (char)((offset + i) % 256))
            {
                printf("FAIL: Data mismatch at offset %lu\n", offset + i);
                return false;
            }
        }

        if (user_interrupt)
        {
            printf("\nTest interrupted by user\n");
            return false;
        }
    }

    // Verify boundary marker
    char boundary_buffer[32];
    size_t bytes_read;
    if (fat32_read(&file, boundary_buffer, strlen(boundary_data), &bytes_read) != FAT32_OK)
    {
        printf("FAIL: Cannot read boundary marker\n");
        return false;
    }

    if (bytes_read != strlen(boundary_data) ||
        memcmp(boundary_buffer, boundary_data, bytes_read) != 0)
    {
        printf("FAIL: Boundary marker mismatch\n");
        return false;
    }

    fat32_close(&file);

    printf("PASS: Cluster boundary test\n");
    return true;
}

static bool fat32_test_many_files()
{
    fat32_file_t dir;
    fat32_file_t file;
    fat32_error_t result;

    printf("\n=== Many Files Test ===\n");

    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        printf("FAIL: Cannot open tests directory\n");
        return false;
    }

    // Create subdirectory for many files test
    if (fat32_dir_create(&dir, "many_files") != FAT32_OK)
    {
        if (fat32_open(&dir, "many_files") != FAT32_OK)
        {
            printf("FAIL: Cannot create many_files directory\n");
            return false;
        }
    }
    else
    {
        if (fat32_set_current_dir("many_files") != FAT32_OK)
        {
            printf("FAIL: Cannot switch to many_files directory\n");
            return false;
        }
    }

    printf("Creating multiple files...\n");

    // Create many small files to test directory entries
    const int num_files = 100;
    char filename[32];
    char file_content[64];

    for (int i = 0; i < num_files; i++)
    {
        snprintf(filename, sizeof(filename), "file_%04d.txt", i);
        snprintf(file_content, sizeof(file_content), "This is test file number %d\n", i);

        if ((result = fat32_create(&file, filename)) != FAT32_OK)
        {
            if (result == FAT32_ERROR_FILE_EXISTS)
            {
                if (fat32_open(&file, filename) != FAT32_OK)
                {
                    printf("FAIL: Cannot create file %s\n", filename);
                    return false;
                }
            }
            else
            {
                printf("FAIL: Cannot create or open file %s, error %s\n", filename, fat32_error_string(result));
                return false;
            }
        }

        size_t bytes_written;
        if ((result = fat32_write(&file, file_content, strlen(file_content), &bytes_written)) != FAT32_OK)
        {
            printf("FAIL: Cannot write to file %s, error %s\n", filename, fat32_error_string(result));
            return false;
        }

        fat32_close(&file);

        if ((i + 1) % 20 == 0)
        {
            printf("Created %d files...\n", i + 1);
        }

        if (user_interrupt)
        {
            printf("\nTest interrupted by user\n");
            return false;
        }
    }

    printf("Verifying files...\n");

    // Verify some of the files
    for (int i = 0; i < num_files; i += 10)
    {
        snprintf(filename, sizeof(filename), "file_%04d.txt", i);
        snprintf(file_content, sizeof(file_content), "This is test file number %d\n", i);

        if (fat32_open(&file, filename) != FAT32_OK)
        {
            printf("FAIL: Cannot open file %s for verification\n", filename);
            return false;
        }

        char read_buffer[64];
        size_t bytes_read;
        if (fat32_read(&file, read_buffer, strlen(file_content), &bytes_read) != FAT32_OK)
        {
            printf("FAIL: Cannot read file %s\n", filename);
            return false;
        }

        if (bytes_read != strlen(file_content) ||
            memcmp(read_buffer, file_content, bytes_read) != 0)
        {
            printf("FAIL: File %s content mismatch\n", filename);
            return false;
        }

        fat32_close(&file);

        if (user_interrupt)
        {
            printf("\nTest interrupted by user\n");
            return false;
        }
    }

    if (fat32_set_current_dir("..") != FAT32_OK)
    {
        printf("FAIL: Cannot switch to parent directory\n");
        return false;
    }

    printf("PASS: Many files test (%d files)\n", num_files);
    return true;
}

static bool fat32_test_large_files()
{
    fat32_file_t file;

    printf("\n=== Large Files Test ===\n");

    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        printf("FAIL: Cannot change to tests directory\n");
        return false;
    }

    // Test files of various sizes around cluster boundaries
    struct
    {
        const char *name;
        uint32_t size;
        const char *description;
    } test_files[] = {
        {"small.bin", 511, "Just under 1 sector"},
        {"sector.bin", 512, "Exactly 1 sector"},
        {"sector_plus.bin", 513, "Just over 1 sector"},
        {"multi_sector.bin", 2048, "Multiple sectors"},
        {"cluster_minus.bin", 32767, "Just under 1 cluster"},
        {"cluster.bin", 32768, "Exactly 1 cluster"},
        {"cluster_plus.bin", 32769, "Just over 1 cluster"},
        {"large.bin", 65536, "2 clusters"}};

    const int num_test_files = sizeof(test_files) / sizeof(test_files[0]);

    for (int test_idx = 0; test_idx < num_test_files; test_idx++)
    {
        printf("Testing %s\n  %s...\n",
               test_files[test_idx].name,
               test_files[test_idx].description);

        if (fat32_create(&file, test_files[test_idx].name) != FAT32_OK)
        {
            if (fat32_open(&file, test_files[test_idx].name) != FAT32_OK)
            {
                printf("FAIL: Cannot create %s\n", test_files[test_idx].name);
                return false;
            }
        }

        // Write test pattern
        const uint32_t chunk_size = 1024;
        char chunk_data[chunk_size];
        uint32_t remaining = test_files[test_idx].size;
        uint32_t offset = 0;

        while (remaining > 0)
        {
            uint32_t write_size = (remaining < chunk_size) ? remaining : chunk_size;

            // Fill chunk with pattern
            for (uint32_t i = 0; i < write_size; i++)
            {
                chunk_data[i] = (char)((offset + i) % 256);
            }

            size_t bytes_written;
            if (fat32_write(&file, chunk_data, write_size, &bytes_written) != FAT32_OK)
            {
                printf("FAIL: Cannot write to %s at offset %lu\n",
                       test_files[test_idx].name, offset);
                return false;
            }

            if (bytes_written != write_size)
            {
                printf("FAIL: Wrote %lu bytes, expected %lu\n",
                       bytes_written, write_size);
                return false;
            }

            remaining -= write_size;
            offset += write_size;

            if (user_interrupt)
            {
                printf("\nTest interrupted by user\n");
                return false;
            }
        }

        fat32_close(&file);

        // Verify file size and some content
        if (fat32_open(&file, test_files[test_idx].name) != FAT32_OK)
        {
            printf("FAIL: Cannot reopen %s\n", test_files[test_idx].name);
            return false;
        }

        // Read and verify first chunk
        size_t verify_size = (test_files[test_idx].size < chunk_size) ? test_files[test_idx].size : chunk_size;

        size_t bytes_read;
        if (fat32_read(&file, chunk_data, verify_size, &bytes_read) != FAT32_OK)
        {
            printf("FAIL: Cannot read from %s\n", test_files[test_idx].name);
            return false;
        }

        if (bytes_read != verify_size)
        {
            printf("FAIL: Read %lu bytes, expected %lu\n", bytes_read, verify_size);
            return false;
        }

        // Verify pattern
        for (uint32_t i = 0; i < verify_size; i++)
        {
            if (chunk_data[i] != (char)(i % 256))
            {
                printf("FAIL: Data mismatch in %s at byte %lu\n",
                       test_files[test_idx].name, i);
                return false;
            }
        }

        fat32_close(&file);

        if (user_interrupt)
        {
            printf("\nTest interrupted by user\n");
            return false;
        }
    }

    printf("PASS: Large files test\n");
    return true;
}

static bool fat32_test_delete_operations()
{
    fat32_file_t file;
    fat32_file_t dir;

    printf("\n=== Delete Operations Test ===\n");

    if (fat32_set_current_dir("/tests") != FAT32_OK)
    {
        printf("FAIL: Cannot change to tests directory\n");
        return false;
    }

    // Create and delete a file
    if (fat32_create(&file, "delete_me.txt") != FAT32_OK &&
        fat32_open(&file, "delete_me.txt") != FAT32_OK)
    {
        printf("FAIL: Cannot create or open delete_me.txt\n");
        return false;
    }
    fat32_close(&file);

    if (fat32_delete("delete_me.txt") != FAT32_OK)
    {
        printf("FAIL: Cannot delete delete_me.txt\n");
        return false;
    }

    // Verify file is deleted
    if (fat32_open(&file, "delete_me.txt") == FAT32_OK)
    {
        printf("FAIL: delete_me.txt still exists after deletion\n");
        fat32_close(&file);
        return false;
    }

    // Create and delete a directory
    if (fat32_dir_create(&dir, "delete_dir") != FAT32_OK &&
        fat32_open(&dir, "delete_dir") != FAT32_OK)
    {
        printf("FAIL: Cannot create or open delete_dir\n");
        return false;
    }
    fat32_close(&dir);

    if (fat32_delete("delete_dir") != FAT32_OK)
    {
        printf("FAIL: Cannot delete delete_dir\n");
        return false;
    }

    // Verify directory is deleted
    if (fat32_open(&dir, "delete_dir") == FAT32_OK)
    {
        printf("FAIL: delete_dir still exists after deletion\n");
        fat32_close(&dir);
        return false;
    }

    printf("PASS: Delete operations test\n");
    return true;
}

void fat32test()
{
    printf("Comprehensive FAT32 File System Test\n");
    printf("====================================\n");
    printf("Sector size: 512 bytes\n");
    printf("Cluster size: 32 KiB (32768 bytes)\n");
    printf("Test directory: tests/\n\n");
    printf("Press BREAK to interrupt tests.\n\n");

    // Setup test environment
    if (!fat32_test_setup())
    {
        printf("\nFAT32 test setup FAILED!\n");
        return;
    }

    // Run basic operations test
    if (!fat32_test_basic_operations())
    {
        printf("\nFAT32 basic operations test FAILED!\n");
        printf("Check file system initialization.\n");
        return;
    }

    if (user_interrupt)
    {
        printf("\nTest suite interrupted by user.\n");
        return;
    }

    // Run sector boundary test
    if (!fat32_test_sector_boundaries())
    {
        printf("\nFAT32 sector boundary test FAILED!\n");
        printf("Check sector alignment handling.\n");
        return;
    }

    if (user_interrupt)
    {
        printf("\nTest suite interrupted by user.\n");
        return;
    }

    // Run cluster boundary test
    if (!fat32_test_cluster_boundaries())
    {
        printf("\nFAT32 cluster boundary test FAILED!\n");
        printf("Check cluster allocation logic.\n");
        return;
    }

    if (user_interrupt)
    {
        printf("\nTest suite interrupted by user.\n");
        return;
    }

    // Run many files test
    if (!fat32_test_many_files())
    {
        printf("\nFAT32 many files test FAILED!\n");
        printf("Check directory entry handling.\n");
        return;
    }

    if (user_interrupt)
    {
        printf("\nTest suite interrupted by user.\n");
        return;
    }

    // Run large files test
    if (!fat32_test_large_files())
    {
        printf("\nFAT32 large files test FAILED!\n");
        printf("Check large file handling.\n");
        return;
    }

    if (user_interrupt)
    {
        printf("\nTest suite interrupted by user.\n");
        return;
    }

    // Run delete operations test
    if (!fat32_test_delete_operations())
    {
        printf("\nFAT32 delete operations test FAILED!\n");
        printf("Check file/directory deletion logic.\n");
        return;
    }

    if (user_interrupt)
    {
        printf("\nTest suite interrupted by user.\n");
        return;
    }

    // Cleanup
    fat32_test_cleanup();

    printf("\n====================================\n");
    printf("All FAT32 tests PASSED!\n");
    printf("File system implementation verified.\n");
    printf("Tested:\n");
    printf("- Basic file/directory operations\n");
    printf("- Sector boundary conditions\n");
    printf("- Cluster boundary conditions\n");
    printf("- Multiple file creation\n");
    printf("- Various file sizes\n");
    printf("- Data integrity across boundaries\n");
}

// Song table for easy access
const test_t tests[] = {
    {"audio", audiotest, "Audio Driver Test"},
    {"display", displaytest, "Display Driver Test"},
    {"keyboard", keyboardtest, "Keyboard Driver Test"},
    {"fat32", fat32test, "FAT32 File System Test"},
    {NULL, NULL, NULL} // End marker
};

// Function to find a song by name
const test_t *find_test(const char *test_name)
{
    for (int i = 0; tests[i].name != NULL; i++)
    {
        if (strcmp(tests[i].name, test_name) == 0)
        {
            return &tests[i];
        }
    }
    return NULL; // Song not found
}

// Function to list all available songs
void show_test_library(void)
{
    printf("\033[?25l\033[4mTest Library\033[0m\n\n");

    for (int i = 0; tests[i].name != NULL; i++)
    {
        printf("  \033[1m%s\033[0m - %s\n", tests[i].name, tests[i].description);
    }
    printf("\033[?25h\n");
}
