# sdcard host test

A PC-only test of how `drivers/sdcard.c` copes with a card that is busy programming a
block. `sd_busy_probe.c` contains a small model of an SD card in SPI mode with a simulated
clock: after each write it holds MISO low for a chosen time and ignores everything clocked
in meanwhile. `sdcard.c` runs unchanged on top of it, with the Pico SDK calls replaced by
the functions in the same file (`stubs/` has the two headers). The program writes 100 blocks
and reads them back, for several busy times and SPI clocks, and prints how many writes
failed, how many reads failed and how many blocks hold the wrong data.

```
gcc -O1 -std=gnu99 -Istubs -I../../drivers sd_busy_probe.c ../../drivers/sdcard.c -o sd_busy_probe
./sd_busy_probe
```

How long a real card is busy, and what one `spi_write_read_blocking()` call costs besides
the bits on the wire (`SPI_CALL_US`, 2 microseconds), are assumptions. The test shows the
mechanism, not the timing of any particular card.
