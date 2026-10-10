# fat32 host test

A PC-only test of `drivers/fat32.c`. It needs no Pico SDK: `fat32_probe.c` formats a block
of RAM as a FAT32 volume, implements the `sd_*` functions on it, and runs four tests through
the public API. `stubs/` has empty stand-ins for the three Pico SDK headers that `fat32.c`
includes.

```
gcc -O1 -std=gnu99 -Istubs -I../../drivers fat32_probe.c ../../drivers/fat32.c -o fat32_probe
./fat32_probe
./fat32_probe -v    # also dumps the raw directory entries around a delete
```

The Pico toolchain has an unsigned `char` and a PC compiler has a signed one. Add
`-funsigned-char` to the command to see what the device does.

`lfn-bounds-instrument.patch` adds a check in front of the `lfn_to_str()` call in
`fat32_dir_read()` that prints a line when the write would fall outside the filename
buffer. Apply it to a scratch copy of an unfixed `fat32.c`, build with `-funsigned-char`
and run the probe to see the out-of-bounds write the fix removes.

The exit status is the number of failed tests. The last test (the cost of writing a
growing file) reports the cost of one 4 KB `fat32_write()` call at two positions in the file;
it fails by design while that cost grows with the position.
