# FatFs on disk images, checked by fsck.fat

A PC-only test of the parts of the file system that are this project's own:
`drivers/diskio.c` (FatFs's disk layer) and `drivers/clib.c` (the C
library's file functions on top of FatFs), with FatFs itself in between.
They run against real FAT32 disk images, and `fsck.fat` (from dosfstools)
checks the image after every step.

```
PICO_SDK_PATH=/path/to/pico-sdk ./run.sh
```

FatFs is taken from the Pico SDK, as the project's `CMakeLists.txt` takes
it, and built with `drivers/fatfs/ffconf.h`.

It makes a 64 MB image with 0.5 KB clusters and a 3 GB sparse image with
32 KB clusters, and runs on each:

- `steps`: what PicoCalc programs do to a card, a step at a time: folders
  and files with long names that are alike, a Diamond-style save (write a
  new file, rename the old one to `.bak`, rename the new one into place)
  many times, screenshots made and deleted, a file saved in place, a
  settings file deleted and made again, a log appended to, an empty file
  written to and emptied, a file made longer by seeking past its end, a
  folder emptied and deleted, and the card taken out and put back with a
  file open. It also checks file dates with and without a clock, and that
  a file can't be opened twice to write. After each step fsck.fat looks;
  any problem is printed, then fixed so the next step starts clean.
- `random`: 30 rounds of random creates, writes, appends, renames, deletes
  and reads in a few folders, with seeds 1, 2 and 3, with the contents
  checked and fsck.fat after each round.

Between steps the card is taken out and put back (`sd_set_present()`), so
FatFs mounts it afresh and reads what is really on the image.

Taking the card out with a file open loses that file's first cluster, as
on any FAT card: FatFs has taken the cluster but not yet written the file's
entry. The step allows that and nothing else.

`clib.c`'s functions are built with other names (`_open` as `pc_open` and
so on; see `run.sh`) so that they don't take the place of the PC's own C
library. `fake/` has stand-ins for the Pico SDK headers, and `sdimg.c`
implements the `sd_*` functions on the image file. Needs gcc, `mkfs.fat`
and `fsck.fat`.
