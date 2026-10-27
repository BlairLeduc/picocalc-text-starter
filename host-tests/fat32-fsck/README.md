# fat32 on disk images, checked by fsck.fat

A PC-only test of `drivers/fat32.c` against real FAT32 disk images, with
`fsck.fat` (from dosfstools) checking the image after every step. Where
`../fat32` tests the driver's own behaviour in RAM, this one asks the
filesystem's own checker whether what the driver left behind is right.

```
./run.sh
```

It makes a 64 MB image with 0.5 KB clusters and a 3 GB sparse image with
32 KB clusters, and runs on each:

- `steps`: what PicoCalc programs do to a card, a step at a time: folders
  and files with long names that are alike, a Diamond-style save (write a
  new file, delete the old one, rename the new one into place) many times,
  screenshots made and deleted, a small settings file rewritten, an empty
  file made, written to and deleted. After each step, fsck.fat looks; any
  problem is printed, then fixed so the next step starts clean.
- `random`: 30 rounds of random creates, writes, renames and deletes in a
  few folders, with seeds 1, 2 and 3, checked after each round.

`fake/` has stand-ins for the Pico SDK headers, and `sdimg.c` implements the
`sd_*` functions on the image file. Needs `mkfs.fat` and `fsck.fat`.
