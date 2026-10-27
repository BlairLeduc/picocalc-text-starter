#!/bin/sh
# The FAT32 driver on a PC, against disk images, checked by fsck.fat
# (dosfstools) after each step: ./run.sh
set -e
cd "$(dirname "$0")"
D=../../drivers
gcc -O1 -g -Ifake -I$D steps.c $D/fat32.c sdimg.c -o steps
gcc -O1 -g -Ifake -I$D random.c $D/fat32.c sdimg.c -o random
for spc in 1 64; do
	rm -f test.img
	if [ $spc = 64 ]; then truncate -s 3G test.img; else truncate -s 64M test.img; fi
	mkfs.fat -F 32 -s $spc test.img > /dev/null
	echo "== $((spc / 2)).5 KB clusters" | sed 's/0\.5/0.5/; s/32\.5/32/'
	./steps test.img
	for seed in 1 2 3; do
		rm -f test.img
		if [ $spc = 64 ]; then truncate -s 3G test.img; else truncate -s 64M test.img; fi
		mkfs.fat -F 32 -s $spc test.img > /dev/null
		./random test.img $seed | tail -1
	done
done
rm -f test.img steps random fsck.out fsck.rc rfsck.out
