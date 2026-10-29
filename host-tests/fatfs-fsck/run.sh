#!/bin/sh
# clib.c and the FatFs disk layer (diskio.c) on a PC, against disk images,
# checked by fsck.fat (dosfstools) after each step: ./run.sh
# FatFs comes from the Pico SDK: PICO_SDK_PATH must be set.
set -e
cd "$(dirname "$0")"
: "${PICO_SDK_PATH:?set PICO_SDK_PATH to the Pico SDK}"
D=../../drivers
F=build/fatfs
mkdir -p $F
for f in ff.c ff.h ffunicode.c diskio.h; do cp "$PICO_SDK_PATH/lib/tinyusb/lib/fatfs/source/$f" $F/; done
cp $D/fatfs/ffconf.h $F/
CFLAGS="-O1 -g -Wall -Ifake -I$F -I$D"
# clib.c's functions would stand in for the PC's own: give them other names
R="-D_open=pc_open -D_close=pc_close -D_lseek=pc_lseek -D_read=pc_read -D_write=pc_write -D_fstat=pc_fstat -D_stat=pc_stat -D_link=pc_link -D_unlink=pc_unlink -Drename=pc_rename -Dmkdir=pc_mkdir"
gcc $CFLAGS -c $F/ff.c -o build/ff.o
gcc $CFLAGS -c $F/ffunicode.c -o build/ffunicode.o
gcc $CFLAGS -c $D/diskio.c -o build/diskio.o
gcc $CFLAGS -c $D/filesystem.c -o build/filesystem.o
gcc $CFLAGS $R -c $D/clib.c -o build/clib.o
gcc $CFLAGS -c sdimg.c -o build/sdimg.o
gcc $CFLAGS -c common.c -o build/common.o
O="build/ff.o build/ffunicode.o build/diskio.o build/filesystem.o build/clib.o build/sdimg.o build/common.o"
gcc $CFLAGS steps.c $O -o build/steps
gcc $CFLAGS random.c $O -o build/random
status=0
for spc in 1 64; do
	if [ $spc = 64 ]; then size=3G; echo "== 32 KB clusters"; else size=64M; echo "== 0.5 KB clusters"; fi
	rm -f build/test.img; truncate -s $size build/test.img; mkfs.fat -F 32 -s $spc build/test.img > /dev/null
	./build/steps build/test.img || status=1
	for seed in 1 2 3; do
		rm -f build/test.img; truncate -s $size build/test.img; mkfs.fat -F 32 -s $spc build/test.img > /dev/null
		./build/random build/test.img $seed | tail -1 || status=1
	done
done
rm -rf build fsck.out rfsck.out
exit $status
