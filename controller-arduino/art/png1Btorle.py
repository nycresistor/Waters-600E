#!/usr/bin/env python3

import sys
from PIL import Image

i = Image.open(sys.argv[1])
if len(sys.argv) > 2:
    prefix = sys.argv[2] + "_"
else:
    prefix = "IMAGE_"

(w,h) = i.size

boundaries = (w, h, 0, 0)
print(f"Scanning for crop boundaries in {w}x{h} image", file=sys.stderr)


foundy = False
for scany in range(h):
    for scanx in range(w):
        if i.getpixel((scanx, scany)) != 0:
            boundaries = (
                min(scanx, boundaries[0]),
                min(scany, boundaries[1]),
                max(scanx, boundaries[2]),
                max(scany, boundaries[3]))

# The RLE format is as follows:
# high bit 0: this run is a 15-bit value
# high bit 1: this run is a 7-bit value
# If the value is 0x7fff: interpret next value as an extension of
# the current value
# (The following value can be 0)
# Pixel color starts at 0
# (A leading 0 means switch to 1)
def encode_run_length(rl):
    if (rl <= 0x7f):
        return [0x80 | rl]
    if (rl < 0x7fff):
        return [rl >> 8, rl & 0xff]
    else:
        return [0x7fff].extend(encode_run_length(rl - 0x7fff))


print(f"Boundaries f{boundaries}", file=sys.stderr)
if (boundaries[0] > boundaries[2]):
    print("Invalid boundaries/empty image, aborting",file=sys.stderr)

pos = (boundaries[0], boundaries[1])
sz = (1+boundaries[2]-boundaries[0],1+boundaries[3]-boundaries[1])

print("#include <stdint.h>")
print("#include \"bitmaps.h\"")
print(f"const uint16_t {prefix}POS[2] = {{ {pos[0]}, {pos[1]} }};")
print(f"const uint16_t {prefix}SZ[2] = {{ {sz[0]}, {sz[1]} }};")
data=[]
cur_val = 0
cur_run = 0
for y in range(boundaries[1],boundaries[3]+1):
    for x in range(boundaries[0],boundaries[2]+1):
        px = i.getpixel((x,y))
        if px == cur_val:
            cur_run += 1
        else:
            cur_val = px
            data.extend(encode_run_length(cur_run))
            cur_run = 1

# We can actually just leave the last run off :)
print(f"Data size: {len(data)} bytes",file=sys.stderr)

print(f"const uint8_t {prefix}DATA [{len(data)}] = {{")
BPL = 20
d = data
while d:
    line,d = d[0:BPL], d[BPL:]
    line = ",".join(map(hex,line))
    print(f"{line},")
print("};")

print(f"RLEBitmap {prefix}bitmap({sz[0]},{sz[1]},{pos[0]},{pos[1]},{prefix}DATA,{len(data)});")

# Reminder to self: YOU DO NOT NEED TO BE AS GOOD AS PNG,
# CUT YOURSELF SOME SLACK
#print(data)

def test(sz,data):
    outim = Image.new(mode="RGB",size=sz)
    x = 0
    y = 0
    partial = -1
    cur = 0
    for d in data:
        if partial != -1:
            rl = (partial << 8) | d
            partial = -1
        elif (d & 0x80) == 0:
            partial = d
            continue
        else:
            rl = d & 0x7f
        if cur == 0:
            color = (0,0,0)
            cur = 1
        else:
            color = (0xff,0,0)
            cur = 0
        while rl > 0:
            outim.putpixel((x,y),color)
            x += 1
            if x >= sz[0]:
                x = 0
                y += 1
            rl -= 1
    while y < sz[1] and x < sz[0]:
        outim.putpixel((x,y),(0,0xff,0))
        x += 1
        if x >= sz[0]:
            x = 0
            y += 1
    outim.show()

#test(sz,data)
