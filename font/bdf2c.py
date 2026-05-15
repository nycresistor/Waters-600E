#!/usr/bin/env python3

import sys
import re
import math


# this is not a general parser, this is just for this one
# font, don't @ me

pat_enc = re.compile("ENCODING ([0-9]+)")
pat_bitmap = re.compile("BITMAP")

char_map = {}
data = []
max_encoding = 0

with open(sys.argv[1]) as bdf:
    encoding = None
    offset = 0
    while True:
        l = bdf.readline()
        if l == '':
            break
        if (m := pat_enc.match(l)) is not None:
            encoding = int(m.group(1))
            max_encoding = max(max_encoding, encoding)
        elif pat_bitmap.match(l):
            for i in range(16):
                val = int(bdf.readline().strip(),16)
                data.append(val)
            #print(f"enc {encoding} data {data}")
            char_map[encoding] = offset
            offset = offset + 16

flat = [-1] * (max_encoding+1)
for (e,o) in char_map.items():
    flat[e] = o

print("#include <stdint.h>")
print(f"#define MAX_ENCODING {max_encoding}\n")
print(f"static const int16_t font_enc_map[MAX_ENCODING+1] = {{ ")
print(", ".join(list(map(str,flat))))
print("};\n")
print(f"static const uint8_t font_data[{len(data)}] = {{ ");
print(", ".join(list(map(str,data))))
print("};")

            
            

            
