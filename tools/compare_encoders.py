#!/usr/bin/env python3
"""Compare helloVFD's 240-bit scan encoding with mariosgit/VFD_MN12832L.

The reference model follows that project's public VFD_MN12832L.inl:
drawPixel() (gate = x // 6 + 1), nextGate(), and refresh() (24-byte
group at scan // 2, masked 0xAA/0x55, followed by six grid bytes).
It does not import or require Arduino, ESP-IDF, or the original library.
"""

FRAME_BYTES = 30
SCANS = 44
WIDTH = 128
HEIGHT = 32
ANODE_BIT = (0, 2, 4, 5, 3, 1)  # physical A B C D E F in afbecd order


def set_bit(data, index):
    data[index // 8] |= 0x80 >> (index % 8)


def current(fr):
    result = []
    for scan in range(SCANS):
        data = bytearray(FRAME_BYTES)
        if scan:
            set_bit(data, 192 + scan - 1)
        set_bit(data, 192 + scan)
        for y in range(HEIGHT):
            for col in range(3):
                x = scan * 3 + col
                if x < WIDTH and (x, y) in fr:
                    lane = col if scan % 2 == 0 else col + 3
                    set_bit(data, y * 6 + ANODE_BIT[lane])
        result.append(bytes(data))
    return result


def reference(fr):
    # The reference keeps the first six-column memory group empty: its
    # drawPixel() stores x=0..5 at group 1, not group 0.
    groups = [bytearray(24) for _ in range(23)]
    for x, y in fr:
        group = x // 6 + 1
        if group >= len(groups):
            continue
        set_bit(groups[group], y * 6 + ANODE_BIT[x % 6])

    result = []
    for scan in range(SCANS):
        data = bytearray(FRAME_BYTES)
        mask = 0x55 if scan % 2 else 0xAA
        source = groups[scan // 2]
        for i in range(24):
            data[i] = source[i] & mask
        if scan:
            set_bit(data, 192 + scan - 1)
        set_bit(data, 192 + scan)
        result.append(bytes(data))
    return result


def compare(label, pixels):
    ours = current(pixels)
    theirs = reference(pixels)
    mismatches = [i for i in range(SCANS) if ours[i] != theirs[i]]
    anode_mismatches = [i for i in range(SCANS)
                        if ours[i][:24] != theirs[i][:24]]
    grid_mismatches = [i for i in range(SCANS)
                       if ours[i][24:] != theirs[i][24:]]
    print(f"{label}: differing scans={len(mismatches)}/{SCANS}, "
          f"anode={len(anode_mismatches)}, grid={len(grid_mismatches)}")
    if label == "pixel (0,16)":
        for name, scans in (("ours", ours), ("reference", theirs)):
            active = [i for i, frame in enumerate(scans) if any(frame[:24])]
            print(f"  {name} anode-active scan(s): {active}")
            for i in active:
                print(f"    scan {i:02d}: {scans[i][:24].hex(' ')}")
    # Check whether it is purely a two-scan shift in the anode payload.
    shifted_matches = all(theirs[scan][:24] == ours[scan - 2][:24]
                          for scan in range(2, SCANS))
    print(f"  reference anodes = ours shifted +2 scans: {shifted_matches}")


if __name__ == "__main__":
    compare("all-off", set())
    compare("pixel (0,16)", {(0, 16)})
    compare("pixel (3,16)", {(3, 16)})
    compare("pixel (6,16)", {(6, 16)})
    compare("pixel (127,16)", {(127, 16)})
    compare("vertical stripe", {(9, y) for y in range(8, 24)})
    compare("checkerboard", {(x, y) for y in range(HEIGHT)
                             for x in range(WIDTH) if (x + y) % 2 == 0})
