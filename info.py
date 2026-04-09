import math

TARGETED_SAMPLE = "ACID"

THRESHOLD_TIMES = [20]

THRESHOLD_CARRY = 3

srlnums = ["SR0KEH","SR0VF6"]

MARKER_FACING = {

    **{i: 3 * math.pi / 2 for i in range(0, 5)},

    **{i: math.pi for i in range(5, 10)},

    **{i:  math.pi / 2 for i in range(10, 15)},

    **{i: 0 for i in range(15, 20)},
}

ZONE_FIDUCIAL_MARKERS = {

    0: [0,18,19],

    1: [3,4,5],

    2: [8,9,10],

    3: [13,14,15]

}

MARKER_TO_ZONE = {marker: zone for zone, markers in ZONE_FIDUCIAL_MARKERS.items() for marker in markers}

ZONE_BOUNDARIES = {
    0: [(0, 4575), (1000, 2575)],
    1:  [(2575, 4575), (4575, 3575)],
    2: [(3575, 2000), (4575, 0)],
    3: [(0, 0), (2000, 1000)]
}

PLATFORM_BOUNDARIES = [
    1677.5, 1677.5, 2897.5, 2897.5
]