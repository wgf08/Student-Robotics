import math

TARGETED_SAMPLE = "ACID"

THRESHOLD_TIMES = [20]

THRESHOLD_CARRY = 3

srlnums = ["srOKET1","srOVF6"]

MARKER_FACING = {

    **{i: 3 * math.pi / 2 for i in range(0, 5)},

    **{i: 0.0 for i in range(5, 10)},

    **{i:  math.pi / 2 for i in range(10, 15)},

    **{i: math.pi/2 for i in range(15, 20)},
}

ZONE_FIDUCIAL_MARKERS = {

    0: [0,18,19],

    1: [3,4,5],

    2: [8,9,10],

    3: [13,14,15]

}

MARKER_TO_ZONE = {marker: zone for zone, markers in ZONE_FIDUCIAL_MARKERS.items() for marker in markers}