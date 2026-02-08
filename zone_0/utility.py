import math
import time
from movement import *
from info import * 

def start_timer():
    return time.time()

def time_left(start_time):
    time_elapsed = start_time-time.time()
    return 150-time_elapsed

def set_zone(robot):
    """
    Returns:
        A tuple containing the zone and the fiducial markers inside this zone
    """
    if robot.mode == "COMP":
        zone = robot.zone
    else:
        zone = 0
    return (zone, ZONE_FIDUCIAL_MARKERS[zone])

def sorted_boxes(markers):
    """
    Returns a list of boxes sorted by type and then distance. [samples, acids, bases, walls]
    """
    acids = []
    bases = []
    walls = []

    for marker in markers:
        if 0 <= marker.id <= 19:
            walls.append(marker)
        elif 100 <= marker.id <= 139:
            acids.append(marker)
        elif 140 <= marker.id <= 179:
            bases.append(marker)

    # Create samples by concatenating acids and bases
    samples = acids + bases

    # Sort all lists by marker.distance
    samples.sort(key=lambda m: m.position.distance)
    acids.sort(key=lambda m: m.position.distance)
    bases.sort(key=lambda m: m.position.distance)
    walls.sort(key=lambda m: m.position.distance)

    return [samples, acids, bases, walls]

def find_marker(markers, marker_id):
    """
    Returns a marker given its id
    """

    for marker in markers:
        if marker.id == marker_id:
            return marker
    return None

def find_position(markers):
    """
    Returns:
        (marker_id, distance_m, marker_angle)
    or None if no suitable marker is visible

    marker angle:
        betwen -pi and pi
    """

    #Get wall Fiducial Markers and sort by distance, taking the nearest marker
    markers = [m for m in markers if m.id in MARKER_FACING]
    if not markers:
        return None
    m = min(markers, key=lambda m: m.position.distance)
    md = m.position.distance
    ma = m.position.horizontal_angle

    # Robot facing angle in arena frame

    return m.id, md, ma


