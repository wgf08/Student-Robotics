import math
import time
from movement import *
from info import * 

def start_timer():
    return time.time()

def time_left(start_time):
    time_elapsed = time.time() - start_time
    return 180 - time_elapsed

def set_zone(robot):
    """
    Returns:
        A tuple containing the zone and the fiducial markers inside this zone
    """
    zone = (robot.zone, ZONE_FIDUCIAL_MARKERS[robot.zone])
    return zone

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


def assess_base_value(b_marker, markers):
    """
    Returns a int "value" representing the acidity/alkalinity of a base.
    Removes duplicate markers (same ID) before processing.
    """
    value = 0
    b_d = b_marker.position.distance
    if b_marker.id % 5 == 0:
        threshold_distance = b_d - 1900
        horizontal_threshold = b_marker.position.distance*math.sin(b_marker.position.horizontal_angle) + 100
    else:
        threshold_distance = b_d - 1100
        horizontal_threshold = None

    unique_markers = {m.id: m for m in markers}.values()

    for m in unique_markers:
        # Check if it's a sample marker (ID > 99) and within range
        cond1 = (b_marker.id%5 == 0 and m.position.horizontal_angle > 0.2 and m.position.distance > threshold_distance and (m.position.distance * math.sin(m.position.horizontal_angle)) < horizontal_threshold)
        cond2 = (not m.position.horizontal_angle > 0.2) and m.position.distance > threshold_distance
        if m.id > 99:
            if cond1 or cond2:
                print(f'ID: {m.id}, \n DIST: {m.position.distance} ANGLE {m.position.horizontal_angle}')
                value += sample_value(m)
            
    return value

def sample_value(marker):
    global TARGETED_SAMPLE
    if 99 < marker.id < 140 and TARGETED_SAMPLE == 'ACID' or (139 < marker.id < 200) and TARGETED_SAMPLE == 'BASIC':
        return 1
    if 99 < marker.id < 140 and TARGETED_SAMPLE == 'BASIC' or (139 < marker.id < 200) and TARGETED_SAMPLE == 'ACID':
        return -1

def wall_xyz(marker):
    """
    Returns:
        perpendicular_distance  (robot → wall, along wall normal)
        along_wall_distance     (robot position along wall)
    """
    d   = marker.position.distance
    ha  = marker.position.horizontal_angle
    yaw = marker.orientation.yaw

    # ha - yaw gives the angle between the marker vector and the wall normal
    perp  = d * math.cos(ha - yaw)   # component along wall normal
    along = d * math.sin(ha - yaw)   # component along wall surface

    return perp, along, 0 

def sample_xyz(marker):
    HALF_BOX = 65
    d   = marker.position.distance
    ha  = marker.position.horizontal_angle
    hv  = marker.position.vertical_angle
    yaw = marker.orientation.yaw

    # 3D position of the marker itself
    horiz = d * math.cos(hv)
    mx    = horiz * math.sin(ha)
    my    = horiz * math.cos(ha)
    mz    = d     * math.sin(hv)
    pitch = marker.orientation.pitch

    # offset back along the face normal to get box centre
    # (same logic as wall code — yaw defines the face normal direction)

    if abs(pitch) > 0.4:   # pitched significantly = top face
        return mx, my, mz - HALF_BOX

    cx = mx + HALF_BOX * math.sin(yaw)
    cy = my + HALF_BOX * math.cos(yaw)
    cz = mz  # assuming box sits flat, no vertical offset needed
    print('working')
    return cx, cy, cz

