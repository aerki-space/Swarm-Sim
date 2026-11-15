"""Local east/north metres <-> WGS-84, flat-earth approximation (fine over a few km)."""
import math

EARTH_M_PER_DEG = 111_320.0
DEFAULT_ORIGIN = (1.3540, 103.7760)    # lat, lon of the area's south-west corner (Bukit Timah Nature Reserve, SG)


def to_latlon(x, y, origin=DEFAULT_ORIGIN):
    lat0, lon0 = origin
    return lat0 + y / EARTH_M_PER_DEG, lon0 + x / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))


def to_local(lat, lon, origin=DEFAULT_ORIGIN):
    lat0, lon0 = origin
    return (lon - lon0) * EARTH_M_PER_DEG * math.cos(math.radians(lat0)), (lat - lat0) * EARTH_M_PER_DEG
