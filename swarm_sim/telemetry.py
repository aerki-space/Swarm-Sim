"""Stream simulation state as JSON over UDP, in the format the AERKI ground station reads.

One datagram per message, UTF-8 JSON. Schema: docs/telemetry.md (shared with the ground station).
"""
import json
import math
import socket
import time

from .geo import DEFAULT_ORIGIN, to_latlon

SCHEMA_VERSION = 1
ROLE = {'PA': 'radar', 'EO': 'thermal', 'AQ': 'sampler', 'CP': 'relay'}


def drone_message(sim, d, origin=DEFAULT_ORIGIN, alt=60.0):
    lat, lon = to_latlon(d.x, d.y, origin)
    track = sim.tracks[d.id]
    (px, py), (cx, cy) = (track[-2] if len(track) > 1 else track[-1]), track[-1]
    heading = (math.degrees(math.atan2(cx - px, cy - py)) + 360) % 360
    state = 'landed' if d.landed else ('confirming' if d.kind == 'AQ' and d.task is not None else
                                       ('surveying' if d.route else 'holding'))
    return {'v': SCHEMA_VERSION, 'type': 'drone', 't': round(sim.t, 2), 'id': f'{d.kind}-{d.id}',
            'role': ROLE[d.kind], 'lat': round(lat, 7), 'lon': round(lon, 7), 'alt': alt,
            'heading': round(heading, 1), 'battery': round(max(0.0, d.battery_wh / 60.0), 3), 'state': state}


def candidate_message(sim, c, origin=DEFAULT_ORIGIN):
    lat, lon = to_latlon(c.x, c.y, origin)
    status = 'confirmed' if c.confirmed else 'rejected' if c.rejected else (
        'checking' if c.assigned_to is not None else 'new')
    reporter = sim.drones[c.by]
    return {'v': SCHEMA_VERSION, 'type': 'detection', 't': round(c.t, 2), 'id': f'det-{c.id}',
            'by': f'{reporter.kind}-{reporter.id}', 'lat': round(lat, 7), 'lon': round(lon, 7),
            'kind': 'hotspot', 'status': status}


class UdpPublisher:
    """on_step hook: sends every drone each `every` seconds of sim time, and detections when they change."""

    def __init__(self, host='127.0.0.1', port=14560, every=1.0, realtime=1.0, origin=DEFAULT_ORIGIN):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.every, self.realtime, self.origin = every, realtime, origin
        self._next = 0.0
        self._sent = {}
        self._wall0 = None

    def send(self, msg):
        self.sock.sendto(json.dumps(msg, separators=(',', ':')).encode(), self.addr)

    def __call__(self, sim):
        if self._wall0 is None:
            self._wall0 = time.monotonic()
        if sim.t + 1e-9 < self._next:
            return
        self._next = sim.t + self.every
        for d in sim.drones:
            self.send(drone_message(sim, d, self.origin))
        for c in sim.candidates:
            m = candidate_message(sim, c, self.origin)
            if self._sent.get(m['id']) != m['status']:
                self._sent[m['id']] = m['status']
                self.send(m)
        if self.realtime > 0:   # pace the simulation against the wall clock
            ahead = sim.t / self.realtime - (time.monotonic() - self._wall0)
            if ahead > 0:
                time.sleep(ahead)
