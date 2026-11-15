# Telemetry format (v1)

Swarm-Sim streams, and the [AERKI ground station](https://github.com/aerki-space/ground-station-fleet-dashboard-)
reads, one UTF-8 JSON object per UDP datagram. The default port is 14560. Every message has `v` (the schema version,
currently 1), `type` and `t` (mission time in seconds).

The ground station accepts the same format from real vehicles. A small bridge on the companion computer turns
MAVLink and DroneCAN data into these messages.

## `drone`

Sent for every vehicle about once a second.

```json
{"v":1,"type":"drone","t":412.0,"id":"PA-0","role":"radar","lat":1.3591226,"lon":103.7769834,
 "alt":60.0,"heading":0.0,"battery":0.917,"state":"surveying"}
```

| Field | Type | Meaning |
|---|---|---|
| `id` | string | Unique vehicle ID |
| `role` | string | `radar`, `thermal`, `sampler`, `relay` (other values are shown as-is) |
| `lat`, `lon` | number | WGS-84 degrees |
| `alt` | number | m above launch |
| `heading` | number | degrees, 0 = north, clockwise |
| `battery` | number | 0 – 1, remaining fraction |
| `state` | string | `surveying`, `confirming`, `holding`, `landed` |

## `detection`

Sent when a detection appears or its status changes.

```json
{"v":1,"type":"detection","t":388.0,"id":"det-2","by":"PA-1","lat":1.3577,"lon":103.7801,
 "kind":"hotspot","status":"checking"}
```

| Field | Type | Meaning |
|---|---|---|
| `id` | string | Unique detection ID; later messages with the same ID update it |
| `by` | string | Vehicle that reported it |
| `kind` | string | `hotspot` today; `gas`, `radar` and others later |
| `status` | string | `new`, then `checking` (a sampler is on its way), then `confirmed` or `rejected` |

## Rules

- Receivers ignore unknown fields and unknown `type` values, so new fields can be added without breaking anyone.
- A breaking change increments `v`.
