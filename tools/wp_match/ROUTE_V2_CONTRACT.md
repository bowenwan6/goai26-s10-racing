# S10 route v2 contract (shared by WP matching, planner/router, simulation)

Decided with the user on 2026-09-19:
- Official course order starts at the LOW Start by the building door and climbs the B stairs.
  Photo time order is the REVERSE: official WP01 = photo #30 (IMG_7018 2.HEIC, at map ~(1.6,-0.2)),
  official WP30 = photo #1 (IMG_6988.HEIC, far/high end at map ~(-39,-1)).
- Native gaits available: flat (0x3002) and stairs (0x3003). Every non-flat hard section
  (B stairs, gabion ledge, artificial low platform / yellow bumps, rock creek bed) uses `stairs`,
  speed limit 0.15 m/s, `allow_detour: false`. Flat sections use `flat`, 0.20 m/s, detour allowed.
- Map: `0914_fr_v3-20260914-142008`, frame `map` (v3 original), metres. MuJoCo package uses the same frame.

## File: `route_v2.json`

```json
{
  "schema": "s10_route_v2",
  "map_id": "0914_fr_v3-20260914-142008",
  "frame": "map",
  "z_reference": "ground",
  "status": "DRAFT",
  "waypoints": [
    {
      "id": "WP01",
      "position": [x, y, z_ground],
      "yaw": 0.0,                 // rad, map frame, direction of travel leaving this WP; may be null
      "radius_xy": 0.20,
      "tol_z": 0.20,
      "terrain": "finish_plaza",  // free-text label from photo
      "confidence": "high"        // high | medium | low
    }
  ],
  "segments": [
    {
      "id": "WP01-WP02",
      "from": "WP01",
      "to": "WP02",
      "gait": "flat",             // flat | stairs  (gait used while driving INTO `to`)
      "speed_limit": 0.20,        // m/s forward cap
      "allow_detour": true,       // false => local planner may not leave the centerline band
      "corridor_half_width": 0.8, // metres of lateral deviation allowed from centerline (0.25 when allow_detour=false)
      "centerline": [[x, y, z_ground], ...]  // dense (<=0.25 m spacing) taught path from the mapping run, from `from` to `to`
    }
  ]
}
```

Rules:
- `segments[i].to == segments[i+1].from`; centerline first/last points are near the WP positions.
- Gait switch happens while stopped at the `from` WP of a segment whose gait differs from the previous segment
  (this matches the existing `kind` semantics in `native_transfer/config/start_b.draft.json`: kind = gait for the incoming segment).
- z is GROUND height. Robot base reference is roughly ground + body_z_offset (native config field); consumers add it.
- Unknown terrain is never free space.
