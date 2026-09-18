# Sources

Visual meshes of [franka_description](https://github.com/frankarobotics/franka_description)
(Apache-2.0, Copyright 2023 Franka Robotics GmbH), commit `7aeeddc`, converted from Collada
to glTF and decimated to about 15% of their triangles (30% for the hand, the
finger whole, parts under 200 triangles kept) by `tools/franka-meshes`:

```sh
python3 tools/franka-meshes/convert.py <franka_description> crates/franka-description/meshes \
  --robots fer,fr3 --decimate 0.15
```

Deviation: distance of points sampled on either surface to the other, mm.

| mesh | original triangles | triangles | bytes | p99 | max |
|---|---|---|---|---|---|
| fer/link0.glb | 20483 | 3496 | 72644 | 0.58 | 1.87 |
| fer/link1.glb | 12516 | 1876 | 34860 | 0.29 | 0.68 |
| fer/link2.glb | 12716 | 1906 | 35396 | 0.30 | 0.57 |
| fer/link3.glb | 14233 | 2131 | 42088 | 1.00 | 1.49 |
| fer/link4.glb | 14621 | 2190 | 43156 | 0.97 | 1.82 |
| fer/link5.glb | 18327 | 2743 | 52940 | 0.43 | 1.15 |
| fer/link6.glb | 21620 | 3798 | 85972 | 0.50 | 1.23 |
| fer/link7.glb | 12082 | 1802 | 40056 | 0.79 | 1.42 |
| fer/hand.glb | 7078 | 2150 | 43564 | 0.76 | 2.51 |
| fer/finger.glb | 624 | 624 | 13308 | 0.00 | 0.16 |
| fr3/link0.glb | 63170 | 9468 | 182892 | 0.54 | 1.20 |
| fr3/link1.glb | 8028 | 1204 | 22788 | 0.45 | 1.07 |
| fr3/link2.glb | 8028 | 1204 | 22784 | 0.46 | 0.87 |
| fr3/link3.glb | 21566 | 3233 | 61152 | 0.60 | 1.29 |
| fr3/link4.glb | 21566 | 3233 | 61176 | 0.64 | 1.33 |
| fr3/link5.glb | 27145 | 4069 | 78516 | 0.39 | 0.70 |
| fr3/link6.glb | 44837 | 7040 | 139428 | 0.21 | 0.60 |
| fr3/link7.glb | 37958 | 5691 | 108408 | 0.63 | 2.48 |
| fr3/hand.glb | 7078 | 2150 | 43564 | 0.76 | 2.51 |
| fr3/finger.glb | 624 | 624 | 13308 | 0.00 | 0.16 |
