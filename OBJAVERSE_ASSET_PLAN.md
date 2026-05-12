# Objaverse / Realistic 3D Waste Asset Plan

## Goal

The project will use a realistic 3D waste asset library for conveyor-based waste sorting simulation.

Target base asset count:

| Class | Target Base Assets |
|---|---:|
| plastic | 25 |
| metal | 25 |
| paper | 25 |
| glass | 25 |
| **Total** | **100** |

The base assets will be expanded into 1500–2000 randomized simulation trials using:

- random position
- random yaw/rotation
- random scale
- random lighting
- random material variation
- randomized conveyor spawn timing
- randomized object order

This means the project does not require 2000 manually downloaded mesh files. Instead, 100 curated 3D models can generate thousands of simulation instances through domain randomization.

---

## Main Sources

Primary source:

- Objaverse-XL

Additional sources if needed:

- Sketchfab downloadable Creative Commons assets
- Free3D free OBJ/FBX/DAE assets
- ShapeNet bottle/can/container categories
- Other open 3D object datasets with compatible licenses

---

## License Rules

Every downloaded 3D asset must have metadata.

Allowed or preferred licenses:

- CC0
- Public Domain
- Creative Commons Attribution
- Research/academic use license, if the thesis use is allowed

Avoid:

- unclear license
- editorial-only license
- restricted redistribution
- paid assets that cannot be included in a public repository
- assets without attribution information

Each selected asset must be recorded in:

```text
ASSET_CREDITS.md


