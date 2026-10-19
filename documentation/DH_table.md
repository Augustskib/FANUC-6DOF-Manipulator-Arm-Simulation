# FANUC LR Mate 200iD: DH Parameters

Standard DH convention. Lengths in mm, angles in degrees.

| Link | θᵢ | dᵢ | aᵢ | αᵢ | Offset |
|------|----|----|----|----|--------|
| 1 | θ₁ | 330 | 50  | -90 | 0   |
| 2 | θ₂ | 0   | 330 | 0   | -90 |
| 3 | θ₃ | 0   | 35  | -90 | 0   |
| 4 | θ₄ | 335 | 0   | +90 | 0   |
| 5 | θ₅ | 0   | 0   | -90 | 0   |
| 6 | θ₆ | 80  | 0   | 0   | 0   |

- d₁ = 330: J2 axis height above base
- a₁ = 50: J1 to J2 horizontal offset
- a₂ = 330: upper arm (J2 to J3)
- a₃ = 35: J3 to forearm offset
- d₄ = 335: forearm (J3 to wrist center)
- d₆ = 80: wrist center to faceplate
- The -90° offset on J2 aligns FANUC's J2 = 0 (upper arm vertical) with the DH zero.
- Tool: 100 mm pen along flange z (`robot.tool = transl(0, 0, 100)`).

Link lengths from the FANUC LR Mate 200iD datasheet; frames assigned using the standard DH convention.

## Joint limits

| Joint | Min | Max |
|-------|------|------|
| J1 | -170 | 170 |
| J2 | -100 | 145 |
| J3 | -205 | 70 (215 in `ParameterCheck.m`) |
| J4 | -190 | 190 |
| J5 | -125 | 125 |
| J6 | -360 | 360 |

## MATLAB (Peter Corke Robotics Toolbox)

```matlab
L(1) = Link([0 330  50 -pi/2 0 0]);
L(2) = Link([0   0 330   0   0 -pi/2]);
L(3) = Link([0   0  35 -pi/2 0 0]);
L(4) = Link([0 335   0  pi/2 0 0]);
L(5) = Link([0   0   0 -pi/2 0 0]);
L(6) = Link([0  80   0   0   0 0]);
robot = SerialLink(L, 'name', 'FANUC LR Mate 200iD');
```
