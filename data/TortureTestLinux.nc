%
( Cutter comp torture test )
( Backplot without comp first, then with G41 / G42 )
( Intended to expose local gouge / corner / arc issues )

G20 G17 G40 G49 G80 G90
G54
T1 M6
S3000 M3
G0 X-0.500 Y-0.500
G43 H1 Z0.100
Z0.020

(lead-in long enough for 1/8 endmill)
G1 Z-0.050 F10.0
G1 X0.000 Y0.000 F20.0
G41 D1 X0.125 Y0.000

(1 - straight, then convex outside corner: should roll around)
G1 X1.000 Y0.000
G1 X1.000 Y1.000

(2 - tangent arc from line: should stay smooth)
G3 X0.500 Y1.500 I-0.500 J0.000

(3 - tangent back to line)
G1 X0.000 Y1.500

(4 - concave inside corner: trim or gouge depending on tool radius)
G1 X0.000 Y0.900
G1 X0.200 Y0.700

(5 - tiny inside fillet arc: often fails if comp math is weak)
G2 X0.400 Y0.500 I0.200 J0.000

(6 - narrow slot-like neck, likely impossible with larger tool radius)
G1 X0.700 Y0.500
G1 X0.700 Y0.650
G1 X0.300 Y0.650
G1 X0.300 Y0.350
G1 X0.850 Y0.350

(7 - arc smaller than tool radius candidate)
G3 X1.050 Y0.550 I0.000 J0.200
G1 X1.050 Y0.950

(8 - sharp inside return)
G1 X0.600 Y0.950
G1 X0.600 Y0.800
G1 X0.950 Y0.800

(9 - near loop / recross region)
G3 X1.250 Y0.500 I0.150 J-0.150
G1 X1.250 Y0.100
G1 X0.500 Y0.100
G1 X0.500 Y0.250
G1 X1.100 Y0.250

(10 - close out with another convex corner)
G1 X1.100 Y1.100
G1 X0.200 Y1.100

(cancel comp with exit move)
G40 G1 X-0.100 Y1.100
G0 Z0.100
M5
M30
%