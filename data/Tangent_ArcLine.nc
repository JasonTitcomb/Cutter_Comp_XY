(Forcing handleArcLine -> IT_TANGENT)
G90 G17 G21
G0 X-2.0 Y1.0
G1 F100.0

(Comp-in move, longer than tool radius)
G42 G1 X-0.7320508 Y1.0

(Steady line: A -> S)
G1 X1.0 Y0.0

(Steady arc: starts at S, CW, center at 0,0)
G2 X0.0 Y-1.0 I-1.0 J0.0

(Comp out)
G40 G1 X-1.0 Y-1.0