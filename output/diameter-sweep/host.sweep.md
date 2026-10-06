# Host diameter sweep

Engine: Host C++ runner

Tool diameter: 0.0001 in to 1.0 in; increment: 1.123% (multiply by 1.01123); offset radius = diameter / 2. Chamfer corners, crossing lookahead enabled, merging disabled.

Increments executed counts diameter increases after the initial 0.0001 in attempt, including the increase to a failed attempt or the clamped 1.0 in endpoint. Total attempts = increments + 1.

Files processed: 23.

| File | Entry units | Final diameter (in) | Final diameter (mm) | Increments executed | Total attempts | Termination | First failed diameter (in) | Error code | Source line | Message |
| --- | --- | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| RapidComp.nc | inch | 1.000000000 | 25.400000000 | 825 | 826 | Maximum reached | - | 0 | 0 |  |
| G41_1.nc | mm | 0.021043782 | 0.534512054 | 480 | 481 | First error | 0.021280103 | 3 | 36 | Comp move too short |
| ThreadMill.nc | inch | 0.383804054 | 9.748622981 | 740 | 741 | First error | 0.388114174 | 4 | 23 | Arc smaller than tool radius |
| G41_2.nc | mm | 0.032167826 | 0.817062773 | 518 | 519 | First error | 0.032529070 | 3 | 5 | Comp move too short |
| TortureTestG91.nc | inch | 0.396880164 | 10.080756177 | 743 | 744 | First error | 0.401337129 | 3 | 12 | Comp move too short |
| Sample2.nc | inch | 0.396880164 | 10.080756177 | 743 | 744 | First error | 0.401337129 | 5 | 7 | Comp-in crossing |
| Sample3.nc | inch | 0.648719550 | 16.477476568 | 787 | 788 | First error | 0.656004670 | 5 | 7 | Comp-in crossing |
| Sample2mm.nc | mm | 0.750077736 | 19.051974488 | 800 | 801 | First error | 0.758501109 | 3 | 5 | Comp move too short |
| TortureTestmm.nc | mm | 0.198592474 | 5.044248849 | 681 | 682 | First error | 0.200822668 | 4 | 12 | Arc smaller than tool radius |
| _TortureTestG90.nc | mm | 0.007788945 | 0.197839212 | 391 | 392 | First error | 0.007876415 | 4 | 14 | Arc smaller than tool radius |
| TortureTestG90.nc | inch | 0.198592474 | 5.044248849 | 681 | 682 | First error | 0.200822668 | 4 | 12 | Arc smaller than tool radius |
| TortureTestG90LARGE.nc | inch | 1.000000000 | 25.400000000 | 825 | 826 | Maximum reached | - | 0 | 0 |  |
| TortureTestG90LARGE2X.nc | inch | 1.000000000 | 25.400000000 | 825 | 826 | Maximum reached | - | 0 | 0 |  |
| TortureTestG90SMALL.nc | inch | 0.016644660 | 0.422774371 | 459 | 460 | First error | 0.016831580 | 4 | 12 | Arc smaller than tool radius |
| ArcTooSmall.nc | inch | 0.248291409 | 6.306601780 | 701 | 702 | First error | 0.251079721 | 3 | 4 | Comp move too short |
| TortureTestLines.nc | inch | 0.219589866 | 5.577582606 | 690 | 691 | First error | 0.222055861 | 5 | 20 | Comp-in crossing |
| TortureTestSmallFilletsG91.nc | inch | 0.448754475 | 11.398363655 | 754 | 755 | First error | 0.453793987 | 3 | 7 | Comp move too short |
| SimpleSquarePocket.nc | mm | 0.015565937 | 0.395374796 | 453 | 454 | First error | 0.015740742 | 3 | 7 | Comp move too short |
| SimpleSquarePocketOverlap.nc | inch | 0.396880164 | 10.080756177 | 743 | 744 | First error | 0.401337129 | 3 | 7 | Comp move too short |
| CompErrorTest.nc | mm | 0.003897463 | 0.098995571 | 329 | 330 | First error | 0.003941232 | 6 | 12 | Comp-out crossing |
| PauseMarkers.nc | mm | 0.784343056 | 19.922313613 | 804 | 805 | First error | 0.793151228 | 3 | 5 | Comp move too short |
| MultipleZmoves.nc | mm | 0.784343056 | 19.922313613 | 804 | 805 | First error | 0.793151228 | 3 | 5 | Comp move too short |
| Comp_Err_out_Test.nc | mm | 0.003897463 | 0.098995571 | 329 | 330 | First error | 0.003941232 | 6 | 10 | Comp-out crossing |
