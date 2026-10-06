# MCU diameter sweep

Engine: MCU C99 core / desktop grblHAL shim

Tool diameter: 0.0001 in to 1.0 in; increment: 1.123% (multiply by 1.01123); offset radius = diameter / 2. Chamfer corners, crossing lookahead enabled, merging disabled.

Increments executed counts diameter increases after the initial 0.0001 in attempt, including the increase to a failed attempt or the clamped 1.0 in endpoint. Total attempts = increments + 1.

Files processed: 23.

| File | Entry units | Final diameter (in) | Final diameter (mm) | Increments executed | Total attempts | Termination | First failed diameter (in) | Error code | Source line | Message |
| --- | --- | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- |
| RapidComp.nc | inch | 0.969739344 | 24.631379335 | 823 | 824 | First error | 0.980629517 | 105 | 0 | Crossing error: move in cutting area |
| G41_1.nc | mm | 0.021043782 | 0.534512054 | 480 | 481 | First error | 0.021280103 | 103 | 36 | Move too short to compensate |
| ThreadMill.nc | inch | 0.383804054 | 9.748622981 | 740 | 741 | First error | 0.388114174 | 104 | 16 | Arc radius less than tool radius |
| G41_2.nc | mm | 0.032167826 | 0.817062773 | 518 | 519 | First error | 0.032529070 | 103 | 5 | Move too short to compensate |
| TortureTestG91.nc | inch | 0.396880164 | 10.080756177 | 743 | 744 | First error | 0.401337129 | 103 | 12 | Move too short to compensate |
| Sample2.nc | inch | 0.396880164 | 10.080756177 | 743 | 744 | First error | 0.401337129 | 105 | 7 | Crossing error: move in cutting area |
| Sample3.nc | inch | 0.129916666 | 3.299883313 | 643 | 644 | First error | 0.131375630 | 104 | 7 | Arc radius less than tool radius |
| Sample2mm.nc | mm | 0.750077736 | 19.051974488 | 800 | 801 | First error | 0.758501109 | 103 | 5 | Move too short to compensate |
| TortureTestmm.nc | mm | 0.198592474 | 5.044248849 | 681 | 682 | First error | 0.200822668 | 104 | 10 | Arc radius less than tool radius |
| _TortureTestG90.nc | mm | 0.007788945 | 0.197839212 | 391 | 392 | First error | 0.007876415 | 104 | 10 | Arc radius less than tool radius |
| TortureTestG90.nc | inch | 0.198592474 | 5.044248849 | 681 | 682 | First error | 0.200822668 | 104 | 10 | Arc radius less than tool radius |
| TortureTestG90LARGE.nc | inch | 1.000000000 | 25.400000000 | 825 | 826 | Maximum reached | - | 0 | 0 |  |
| TortureTestG90LARGE2X.nc | inch | 1.000000000 | 25.400000000 | 825 | 826 | Maximum reached | - | 0 | 0 |  |
| TortureTestG90SMALL.nc | inch | 0.016644660 | 0.422774371 | 459 | 460 | First error | 0.016831580 | 105 | 0 | Crossing error: move in cutting area |
| ArcTooSmall.nc | inch | 0.248291409 | 6.306601780 | 701 | 702 | First error | 0.251079721 | 103 | 4 | Move too short to compensate |
| TortureTestLines.nc | inch | 0.217151258 | 5.515641947 | 689 | 690 | First error | 0.219589866 | 105 | 27 | Crossing error: move in cutting area |
| TortureTestSmallFilletsG91.nc | inch | 0.049724402 | 1.262999822 | 557 | 558 | First error | 0.050282807 | 104 | 44 | Arc radius less than tool radius |
| SimpleSquarePocket.nc | mm | 0.015565937 | 0.395374796 | 453 | 454 | First error | 0.015740742 | 103 | 7 | Move too short to compensate |
| SimpleSquarePocketOverlap.nc | inch | 0.396880164 | 10.080756177 | 743 | 744 | First error | 0.401337129 | 103 | 7 | Move too short to compensate |
| CompErrorTest.nc | mm | 0.003897463 | 0.098995571 | 329 | 330 | First error | 0.003941232 | 106 | 12 | Crossing error: move out of cutting area |
| PauseMarkers.nc | mm | 0.784343056 | 19.922313613 | 804 | 805 | First error | 0.793151228 | 103 | 5 | Move too short to compensate |
| MultipleZmoves.nc | mm | 0.784343056 | 19.922313613 | 804 | 805 | First error | 0.793151228 | 103 | 5 | Move too short to compensate |
| Comp_Err_out_Test.nc | mm | 0.003897463 | 0.098995571 | 329 | 330 | First error | 0.003941232 | 106 | 10 | Crossing error: move out of cutting area |
