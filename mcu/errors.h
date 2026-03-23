
#ifndef ERRORS_H
#define ERRORS_H

typedef enum
{
    Status_OK = 0,
    Status_ArcRadiusMismatch = 101,
    Status_InvalidMove = 102,
    Status_MoveTooShort = 103,
    Status_ArcLtToolRad = 104,
    Status_FlippedArc = 105,
    Status_CompInCrossing = 106,
    Status_CompOutCrossing = 107,
    Status_UnresolvedGap = 108,
    Status_InputBufferOverflow = 109,
    Status_OutputBufferOverflow = 110
} status_code_t;

#endif