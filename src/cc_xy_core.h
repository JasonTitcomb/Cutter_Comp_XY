/*
 * cc_xy_core.h
 * Jason Titcomb 2026
 * MIT License - see LICENSE file in repository root
 *
 * Thin grblHAL-oriented runner for the standalone cc_xy engine.
 * Uses fixed-size callbacks and move structs only.
 */

#pragma once

#include "cc_xy.h"

typedef void (*CcXyCoreEmitMoveCB)(const CcXyMove2D *move);
typedef void (*CcXyCoreErrorCB)(CcXyCompError err, uint32_t seqNum);

typedef struct
{
	struct
	{
		CcXyCoreEmitMoveCB emitMove;
		CcXyCoreErrorCB error;
	} callbacks;
	float toolRadius;
	CcXyUnits units;
} CcXyCoreOptions;

typedef struct
{
	CcXyContext engine;
	CcXyCoreEmitMoveCB emitMoveCB;
	CcXyCoreErrorCB errorCB;
} CcXyCoreRunner;

static inline void ccxy_core_reset(CcXyCoreRunner *runner)
{
	runner->engine.havePrevMove = false;
	runner->engine.inHead = 0;
	runner->engine.inCount = 0;
	runner->engine.outHead = 0;
	runner->engine.outCount = 0;
	runner->engine.hasCompError = false;
	runner->engine.lastError = CCXY_CE_ERROR;
	runner->engine.lastSeqNum = 0;
}

static inline void ccxy_core_drain(CcXyCoreRunner *runner)
{
	CcXyMove2D out;
	if (!runner->emitMoveCB)
		return;
	while (ccxy_pop_out(&runner->engine, &out))
		runner->emitMoveCB(&out);
}

static inline bool ccxy_core_report_error(CcXyCoreRunner *runner)
{
	if (runner->errorCB)
		runner->errorCB(runner->engine.lastError, runner->engine.lastSeqNum);
	return false;
}

static inline bool ccxy_core_begin(CcXyCoreRunner *runner, const CcXyCoreOptions *options)
{
	CcXyOptions engineOptions;
	engineOptions.toolRadius = 0.0f;
	engineOptions.error = (CcXyErrorCB)0;
	if (options)
	{
		engineOptions.toolRadius = options->toolRadius;
		runner->emitMoveCB = options->callbacks.emitMove;
		runner->errorCB = options->callbacks.error;
	}
	else
	{
		runner->emitMoveCB = (CcXyCoreEmitMoveCB)0;
		runner->errorCB = (CcXyCoreErrorCB)0;
	}
	ccxy_init(&runner->engine, &engineOptions);
	ccxy_set_units(&runner->engine, options ? options->units : CCXY_UNITS_MM);
	return true;
}

static inline void ccxy_core_set_comp(CcXyCoreRunner *runner, CcXyCompSide side)
{
	ccxy_set_comp(&runner->engine, side);
}

static inline void ccxy_core_set_units(CcXyCoreRunner *runner, CcXyUnits units)
{
	ccxy_set_units(&runner->engine, units);
}

static inline void ccxy_core_set_tool_radius(CcXyCoreRunner *runner, float radius)
{
	ccxy_set_tool_radius(&runner->engine, radius);
}

static inline bool ccxy_core_process_move(CcXyCoreRunner *runner, const CcXyMove2D *move)
{
	if (!ccxy_push_in(&runner->engine, move))
		return ccxy_core_report_error(runner);
	if (!ccxy_process(&runner->engine))
		return ccxy_core_report_error(runner);
	ccxy_core_drain(runner);
	return !runner->engine.hasCompError;
}

static inline bool ccxy_core_flush(CcXyCoreRunner *runner)
{
	ccxy_flush(&runner->engine);
	if (runner->engine.hasCompError)
		return ccxy_core_report_error(runner);
	ccxy_core_drain(runner);
	return true;
}

static inline bool ccxy_core_end(CcXyCoreRunner *runner)
{
	return ccxy_core_flush(runner);
}