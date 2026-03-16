/*
 * cc_xy.h
 * Jason Titcomb 2026
 * MIT License - see LICENSE file in repository root
 *
 * Minimal standalone 2D cutter compensation engine intended for grblHAL-style
 * integration. This version performs only local pairwise resolution between
 * adjacent moves. It does not do any global trim or look-ahead.
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef CCXY_TOL
#define CCXY_TOL 0.0001f
#endif

#ifndef CCXY_ARC_TOL_IN
#define CCXY_ARC_TOL_IN 0.0005f
#endif

#ifndef CCXY_GAP_TOL_IN
#define CCXY_GAP_TOL_IN 0.001f
#endif

#ifndef CCXY_EPS
#define CCXY_EPS 1e-7f
#endif

#ifndef CCXY_PARALLEL_TOL
#define CCXY_PARALLEL_TOL 1e-3f
#endif

#ifndef CCXY_BEVEL_VEC_TOL
#define CCXY_BEVEL_VEC_TOL 1.0e-1f
#endif

#ifndef CCXY_PI
#define CCXY_PI 3.14159265358979323846f
#endif

#ifndef CCXY_TWO_PI
#define CCXY_TWO_PI 6.2831853071795864769f
#endif

#ifndef CCXY_MAX_SWEEP_DEG
#define CCXY_MAX_SWEEP_DEG 359.9f
#endif

#ifndef CCXY_MIN_ARC_LEN
#define CCXY_MIN_ARC_LEN 0.001f
#endif

#ifndef CCXY_IN_CAP
#define CCXY_IN_CAP 2
#endif

#ifndef CCXY_OUT_CAP
#define CCXY_OUT_CAP 2
#endif

#ifndef CCXY_INSERT_CAP
#define CCXY_INSERT_CAP 1
#endif

#define CCXY_MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct
{
	float x;
	float y;
} CcXyVec2;

typedef enum
{
	CCXY_CM_NONE = 0,
	CCXY_CM_IN = 1,
	CCXY_CM_STEADY = 2,
	CCXY_CM_OUT = 3
} CcXyCompMode;

typedef enum
{
	CCXY_MOT_EMPTY = 0,
	CCXY_MOT_RAPID = 1,
	CCXY_MOT_LINE = 2,
	CCXY_MOT_ARC = 3
} CcXyMotionType;

typedef enum
{
	CCXY_ARC_CW = 0,
	CCXY_ARC_CCW = 1
} CcXyArcDir;

typedef enum
{
	CCXY_IT_NONE = 0,
	CCXY_IT_TANGENT = 1,
	CCXY_IT_INTERSECT = 2
} CcXyIntersectType;

typedef enum
{
	CCXY_COMP_OFF = 0,
	CCXY_COMP_LEFT = 1,
	CCXY_COMP_RIGHT = -1
} CcXyCompSide;

typedef enum
{
	CCXY_CE_ERROR = 0,
	CCXY_CE_ARC_RADIUS_MISMATCH,
	CCXY_CE_INVALID_MOVE,
	CCXY_CE_COMP_MOVE_TOO_SHORT,
	CCXY_CE_ARC_LT_TOOL_RAD,
	CCXY_CE_FLIPPED_ARC,
	CCXY_CE_COMP_IN_CROSSING,
	CCXY_CE_COMP_OUT_CROSSING,
	CCXY_CE_UNRESOLVED_GAP,
	CCXY_CE_OUTPUT_BUFFER_OVERFLOW
} CcXyCompError;

typedef enum
{
	CCXY_UNITS_MM = 0,
	CCXY_UNITS_INCH = 1
} CcXyUnits;

typedef struct
{
	CcXyVec2 p_0;
	CcXyVec2 p_1;
	CcXyVec2 center;
	CcXyVec2 startDir;
	CcXyVec2 endDir;
	float radius;
	float feed;
	float z_0;
	float z_1;
	uint32_t seqNum;
	uint8_t type;
	uint8_t arcDir;
	uint8_t compMode;
	bool hasXY;
	bool hasZ;
	bool valid;
} CcXyMove2D;

typedef void (*CcXyErrorCB)(const char *message, CcXyCompError err, uint32_t seqNum);

typedef struct
{
	float toolRadius;
	CcXyErrorCB error;
} CcXyOptions;

typedef enum
{
	CCXY_JT_NONE = 0,
	CCXY_JT_TRIM_TO_INTERSECTION,
	CCXY_JT_EXTEND_TO_INTERSECTION,
	CCXY_JT_ROLL_AROUND,
	CCXY_JT_GOUGE
} CcXyJunctionType;

typedef struct
{
	CcXyJunctionType type;
	CcXyVec2 p;
} CcXyJunction;

typedef struct
{
	float a0;
	float a1;
	uint8_t dir;
} CcXyArcAngles;

typedef struct
{
	CcXyErrorCB errorCB;
	CcXyUnits units;
	bool hasCompError;
	CcXyCompError lastError;
	uint32_t lastSeqNum;
	float toolR;
	int8_t toolSign;
	float arcTol;
	float gapTol;
	CcXyCompSide compState;
	bool havePrevMove;
	CcXyMove2D prevOff;
	CcXyMove2D input_buffer[CCXY_IN_CAP];
	int inHead;
	int inCount;
	CcXyMove2D output_buffer[CCXY_OUT_CAP];
	int outHead;
	int outCount;
} CcXyContext;

static inline float ccxy_clamp(float x, float lo, float hi)
{
	return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline CcXyVec2 ccxy_v2(float x, float y)
{
	CcXyVec2 v;
	v.x = x;
	v.y = y;
	return v;
}

static inline CcXyVec2 ccxy_add(CcXyVec2 a, CcXyVec2 b)
{
	return ccxy_v2(a.x + b.x, a.y + b.y);
}

static inline CcXyVec2 ccxy_sub(CcXyVec2 a, CcXyVec2 b)
{
	return ccxy_v2(a.x - b.x, a.y - b.y);
}

static inline CcXyVec2 ccxy_scale(CcXyVec2 v, float s)
{
	return ccxy_v2(v.x * s, v.y * s);
}

static inline float ccxy_dot(CcXyVec2 a, CcXyVec2 b)
{
	return a.x * b.x + a.y * b.y;
}

static inline float ccxy_cross(CcXyVec2 a, CcXyVec2 b)
{
	return a.x * b.y - a.y * b.x;
}

static inline float ccxy_len(CcXyVec2 v)
{
	return sqrtf(ccxy_dot(v, v));
}

static inline float ccxy_dist(CcXyVec2 a, CcXyVec2 b)
{
	return ccxy_len(ccxy_sub(a, b));
}

static inline CcXyVec2 ccxy_normalize(CcXyVec2 v)
{
	float l = ccxy_len(v);
	if (l < CCXY_TOL)
		return ccxy_v2(0.0f, 0.0f);
	return ccxy_v2(v.x / l, v.y / l);
}

static inline CcXyVec2 ccxy_left_normal(CcXyVec2 v)
{
	return ccxy_v2(-v.y, v.x);
}

static inline CcXyVec2 ccxy_right_normal(CcXyVec2 v)
{
	return ccxy_v2(v.y, -v.x);
}

static inline float ccxy_wrap2pi(float a)
{
	a = fmodf(a, CCXY_TWO_PI);
	if (a < 0.0f)
		a += CCXY_TWO_PI;
	return a;
}

static inline float ccxy_angle_norm(float a)
{
	while (a < 0.0f)
		a += CCXY_TWO_PI;
	while (a >= CCXY_TWO_PI)
		a -= CCXY_TWO_PI;
	return a;
}

static inline float ccxy_sweep_ccw(float a0, float a1)
{
	float d;
	a0 = ccxy_angle_norm(a0);
	a1 = ccxy_angle_norm(a1);
	d = a1 - a0;
	if (d < 0.0f)
		d += CCXY_TWO_PI;
	return d;
}

static inline float ccxy_sweep_cw(float a0, float a1)
{
	return ccxy_sweep_ccw(a1, a0);
}

static inline bool ccxy_angle_on_sweep_ccw(float a0, float a1, float ap)
{
	a0 = ccxy_angle_norm(a0);
	a1 = ccxy_angle_norm(a1);
	ap = ccxy_angle_norm(ap);
	if (a0 <= a1)
		return (ap + CCXY_EPS >= a0) && (ap <= a1 + CCXY_EPS);
	return (ap >= a0 - CCXY_EPS) || (ap <= a1 + CCXY_EPS);
}

static inline bool ccxy_angle_on_sweep_cw(float a0, float a1, float ap)
{
	return ccxy_angle_on_sweep_ccw(a1, a0, ap);
}

static inline int ccxy_get_winding_dir(CcXyVec2 a, CcXyVec2 b)
{
	float z = ccxy_cross(a, b);
	if (z > CCXY_TOL)
		return 1;
	if (z < -CCXY_TOL)
		return -1;
	return 0;
}

static inline bool ccxy_is_near(CcXyVec2 a, CcXyVec2 b)
{
	CcXyVec2 d = ccxy_sub(a, b);
	return ccxy_dot(d, d) <= CCXY_TOL * CCXY_TOL;
}

static inline void ccxy_update_vectors(CcXyMove2D *m)
{
	if (m->type == CCXY_MOT_LINE || m->type == CCXY_MOT_RAPID)
	{
		CcXyVec2 d = ccxy_sub(m->p_1, m->p_0);
		CcXyVec2 u = ccxy_normalize(d);
		m->startDir = u;
		m->endDir = u;
		return;
	}

	if (m->type == CCXY_MOT_ARC)
	{
		CcXyVec2 rs = ccxy_normalize(ccxy_sub(m->p_0, m->center));
		CcXyVec2 re = ccxy_normalize(ccxy_sub(m->p_1, m->center));
		if (m->arcDir == CCXY_ARC_CCW)
		{
			m->startDir = ccxy_left_normal(rs);
			m->endDir = ccxy_left_normal(re);
		}
		else
		{
			m->startDir = ccxy_right_normal(rs);
			m->endDir = ccxy_right_normal(re);
		}
		return;
	}

	m->startDir = ccxy_v2(0.0f, 0.0f);
	m->endDir = ccxy_v2(0.0f, 0.0f);
}

static inline CcXyVec2 ccxy_original_endpoint(CcXyVec2 p_offset, CcXyVec2 dir, bool useLeft, float toolR)
{
	CcXyVec2 normal = useLeft ? ccxy_left_normal(dir) : ccxy_right_normal(dir);
	return ccxy_sub(p_offset, ccxy_scale(normal, toolR));
}

static inline bool ccxy_is_radius_consistent(const CcXyContext *ctx, const CcXyMove2D *m)
{
	float r0 = ccxy_len(ccxy_sub(m->p_0, m->center));
	float r1 = ccxy_len(ccxy_sub(m->p_1, m->center));
	return fabsf(r0 - r1) <= ctx->arcTol;
}

static inline float ccxy_arc_sweep_deg(const CcXyMove2D *m)
{
	float a0 = ccxy_wrap2pi(atan2f(m->p_0.y - m->center.y, m->p_0.x - m->center.x));
	float a1 = ccxy_wrap2pi(atan2f(m->p_1.y - m->center.y, m->p_1.x - m->center.x));
	float sw;

	if (m->arcDir == CCXY_ARC_CCW)
	{
		sw = a1 - a0;
		if (sw < 0.0f)
			sw += CCXY_TWO_PI;
	}
	else
	{
		sw = a0 - a1;
		if (sw < 0.0f)
			sw += CCXY_TWO_PI;
	}

	return sw * (180.0f / CCXY_PI);
}

static inline float ccxy_line_t(const CcXyMove2D *m, CcXyVec2 p)
{
	CcXyVec2 d = ccxy_sub(m->p_1, m->p_0);
	float l2 = ccxy_dot(d, d);
	if (l2 < 1e-12f)
		return 0.0f;
	return ccxy_dot(ccxy_sub(p, m->p_0), d) / l2;
}

static inline float ccxy_dist_from_start_along(const CcXyMove2D *m, CcXyVec2 p)
{
	if (m->type == CCXY_MOT_LINE || m->type == CCXY_MOT_RAPID)
	{
		float t = ccxy_line_t(m, p);
		t = ccxy_clamp(t, 0.0f, 1.0f);
		return ccxy_len(ccxy_sub(m->p_1, m->p_0)) * t;
	}

	if (m->type == CCXY_MOT_ARC)
	{
		float a0 = atan2f(m->p_0.y - m->center.y, m->p_0.x - m->center.x);
		float ap = atan2f(p.y - m->center.y, p.x - m->center.x);
		float sw = (m->arcDir == CCXY_ARC_CCW) ? ccxy_sweep_ccw(a0, ap) : ccxy_sweep_cw(a0, ap);
		return fabsf(m->radius) * sw;
	}

	return 0.0f;
}

static inline bool ccxy_point_on_segment(CcXyVec2 a, CcXyVec2 b, CcXyVec2 p)
{
	CcXyVec2 ab = ccxy_sub(b, a);
	float lab2 = ccxy_dot(ab, ab);
	float t;
	float d;

	if (lab2 < CCXY_TOL)
		return ccxy_len(ccxy_sub(p, a)) < CCXY_TOL;

	t = ccxy_dot(ccxy_sub(p, a), ab) / lab2;
	if (t < -CCXY_TOL || t > 1.0f + CCXY_TOL)
		return false;

	d = fabsf(ccxy_cross(ccxy_sub(p, a), ab)) / sqrtf(lab2);
	return d < CCXY_TOL;
}

static inline CcXyArcAngles ccxy_precompute_arc_angles(const CcXyMove2D *m)
{
	CcXyArcAngles aa;
	aa.a0 = atan2f(m->p_0.y - m->center.y, m->p_0.x - m->center.x);
	aa.a1 = atan2f(m->p_1.y - m->center.y, m->p_1.x - m->center.x);
	aa.dir = m->arcDir;
	return aa;
}

static inline bool ccxy_point_on_arc_cached(const CcXyMove2D *a, CcXyVec2 p, const CcXyArcAngles *aa)
{
	float rp = ccxy_len(ccxy_sub(p, a->center));
	float ap;

	if (fabsf(rp - fabsf(a->radius)) > CCXY_TOL)
		return false;

	ap = atan2f(p.y - a->center.y, p.x - a->center.x);
	if (aa->dir == CCXY_ARC_CCW)
		return ccxy_angle_on_sweep_ccw(aa->a0, aa->a1, ap);
	return ccxy_angle_on_sweep_cw(aa->a0, aa->a1, ap);
}

static inline CcXyIntersectType ccxy_intersect_line_line(const CcXyMove2D *ln1, const CcXyMove2D *ln2, CcXyVec2 *ip, bool *tip)
{
	CcXyVec2 p = ln1->p_0;
	CcXyVec2 r = ccxy_sub(ln1->p_1, ln1->p_0);
	CcXyVec2 q = ln2->p_0;
	CcXyVec2 s = ccxy_sub(ln2->p_1, ln2->p_0);
	float lr = ccxy_len(r);
	float ls = ccxy_len(s);
	float den;
	float denTol;
	float t;
	float u;

	if (lr < CCXY_TOL || ls < CCXY_TOL)
	{
		*tip = false;
		return CCXY_IT_NONE;
	}

	den = ccxy_cross(r, s);
	denTol = CCXY_PARALLEL_TOL * lr * ls;
	if (fabsf(den) <= denTol)
	{
		*tip = false;
		return CCXY_IT_NONE;
	}

	t = ccxy_cross(ccxy_sub(q, p), s) / den;
	u = ccxy_cross(ccxy_sub(q, p), r) / den;
	*ip = ccxy_add(p, ccxy_scale(r, t));
	*tip = (t >= -CCXY_TOL && t <= 1.0f + CCXY_TOL && u >= -CCXY_TOL && u <= 1.0f + CCXY_TOL);
	return CCXY_IT_INTERSECT;
}

static inline CcXyIntersectType ccxy_intersect_circle_circle(const CcXyMove2D *a1, const CcXyMove2D *a2, CcXyVec2 *p1, CcXyVec2 *p2, int *count)
{
	CcXyVec2 c0 = a1->center;
	CcXyVec2 c1 = a2->center;
	float r0 = fabsf(a1->radius);
	float r1 = fabsf(a2->radius);
	CcXyVec2 d = ccxy_sub(c1, c0);
	float distc = ccxy_len(d);
	float a;
	float h2;
	CcXyVec2 u;
	CcXyVec2 mid;
	float h;
	CcXyVec2 perp;

	*count = 0;

	if (distc < CCXY_TOL)
		return CCXY_IT_NONE;
	if (distc > r0 + r1 + CCXY_TOL)
		return CCXY_IT_NONE;
	if (distc < fabsf(r0 - r1) - CCXY_TOL)
		return CCXY_IT_NONE;

	a = (r0 * r0 - r1 * r1 + distc * distc) / (2.0f * distc);
	h2 = r0 * r0 - a * a;
	u = ccxy_scale(d, 1.0f / distc);
	mid = ccxy_add(c0, ccxy_scale(u, a));

	if (fabsf(h2) < CCXY_TOL)
	{
		*p1 = mid;
		*count = 1;
		return CCXY_IT_TANGENT;
	}

	h = sqrtf(fmaxf(0.0f, h2));
	perp = ccxy_left_normal(u);
	*p1 = ccxy_add(mid, ccxy_scale(perp, h));
	*p2 = ccxy_sub(mid, ccxy_scale(perp, h));
	*count = 2;
	return CCXY_IT_INTERSECT;
}

static inline CcXyIntersectType ccxy_intersect_line_circle(CcXyVec2 l1, CcXyVec2 l2, CcXyVec2 ctr, float r, CcXyVec2 *p1, CcXyVec2 *p2, int *count)
{
	CcXyVec2 d = ccxy_sub(l2, l1);
	float dd = ccxy_dot(d, d);
	CcXyVec2 f;
	float t0;
	CcXyVec2 q;
	CcXyVec2 qc;
	float dist2;
	float r2;
	float h2;
	float h;
	float invLen;
	CcXyVec2 u;
	const float eps = 1e-5f;
	const float eps2 = eps * eps;

	*count = 0;
	if (dd < 1e-20f)
		return CCXY_IT_NONE;

	f = ccxy_sub(l1, ctr);
	t0 = -ccxy_dot(f, d) / dd;
	q = ccxy_add(l1, ccxy_scale(d, t0));
	qc = ccxy_sub(q, ctr);
	dist2 = ccxy_dot(qc, qc);
	r2 = r * r;
	h2 = r2 - dist2;

	if (h2 < -eps2)
		return CCXY_IT_NONE;

	if (fabsf(h2) <= eps2)
	{
		*p1 = q;
		*count = 1;
		return CCXY_IT_TANGENT;
	}

	h = sqrtf(h2);
	invLen = 1.0f / sqrtf(dd);
	u = ccxy_scale(d, invLen);
	*p1 = ccxy_sub(q, ccxy_scale(u, h));
	*p2 = ccxy_add(q, ccxy_scale(u, h));
	*count = 2;
	return CCXY_IT_INTERSECT;
}

static inline void ccxy_report_error(CcXyContext *ctx, CcXyCompError err)
{
	const char *message = "Unknown comp error";
	ctx->hasCompError = true;
	ctx->lastError = err;

	if (!ctx->errorCB)
		return;

	switch (err)
	{
	case CCXY_CE_ARC_RADIUS_MISMATCH:
		message = "Arc radius inconsistency";
		break;
	case CCXY_CE_INVALID_MOVE:
		message = "Invalid move";
		break;
	case CCXY_CE_COMP_MOVE_TOO_SHORT:
		message = "Comp move too short";
		break;
	case CCXY_CE_ARC_LT_TOOL_RAD:
		message = "Arc smaller than tool radius";
		break;
	case CCXY_CE_FLIPPED_ARC:
		message = "Flipped arc";
		break;
	case CCXY_CE_COMP_IN_CROSSING:
		message = "Comp-in crossing";
		break;
	case CCXY_CE_COMP_OUT_CROSSING:
		message = "Comp-out crossing";
		break;
	case CCXY_CE_UNRESOLVED_GAP:
		message = "Unresolved gap";
		break;
	case CCXY_CE_OUTPUT_BUFFER_OVERFLOW:
		message = "Output buffer overflow";
		break;
	default:
		break;
	}

	ctx->errorCB(message, err, ctx->lastSeqNum);
}

static inline bool ccxy_validate(CcXyContext *ctx, CcXyMove2D *m)
{
	if (m->type == CCXY_MOT_LINE || m->type == CCXY_MOT_RAPID)
	{
		m->valid = ccxy_len(ccxy_sub(m->p_1, m->p_0)) >= CCXY_TOL;
		if (!m->valid)
			ccxy_report_error(ctx, CCXY_CE_INVALID_MOVE);
		return m->valid;
	}

	if (m->type == CCXY_MOT_ARC)
	{
		float d = ccxy_dist_from_start_along(m, m->p_1);
		float chordLen = ccxy_len(ccxy_sub(m->p_1, m->p_0));
		bool radius_ok = ccxy_is_radius_consistent(ctx, m);
		float sw = ccxy_arc_sweep_deg(m);
		bool sweep_ok = !(sw > CCXY_MAX_SWEEP_DEG || sw < CCXY_MIN_ARC_LEN);

		if (ctx->compState == CCXY_COMP_LEFT && m->arcDir == CCXY_ARC_CCW)
		{
			if (chordLen <= ctx->toolR)
			{
				ccxy_report_error(ctx, CCXY_CE_ARC_LT_TOOL_RAD);
				m->valid = false;
				return false;
			}
		}

		if (ctx->compState == CCXY_COMP_RIGHT && m->arcDir == CCXY_ARC_CW)
		{
			if (chordLen <= ctx->toolR)
			{
				ccxy_report_error(ctx, CCXY_CE_ARC_LT_TOOL_RAD);
				m->valid = false;
				return false;
			}
		}

		m->valid = d >= CCXY_TOL && radius_ok && sweep_ok && fabsf(m->radius) >= CCXY_TOL;
		if (!radius_ok)
			ccxy_report_error(ctx, CCXY_CE_ARC_RADIUS_MISMATCH);
		if (!sweep_ok || fabsf(m->radius) < CCXY_TOL)
			ccxy_report_error(ctx, CCXY_CE_INVALID_MOVE);
		return m->valid;
	}

	m->valid = false;
	return false;
}

static inline bool ccxy_motion_valid(const CcXyMove2D *m)
{
	return m->valid && m->type != CCXY_MOT_EMPTY;
}

static inline bool ccxy_point_on_finite_elem(const CcXyMove2D *m, CcXyVec2 p)
{
	if (m->type == CCXY_MOT_LINE || m->type == CCXY_MOT_RAPID)
		return ccxy_point_on_segment(m->p_0, m->p_1, p);

	if (m->type == CCXY_MOT_ARC)
	{
		CcXyArcAngles aa = ccxy_precompute_arc_angles(m);
		return ccxy_point_on_arc_cached(m, p, &aa);
	}

	return false;
}

static inline int ccxy_intersect_carrier(const CcXyMove2D *a, const CcXyMove2D *b, CcXyVec2 pts[2])
{
	if ((a->type == CCXY_MOT_LINE || a->type == CCXY_MOT_RAPID) && (b->type == CCXY_MOT_LINE || b->type == CCXY_MOT_RAPID))
	{
		bool tip = false;
		CcXyIntersectType it = ccxy_intersect_line_line(a, b, &pts[0], &tip);
		return (it == CCXY_IT_NONE) ? 0 : 1;
	}

	if (a->type == CCXY_MOT_ARC && b->type == CCXY_MOT_ARC)
	{
		int count = 0;
		CcXyIntersectType it;
		if (ccxy_is_near(a->center, b->center))
			return 0;
		it = ccxy_intersect_circle_circle(a, b, &pts[0], &pts[1], &count);
		if (it == CCXY_IT_NONE)
			return 0;
		return count;
	}

	{
		const CcXyMove2D *line = ((a->type == CCXY_MOT_LINE || a->type == CCXY_MOT_RAPID) ? a : b);
		const CcXyMove2D *arc = (a->type == CCXY_MOT_ARC ? a : b);
		int count = 0;
		CcXyIntersectType it = ccxy_intersect_line_circle(line->p_0, line->p_1, arc->center, fabsf(arc->radius), &pts[0], &pts[1], &count);
		if (it == CCXY_IT_NONE)
			return 0;
		return count;
	}
}

static inline int ccxy_finite_intersection_points(const CcXyMove2D *a, const CcXyMove2D *b, CcXyVec2 pts[2])
{
	CcXyVec2 carrierPts[2];
	int carrierCount = ccxy_intersect_carrier(a, b, carrierPts);
	int finiteCount = 0;
	int i;

	for (i = 0; i < carrierCount; ++i)
	{
		CcXyVec2 p = carrierPts[i];
		if (!ccxy_point_on_finite_elem(a, p) || !ccxy_point_on_finite_elem(b, p))
			continue;
		if (finiteCount > 0 && ccxy_is_near(pts[0], p))
			continue;
		pts[finiteCount++] = p;
	}

	return finiteCount;
}

static inline bool ccxy_is_forward_extension_point(const CcXyMove2D *a, const CcXyMove2D *b, CcXyVec2 p)
{
	float fipDir1 = ccxy_dot(ccxy_sub(p, a->p_1), a->endDir);
	float fipDir2 = ccxy_dot(ccxy_sub(p, b->p_0), b->startDir);
	return fipDir1 > 0.0f && fipDir2 < 0.0f;
}

static inline bool ccxy_convex_from_winding(const CcXyContext *ctx, int winding)
{
	bool isLeft;
	if (winding == 0)
		return false;
	isLeft = (ctx->compState == CCXY_COMP_LEFT);
	if (ctx->toolSign < 0)
		isLeft = !isLeft;
	if (isLeft)
		return !(winding > 0);
	return winding > 0;
}

static inline bool ccxy_is_convex(const CcXyContext *ctx, const CcXyMove2D *a, const CcXyMove2D *b)
{
	int winding = ccxy_get_winding_dir(a->endDir, b->startDir);
	return ccxy_convex_from_winding(ctx, winding);
}

static inline bool ccxy_solve_junction(const CcXyContext *ctx, const CcXyMove2D *a, const CcXyMove2D *b, bool allowExtend, CcXyJunction *outjunc)
{
	CcXyVec2 trimPts[2];
	CcXyVec2 carrierPts[2];
	int trimCount = ccxy_finite_intersection_points(a, b, trimPts);
	int carrierCount = ccxy_intersect_carrier(a, b, carrierPts);
	float bestTrimScore = 0.0f;
	float bestExtendScore = 0.0f;
	bool foundTrim = false;
	bool foundExtend = false;
	int i;

	outjunc->type = CCXY_JT_NONE;
	outjunc->p = ccxy_v2(0.0f, 0.0f);

	for (i = 0; i < trimCount; ++i)
	{
		CcXyVec2 p = trimPts[i];
		float score = ccxy_dist_from_start_along(a, p) + ccxy_dist_from_start_along(b, p);
		if (!foundTrim || score < bestTrimScore)
		{
			outjunc->type = CCXY_JT_TRIM_TO_INTERSECTION;
			outjunc->p = p;
			bestTrimScore = score;
			foundTrim = true;
		}
	}

	for (i = 0; i < carrierCount; ++i)
	{
		CcXyVec2 p = carrierPts[i];
		if (allowExtend && ccxy_is_forward_extension_point(a, b, p))
		{
			float score = ccxy_dist(a->p_1, p) + ccxy_dist(b->p_0, p);
			if (!foundExtend || score < bestExtendScore)
			{
				outjunc->type = CCXY_JT_EXTEND_TO_INTERSECTION;
				outjunc->p = p;
				bestExtendScore = score;
				foundExtend = true;
			}
		}
	}

	if (foundTrim)
		return true;
	if (foundExtend)
		return true;
	if (ccxy_is_convex(ctx, a, b))
	{
		outjunc->type = CCXY_JT_ROLL_AROUND;
		return true;
	}

	outjunc->type = CCXY_JT_GOUGE;
	return false;
}

static inline bool ccxy_out_has_space(CcXyContext *ctx, int n)
{
	bool ok = (ctx->outCount + n) <= CCXY_OUT_CAP;
	if (!ok)
		ccxy_report_error(ctx, CCXY_CE_OUTPUT_BUFFER_OVERFLOW);
	return ok;
}

static inline CcXyMove2D ccxy_pop_in(CcXyContext *ctx)
{
	CcXyMove2D m = ctx->input_buffer[ctx->inHead];
	ctx->inHead = (ctx->inHead + 1) % CCXY_IN_CAP;
	ctx->inCount--;
	return m;
}

static inline void ccxy_push_out(CcXyContext *ctx, const CcXyMove2D *m)
{
	if (!ccxy_out_has_space(ctx, 1))
		return;

	ctx->output_buffer[(ctx->outHead + ctx->outCount) % CCXY_OUT_CAP] = *m;
	ctx->outCount++;
}

static inline CcXyMove2D ccxy_make_bevel(const CcXyMove2D *a, const CcXyMove2D *b)
{
	CcXyMove2D m = {0};
	m.hasXY = true;
	m.valid = true;
	m.type = CCXY_MOT_LINE;
	m.feed = (a->feed > 0.0f) ? a->feed : b->feed;
	m.compMode = CCXY_CM_STEADY;
	m.p_0 = a->p_1;
	m.p_1 = b->p_0;
	ccxy_update_vectors(&m);
	return m;
}

static inline bool ccxy_offset_line(const CcXyContext *ctx, const CcXyMove2D *src, CcXyMove2D *dst)
{
	CcXyVec2 v;
	float l;
	CcXyVec2 u;
	bool useLeft;
	CcXyVec2 n;
	CcXyVec2 off;

	*dst = *src;
	v = ccxy_sub(src->p_1, src->p_0);
	l = ccxy_len(v);
	if (l < CCXY_TOL)
	{
		dst->valid = false;
		return false;
	}

	u = ccxy_scale(v, 1.0f / l);
	useLeft = (ctx->compState == CCXY_COMP_LEFT);
	if (ctx->toolSign < 0)
		useLeft = !useLeft;
	n = useLeft ? ccxy_left_normal(u) : ccxy_right_normal(u);
	off = ccxy_scale(n, ctx->toolR);

	if (src->compMode == CCXY_CM_IN || src->compMode == CCXY_CM_OUT)
		off = ccxy_v2(0.0f, 0.0f);

	dst->p_0 = ccxy_add(src->p_0, off);
	dst->p_1 = ccxy_add(src->p_1, off);
	return true;
}

static inline bool ccxy_offset_arc(CcXyContext *ctx, const CcXyMove2D *src, CcXyMove2D *dst)
{
	float r0 = src->radius;
	float dr;
	bool ccw;
	bool left;
	float r1;
	CcXyVec2 v0;
	CcXyVec2 v1;
	float lv0;
	float lv1;

	if (r0 < CCXY_TOL)
		r0 = ccxy_len(ccxy_sub(src->p_0, src->center));
	if (r0 < CCXY_TOL)
		return false;

	*dst = *src;
	dr = ctx->toolR;
	ccw = (src->arcDir == CCXY_ARC_CCW);
	left = (ctx->compState == CCXY_COMP_LEFT);
	if (ctx->toolSign < 0)
		left = !left;

	if (ccw)
		r1 = r0 + (left ? -dr : dr);
	else
		r1 = r0 + (left ? dr : -dr);

	if (r1 <= CCXY_TOL)
	{
		ccxy_report_error(ctx, CCXY_CE_ARC_LT_TOOL_RAD);
		dst->valid = false;
		return false;
	}

	v0 = ccxy_sub(src->p_0, src->center);
	v1 = ccxy_sub(src->p_1, src->center);
	lv0 = ccxy_len(v0);
	lv1 = ccxy_len(v1);
	if (lv0 < CCXY_TOL || lv1 < CCXY_TOL)
	{
		dst->valid = false;
		return false;
	}

	dst->center = src->center;
	dst->radius = r1;
	dst->p_0 = ccxy_add(src->center, ccxy_scale(v0, r1 / lv0));
	dst->p_1 = ccxy_add(src->center, ccxy_scale(v1, r1 / lv1));
	return true;
}

static inline bool ccxy_offset_move(CcXyContext *ctx, const CcXyMove2D *src, CcXyMove2D *dst)
{
	if (src->type == CCXY_MOT_LINE || src->type == CCXY_MOT_RAPID)
		return ccxy_offset_line(ctx, src, dst);
	if (src->type == CCXY_MOT_ARC)
		return ccxy_offset_arc(ctx, src, dst);
	return false;
}

static inline bool ccxy_trim_to(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, CcXyVec2 tip)
{
	a->p_1 = tip;
	b->p_0 = tip;
	ccxy_update_vectors(a);
	ccxy_update_vectors(b);
	ccxy_validate(ctx, a);
	ccxy_validate(ctx, b);
	return a->valid && b->valid;
}

static inline bool ccxy_extend_to(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, CcXyVec2 fip)
{
	float fipDir1 = ccxy_dot(ccxy_sub(fip, a->p_1), a->endDir);
	float fipDir2 = ccxy_dot(ccxy_sub(fip, b->p_0), b->startDir);

	if (fipDir1 > 0.0f && fipDir2 < 0.0f)
	{
		a->p_1 = fip;
		b->p_0 = fip;
		ccxy_update_vectors(a);
		ccxy_update_vectors(b);
		ccxy_validate(ctx, a);
		ccxy_validate(ctx, b);
	}

	return a->valid && b->valid;
}

static inline CcXyMove2D ccxy_make_roll_arc(const CcXyContext *ctx, const CcXyMove2D *a, const CcXyMove2D *b)
{
	CcXyMove2D roll = {0};
	bool useLeft = (ctx->compState == CCXY_COMP_LEFT);
	CcXyVec2 v0;
	CcXyVec2 v1;
	float r0;
	float r1;
	float r;
	CcXyArcDir preferredDir;
	const float turnEps = 1.0e-4f;

	if (ctx->toolSign < 0)
		useLeft = !useLeft;

	roll.type = CCXY_MOT_ARC;
	roll.compMode = CCXY_CM_STEADY;
	roll.feed = (a->feed > 0.0f) ? a->feed : b->feed;
	roll.p_0 = a->p_1;
	roll.p_1 = b->p_0;
	roll.center = ccxy_original_endpoint(a->p_1, a->endDir, useLeft, ctx->toolR);

	v0 = ccxy_sub(roll.p_0, roll.center);
	v1 = ccxy_sub(roll.p_1, roll.center);
	r0 = ccxy_len(v0);
	r1 = ccxy_len(v1);

	if (r0 >= CCXY_TOL && r1 >= CCXY_TOL)
		r = 0.5f * (r0 + r1);
	else if (r0 >= CCXY_TOL)
		r = r0;
	else if (r1 >= CCXY_TOL)
		r = r1;
	else
		r = ctx->toolR;

	if (r0 >= CCXY_TOL)
		roll.p_0 = ccxy_add(roll.center, ccxy_scale(v0, r / r0));
	if (r1 >= CCXY_TOL)
		roll.p_1 = ccxy_add(roll.center, ccxy_scale(v1, r / r1));

	roll.radius = r;
	preferredDir = useLeft ? CCXY_ARC_CW : CCXY_ARC_CCW;
	roll.arcDir = (uint8_t)preferredDir;

	if (r0 >= CCXY_TOL && r1 >= CCXY_TOL)
	{
		float turnSign = ccxy_cross(v0, v1) / (r0 * r1);
		if (turnSign > turnEps && preferredDir != CCXY_ARC_CCW)
			roll.arcDir = CCXY_ARC_CCW;
		else if (turnSign < -turnEps && preferredDir != CCXY_ARC_CW)
			roll.arcDir = CCXY_ARC_CW;
	}

	roll.hasXY = true;
	roll.valid = true;
	ccxy_update_vectors(&roll);
	return roll;
}

static inline bool ccxy_insert_roll(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, CcXyMove2D inserts[CCXY_INSERT_CAP], int *insertCount)
{
	float gap = ccxy_len(ccxy_sub(b->p_0, a->p_1));

	if (gap < ctx->gapTol)
	{
		CcXyMove2D bevel = ccxy_make_bevel(a, b);
		if (!ccxy_validate(ctx, &bevel))
			return false;
		inserts[(*insertCount)++] = bevel;
		return true;
	}

	if (*insertCount >= CCXY_INSERT_CAP)
		return false;

	{
		CcXyMove2D roll = ccxy_make_roll_arc(ctx, a, b);
		if (!ccxy_validate(ctx, &roll))
			return false;
		inserts[(*insertCount)++] = roll;
		return true;
	}
}

static inline void ccxy_handle_line_line(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, bool comping, CcXyMove2D inserts[CCXY_INSERT_CAP], int *insertCount)
{
	CcXyJunction junction;
	float gap = ccxy_dist(b->p_0, a->p_1);
	bool allowExtend = (gap < ctx->gapTol) || comping;
	bool resolved = ccxy_solve_junction(ctx, a, b, allowExtend, &junction);

	if (junction.type == CCXY_JT_TRIM_TO_INTERSECTION)
	{
		ccxy_trim_to(ctx, a, b, junction.p);
		if (!a->valid)
		{
			b->p_0 = a->p_1;
			return;
		}
		if (!b->valid)
		{
			a->p_1 = b->p_0;
			return;
		}
		return;
	}

	if (junction.type == CCXY_JT_EXTEND_TO_INTERSECTION && ccxy_extend_to(ctx, a, b, junction.p))
		return;

	if (a->compMode == CCXY_CM_IN)
	{
		a->p_1 = b->p_0;
		ccxy_update_vectors(a);
		ccxy_validate(ctx, a);
		return;
	}

	if (b->compMode == CCXY_CM_OUT)
	{
		b->p_0 = a->p_1;
		ccxy_update_vectors(b);
		ccxy_validate(ctx, b);
		return;
	}

	if (resolved && junction.type == CCXY_JT_ROLL_AROUND)
	{
		if (!ccxy_insert_roll(ctx, a, b, inserts, insertCount))
			ccxy_report_error(ctx, CCXY_CE_UNRESOLVED_GAP);
		return;
	}

	inserts[(*insertCount)++] = ccxy_make_bevel(a, b);
}

static inline void ccxy_handle_arc_arc(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, CcXyMove2D inserts[CCXY_INSERT_CAP], int *insertCount)
{
	CcXyJunction junction;
	float gap;
	bool resolved;

	if (ccxy_is_near(a->p_1, b->p_0) || ccxy_is_near(a->center, b->center))
		return;

	gap = ccxy_len(ccxy_sub(b->p_0, a->p_1));
	resolved = ccxy_solve_junction(ctx, a, b, gap < ctx->gapTol, &junction);

	if (junction.type == CCXY_JT_TRIM_TO_INTERSECTION && ccxy_trim_to(ctx, a, b, junction.p))
		return;

	if (junction.type == CCXY_JT_EXTEND_TO_INTERSECTION && ccxy_extend_to(ctx, a, b, junction.p))
		return;

	if (resolved && junction.type == CCXY_JT_ROLL_AROUND)
	{
		if (!ccxy_insert_roll(ctx, a, b, inserts, insertCount))
			ccxy_report_error(ctx, CCXY_CE_UNRESOLVED_GAP);
		return;
	}

	inserts[(*insertCount)++] = ccxy_make_bevel(a, b);
}

static inline void ccxy_handle_arc_line(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, CcXyMove2D inserts[CCXY_INSERT_CAP], int *insertCount)
{
	CcXyJunction junction;
	float gap;
	bool resolved;

	if (ccxy_is_near(a->p_1, b->p_0))
	{
		b->p_0 = a->p_1;
		ccxy_update_vectors(b);
		ccxy_validate(ctx, b);
		return;
	}

	gap = ccxy_len(ccxy_sub(b->p_0, a->p_1));
	resolved = ccxy_solve_junction(ctx, a, b, gap < ctx->gapTol, &junction);

	if (junction.type == CCXY_JT_TRIM_TO_INTERSECTION && ccxy_trim_to(ctx, a, b, junction.p))
		return;

	if (junction.type == CCXY_JT_EXTEND_TO_INTERSECTION && ccxy_extend_to(ctx, a, b, junction.p))
		return;

	if (resolved && junction.type == CCXY_JT_ROLL_AROUND)
	{
		if (!ccxy_insert_roll(ctx, a, b, inserts, insertCount))
			ccxy_report_error(ctx, CCXY_CE_UNRESOLVED_GAP);
		return;
	}

	inserts[(*insertCount)++] = ccxy_make_bevel(a, b);
}

static inline void ccxy_apply_logic(CcXyContext *ctx, CcXyMove2D *a, CcXyMove2D *b, CcXyMove2D inserts[CCXY_INSERT_CAP], int *insertCount)
{
	bool comping = (a->compMode == CCXY_CM_IN || a->compMode == CCXY_CM_OUT || b->compMode == CCXY_CM_IN || b->compMode == CCXY_CM_OUT);
	*insertCount = 0;

	if ((a->type == CCXY_MOT_LINE || a->type == CCXY_MOT_RAPID) && (b->type == CCXY_MOT_LINE || b->type == CCXY_MOT_RAPID))
	{
		ccxy_handle_line_line(ctx, a, b, comping, inserts, insertCount);
		return;
	}

	if (a->type == CCXY_MOT_ARC && b->type == CCXY_MOT_ARC)
	{
		ccxy_handle_arc_arc(ctx, a, b, inserts, insertCount);
		return;
	}

	if ((a->type == CCXY_MOT_ARC && (b->type == CCXY_MOT_LINE || b->type == CCXY_MOT_RAPID)) ||
		((a->type == CCXY_MOT_LINE || a->type == CCXY_MOT_RAPID) && b->type == CCXY_MOT_ARC))
	{
		ccxy_handle_arc_line(ctx, a, b, inserts, insertCount);
	}
}

static inline void ccxy_reset_state(CcXyContext *ctx)
{
	ctx->havePrevMove = false;
}

static inline void ccxy_init(CcXyContext *ctx, const CcXyOptions *options)
{
	CcXyContext zero = {0};
	*ctx = zero;
	ctx->units = CCXY_UNITS_MM;
	ctx->lastError = CCXY_CE_ERROR;
	ctx->toolSign = 1;
	ctx->arcTol = CCXY_ARC_TOL_IN * 25.4f;
	ctx->gapTol = CCXY_GAP_TOL_IN * 25.4f;
	ctx->compState = CCXY_COMP_OFF;
	if (options)
	{
		ctx->errorCB = options->error;
		ctx->toolR = (options->toolRadius < 0.0f) ? -options->toolRadius : options->toolRadius;
		ctx->toolSign = (options->toolRadius < 0.0f) ? -1 : 1;
	}
}

static inline void ccxy_set_units(CcXyContext *ctx, CcXyUnits units)
{
	ctx->units = units;
	if (units == CCXY_UNITS_INCH)
	{
		ctx->arcTol = CCXY_ARC_TOL_IN;
		ctx->gapTol = CCXY_GAP_TOL_IN;
	}
	else
	{
		ctx->arcTol = CCXY_ARC_TOL_IN * 25.4f;
		ctx->gapTol = CCXY_GAP_TOL_IN * 25.4f;
	}
}

static inline void ccxy_set_tool_radius(CcXyContext *ctx, float radius)
{
	ctx->toolR = (radius < 0.0f) ? -radius : radius;
	ctx->toolSign = (radius < 0.0f) ? -1 : 1;
}

static inline void ccxy_set_error_callback(CcXyContext *ctx, CcXyErrorCB cb)
{
	ctx->errorCB = cb;
}

static inline void ccxy_set_comp(CcXyContext *ctx, CcXyCompSide side)
{
	ctx->compState = side;
	ccxy_reset_state(ctx);
}

static inline bool ccxy_push_in(CcXyContext *ctx, const CcXyMove2D *m)
{
	if (ctx->inCount >= CCXY_IN_CAP)
		return false;
	ctx->input_buffer[(ctx->inHead + ctx->inCount) % CCXY_IN_CAP] = *m;
	ctx->inCount++;
	return true;
}

static inline bool ccxy_process(CcXyContext *ctx)
{
	if (ctx->hasCompError)
		return false;

	if (ctx->compState == CCXY_COMP_OFF || ctx->toolR < CCXY_TOL)
	{
		while (ctx->inCount > 0)
		{
			CcXyMove2D m;
			if (!ccxy_out_has_space(ctx, 1))
				return false;
			m = ccxy_pop_in(ctx);
			ccxy_push_out(ctx, &m);
		}
		return true;
	}

	while (ctx->inCount > 0)
	{
		CcXyMove2D raw;
		CcXyMove2D curOff;
		CcXyMove2D inserts[CCXY_INSERT_CAP];
		int insertCount = 0;

		if (!ccxy_out_has_space(ctx, 1 + CCXY_INSERT_CAP))
			return false;

		raw = ccxy_pop_in(ctx);
		if (raw.seqNum != 0)
			ctx->lastSeqNum = raw.seqNum;

		if (raw.type == CCXY_MOT_EMPTY)
			continue;

		ccxy_update_vectors(&raw);
		if (!ccxy_offset_move(ctx, &raw, &curOff))
			return false;
		ccxy_update_vectors(&curOff);
		if(!ccxy_validate(ctx, &curOff)) return false;

		if (!raw.hasXY && raw.hasZ)
		{
			if (ctx->havePrevMove)
			{
				curOff.p_0 = ctx->prevOff.p_0;
				curOff.p_1 = ctx->prevOff.p_0;
			}
			if (!ccxy_out_has_space(ctx, 1))
				return false;
			ccxy_push_out(ctx, &curOff);
			continue;
		}

		if (raw.compMode == CCXY_CM_IN || raw.compMode == CCXY_CM_OUT)
		{
			float moveLen = ccxy_len(ccxy_sub(raw.p_1, raw.p_0));
			if (moveLen <= ctx->toolR)
			{
				ccxy_report_error(ctx, CCXY_CE_COMP_MOVE_TOO_SHORT);
				return false;
			}
		}

		if (!ctx->havePrevMove)
		{
			ctx->prevOff = curOff;
			ctx->havePrevMove = true;
			continue;
		}

		if (ctx->prevOff.compMode == CCXY_CM_IN)
			ctx->prevOff.p_1 = curOff.p_0;

		if (curOff.compMode == CCXY_CM_OUT)
			curOff.p_0 = ctx->prevOff.p_1;

		ccxy_apply_logic(ctx, &ctx->prevOff, &curOff, inserts, &insertCount);

		if (ctx->prevOff.valid)
		{
			int i;
			ccxy_push_out(ctx, &ctx->prevOff);
			for (i = 0; i < insertCount; ++i)
				ccxy_push_out(ctx, &inserts[i]);
		}

		ctx->prevOff = curOff;
	}

	return true;
}

static inline void ccxy_flush(CcXyContext *ctx)
{
	ccxy_process(ctx);
	if (ctx->havePrevMove && ccxy_out_has_space(ctx, 1))
	{
		ccxy_push_out(ctx, &ctx->prevOff);
		ctx->havePrevMove = false;
	}
}

static inline bool ccxy_pop_out(CcXyContext *ctx, CcXyMove2D *m)
{
	if (ctx->outCount == 0)
		return false;

	*m = ctx->output_buffer[ctx->outHead];
	ctx->outHead = (ctx->outHead + 1) % CCXY_OUT_CAP;
	ctx->outCount--;
	return true;
}
