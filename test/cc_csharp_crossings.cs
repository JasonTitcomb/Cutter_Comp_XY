namespace CutterCompXY.Port;

internal static class CrossingSelfTest
{
    private static Move2D LineMove(Vec2 start, Vec2 end)
    {
        Move2D move = new Move2D
        {
            type = MotionType.MOT_LINE,
            compMode = CompMode.CM_STEADY,
            p_0 = start,
            p_1 = end
        };
        CcMath.UpdateVectors(ref move);
        return move;
    }

    private static bool CheckSharedLeadEndpoint(float scale)
    {
        float toolRadius = 0.0010225861129f * 0.5f * scale;
        Vec2 entry = new Vec2(0.25f * scale - toolRadius, 0.55f * scale);
        Vec2 exit = new Vec2(0.25f * scale + 0.21693046f * toolRadius,
                             0.55f * scale - 0.97618706f * toolRadius);
        Vec2 origin = new Vec2(0.0f, 0.0f);
        for (int reverseFirst = 0; reverseFirst < 2; ++reverseFirst)
        {
            for (int reverseSecond = 0; reverseSecond < 2; ++reverseSecond)
            {
                Move2D first = LineMove(reverseFirst != 0 ? entry : origin,
                                        reverseFirst != 0 ? origin : entry);
                Move2D second = LineMove(reverseSecond != 0 ? origin : exit,
                                         reverseSecond != 0 ? exit : origin);
                first.type = MotionType.MOT_RAPID;
                second.type = MotionType.MOT_RAPID;
                if (CcMath.IntersectLineLine(first, second, out Vec2 tip) != IntersectType.IT_INTERSECT ||
                    !CcMath.SamePoint(tip, origin) ||
                    CcMath.IntersectLineLine(second, first, out tip) != IntersectType.IT_INTERSECT ||
                    !CcMath.SamePoint(tip, origin))
                    return false;
            }
        }

        Move2D lead = LineMove(origin, entry);
        Move2D crossing = LineMove(new Vec2(0.0f, 0.275f * scale),
                                   new Vec2(0.5f * scale, 0.275f * scale));
        if (CcMath.IntersectLineLine(lead, crossing, out Vec2 crossingTip) != IntersectType.IT_INTERSECT ||
            !CcMath.IsNear(crossingTip, entry * 0.5f))
            return false;

        Move2D parallel = LineMove(entry, entry * 2.0f);
        if (CcMath.IntersectLineLine(lead, parallel, out _) != IntersectType.IT_NONE)
            return false;

        Move2D collapsed = LineMove(entry, entry);
        if (CcMath.IntersectLineLine(lead, collapsed, out _) != IntersectType.IT_NONE)
            return false;

        collapsed.junctionOnly = true;
        collapsed.startDir = new Vec2(1.0f, 0.0f);
        collapsed.endDir = collapsed.startDir;
        return CcMath.IntersectLineLine(lead, collapsed, out Vec2 junctionTip) == IntersectType.IT_INTERSECT &&
               CcMath.SamePoint(junctionTip, entry);
    }

    internal static int Run()
    {
        if (!CheckSharedLeadEndpoint(1.0f) || !CheckSharedLeadEndpoint(25.4f))
        {
            Console.Error.WriteLine("Shared lead endpoint drifted or crossing/parallel behavior changed.");
            return 1;
        }

        Console.WriteLine("C# crossing checks passed.");
        return 0;
    }
}
