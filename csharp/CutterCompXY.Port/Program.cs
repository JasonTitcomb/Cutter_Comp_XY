namespace CutterCompXY.Port;

internal static class Program
{
    private const float TOOL_RADIUS = 0.0651f;
    private const CornerType CORNER_TREATMENT = CornerType.CORNER_ROLL;

    private static StreamWriter currentWriter;
    private static bool hasStopError;

    private static void OutputHandler(string text, int len)
    {
        if (currentWriter == null || string.IsNullOrEmpty(text))
            return;

        int count = Math.Min(len, text.Length);
        currentWriter.Write(text.AsSpan(0, count));
    }

    private static void ErrorHandler(string message, int err, uint seqNum)
    {
        Console.WriteLine($"{message} ({(CompError)err}, N{seqNum})");
        hasStopError = true;
    }

    private static void StartCompHandler(int toolRegister, int diaRegister)
    {
        _ = toolRegister;
        _ = diaRegister;
    }

    private static bool RunProfileStreaming(string inputPath, string emitGcodePath)
    {
        if (!File.Exists(inputPath))
        {
            Console.WriteLine($"Failed to open input file: {inputPath}");
            return false;
        }

        hasStopError = false;
        using StreamWriter outFile = new StreamWriter(emitGcodePath);
        currentWriter = outFile;

        CcMainOptions options = new CcMainOptions
        {
            toolRadius = TOOL_RADIUS,
            cornerTreatment = CORNER_TREATMENT,
            globalTrimCrossing = true,
            globalMerge = true,
            emitStatusComments = true,
            callbacks = new CcMainOptions.CcMainCallbacks
            {
                output = OutputHandler,
                error = ErrorHandler,
                startComp = StartCompHandler
            }
        };

        CcMainRunner runner = new CcMainRunner();
        if (!runner.Begin(options))
            return false;

        foreach (string rawLine in File.ReadLines(inputPath))
        {
            if (hasStopError)
                break;

            if (!runner.ProcessLine(rawLine.TrimEnd('\r')))
                return false;
        }

        if (!runner.Finish())
            return false;

        return !hasStopError;
    }

    private static int Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--geometry-self-test")
            return CrossingSelfTest.Run();

        string repoRoot = FindRepoRoot();
        string defaultInput = Path.Combine(repoRoot, "data", "TortureTestG90.nc");
        string inputPath = args.Length > 0 ? args[0] : defaultInput;

        string outputDir = Path.Combine(repoRoot, "output");
        Directory.CreateDirectory(outputDir);

        string inputBaseName = Path.GetFileNameWithoutExtension(inputPath);
        string ngcPath = Path.Combine(outputDir, inputBaseName + ".cs.ngc");

        bool ok = RunProfileStreaming(inputPath, ngcPath);
        if (!ok)
        {
            Console.WriteLine("(warning: profile validation failed)");
            return 1;
        }

        Console.WriteLine($"Wrote: {ngcPath}");
        return 0;
    }

    private static string FindRepoRoot()
    {
        DirectoryInfo dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir != null)
        {
            string candidate = Path.Combine(dir.FullName, "data");
            if (Directory.Exists(candidate))
                return dir.FullName;
            dir = dir.Parent;
        }

        return Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, "..", "..", "..", "..", "..", ".."));
    }
}
