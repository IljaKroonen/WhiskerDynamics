using System.Runtime.Loader;
using WhiskerDynamics.Compatibility;
using WhiskerDynamics.Compatibility.Patching;

namespace WhiskerDynamics.Compatibility.Cli;

internal static class Program
{
    public static int Main(string[] args) => CompatibilityCli.Run(args);
}

internal static class CompatibilityCli
{
    private const string DefaultGameDir = @"C:\Program Files\Kitten Space Agency";

    public static int Run(string[] args)
    {
        ParseResult parsed = ParseArgs(args);
        if (parsed.ShowedHelp) return 0;
        if (parsed.Error is not null)
        {
            Console.Error.WriteLine($"error: {parsed.Error}");
            PrintUsage();
            return 2;
        }

        string gameDir = Path.GetFullPath(parsed.GameDir!);
        string ksaPath = Path.Combine(gameDir, "KSA.dll");
        if (!File.Exists(ksaPath))
        {
            Console.Error.WriteLine($"error: KSA.dll not found in '{gameDir}'");
            return 2;
        }

        AssemblyLoadContext.Default.Resolving += (_, name) =>
        {
            string candidate = Path.Combine(gameDir, $"{name.Name}.dll");
            return File.Exists(candidate)
                ? AssemblyLoadContext.Default.LoadFromAssemblyPath(candidate)
                : null;
        };

        try
        {
            return Validate(gameDir);
        }
        catch (Exception e)
        {
            Console.Error.WriteLine("INCOMPATIBLE: the compatibility contract could not be loaded.");
            Console.Error.WriteLine(Describe(e));
            return 1;
        }
    }

    private static int Validate(string gameDir)
    {
        string version = typeof(KSA.Program).Assembly.GetName().Version?.ToString() ?? "unknown";
        Console.WriteLine("Whisker Dynamics game compatibility report");
        Console.WriteLine($"Game directory: {gameDir}");
        Console.WriteLine($"KSA assembly version: {version}");
        Console.WriteLine($"Verified KSA build: {GameBuildPolicy.VerifiedBuild}");

        bool buildOk = GameBuildPolicy.IsVerified(version);
        Console.WriteLine($"Build verification: {(buildOk ? "VERIFIED" : "UNSUPPORTED")}");

        bool panelOk = PrintSection("Panel targets", PanelTargets.Panel);
        bool gameplayOk = PrintSection("Gameplay targets", GameplayTargets.Gameplay);
        bool enumsOk = EnumContract.Validate(out var enumMismatches);
        PrintResult("Enum values", enumsOk, enumMismatches, 4);

        int targetCount = PanelTargets.Panel.Length + GameplayTargets.Gameplay.Length;
        bool compatible = buildOk && panelOk && gameplayOk && enumsOk;
        Console.WriteLine();
        Console.WriteLine(compatible
            ? $"SUPPORTED: verified build; all {targetCount} member targets and 4 enum values match."
            : !buildOk && panelOk && gameplayOk && enumsOk
                ? "UNSUPPORTED: API checks pass, but this KSA build has not been behaviorally verified."
                : "INCOMPATIBLE: one or more game API assumptions changed.");
        return compatible ? 0 : 1;
    }

    private static bool PrintSection(string heading, TargetSpec[] specs)
    {
        bool ok = PatchValidator.ValidateAll(specs, out var mismatches);
        PrintResult(heading, ok, mismatches, specs.Length);
        return ok;
    }

    private static void PrintResult(string heading, bool ok, IReadOnlyList<string> mismatches, int count)
    {
        Console.WriteLine();
        Console.WriteLine($"{heading}: {(ok ? "OK" : "FAILED")} ({count} checked)");
        foreach (string mismatch in mismatches)
            Console.WriteLine($"  - {mismatch}");
    }

    private static ParseResult ParseArgs(string[] args)
    {
        string gameDir = Environment.GetEnvironmentVariable("KSA_INSTALL_DIR") ?? DefaultGameDir;
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] is "-h" or "--help")
            {
                PrintUsage();
                return new(null, null, true);
            }
            if (args[i] == "--game-dir" && i + 1 < args.Length)
            {
                gameDir = args[++i];
                continue;
            }
            return new(null, $"unknown or incomplete argument '{args[i]}'", false);
        }
        return new(gameDir, null, false);
    }

    private static void PrintUsage() => Console.WriteLine(
        "Usage: dotnet run --project src/WhiskerDynamics.Compatibility.Cli -- [--game-dir <path>]\n" +
        "Exit codes: 0 compatible, 1 incompatible, 2 usage/install error.");

    private static string Describe(Exception exception)
    {
        Exception root = exception;
        while (root.InnerException is not null) root = root.InnerException;
        return root.Message;
    }

    private sealed record ParseResult(string? GameDir, string? Error, bool ShowedHelp);
}
