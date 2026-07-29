using System.Globalization;
using System.Text.Json;
using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Host;

internal sealed class GameTestHost(GameTestHostServices services)
{
    internal async Task<int> RunAsync(string[] args)
    {
        try
        {
            IReadOnlyDictionary<string, IGameTestScenario> available =
                services.DiscoverScenarios();
            if (args.Length == 0 || args.Contains("--help") || args.Contains("--list"))
            {
                services.Output.WriteLine(
                    "usage: dotnet run --project tests/WhiskerDynamics.GameTests -- "
                    + "<scenario-id> [--timeout <wall-seconds>] "
                    + "[--keep-game-running] [--no-deploy]");
                services.Output.WriteLine("available scenarios:");
                foreach (string id in available.Keys.Order())
                    services.Output.WriteLine($"  {id}");
                return args.Length == 0 ? 2 : 0;
            }

            string scenarioId = args[0];
            if (!available.TryGetValue(scenarioId, out IGameTestScenario? definition))
                throw new ArgumentException(
                    $"unknown scenario '{scenarioId}'; use --list to show available scenarios");
            bool keepRunning = args.Contains("--keep-game-running");
            bool deploy = !args.Contains("--no-deploy");
            GameTestScenario scenario = definition.Create();
            scenario.RunId = services.CreateRunId();
            Validate(scenario);

            double hostTimeout =
                ReadOption(args, "--timeout") ?? scenario.TimeoutSeconds + 120.0;
            if (!(hostTimeout > 0) || !double.IsFinite(hostTimeout))
                throw new ArgumentException("--timeout must be finite and positive");

            IGameProcess[] existing = FindGameProcesses();
            if (existing.Length != 0)
                throw new InvalidOperationException(
                    "KSA/StarMap is already running; close it before starting an isolated game test");

            string repoRoot = FindRepoRoot(services.Environment.BaseDirectory);
            if (deploy)
            {
                await services.Commands.RunAsync(
                    "dotnet",
                    [
                        "publish",
                        "-c", "Release",
                        "tests/WhiskerDynamics.GameTestDriver/WhiskerDynamics.GameTestDriver.csproj",
                        "--disable-build-servers",
                        "-m:1",
                    ],
                    repoRoot);
                await services.Commands.RunAsync(
                    "dotnet",
                    ["run", "--file", "scripts/deploy-mod.cs", "--", "Release"],
                    repoRoot);
                await services.Commands.RunAsync(
                    "dotnet",
                    [
                        "run", "--file", "scripts/deploy-mod.cs", "--",
                        "--game-test-driver", "Release",
                    ],
                    repoRoot);
            }

            string documents = services.Environment.GetFolderPath(
                Environment.SpecialFolder.MyDocuments);
            string modDirectory = Path.Combine(
                documents, "My Games", "Kitten Space Agency", "mods", "WhiskerDynamics");

            string requestPath = Path.Combine(
                modDirectory, GameTestProtocol.RequestFileName);
            string resultPath = Path.Combine(
                modDirectory, GameTestProtocol.ResultFileName);
            services.Files.CreateDirectory(modDirectory);
            services.Files.DeleteFile(resultPath);
            services.Files.DeleteFile(resultPath + ".tmp");
            GameTestResult? result = null;
            try
            {
                await services.Files.WriteAllTextAsync(
                    requestPath,
                    JsonSerializer.Serialize(scenario, GameTestProtocol.JsonOptions));

                string starMapDirectory = Path.Combine(
                    services.Environment.GetFolderPath(
                        Environment.SpecialFolder.LocalApplicationData),
                    "StarMap");
                string starMapPath = Path.Combine(starMapDirectory, "StarMap.exe");
                if (!services.Files.FileExists(starMapPath))
                    throw new FileNotFoundException(
                        "StarMap launcher was not found", starMapPath);

                services.Output.WriteLine($"starting '{scenario.Name}' ({scenario.RunId})");
                ILaunchedGameSession launchedSession =
                    services.Processes.StartShell(starMapPath, starMapDirectory)
                    ?? throw new InvalidOperationException("could not launch StarMap");
                try
                {
                    IElapsedTimer timer = services.Clock.StartTimer();
                    while (timer.ElapsedTotalSeconds < hostTimeout)
                    {
                        await services.Clock.DelayAsync(TimeSpan.FromMilliseconds(250));
                        if (!services.Files.FileExists(resultPath)) continue;
                        try
                        {
                            result = JsonSerializer.Deserialize<GameTestResult>(
                                await services.Files.ReadAllTextAsync(resultPath),
                                GameTestProtocol.JsonOptions);
                            if (result?.RunId == scenario.RunId) break;
                            result = null;
                        }
                        catch (JsonException)
                        {
                            // Atomic rename makes this unlikely; tolerate antivirus/indexer races.
                        }
                    }
                }
                finally
                {
                    if (!keepRunning) StopGameSession(launchedSession);
                }
            }
            finally
            {
                services.Files.DeleteFile(requestPath);
            }

            if (result is null)
                throw new TimeoutException(
                    $"no result after {hostTimeout:F1} wall seconds; inspect "
                    + Path.Combine(modDirectory, "whiskerdynamics.log"));

            foreach (GameTestStepResult step in result.Steps)
                services.Output.WriteLine(
                    $"{(step.Passed ? "PASS" : "FAIL")} {step.Index}: "
                    + $"{step.Action} - {step.Detail}");
            services.Output.WriteLine(
                $"{(result.Passed ? "PASS" : "FAIL")}: {result.Scenario} "
                + $"({result.ElapsedWallSeconds:F1} s)");
            if (!result.Passed && result.Error is not null)
                services.Error.WriteLine(result.Error);
            return result.Passed ? 0 : 1;
        }
        catch (Exception e)
        {
            services.Error.WriteLine($"game test host failed: {e.Message}");
            return 2;
        }
    }

    private static void Validate(GameTestScenario scenario)
    {
        if (string.IsNullOrWhiteSpace(scenario.Name))
            throw new InvalidDataException("name is required");
        if (!(scenario.TimeoutSeconds > 0) || !double.IsFinite(scenario.TimeoutSeconds))
            throw new InvalidDataException("timeoutSeconds must be finite and positive");
        if (scenario.Steps.Count == 0)
            throw new InvalidDataException("at least one step is required");
    }

    private static double? ReadOption(string[] args, string name)
    {
        int index = Array.IndexOf(args, name);
        if (index < 0) return null;
        if (index + 1 >= args.Length || !double.TryParse(
                args[index + 1],
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out double value))
            throw new ArgumentException($"{name} requires a number");
        return value;
    }

    private string FindRepoRoot(string start)
    {
        DirectoryInfo? current = new(start);
        while (current is not null)
        {
            if (services.Files.FileExists(
                    Path.Combine(current.FullName, "WhiskerDynamics.slnx")))
                return current.FullName;
            current = current.Parent;
        }
        throw new DirectoryNotFoundException("could not locate repository root");
    }

    private IGameProcess[] FindGameProcesses() => services.Processes.EnumerateProcesses()
        .Where(process => process.ProcessName is "KSA" or "StarMap")
        .ToArray();

    private void StopGameSession(ILaunchedGameSession session)
    {
        try { session.CloseMainWindow(); } catch { }
        try
        {
            services.Clock.Sleep(TimeSpan.FromMilliseconds(1500));
        }
        finally
        {
            try { session.KillTree(); } catch { }
        }
    }
}
