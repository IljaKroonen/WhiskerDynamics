using System.Text.Json;
using WhiskerDynamics.GameTesting;
using WhiskerDynamics.GameTests.Host;
using HostRunner = WhiskerDynamics.GameTests.Host.GameTestHost;

namespace WhiskerDynamics.GameTestHost.Tests;

public class GameTestHostTests
{
    [Fact]
    public void Scenario_catalog_discovers_the_executable_scenarios_case_insensitively()
    {
        IReadOnlyDictionary<string, IGameTestScenario> scenarios =
            ScenarioCatalog.Discover();

        Assert.Equal(
            ["create-fixtures", "moon-transfer", "nrho-station-keeping", "rendezvous", "smoke"],
            scenarios.Keys.Order());
        Assert.True(scenarios.ContainsKey("SMOKE"));
        Assert.Equal("smoke", scenarios["SMOKE"].Id);
    }

    [Fact]
    public async Task No_arguments_prints_usage_and_sorted_scenarios_without_side_effects()
    {
        var harness = new Harness(
            new Dictionary<string, IGameTestScenario>
            {
                ["z-last"] = new ScenarioDefinition("z-last"),
                ["a-first"] = new ScenarioDefinition("a-first"),
            });

        int exitCode = await harness.Host.RunAsync([]);

        Assert.Equal(2, exitCode);
        Assert.Contains("usage: dotnet run --project tests/WhiskerDynamics.GameTests",
            harness.Output.ToString());
        Assert.True(
            harness.Output.ToString().IndexOf("a-first", StringComparison.Ordinal)
            < harness.Output.ToString().IndexOf("z-last", StringComparison.Ordinal));
        Assert.Empty(harness.Files.Operations);
        Assert.Equal(0, harness.Processes.EnumerateCalls);
    }

    [Fact]
    public async Task Successful_run_preserves_deploy_exchange_launch_poll_and_cleanup_order()
    {
        var harness = new Harness();
        var session = new FakeProcess("StarMap", harness.Trace);
        harness.Processes.LaunchedSession = session;
        harness.Processes.Enumerations.Enqueue([]);
        harness.Clock.OnDelay = () => harness.WriteResult(passed: true);

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(0, exitCode);
        Assert.Collection(harness.Commands.Runs,
            publish =>
            {
                Assert.Equal("dotnet", publish.FileName);
                Assert.Equal(
                [
                    "publish",
                    "-c", "Release",
                    "tests/WhiskerDynamics.GameTestDriver/WhiskerDynamics.GameTestDriver.csproj",
                    "--disable-build-servers",
                    "-m:1",
                ], publish.Arguments);
                Assert.Equal(harness.Root, publish.WorkingDirectory);
            },
            deployMod => Assert.Equal(
                ["run", "--file", "scripts/deploy-mod.cs", "--", "Release"],
                deployMod.Arguments),
            deployDriver => Assert.Equal(
                [
                    "run", "--file", "scripts/deploy-mod.cs", "--",
                    "--game-test-driver", "Release",
                ], deployDriver.Arguments));
        Assert.Equal(
            (harness.StarMapPath, harness.StarMapDirectory),
            Assert.Single(harness.Processes.Launches));
        Assert.Equal(1, session.CloseCalls);
        Assert.Equal(1, session.KillCalls);
        Assert.Equal(1, harness.Processes.EnumerateCalls);
        Assert.Equal([TimeSpan.FromMilliseconds(1500)], harness.Clock.Sleeps);
        Assert.False(harness.Files.Files.ContainsKey(harness.RequestPath));
        Assert.True(harness.Files.Files.ContainsKey(harness.ResultPath));
        string requestJson = Assert.Single(
            harness.Files.Writes, write => write.Path == harness.RequestPath).Contents;
        var request = JsonSerializer.Deserialize<GameTestScenario>(
            requestJson, GameTestProtocol.JsonOptions);
        Assert.Equal("run-1", request!.RunId);
        Assert.Contains("PASS: scenario (1.5 s)", harness.Output.ToString());
        Assert.Equal("", harness.Error.ToString());

        AssertOrdered(
            harness.Trace,
            "processes:enumerate",
            "command:dotnet",
            $"mkdir:{harness.ModDirectory}",
            $"delete:{harness.ResultPath}",
            $"delete:{harness.ResultPath}.tmp",
            $"write:{harness.RequestPath}",
            $"launch:{harness.StarMapPath}",
            "timer:start",
            "delay:250",
            $"read:{harness.ResultPath}",
            "close:StarMap",
            "sleep:1500",
            "kill:StarMap",
            $"delete:{harness.RequestPath}");
    }

    [Fact]
    public async Task Malformed_and_stale_results_are_ignored_until_the_run_id_matches()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);
        harness.Files.ReadResponses.Enqueue("{");
        harness.Files.ReadResponses.Enqueue(JsonSerializer.Serialize(
            harness.Result("stale-run", passed: true), GameTestProtocol.JsonOptions));
        harness.Files.ReadResponses.Enqueue(JsonSerializer.Serialize(
            harness.Result("run-1", passed: true), GameTestProtocol.JsonOptions));
        harness.Clock.OnDelay = () => harness.Files.SetFile(harness.ResultPath, "available");

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(0, exitCode);
        Assert.Equal(3, harness.Clock.DelayCalls);
        Assert.Equal(3, harness.Files.ReadCalls);
    }

    [Fact]
    public async Task Timeout_still_stops_launched_session_deletes_request_and_returns_host_error()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);

        int exitCode = await harness.Host.RunAsync(["smoke", "--timeout", "0.5"]);

        Assert.Equal(2, exitCode);
        Assert.Equal(2, harness.Clock.DelayCalls);
        Assert.Equal(1, harness.Processes.EnumerateCalls);
        AssertLaunchedSessionCleaned(harness);
        Assert.Contains("game test host failed: no result after 0.5 wall seconds",
            harness.Error.ToString());
    }

    [Fact]
    public async Task Launcher_failure_deletes_the_written_request()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);
        harness.Processes.LaunchFailure = new InvalidOperationException("launcher failed");

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(2, exitCode);
        Assert.False(harness.Files.Files.ContainsKey(harness.RequestPath));
        Assert.Equal(0, harness.Processes.LaunchedSession.CloseCalls);
        Assert.Equal(0, harness.Processes.LaunchedSession.KillCalls);
        Assert.Empty(harness.Clock.Sleeps);
        Assert.Contains("game test host failed: launcher failed", harness.Error.ToString());
    }

    [Fact]
    public async Task Poll_failure_stops_the_launched_session_and_deletes_the_request()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);
        harness.Clock.DelayFailure = new IOException("poll failed");

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(2, exitCode);
        AssertLaunchedSessionCleaned(harness);
        Assert.Contains("game test host failed: poll failed", harness.Error.ToString());
    }

    [Fact]
    public async Task Read_failure_stops_the_launched_session_and_deletes_the_request()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);
        harness.Files.ReadFailure = new IOException("read failed");
        harness.Clock.OnDelay = () => harness.Files.SetFile(harness.ResultPath, "available");

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(2, exitCode);
        AssertLaunchedSessionCleaned(harness);
        Assert.Contains("game test host failed: read failed", harness.Error.ToString());
    }

    [Fact]
    public async Task Unrelated_matching_process_started_during_polling_is_not_stopped()
    {
        var harness = new Harness();
        var unrelated = new FakeProcess("KSA", harness.Trace);
        harness.Processes.Enumerations.Enqueue([]);
        harness.Clock.OnDelay = () =>
        {
            harness.Processes.Enumerations.Enqueue([unrelated]);
            harness.WriteResult(passed: true);
        };

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(0, exitCode);
        AssertLaunchedSessionCleaned(harness);
        Assert.Equal(1, harness.Processes.EnumerateCalls);
        Assert.Single(harness.Processes.Enumerations);
        Assert.Equal(0, unrelated.CloseCalls);
        Assert.Equal(0, unrelated.KillCalls);
    }

    [Fact]
    public async Task Keep_running_and_no_deploy_skip_only_their_current_side_effects()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);
        harness.Clock.OnDelay = () => harness.WriteResult(passed: true);

        int exitCode = await harness.Host.RunAsync(
            ["smoke", "--no-deploy", "--keep-game-running"]);

        Assert.Equal(0, exitCode);
        Assert.Empty(harness.Commands.Runs);
        Assert.Equal(1, harness.Processes.EnumerateCalls);
        Assert.Empty(harness.Clock.Sleeps);
        Assert.False(harness.Files.Files.ContainsKey(harness.RequestPath));
    }

    [Fact]
    public async Task Existing_game_process_aborts_before_deploy_or_file_exchange()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([new FakeProcess("KSA")]);

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(2, exitCode);
        Assert.Empty(harness.Commands.Runs);
        Assert.Empty(harness.Files.Operations);
        Assert.Empty(harness.Processes.Launches);
        Assert.Contains("KSA/StarMap is already running", harness.Error.ToString());
    }

    [Fact]
    public async Task Unknown_scenario_and_invalid_timeout_keep_the_host_error_contract()
    {
        var unknown = new Harness();

        Assert.Equal(2, await unknown.Host.RunAsync(["missing"]));
        Assert.Equal(
            "game test host failed: unknown scenario 'missing'; use --list to show available scenarios"
            + Environment.NewLine,
            unknown.Error.ToString());
        Assert.Empty(unknown.Files.Operations);
        Assert.Equal(0, unknown.Processes.EnumerateCalls);

        var invalidTimeout = new Harness();
        Assert.Equal(2, await invalidTimeout.Host.RunAsync(["smoke", "--timeout", "NaN"]));
        Assert.Equal(
            "game test host failed: --timeout must be finite and positive"
            + Environment.NewLine,
            invalidTimeout.Error.ToString());
        Assert.Equal(0, invalidTimeout.Processes.EnumerateCalls);
    }

    [Fact]
    public async Task Failed_game_result_returns_one_and_prints_the_game_error()
    {
        var harness = new Harness();
        harness.Processes.Enumerations.Enqueue([]);
        harness.Clock.OnDelay = () => harness.WriteResult(passed: false);

        int exitCode = await harness.Host.RunAsync(["smoke"]);

        Assert.Equal(1, exitCode);
        Assert.Contains("FAIL: scenario (1.5 s)", harness.Output.ToString());
        Assert.Contains("scenario failed", harness.Error.ToString());
    }

    private static void AssertOrdered(IReadOnlyList<string> actual, params string[] expected)
    {
        int cursor = -1;
        foreach (string operation in expected)
        {
            int found = actual.IndexOf(operation, cursor + 1);
            Assert.True(found > cursor, $"'{operation}' was not found in order");
            cursor = found;
        }
    }

    private static void AssertLaunchedSessionCleaned(Harness harness)
    {
        Assert.False(harness.Files.Files.ContainsKey(harness.RequestPath));
        Assert.Equal(1, harness.Processes.LaunchedSession.CloseCalls);
        Assert.Equal(1, harness.Processes.LaunchedSession.KillCalls);
        Assert.Equal([TimeSpan.FromMilliseconds(1500)], harness.Clock.Sleeps);
    }

    private sealed class Harness
    {
        internal readonly string Root = Path.GetFullPath(
            Path.Combine(Path.GetTempPath(), $"wd-host-{Guid.NewGuid():N}"));
        internal readonly List<string> Trace = [];
        internal readonly FakeFileSystem Files;
        internal readonly FakeProcesses Processes;
        internal readonly FakeClock Clock;
        internal readonly FakeCommands Commands;
        internal readonly StringWriter Output = new();
        internal readonly StringWriter Error = new();
        internal readonly HostRunner Host;

        internal string Documents => Path.Combine(Root, "documents");
        internal string LocalAppData => Path.Combine(Root, "local");
        internal string ModDirectory => Path.Combine(
            Documents, "My Games", "Kitten Space Agency", "mods", "WhiskerDynamics");
        internal string RequestPath =>
            Path.Combine(ModDirectory, GameTestProtocol.RequestFileName);
        internal string ResultPath =>
            Path.Combine(ModDirectory, GameTestProtocol.ResultFileName);
        internal string StarMapDirectory => Path.Combine(LocalAppData, "StarMap");
        internal string StarMapPath => Path.Combine(StarMapDirectory, "StarMap.exe");

        internal Harness(IReadOnlyDictionary<string, IGameTestScenario>? scenarios = null)
        {
            Files = new FakeFileSystem(Trace);
            Processes = new FakeProcesses(Trace);
            Clock = new FakeClock(Trace);
            Commands = new FakeCommands(Trace);
            Files.SetFile(Path.Combine(Root, "WhiskerDynamics.slnx"), "solution");
            Files.SetFile(StarMapPath, "launcher");
            var environment = new FakeEnvironment(
                Path.Combine(Root, "tests", "bin"), Documents, LocalAppData);
            var services = new GameTestHostServices(
                Files,
                environment,
                Processes,
                Clock,
                Commands,
                Output,
                Error,
                () => scenarios ?? new Dictionary<string, IGameTestScenario>
                {
                    ["smoke"] = new ScenarioDefinition("smoke"),
                },
                () => "run-1");
            Host = new HostRunner(services);
        }

        internal GameTestResult Result(string runId, bool passed) => new()
        {
            RunId = runId,
            Scenario = "scenario",
            Passed = passed,
            Error = passed ? null : "scenario failed",
            ElapsedWallSeconds = 1.5,
            Steps =
            [
                new GameTestStepResult
                {
                    Index = 0,
                    Action = "assert",
                    Passed = passed,
                    Detail = passed ? "ok" : "failed",
                },
            ],
        };

        internal void WriteResult(bool passed) => Files.SetFile(
            ResultPath,
            JsonSerializer.Serialize(
                Result("run-1", passed), GameTestProtocol.JsonOptions));
    }

    private sealed class ScenarioDefinition(string id) : IGameTestScenario
    {
        public string Id => id;
        public GameTestScenario Create() => new()
        {
            Name = "scenario",
            TimeoutSeconds = 10,
            Steps = [new GameTestStep { Action = "assert" }],
        };
    }

    private sealed class FakeFileSystem(List<string> trace) : IHostFileSystem
    {
        internal readonly Dictionary<string, string> Files =
            new(StringComparer.OrdinalIgnoreCase);
        internal readonly HashSet<string> Directories =
            new(StringComparer.OrdinalIgnoreCase);
        internal readonly List<string> Operations = [];
        internal readonly List<(string Path, string Contents)> Writes = [];
        internal readonly Queue<string> ReadResponses = [];
        internal Exception? ReadFailure;
        internal int ReadCalls;

        public bool FileExists(string path) => Files.ContainsKey(path);
        public bool DirectoryExists(string path) => Directories.Contains(path);
        public void CreateDirectory(string path)
        {
            Directories.Add(path);
            Operations.Add($"mkdir:{path}");
            trace.Add($"mkdir:{path}");
        }
        public void DeleteFile(string path)
        {
            Files.Remove(path);
            Operations.Add($"delete:{path}");
            trace.Add($"delete:{path}");
        }
        public Task WriteAllTextAsync(string path, string contents)
        {
            Files[path] = contents;
            Writes.Add((path, contents));
            Operations.Add($"write:{path}");
            trace.Add($"write:{path}");
            return Task.CompletedTask;
        }
        public Task<string> ReadAllTextAsync(string path)
        {
            ReadCalls++;
            trace.Add($"read:{path}");
            if (ReadFailure is not null)
                return Task.FromException<string>(ReadFailure);
            return Task.FromResult(
                ReadResponses.Count > 0 ? ReadResponses.Dequeue() : Files[path]);
        }
        internal void SetFile(string path, string contents) => Files[path] = contents;
    }

    private sealed record FakeEnvironment(
        string BaseDirectory,
        string Documents,
        string LocalAppData) : IHostEnvironment
    {
        public string GetFolderPath(Environment.SpecialFolder folder) => folder switch
        {
            Environment.SpecialFolder.MyDocuments => Documents,
            Environment.SpecialFolder.LocalApplicationData => LocalAppData,
            _ => "",
        };
    }

    private sealed class FakeProcess(
        string processName, List<string>? trace = null) : IGameProcess, ILaunchedGameSession
    {
        public string ProcessName => processName;
        internal int CloseCalls;
        internal int KillCalls;
        public void CloseMainWindow()
        {
            CloseCalls++;
            trace?.Add($"close:{processName}");
        }
        public void KillTree()
        {
            KillCalls++;
            trace?.Add($"kill:{processName}");
        }
    }

    private sealed class FakeProcesses(List<string> trace) : IGameProcessPlatform
    {
        internal readonly Queue<IReadOnlyList<IGameProcess>> Enumerations = [];
        internal readonly List<(string FileName, string WorkingDirectory)> Launches = [];
        internal FakeProcess LaunchedSession = new("StarMap", trace);
        internal Exception? LaunchFailure;
        internal int EnumerateCalls;

        public IReadOnlyList<IGameProcess> EnumerateProcesses()
        {
            EnumerateCalls++;
            trace.Add("processes:enumerate");
            return Enumerations.Count == 0 ? [] : Enumerations.Dequeue();
        }

        public ILaunchedGameSession? StartShell(string fileName, string workingDirectory)
        {
            Launches.Add((fileName, workingDirectory));
            trace.Add($"launch:{fileName}");
            if (LaunchFailure is not null) throw LaunchFailure;
            return LaunchedSession;
        }
    }

    private sealed class FakeClock(List<string> trace) : IHostClock
    {
        private readonly FakeTimer _timer = new();
        internal readonly List<TimeSpan> Sleeps = [];
        internal int DelayCalls;
        internal Action? OnDelay;
        internal Exception? DelayFailure;

        public IElapsedTimer StartTimer()
        {
            trace.Add("timer:start");
            return _timer;
        }
        public Task DelayAsync(TimeSpan delay)
        {
            DelayCalls++;
            _timer.Elapsed += delay.TotalSeconds;
            trace.Add($"delay:{delay.TotalMilliseconds:F0}");
            OnDelay?.Invoke();
            return DelayFailure is null
                ? Task.CompletedTask
                : Task.FromException(DelayFailure);
        }
        public void Sleep(TimeSpan delay)
        {
            Sleeps.Add(delay);
            trace.Add($"sleep:{delay.TotalMilliseconds:F0}");
        }

        private sealed class FakeTimer : IElapsedTimer
        {
            internal double Elapsed;
            public double ElapsedTotalSeconds => Elapsed;
        }
    }

    private sealed class FakeCommands(List<string> trace) : IChildCommandRunner
    {
        internal readonly List<(
            string FileName, IReadOnlyList<string> Arguments, string WorkingDirectory)> Runs = [];

        public Task RunAsync(
            string fileName, IReadOnlyList<string> arguments, string workingDirectory)
        {
            Runs.Add((fileName, arguments.ToArray(), workingDirectory));
            trace.Add($"command:{fileName}");
            return Task.CompletedTask;
        }
    }
}

internal static class ListSearch
{
    internal static int IndexOf(
        this IReadOnlyList<string> values, string value, int startIndex)
    {
        for (int i = startIndex; i < values.Count; i++)
            if (values[i] == value) return i;
        return -1;
    }
}
