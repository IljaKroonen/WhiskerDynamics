using System.Reflection;
using WhiskerDynamics.GameTesting;

namespace WhiskerDynamics.GameTests.Host;

internal sealed record GameTestHostServices(
    IHostFileSystem Files,
    IHostEnvironment Environment,
    IGameProcessPlatform Processes,
    IHostClock Clock,
    IChildCommandRunner Commands,
    TextWriter Output,
    TextWriter Error,
    Func<IReadOnlyDictionary<string, IGameTestScenario>> DiscoverScenarios,
    Func<string> CreateRunId)
{
    internal static GameTestHostServices CreateSystem() => new(
        new SystemHostFileSystem(),
        new SystemHostEnvironment(),
        new SystemGameProcessPlatform(),
        new SystemHostClock(),
        new SystemChildCommandRunner(),
        new CurrentConsoleTextWriter(error: false),
        new CurrentConsoleTextWriter(error: true),
        ScenarioCatalog.Discover,
        () => Guid.NewGuid().ToString("N"));
}

internal interface IHostFileSystem
{
    bool FileExists(string path);
    bool DirectoryExists(string path);
    void CreateDirectory(string path);
    void DeleteFile(string path);
    Task WriteAllTextAsync(string path, string contents);
    Task<string> ReadAllTextAsync(string path);
}

internal interface IHostEnvironment
{
    string BaseDirectory { get; }
    string GetFolderPath(Environment.SpecialFolder folder);
}

internal interface IGameProcess
{
    string ProcessName { get; }
}

internal interface ILaunchedGameSession
{
    void CloseMainWindow();
    void KillTree();
}

internal interface IGameProcessPlatform
{
    IReadOnlyList<IGameProcess> EnumerateProcesses();
    ILaunchedGameSession? StartShell(string fileName, string workingDirectory);
}

internal interface IElapsedTimer
{
    double ElapsedTotalSeconds { get; }
}

internal interface IHostClock
{
    IElapsedTimer StartTimer();
    Task DelayAsync(TimeSpan delay);
    void Sleep(TimeSpan delay);
}

internal interface IChildCommandRunner
{
    Task RunAsync(string fileName, IReadOnlyList<string> arguments, string workingDirectory);
}

internal static class ScenarioCatalog
{
    internal static IReadOnlyDictionary<string, IGameTestScenario> Discover()
    {
        var scenarios = new Dictionary<string, IGameTestScenario>(
            StringComparer.OrdinalIgnoreCase);
        foreach (Type type in Assembly.GetExecutingAssembly().GetTypes()
            .Where(type => !type.IsAbstract && typeof(IGameTestScenario).IsAssignableFrom(type)))
        {
            if (Activator.CreateInstance(type) is not IGameTestScenario scenario)
                throw new InvalidOperationException(
                    $"scenario type '{type.FullName}' needs a public parameterless constructor");
            if (string.IsNullOrWhiteSpace(scenario.Id))
                throw new InvalidOperationException($"scenario type '{type.FullName}' has an empty id");
            if (!scenarios.TryAdd(scenario.Id, scenario))
                throw new InvalidOperationException($"duplicate scenario id '{scenario.Id}'");
        }
        return scenarios;
    }
}
