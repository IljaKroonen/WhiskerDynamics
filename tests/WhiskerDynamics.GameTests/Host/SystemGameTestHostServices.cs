using System.Diagnostics;
using System.Text;

namespace WhiskerDynamics.GameTests.Host;

internal sealed class CurrentConsoleTextWriter(bool error) : TextWriter
{
    private TextWriter Current => error ? Console.Error : Console.Out;
    public override Encoding Encoding => Current.Encoding;
    public override void Write(char value) => Current.Write(value);
    public override void Write(string? value) => Current.Write(value);
    public override void WriteLine(string? value) => Current.WriteLine(value);
}

internal sealed class SystemHostFileSystem : IHostFileSystem
{
    public bool FileExists(string path) => File.Exists(path);
    public void CreateDirectory(string path) => Directory.CreateDirectory(path);
    public void DeleteFile(string path) => File.Delete(path);
    public Task WriteAllTextAsync(string path, string contents) =>
        File.WriteAllTextAsync(path, contents);
    public Task<string> ReadAllTextAsync(string path) => File.ReadAllTextAsync(path);
}

internal sealed class SystemHostEnvironment : IHostEnvironment
{
    public string BaseDirectory => AppContext.BaseDirectory;
    public string GetFolderPath(Environment.SpecialFolder folder) =>
        Environment.GetFolderPath(folder);
}

internal sealed class SystemGameProcessPlatform : IGameProcessPlatform
{
    public IReadOnlyList<IGameProcess> EnumerateProcesses() =>
        Process.GetProcesses().Select(process => (IGameProcess)new SystemGameProcess(process))
            .ToArray();

    public ILaunchedGameSession? StartShell(string fileName, string workingDirectory)
    {
        Process? process = Process.Start(new ProcessStartInfo(fileName)
        {
            WorkingDirectory = workingDirectory,
            UseShellExecute = true,
        });
        return process is null ? null : new SystemGameProcess(process);
    }

    private sealed class SystemGameProcess(Process process) : IGameProcess, ILaunchedGameSession
    {
        public string ProcessName => process.ProcessName;
        public void CloseMainWindow() => process.CloseMainWindow();
        public void KillTree() => process.Kill(entireProcessTree: true);
    }
}

internal sealed class SystemHostClock : IHostClock
{
    public IElapsedTimer StartTimer() => new StopwatchTimer(Stopwatch.StartNew());
    public Task DelayAsync(TimeSpan delay) => Task.Delay(delay);
    public void Sleep(TimeSpan delay) => Thread.Sleep(delay);

    private sealed class StopwatchTimer(Stopwatch stopwatch) : IElapsedTimer
    {
        public double ElapsedTotalSeconds => stopwatch.Elapsed.TotalSeconds;
    }
}

internal sealed class SystemChildCommandRunner : IChildCommandRunner
{
    public async Task RunAsync(
        string fileName, IReadOnlyList<string> arguments, string workingDirectory)
    {
        var info = new ProcessStartInfo(fileName)
        {
            WorkingDirectory = workingDirectory,
            UseShellExecute = false,
        };
        foreach (string argument in arguments) info.ArgumentList.Add(argument);
        using Process process = Process.Start(info)
            ?? throw new InvalidOperationException($"could not start {fileName}");
        await process.WaitForExitAsync();
        if (process.ExitCode != 0)
            throw new InvalidOperationException(
                $"{fileName} exited with code {process.ExitCode}");
    }
}
