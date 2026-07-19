namespace WhiskerDynamics.Mod.Diagnostics;

/// <summary>Queued file log plus an atomically published bounded panel snapshot.</summary>
public static class ModLog
{
    private const int RecentLimit = 200;
    private static readonly object RecentGate = new();
    private static readonly object StartGate = new();
    private static readonly Queue<string> Recent = new();
    private static readonly System.Collections.Concurrent.BlockingCollection<string> Pending = new(4096);
    private static string[] _snapshot = [];
    private static string _path = "";
    private static Thread? _thread;
    private static long _version;

    public static void Init(string path)
    {
        Volatile.Write(ref _path, path);
        try { File.WriteAllText(path, ""); } catch { }
        EnsureStarted();
    }

    public static void Info(string message) => Write("INFO ", message);
    public static void Warn(string message) => Write("WARN ", message);
    public static void Error(string message) => Write("ERROR", message);
    public static IReadOnlyList<string> Snapshot() => Volatile.Read(ref _snapshot);
    public static long Version => Volatile.Read(ref _version);

    private static void Write(string level, string message)
    {
        string line = $"{DateTime.Now:HH:mm:ss.fff} [{level}] {message}";
        lock (RecentGate)
        {
            Recent.Enqueue(line);
            while (Recent.Count > RecentLimit) Recent.Dequeue();
            Volatile.Write(ref _snapshot, Recent.ToArray());
            Interlocked.Increment(ref _version);
        }
        EnsureStarted();
        try { if (!Pending.IsAddingCompleted) Pending.TryAdd(line); }
        catch (InvalidOperationException) { /* cooperative shutdown raced this producer */ }
    }

    private static void EnsureStarted()
    {
        if (Volatile.Read(ref _thread) is not null || Pending.IsAddingCompleted) return;
        lock (StartGate)
        {
            if (_thread is not null || Pending.IsAddingCompleted) return;
            _thread = new Thread(WriterLoop)
            {
                IsBackground = true, Name = "whiskerdynamics-log",
                Priority = ThreadPriority.BelowNormal,
            };
            _thread.Start();
            AppDomain.CurrentDomain.ProcessExit += (_, _) => Shutdown(500);
        }
    }

    private static void WriterLoop()
    {
        foreach (string line in Pending.GetConsumingEnumerable())
        {
            try
            {
                string path = Volatile.Read(ref _path);
                if (path.Length > 0) File.AppendAllText(path, line + Environment.NewLine);
            }
            catch { }
        }
    }

    internal static bool Shutdown(int timeoutMs)
    {
        if (!Pending.IsAddingCompleted) Pending.CompleteAdding();
        var thread = Volatile.Read(ref _thread);
        return thread is null || thread.Join(Math.Max(0, timeoutMs));
    }
}
