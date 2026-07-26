namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Latest-wins keyed job queue (KSA-free, offline-tested): one pending slot
/// per key. A producer outrunning the consumer replaces pending work, while the
/// active ticket keeps its publication identity until completion so continuous
/// capture cannot starve it. A throwing job is contained per drain.</summary>
public sealed class KeyedLatestQueue(Action<string, Exception> onJobError)
{
    private sealed record Entry(long Version, Action Job, Action? OnDiscard);
    private readonly object _gate = new();
    private readonly Dictionary<string, Entry> _pending = new(StringComparer.Ordinal);
    private readonly Dictionary<string, long> _running = new(StringComparer.Ordinal);
    private readonly Queue<string> _order = new();
    private readonly HashSet<string> _queued = new(StringComparer.Ordinal);
    private long _nextVersion;

    public int PendingCount { get { lock (_gate) return _pending.Count; } }

    public long Enqueue(string key, Action job)
        => Enqueue(key, _ => job);

    public long Enqueue(string key, Action job, Action onDiscard)
        => Enqueue(key, _ => job, onDiscard);

    public long Enqueue(string key, Func<long, Action> jobFactory)
        => Enqueue(key, jobFactory, onDiscard: null);

    public long Enqueue(string key, Func<long, Action> jobFactory, Action? onDiscard)
    {
        lock (_gate)
            return EnqueueLocked(key, jobFactory, onDiscard);
    }

    public bool TryEnqueue(string key, Func<bool> canEnqueue, Func<long, Action> jobFactory,
        Action? onDiscard = null)
    {
        lock (_gate)
        {
            if (!canEnqueue()) return false;
            EnqueueLocked(key, jobFactory, onDiscard);
            return true;
        }
    }

    private long EnqueueLocked(
        string key, Func<long, Action> jobFactory, Action? onDiscard)
    {
        long version = ++_nextVersion;
        var entry = new Entry(version, jobFactory(version), onDiscard);
        if (_pending.TryGetValue(key, out var replaced))
            DiscardLocked(key, replaced);
        _pending[key] = entry;
        if (_queued.Add(key)) _order.Enqueue(key);
        return version;
    }

    private void DiscardLocked(string key, Entry entry)
    {
        try
        {
            entry.OnDiscard?.Invoke();
        }
        catch (Exception e)
        {
            onJobError(key, e);
        }
    }

    public bool IsCurrent(string key, long version)
    {
        lock (_gate)
            return _running.TryGetValue(key, out long running) && running == version
                || !_running.ContainsKey(key)
                    && _pending.TryGetValue(key, out var pending)
                    && pending.Version == version;
    }

    public bool RunIfCurrent(string key, long version, Action action)
    {
        lock (_gate)
        {
            bool current = _running.TryGetValue(key, out long running)
                ? running == version
                : _pending.TryGetValue(key, out var pending) && pending.Version == version;
            if (!current)
                return false;
            action();
            return true;
        }
    }

    public void Clear(Action? whileLocked = null)
    {
        lock (_gate)
        {
            whileLocked?.Invoke();
            foreach (var (key, entry) in _pending)
                DiscardLocked(key, entry);
            _pending.Clear();
            // Invalidate in-flight tickets too. Their delegates continue only until
            // their next cooperative check; generation-aware publication supplies
            // the same hard session boundary for the production worker.
            _running.Clear();
            _order.Clear();
            _queued.Clear();
        }
    }

    /// <summary>Revokes both queued and running work for one key. A running delegate
    /// continues only until its next <see cref="IsCurrent"/> / <see cref="RunIfCurrent"/>
    /// check; removal from <c>_running</c> makes publication fail atomically against
    /// this call. Stale order entries are harmless and are skipped by <see cref="Drain"/>.</summary>
    public void Cancel(string key)
    {
        lock (_gate)
        {
            if (_pending.Remove(key, out var pending))
                DiscardLocked(key, pending);
            _running.Remove(key);
            _queued.Remove(key);
        }
    }

    /// <summary>Runs everything pending on the CALLER's thread; returns how many ran.</summary>
    public int Drain(int maxJobs = int.MaxValue, long maxElapsedMs = long.MaxValue)
    {
        if (maxJobs <= 0) return 0;
        long started = Environment.TickCount64;
        int ran = 0;
        while (ran < maxJobs && Environment.TickCount64 - started <= maxElapsedMs)
        {
            string key;
            Entry entry;
            lock (_gate)
            {
                if (_order.Count == 0) break;
                key = _order.Dequeue();
                _queued.Remove(key);
                if (!_pending.Remove(key, out entry!)) continue;
                // A newer same-key capture may replace the PENDING slot while this
                // ticket runs, but must not revoke the active ticket: otherwise a
                // producer that stays ahead of a long rebuild can starve the vessel
                // forever. Session Clear still removes this active identity.
                _running[key] = entry.Version;
            }
            try
            {
                entry.Job();
            }
            catch (Exception e)
            {
                onJobError(key, e);
            }
            finally
            {
                lock (_gate)
                {
                    if (_running.TryGetValue(key, out long running)
                        && running == entry.Version)
                        _running.Remove(key);
                    // Enqueue normally installs the order entry itself. Reassert it
                    // here so a same-key capture racing a session reset/finalization
                    // cannot leave a pending slot without an eventual drain.
                    if (_pending.ContainsKey(key) && _queued.Add(key))
                        _order.Enqueue(key);
                }
            }
            ran++;
        }
        return ran;
    }
}

/// <summary>Overlay rebuild worker: vessel batch rebuilds are too heavy to run
/// SYNCHRONOUSLY inside the vehicle update task postfix — a full-horizon
/// display integration plus the dense adaptive sweep, every second per vessel (or
/// continuously during live physics), would stall a physics task thread for tens of
/// milliseconds per cycle: a rhythmic map hitch. Producers instead CAPTURE the
/// rebuild's game-state inputs on the task thread (cheap: burn scan, engine scalars,
/// seed states) and enqueue the sampling/folding/publishing here, on one dedicated
/// background thread — the same split the rails worker uses for celestial curves.
///
/// One thread = global serialization: every TrackedVessel overlay-cache field
/// (LastPlanned*, ghost cache, actual-display reuse) keeps a single writer by
/// construction. Queue semantics live in <see cref="KeyedLatestQueue"/>.</summary>
public static class OverlayWorker
{
    private static readonly KeyedLatestQueue Queue = new(
        (key, e) => ModLog.Warn($"overlay worker job '{key}' contained: {e.Message}"));
    private static readonly AutoResetEvent Signal = new(false);
    private static readonly object StartGate = new();
    private static Thread? _thread;
    private static int _generation;
    private static long _nextLagWarnMs;

    /// <summary>Aggregate-saturation telemetry: a drain cycle longer than this warns
    /// (throttled) — past ~5 s ordinary freshness relies on the bounded rebuild
    /// lease, so the warning fires with margin before stale-while-revalidate becomes
    /// the steady state and while the config knobs can still be lowered.</summary>
    private const long DrainLagWarnMs = 2_500;

    /// <summary>Queue depth (panel/log evidence).</summary>
    public static int PendingCount => Queue.PendingCount;

    /// <summary>Session generation: bumped by the statics sweep. Jobs capture it at
    /// enqueue and refuse to run or publish when it moves, so an in-flight rebuild
    /// captured against the OLD session's rails can never surface a pre-load
    /// trajectory in the new one.</summary>
    public static int CurrentGeneration => Volatile.Read(ref _generation);

    /// <summary>Replaces any pending job under the same key (latest wins) and wakes
    /// the worker. The job runs on the worker thread; it should contain its own
    /// failures (the queue's containment is a backstop, not a reporting channel).</summary>
    public static bool Enqueue(string key, int generation, Action job)
        => Enqueue(key, generation, static () => true, (_, _) => job());

    /// <summary>The predicate turns true when the session resets. New same-key work
    /// coalesces in the pending slot without revoking the active ticket; long-running
    /// rebuilds still poll for the hard session boundary cooperatively.</summary>
    public static bool Enqueue(string key, int generation, Action<Func<bool>> job)
        => Enqueue(key, generation, static () => true,
            (superseded, _) => job(superseded));

    public static bool Enqueue(string key, int generation,
        Action<Func<bool>, Func<Action, bool>> job)
        => Enqueue(key, generation, static () => true, job);

    /// <summary>Admission and same-key replacement are one queue-gate transaction.
    /// A producer that resumes after rebind/reseed therefore cannot borrow the new
    /// worker generation and evict a valid new-session pending job.</summary>
    internal static bool Enqueue(string key, int generation, Func<bool> admissionStillCurrent,
        Action<Func<bool>, Func<Action, bool>> job, Action? onDiscard = null)
    {
        EnsureStarted();
        bool accepted = Queue.TryEnqueue(key,
            () => generation == CurrentGeneration && admissionStillCurrent(),
            ticket => () => job(
                () => generation != CurrentGeneration || !Queue.IsCurrent(key, ticket),
                action => generation == CurrentGeneration
                    && Queue.RunIfCurrent(key, ticket, action)),
            onDiscard);
        if (!accepted) return false;
        Signal.Set();
        return true;
    }

    /// <summary>Statics sweep: a rebind must not run jobs captured against the old
    /// session's objects — pending jobs are dropped and generation-aware buffer
    /// publication rejects any IN-FLIGHT result. The thread itself persists
    /// (background, idles on the signal).</summary>
    internal static void ResetSessionStatics(Action? whileQueueLocked = null)
    {
        // Analysis admission holds its own queue gate while consulting this queue.
        // Take those gates in the same order during reset to avoid an AB/BA deadlock,
        // while keeping both clears atomic against new-session analysis admission.
        OverlayAnalysisWorker.ResetSessionStatics(() => Queue.Clear(() =>
        {
            Interlocked.Increment(ref _generation);
            whileQueueLocked?.Invoke();
        }));
    }

    /// <summary>Revokes one vessel without moving the process-wide session generation.</summary>
    internal static void Cancel(string key) => Queue.Cancel(key);

    private static void EnsureStarted()
    {
        if (_thread is not null) return;
        lock (StartGate)
        {
            if (_thread is not null) return;
            _thread = new Thread(Loop)
            {
                IsBackground = true,
                Name = "whiskerdynamics-overlay",
                Priority = ThreadPriority.BelowNormal,
            };
            _thread.Start();
        }
    }

    private static void Loop()
    {
        while (true)
        {
            try
            {
                Signal.WaitOne(500);
                long startMs = Environment.TickCount64;
                int ran = Queue.Drain(maxJobs: 4, maxElapsedMs: 100);
                long elapsed = Environment.TickCount64 - startMs;
                if (elapsed > DrainLagWarnMs && Environment.TickCount64 >= _nextLagWarnMs)
                {
                    _nextLagWarnMs = Environment.TickCount64 + 30_000;
                    ModLog.Warn($"overlay worker saturated: {ran} rebuild(s) took {elapsed} ms in one "
                        + "cycle (batches age out at 5 s) - consider lowering overlay_max_points, "
                        + "Orbit look-ahead, or the tracked-vessel count");
                }
                if (Queue.PendingCount > 0)
                {
                    Signal.Set();
                    Thread.Yield();
                }
            }
            catch (Exception e)
            {
                // Backstop only — the queue contains job failures. Never let the
                // worker die: a dead thread turns every line stale within 5 s.
                ModLog.Warn($"overlay worker backstop: {e.Message}");
            }
        }
    }
}
