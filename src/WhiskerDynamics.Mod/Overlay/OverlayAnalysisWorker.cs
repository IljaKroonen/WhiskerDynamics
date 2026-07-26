namespace WhiskerDynamics.Mod.Overlay;

/// <summary>Dedicated latest-wins worker for the optional orbit analyser. Analysis
/// may integrate millions of private predictor steps; keeping it off
/// <see cref="OverlayWorker"/> ensures that line rebuilds for every vessel retain
/// their normal cadence. Only the controlled vessel requests analysis, so one
/// serialized background thread is sufficient.</summary>
internal static class OverlayAnalysisWorker
{
    private static readonly KeyedLatestQueue Queue = new(
        (key, e) => ModLog.Warn($"overlay analysis job '{key}' contained: {e.Message}"));
    private static readonly AutoResetEvent Signal = new(false);
    private static readonly object StartGate = new();
    private static Thread? _thread;

    internal static bool Enqueue(string key, int generation,
        Func<bool> admissionStillCurrent,
        Action<Func<bool>, Func<Action, bool>> job,
        Action? onDiscard = null)
    {
        EnsureStarted();
        bool accepted = Queue.TryEnqueue(key,
            () => generation == OverlayWorker.CurrentGeneration
                && admissionStillCurrent(),
            ticket => () => job(
                () => generation != OverlayWorker.CurrentGeneration
                    || !Queue.IsCurrent(key, ticket),
                action => generation == OverlayWorker.CurrentGeneration
                    && Queue.RunIfCurrent(key, ticket, action)),
            onDiscard);
        if (!accepted) return false;
        Signal.Set();
        return true;
    }

    /// <summary>Runs the overlay reset while holding the analysis queue gate, then
    /// clears analysis work before releasing it. Analysis admission may consult the
    /// overlay queue, so every nested acquisition follows analysis -> overlay.</summary>
    internal static void ResetSessionStatics(Action whileAnalysisQueueLocked)
        => Queue.Clear(whileAnalysisQueueLocked);

    /// <summary>Revokes queued and running analysis for one vessel. Cancellation and
    /// <see cref='KeyedLatestQueue.RunIfCurrent'/> publication share the queue gate,
    /// so a reseed either rejects the publication or waits for it and strips it.</summary>
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
                Name = "whiskerdynamics-analysis",
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
                Queue.Drain(maxJobs: 1, maxElapsedMs: 100);
                if (Queue.PendingCount > 0)
                {
                    Signal.Set();
                    Thread.Yield();
                }
            }
            catch (Exception e)
            {
                ModLog.Warn($"overlay analysis worker backstop: {e.Message}");
            }
        }
    }
}
