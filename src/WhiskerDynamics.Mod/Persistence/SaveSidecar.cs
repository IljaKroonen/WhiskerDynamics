using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Persistence;

public sealed class SidecarVessel
{
    public string Id { get; set; } = "";
    public double EpochSeconds { get; set; }
    public double[] PositionEcl { get; set; } = new double[3]; // mod-frame absolute, SI
    public double[] VelocityEcl { get; set; } = new double[3];
    /// <summary>Direct-field (mu/r^2) label — display/diagnostic ONLY.
    /// Sol dominates cislunar space, so this labels a trans-lunar vessel 'Sol'; the
    /// persisted state is absolute mod-frame, making the label choice harmless, and
    /// nothing parent-relative is ever derived from it.</summary>
    public string DominantAttractor { get; set; } = "";
    /// <summary>Human-readable osculating elements around the vessel's ACTUAL stock
    /// orbit parent (never the dominant-attractor label), or the reason none exist.</summary>
    public string OsculatingNote { get; set; } = "";
}

/// <summary>Stable, name-based identity for a selected display frame.</summary>
public sealed class SidecarFrame
{
    public string FrameKind { get; set; } = "";
    public string PrimaryId { get; set; } = "";
    public string? SecondaryId { get; set; }
}

/// <summary>One vessel's selected display-frame identity. Kept at file scope rather
/// than on SidecarVessel so landed/stock-owned vessels and a failed trajectory capture
/// still retain their UI preference.</summary>
public sealed class SidecarFrameSelection
{
    public string VesselId { get; set; } = "";
    public SidecarFrame Frame { get; set; } = new();
}

/// <summary>One vessel's flight-plan metadata. Kept at file scope so landed,
/// stock-owned, reseed-pending, and failed exact-state captures retain their plan.</summary>
public sealed class SidecarPlanRecord
{
    public string VesselId { get; set; } = "";
    public SidecarPlan? Plan { get; set; }
}

/// <summary>Flight-plan DTO sanitized by `FlightPlans.FromSidecar`. Invalid plans or
/// burn metadata are ignored, leaving the stock burn plan as the execution truth.</summary>
public sealed class SidecarPlan
{
    public double CreatedAtSeconds { get; set; }
    public double LengthSeconds { get; set; }
    /// <summary>Finite-plan propulsion selection. Missing, empty, or invalid values
    /// select main engines.</summary>
    public string PropulsionSource { get; set; } = string.Empty;
    public List<SidecarPlanBurn> Burns { get; set; } = [];
    /// <summary>Plan-snapshot anchor. Null or missing anchors are captured lazily on
    /// the first on-rails rebuild.</summary>
    public SidecarPlanAnchor? Anchor { get; set; }
    /// <summary>The snapshot's captured burn list (times + VLF dv as written to the
    /// stock plan at capture) — the display fold's input, distinct from the
    /// frame-authoring metadata in <see cref="Burns"/>.</summary>
    public List<SidecarSnapshotBurn> SnapshotBurns { get; set; } = [];
    /// <summary>True when reality had already left the snapshot's world at save time
    /// (burn flown, not yet rebased) — restored so the ghost stays a ghost.</summary>
    public bool Diverged { get; set; }
}

/// <summary>The plan snapshot's trajectory anchor: epoch + mod-frame absolute state
/// (same convention as <see cref="SidecarVessel"/>'s exact state) + the orbit parent
/// at capture. An empty parent id falls back to the live parent.</summary>
public sealed class SidecarPlanAnchor
{
    public double EpochSeconds { get; set; }
    public double[] PositionEcl { get; set; } = new double[3];
    public double[] VelocityEcl { get; set; } = new double[3];
    public string? ParentId { get; set; }
    /// <summary>Propulsion set represented by the frozen engine scalars. Missing,
    /// empty, or invalid values select main engines.</summary>
    public string PropulsionSource { get; set; } = string.Empty;
    /// <summary>Finite-burn engine scalars at capture. Missing, zero, or invalid
    /// values keep the display fold impulsive until the next capture or rebase.</summary>
    public double MassKg { get; set; }
    public double ExhaustVelocity { get; set; }
    public double MassFlowRate { get; set; }
}

/// <summary>One captured snapshot burn: stock time key, VLF delta-v components, and
/// nullable burn-time basis parent.</summary>
public sealed class SidecarSnapshotBurn
{
    public double TimeSeconds { get; set; }
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public string? BasisParentId { get; set; }
    public bool BasisParentRefreshPending { get; set; }
}

/// <summary>One frame-authored burn's metadata: the burn-time key into the STOCK plan
/// (the burns themselves live in the stock save) plus the authoring frame and the
/// authored components. VLF burns carry no entry. <see cref="Basis"/> is the component
/// semantics discriminator: "prn" means prograde/radial/normal of the
/// frame-relative trajectory. Missing or unsupported bases are dropped so the stock
/// burn remains a plain VLF burn.</summary>
public sealed class SidecarPlanBurn
{
    public double TimeSeconds { get; set; }
    public string FrameKind { get; set; } = "";
    public string PrimaryId { get; set; } = "";
    public string? SecondaryId { get; set; }
    public string Basis { get; set; } = "";
    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
}

public sealed class SidecarFile
{
    /// <summary>Exact stock-save identity when the save API exposes one. Null means
    /// this is an anonymous epoch-paired sidecar.</summary>
    public string? SaveIdentity { get; set; }
    /// <summary>Stock metadata generation for an exact named save. A named load that
    /// supplies a generation accepts only the sidecar captured by that same write.</summary>
    public long? SaveGenerationTicks { get; set; }
    public string GameBuild { get; set; } = "";
    public double ElapsedSeconds { get; set; }
    public List<SidecarVessel> Vessels { get; set; } = [];
    /// <summary>Plan metadata keyed independently of exact predictor-state records.</summary>
    public List<SidecarPlanRecord> Plans { get; set; } = [];
    /// <summary>Display preferences for every controlled vessel, independent of
    /// exact-state sidecar eligibility. Missing selections use the default frame.</summary>
    public List<SidecarFrameSelection> FrameSelections { get; set; } = [];
    /// <summary>Stock nodes owned by an automatic rendezvous transaction that was
    /// mid-commit when this save was taken. On load they are removed before the
    /// planner becomes available, preventing an orphan one-node transfer.</summary>
    public SidecarPendingRendezvous? PendingRendezvous { get; set; }
}

public sealed class SidecarPendingRendezvous
{
    public string VesselId { get; set; } = "";
    public List<SidecarSnapshotBurn> Burns { get; set; } = [];
}

/// <summary>Exact mod state persisted OUTSIDE the save file (spec invariant: the stock
/// game must never see anything it cannot read — the save itself stays pure stock, its
/// vessel conics osculating-fresh via the re-osculation refresh). Named stock saves use
/// their exact game identity; anonymous captures use deterministic
/// nearest-epoch matching within one second. Without a matching sidecar the mod degrades
/// gracefully to reseeding from the stock osculating elements.
/// KSA-free by design: the offline suite covers serialization, pairing, containment and
/// the parent-vs-label caveat.</summary>
public static class SaveSidecar
{
    // KSA writes SimTime seconds with four decimal places. The sidecar intentionally
    // retains full precision, so its capture can be up to 50 microseconds later than
    // the same stock save's restored epoch.
    internal const double StockTimeSerializationToleranceSeconds = 1e-3;
    // AllowNamedFloatingPointLiterals: a hand-edited "NaN" in a snapshot must parse (so
    // IsSane can drop that vessel) instead of poisoning the whole file with a
    // JsonException; legitimate writes never contain them (Capture rejects non-finite).
    private static readonly JsonSerializerOptions Options = new()
    {
        WriteIndented = true,
        NumberHandling = System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals,
    };

    /// <summary>Maximum numbered snapshots retained on disk.</summary>
    internal const int SnapshotKeep = 20;
    /// <summary>Maximum distinct named-save captures retained while their canonical
    /// writes are not durable. Normal writes leave this set immediately; the bound is
    /// a fail-safe for a persistent storage outage plus repeated Save As operations.</summary>
    internal const int MaxPendingNamedIdentities = 64;

    internal static string? DirOverride; // offline tests redirect the sidecar dir
    public static string Dir => DirOverride ?? Path.Combine(ModMain.ModDir, "sidecar");
    private sealed record WriteRequest(long Sequence, SidecarFile File,
        double ElapsedSeconds, string Directory, string? SaveIdentity,
        long? SaveGenerationTicks, string RequestToken, AtomicTextFileHooks? AtomicWriteHooks);
    private sealed record ReadCandidate(SidecarFile File, double Distance,
        bool Pending, long Recency, string StableName);
    private static readonly System.Collections.Concurrent.BlockingCollection<WriteRequest> Writes = new(8);
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<long, WriteRequest>
        PendingMemory = new();
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<long, byte>
        DurableWrites = new();
    private static readonly System.Collections.Concurrent.ConcurrentDictionary<long, byte>
        AttemptedWrites = new();
    private static readonly object WriterGate = new();
    private static readonly object PendingRetentionGate = new();
    // A sidecar publication spans two individually atomic files. Serialize the canonical
    // snapshot -> latest convenience copy -> prune sequence so synchronous and queued
    // callers cannot interleave those transactions and publish mismatched generations.
    private static readonly object SidecarIoGate = new();
    private static readonly AutoResetEvent WriteCompleted = new(false);
    private static Thread? _writer;
    private static long _writesSubmitted, _writesCompleted;
    private static long _writesSuperseded;
    private static long _namedCapturesEvicted;
    private static long _namedCaptureEvictionsReported;
    private static long _nextNamedEvictionLogMs;
    internal static long SupersededWriteCount => Interlocked.Read(ref _writesSuperseded);
    internal static long NamedCaptureEvictionCount =>
        Interlocked.Read(ref _namedCapturesEvicted);
    internal static int PendingNamedIdentityCount => PendingMemory.Values
        .Where(request => request.SaveIdentity is not null)
        .Select(request => (request.Directory, request.SaveIdentity))
        .Distinct()
        .Count();
    internal static int PendingAttemptedNamedIdentityCount => PendingMemory.Values
        .Count(request => request.SaveIdentity is not null
            && AttemptedWrites.ContainsKey(request.Sequence));
    internal static int AttemptedWriteCount => AttemptedWrites.Count;
    internal static Action? WriterBeforeIoForTest;
    internal static Action? WriterAfterNamedRetirementForTest;

    /// <summary>Writes one snapshot (plus the 'latest' convenience copy). Exact states
    /// come from vessels the registry deemed eligible; plans and frame selections are
    /// captured independently from their file-level stores. Per-vessel containment:
    /// one broken vessel must not lose the sidecar; the caller contains everything else
    /// (a failed sidecar never touches the save itself).</summary>
    public static void Write(RailsService rails, IReadOnlyList<TrackedVessel> vessels,
        double elapsedSeconds, string gameBuild,
        SidecarPendingRendezvous? pendingRendezvous = null, string? saveIdentity = null,
        long? saveGenerationTicks = null)
    {
        saveIdentity = NormalizeIdentity(saveIdentity);
        var file = new SidecarFile
        {
            SaveIdentity = saveIdentity,
            SaveGenerationTicks = saveGenerationTicks,
            GameBuild = gameBuild,
            ElapsedSeconds = elapsedSeconds,
            PendingRendezvous = pendingRendezvous,
            Plans = FlightPlans.PlansForSidecar(),
            FrameSelections = FrameManager.FrameSelectionsForSidecar(),
        };
        foreach (var tracked in vessels)
        {
            try
            {
                file.Vessels.Add(Capture(rails, tracked, elapsedSeconds));
            }
            catch (Exception e)
            {
                ModLog.Warn($"sidecar: vessel '{tracked.Id}' skipped ({e.Message}) - "
                    + "it will reseed from its stock osculating state on load");
            }
        }
        string directory = Dir;
        string json = JsonSerializer.Serialize(file, Options);
        string path = SnapshotPath(directory, elapsedSeconds, saveIdentity,
            Guid.NewGuid().ToString("N"));
        lock (SidecarIoGate)
        {
            Directory.CreateDirectory(directory);
            AtomicTextFile.WriteAllText(path, json);
            AtomicTextFile.WriteAllText(
                Path.Combine(directory, "whiskerdynamics-latest.json"), json);
            PruneSnapshots(directory);
        }
        ModLog.Info($"sidecar written: {path} ({file.Vessels.Count} vessels, "
            + $"{file.Plans.Count} flight plans)");
    }

    /// <summary>Captures game/rails state synchronously, then queues only serialization
    /// and filesystem work. Under saturation, older disk work yields to newer saves;
    /// the bounded in-memory capture remains immediately restorable.</summary>
    public static long QueueWrite(RailsService rails, IReadOnlyList<TrackedVessel> vessels,
        double elapsedSeconds, string gameBuild, SidecarPendingRendezvous? pendingRendezvous = null,
        string? saveIdentity = null, long? saveGenerationTicks = null) =>
        QueueWriteCore(rails, vessels, elapsedSeconds, gameBuild, pendingRendezvous,
            saveIdentity, saveGenerationTicks, atomicWriteHooks: null);

    internal static long QueueWriteForTest(RailsService rails,
        IReadOnlyList<TrackedVessel> vessels, double elapsedSeconds, string gameBuild,
        AtomicTextFileHooks hooks, SidecarPendingRendezvous? pendingRendezvous = null,
        string? saveIdentity = null, long? saveGenerationTicks = null)
    {
        ArgumentNullException.ThrowIfNull(hooks);
        return QueueWriteCore(rails, vessels, elapsedSeconds, gameBuild, pendingRendezvous,
            saveIdentity, saveGenerationTicks, hooks);
    }

    private static long QueueWriteCore(RailsService rails,
        IReadOnlyList<TrackedVessel> vessels, double elapsedSeconds, string gameBuild,
        SidecarPendingRendezvous? pendingRendezvous, string? saveIdentity,
        long? saveGenerationTicks, AtomicTextFileHooks? atomicWriteHooks)
    {
        long sequence = Interlocked.Increment(ref _writesSubmitted);
        saveIdentity = NormalizeIdentity(saveIdentity);
        var request = new WriteRequest(sequence,
            BuildFile(rails, vessels, elapsedSeconds, gameBuild, pendingRendezvous,
                saveIdentity, saveGenerationTicks),
            elapsedSeconds, Dir, saveIdentity, saveGenerationTicks,
            Guid.NewGuid().ToString("N"), atomicWriteHooks);
        PendingMemory[sequence] = request;
        EnsureWriterStarted();
        if (!Writes.TryAdd(request))
        {
            if (Writes.TryTake(out var dropped))
            {
                Interlocked.Increment(ref _writesSuperseded);
                AttemptedWrites[dropped.Sequence] = 1;
                if (!PendingMemory.ContainsKey(dropped.Sequence))
                    AttemptedWrites.TryRemove(dropped.Sequence, out _);
                NoteWriteCompleted();
                ModLog.Warn($"sidecar writer saturated; snapshot at {dropped.ElapsedSeconds:F1} s "
                    + "is restorable only while retained in the bounded recent-memory cache; "
                    + "its disk write was superseded");
            }
            if (!Writes.TryAdd(request))
            {
                Interlocked.Increment(ref _writesSuperseded);
                AttemptedWrites[request.Sequence] = 1;
                NoteWriteCompleted();
                ModLog.Warn($"sidecar writer saturated; snapshot at {elapsedSeconds:F1} s "
                    + "is restorable only while retained in the bounded recent-memory cache; "
                    + "its disk write was deferred");
            }
        }
        TrimPendingMemory(request);
        foreach (long stale in DurableWrites.Keys.OrderByDescending(x => x).Skip(128))
            DurableWrites.TryRemove(stale, out _);
        return sequence;
    }

    private static void TrimPendingMemory(WriteRequest newest)
    {
        lock (PendingRetentionGate)
        {
            // One named stock slot has one canonical path. Retain only its newest
            // capture: an older generation cannot be the correct companion after the
            // same stock slot has been overwritten again.
            if (newest.SaveIdentity is not null)
            {
                foreach (var stale in PendingMemory.Values)
                {
                    if (stale.Sequence >= newest.Sequence
                        || !string.Equals(stale.Directory, newest.Directory,
                            StringComparison.OrdinalIgnoreCase)
                        || !string.Equals(stale.SaveIdentity, newest.SaveIdentity,
                            StringComparison.Ordinal))
                        continue;
                    PendingMemory.TryRemove(stale.Sequence, out _);
                    AttemptedWrites.TryRemove(stale.Sequence, out _);
                }
            }

            foreach (long stale in PendingMemory.Values
                         .Where(x => x.SaveIdentity is null)
                         .OrderByDescending(x => x.Sequence)
                         .Skip(16)
                         .Select(x => x.Sequence))
            {
                PendingMemory.TryRemove(stale, out _);
                AttemptedWrites.TryRemove(stale, out _);
            }

            // No finite RAM-only strategy can preserve infinitely many distinct Save
            // As captures while storage remains unavailable. Keep a generous bounded
            // working set, prefer the saves made most recently, and report every
            // overflow episode explicitly rather than silently claiming durability.
            foreach (var stale in PendingMemory.Values
                         .Where(x => x.SaveIdentity is not null)
                         .OrderByDescending(x => x.Sequence)
                         .Skip(MaxPendingNamedIdentities))
            {
                if (!PendingMemory.TryRemove(stale.Sequence, out _)) continue;
                AttemptedWrites.TryRemove(stale.Sequence, out _);
                long total = Interlocked.Increment(ref _namedCapturesEvicted);
                long nowMs = Environment.TickCount64;
                if (nowMs < _nextNamedEvictionLogMs) continue;
                _nextNamedEvictionLogMs = nowMs + 5000;
                ModLog.Error("sidecar named-save recovery cache exhausted after "
                    + $"{MaxPendingNamedIdentities} distinct pending identities; "
                    + $"evicted oldest capture '{stale.SaveIdentity}' before durability "
                    + $"(total evictions {total}). Stock save data remains valid, but "
                    + "its exact mod sidecar may fall back or fail closed until storage recovers.");
            }
        }
    }

    private static SidecarFile BuildFile(RailsService rails, IReadOnlyList<TrackedVessel> vessels,
        double elapsedSeconds, string gameBuild, SidecarPendingRendezvous? pendingRendezvous,
        string? saveIdentity, long? saveGenerationTicks)
    {
        var file = new SidecarFile
        {
            SaveIdentity = saveIdentity,
            SaveGenerationTicks = saveGenerationTicks,
            GameBuild = gameBuild,
            ElapsedSeconds = elapsedSeconds,
            PendingRendezvous = pendingRendezvous,
            Plans = FlightPlans.PlansForSidecar(),
            FrameSelections = FrameManager.FrameSelectionsForSidecar(),
        };
        foreach (var tracked in vessels)
        {
            try { file.Vessels.Add(Capture(rails, tracked, elapsedSeconds)); }
            catch (Exception e)
            {
                ModLog.Warn($"sidecar: vessel '{tracked.Id}' skipped ({e.Message}) - "
                    + "it will reseed from its stock osculating state on load");
            }
        }
        return file;
    }

    private static void EnsureWriterStarted()
    {
        if (Volatile.Read(ref _writer) is not null) return;
        lock (WriterGate)
        {
            if (_writer is not null) return;
            _writer = new Thread(() =>
            {
                foreach (var request in Writes.GetConsumingEnumerable()) CompleteWrite(request);
            })
            {
                IsBackground = true, Name = "whiskerdynamics-sidecar",
                Priority = ThreadPriority.BelowNormal,
            };
            _writer.Start();
            AppDomain.CurrentDomain.ProcessExit += (_, _) => FlushPendingWrites(1000);
        }
    }

    private static void CompleteWrite(WriteRequest request)
    {
        bool durable = false;
        try
        {
            WriteCaptured(request);
            // A named path is canonical for all generations of that identity. Retire
            // superseded in-memory generations before publishing durability, otherwise
            // a concurrent flush can observe completion and requeue stale disk work.
            RetireOlderNamedCaptures(request);
            if (request.SaveIdentity is not null)
                WriterAfterNamedRetirementForTest?.Invoke();
            DurableWrites[request.Sequence] = 1;
            PendingMemory.TryRemove(request.Sequence, out _);
            durable = true;
        }
        catch (Exception e) { ModLog.Error($"sidecar queued write failed: {e}"); }
        finally
        {
            if (durable) AttemptedWrites.TryRemove(request.Sequence, out _);
            else
            {
                AttemptedWrites[request.Sequence] = 1;
                if (!PendingMemory.ContainsKey(request.Sequence))
                    AttemptedWrites.TryRemove(request.Sequence, out _);
            }
            NoteWriteCompleted();
        }
    }

    private static void NoteWriteCompleted()
    {
        Interlocked.Increment(ref _writesCompleted);
        WriteCompleted.Set();
    }

    private static void RetireOlderNamedCaptures(WriteRequest durable)
    {
        if (durable.SaveIdentity is null) return;
        foreach (var stale in PendingMemory.Values)
        {
            if (stale.Sequence >= durable.Sequence
                || !string.Equals(stale.Directory, durable.Directory,
                    StringComparison.OrdinalIgnoreCase)
                || !string.Equals(stale.SaveIdentity, durable.SaveIdentity,
                    StringComparison.Ordinal))
                continue;
            PendingMemory.TryRemove(stale.Sequence, out _);
            AttemptedWrites.TryRemove(stale.Sequence, out _);
        }
    }

    public static bool WaitForDurability(long sequence, int timeoutMs)
    {
        long deadline = Environment.TickCount64 + Math.Max(0, timeoutMs);
        while (!DurableWrites.ContainsKey(sequence))
        {
            int remaining = (int)Math.Min(int.MaxValue, deadline - Environment.TickCount64);
            if (remaining <= 0 || !WriteCompleted.WaitOne(remaining)) return false;
        }
        return true;
    }

    public static bool FlushPendingWrites(int timeoutMs)
    {
        // A bounded recovery-cache overflow is irreversible for any capture that was
        // not already in flight. Surface it on the next flush even if every retained
        // request succeeds; a later flush reports the current pending set normally.
        bool namedCaptureEviction = AcknowledgeNamedCaptureEvictions();
        long target = Interlocked.Read(ref _writesSubmitted);
        long deadline = Environment.TickCount64 + Math.Max(0, timeoutMs);
        var newestPending = PendingMemory.Values
                     .Where(x => x.Sequence <= target)
                     .GroupBy(x => x.Directory, StringComparer.OrdinalIgnoreCase)
                     .SelectMany(directory => directory
                         .GroupBy(x => x.SaveIdentity, StringComparer.Ordinal)
                         .Select(identity => identity
                             .OrderByDescending(x => x.Sequence).First()))
                     .OrderByDescending(x => x.Sequence)
                     .ToArray();
        bool allDurable = true;
        foreach (var request in newestPending)
        {
            if (WaitForAttemptAndRetry(request, deadline)) continue;
            allDurable = false;
            if (Environment.TickCount64 >= deadline) break;
        }
        return allDurable && !namedCaptureEviction;
    }

    private static bool AcknowledgeNamedCaptureEvictions()
    {
        long current = Interlocked.Read(ref _namedCapturesEvicted);
        while (true)
        {
            long reported = Interlocked.Read(ref _namedCaptureEvictionsReported);
            if (current <= reported) return false;
            if (Interlocked.CompareExchange(
                    ref _namedCaptureEvictionsReported, current, reported) == reported)
                return true;
        }
    }

    private static bool WaitForAttemptAndRetry(WriteRequest request, long deadline)
    {
        while (!DurableWrites.ContainsKey(request.Sequence)
               && !AttemptedWrites.ContainsKey(request.Sequence))
        {
            int remaining = (int)Math.Min(int.MaxValue, deadline - Environment.TickCount64);
            if (remaining <= 0 || !WriteCompleted.WaitOne(remaining)) return false;
        }
        if (DurableWrites.ContainsKey(request.Sequence)) return true;

        // The first attempt failed or was superseded. Queue exactly one retry on the
        // writer; the flush caller never executes serialization or filesystem work.
        AttemptedWrites.TryRemove(request.Sequence, out _);
        if (!Writes.TryAdd(request))
        {
            // Keep the request retryable by a later flush. Without restoring this
            // marker, a full retry queue leaves the request in a neither-queued-nor-
            // attempted state and every future flush waits for an event that cannot occur.
            AttemptedWrites[request.Sequence] = 1;
            if (!PendingMemory.ContainsKey(request.Sequence))
                AttemptedWrites.TryRemove(request.Sequence, out _);
            return false;
        }
        while (!DurableWrites.ContainsKey(request.Sequence))
        {
            if (AttemptedWrites.ContainsKey(request.Sequence)) return false;
            int remaining = (int)Math.Min(int.MaxValue, deadline - Environment.TickCount64);
            if (remaining <= 0 || !WriteCompleted.WaitOne(remaining)) return false;
        }
        return true;
    }

    private static void WriteCaptured(WriteRequest request)
    {
        WriterBeforeIoForTest?.Invoke();
        string json = JsonSerializer.Serialize(request.File, Options);
        string path = SnapshotPath(request.Directory, request.ElapsedSeconds,
            request.SaveIdentity, request.RequestToken);
        lock (SidecarIoGate)
        {
            Directory.CreateDirectory(request.Directory);
            AtomicTextFile.WriteAllText(path, json, request.AtomicWriteHooks);
            AtomicTextFile.WriteAllText(
                Path.Combine(request.Directory, "whiskerdynamics-latest.json"), json,
                request.AtomicWriteHooks);
            PruneSnapshots(request.Directory);
        }
        ModLog.Info($"sidecar written: {path} ({request.File.Vessels.Count} vessels, "
            + $"{request.File.Plans.Count} flight plans)");
    }

    private static string SnapshotPath(string directory, double elapsedSeconds,
        string? saveIdentity, string requestToken)
    {
        if (saveIdentity is not null)
            return IdentitySnapshotPath(directory, saveIdentity);
        ulong bits = unchecked((ulong)BitConverter.DoubleToInt64Bits(elapsedSeconds));
        string epoch = bits.ToString("x16", CultureInfo.InvariantCulture);
        return Path.Combine(directory,
            $"whiskerdynamics-epoch-{epoch}-{requestToken}.json");
    }

    internal static string IdentitySnapshotPath(string directory, string saveIdentity)
    {
        byte[] digest = SHA256.HashData(Encoding.UTF8.GetBytes(saveIdentity));
        return Path.Combine(directory,
            "whiskerdynamics-save-" + Convert.ToHexStringLower(digest) + ".json");
    }

    private static string? NormalizeIdentity(string? identity) =>
        string.IsNullOrEmpty(identity) ? null : identity;

    private static SidecarVessel Capture(RailsService rails, TrackedVessel tracked, double elapsedSeconds)
    {
        StateVector state;
        lock (rails.Gate) state = tracked.Predictor.StateAt(elapsedSeconds);
        if (!IsFinite(state.Position) || !IsFinite(state.Velocity))
            throw new InvalidOperationException("predictor state is non-finite");
        string dominant = DominantAttractor.Compute(rails, state.Position, elapsedSeconds);
        // CAVEAT: the note's reference body is
        // the vessel's ACTUAL stock orbit parent (LastParentId — always an integrated
        // body, stamped by every seed/reseed), NOT the direct-field dominant label,
        // which reads 'Sol' across most of cislunar space. The label is persisted as
        // display data only. Fallback to the label covers only never-staged vessels
        // (offline-constructed; in-game every eligible vessel has been staged).
        string noteParent = tracked.LastParentId ?? dominant;
        string note;
        try
        {
            var parentAbs = rails.GetAbsolute(noteParent, elapsedSeconds);
            var relative = new StateVector(state.Position - parentAbs.Position, state.Velocity - parentAbs.Velocity);
            var elements = Kepler.ElementsFromState(relative, rails.MuOf(noteParent), elapsedSeconds);
            note = $"osculating around {noteParent}: a={elements.SemiMajorAxis:E6} m e={elements.Eccentricity:F6} "
                 + $"i={elements.Inclination:F6} rad";
        }
        catch (NotSupportedException e)
        {
            // Near-circular or exactly-parabolic states have no quotable elements
            // (hyperbolic escape states quote fine — a < 0, e > 1): the exact state
            // is persisted regardless; the note only explains.
            note = $"no elliptic osculating elements around {noteParent}: {e.Message}";
        }
        return new SidecarVessel
        {
            Id = tracked.Id,
            EpochSeconds = elapsedSeconds,
            PositionEcl = [state.Position.X, state.Position.Y, state.Position.Z],
            VelocityEcl = [state.Velocity.X, state.Velocity.Y, state.Velocity.Z],
            DominantAttractor = dominant,
            OsculatingNote = note,
        };
    }

    /// <summary>Compares restored and seeded states at their later epoch, avoiding
    /// position error caused solely by their allowed timestamp skew.</summary>
    public static double RestoreDeltaMeters(RailsService rails, TrackedVessel tracked,
        StateVector sidecarAbsolute, double epochSeconds)
    {
        double commonTime = Math.Max(tracked.SeedTime, epochSeconds);
        lock (rails.Gate)
        {
            var probe = new TrajectoryPredictor(rails.VesselGravity, sidecarAbsolute, epochSeconds, tracked.Options);
            return (probe.StateAt(commonTime).Position - tracked.Predictor.StateAt(commonTime).Position).Length();
        }
    }

    /// <summary>Aligns the full-precision sidecar state with KSA's rounded stock-save
    /// epoch. Only sub-millisecond future skew is normalization-safe; a materially
    /// future sidecar is not the state represented by the stock save and is rejected.</summary>
    internal static bool TryNormalizeRestoreEpoch(StateVector sidecarAbsolute,
        double epochSeconds, double seedTime, out StateVector normalized,
        out double normalizedEpoch)
    {
        normalized = sidecarAbsolute;
        normalizedEpoch = epochSeconds;
        double futureSkew = epochSeconds - seedTime;
        if (futureSkew <= 0) return true;
        if (futureSkew > StockTimeSerializationToleranceSeconds) return false;

        // Over <= 1 ms, a linear rewind is far below the sidecar's restore sanity
        // threshold and avoids seeding a predictor after KSA's first staging query.
        normalized = sidecarAbsolute with
        {
            Position = sidecarAbsolute.Position - sidecarAbsolute.Velocity * futureSkew,
        };
        normalizedEpoch = seedTime;
        return true;
    }

    /// <summary>Sidecar for the exact named stock save and metadata generation when
    /// supplied. Named loads fail closed; only anonymous callers use deterministic
    /// nearest-epoch matching. Hand-broken or non-finite vessel entries are
    /// dropped, and unreadable snapshots are skipped.</summary>
    public static SidecarFile? TryRead(double elapsedSeconds, string? saveIdentity = null,
        long? saveGenerationTicks = null)
    {
        string directory = Dir;
        saveIdentity = NormalizeIdentity(saveIdentity);
        if (saveIdentity is not null)
        {
            // Repeated writes to one named slot intentionally replace its state. If the
            // newest capture is still queued, it is more authoritative than disk.
            var memory = PendingMemory.Values
                .Where(x => string.Equals(x.Directory, directory,
                        StringComparison.OrdinalIgnoreCase)
                    && string.Equals(x.SaveIdentity, saveIdentity,
                        StringComparison.Ordinal)
                    && GenerationMatches(x.SaveGenerationTicks, saveGenerationTicks)
                    && WithinEpochTolerance(x.ElapsedSeconds, elapsedSeconds))
                .OrderByDescending(x => x.Sequence)
                .FirstOrDefault();
            if (memory is not null)
                return CloneAndSanitize(memory.File,
                    $"pending named save '{saveIdentity}'");

            if (Directory.Exists(directory))
            {
                string exact = IdentitySnapshotPath(directory, saveIdentity);
                var file = TryReadExactIdentity(exact, saveIdentity,
                    saveGenerationTicks, elapsedSeconds);
                if (file is not null) return file;

                // The convenience copy is safe only when its embedded exact identity
                // agrees with the requested save.
                string latest = Path.Combine(directory, "whiskerdynamics-latest.json");
                file = TryReadExactIdentity(latest, saveIdentity,
                    saveGenerationTicks, elapsedSeconds);
                if (file is not null) return file;
            }

            // Supplying an identity declares that the stock save has exact pairing
            // metadata. Falling back to an anonymous epoch candidate would reintroduce
            // the cross-save state import this contract exists to prevent.
            return null;
        }

        // Epoch pairing is available only to callers without a stock identity.
        var candidates = new List<ReadCandidate>();
        foreach (var request in PendingMemory.Values)
        {
            if (!string.Equals(request.Directory, directory,
                    StringComparison.OrdinalIgnoreCase)
                || NormalizeIdentity(request.SaveIdentity) is not null
                || !WithinEpochTolerance(request.ElapsedSeconds, elapsedSeconds))
                continue;
            candidates.Add(new ReadCandidate(request.File,
                Math.Abs(request.ElapsedSeconds - elapsedSeconds), Pending: true,
                request.Sequence,
                "memory-" + request.Sequence.ToString("D20",
                    CultureInfo.InvariantCulture)));
        }

        if (Directory.Exists(directory))
        {
            foreach (var info in new DirectoryInfo(directory)
                         .GetFiles("whiskerdynamics-*.json"))
            {
                var file = TryDeserialize(info.FullName);
                if (file is null || NormalizeIdentity(file.SaveIdentity) is not null
                    || !WithinEpochTolerance(file.ElapsedSeconds, elapsedSeconds))
                    continue;
                candidates.Add(new ReadCandidate(file,
                    Math.Abs(file.ElapsedSeconds - elapsedSeconds), Pending: false,
                    info.LastWriteTimeUtc.Ticks, info.Name));
            }
        }

        // Explicit ambiguity policy: exact epoch distance, then an in-process pending
        // capture over disk, then newest sequence/write time, then ordinal stable name.
        var selected = candidates
            .OrderBy(x => x.Distance)
            .ThenByDescending(x => x.Pending)
            .ThenByDescending(x => x.Recency)
            .ThenBy(x => x.StableName, StringComparer.Ordinal)
            .FirstOrDefault();
        return selected is null
            ? null
            : CloneAndSanitize(selected.File, selected.StableName);
    }

    private static SidecarFile? TryReadExactIdentity(string path,
        string saveIdentity, long? saveGenerationTicks, double elapsedSeconds)
    {
        var file = TryDeserialize(path);
        if (file is null
            || !string.Equals(file.SaveIdentity, saveIdentity, StringComparison.Ordinal)
            || !GenerationMatches(file.SaveGenerationTicks, saveGenerationTicks)
            || !WithinEpochTolerance(file.ElapsedSeconds, elapsedSeconds))
            return null;
        return CloneAndSanitize(file, Path.GetFileName(path));
    }

    private static SidecarFile? TryDeserialize(string path)
    {
        try
        {
            if (!File.Exists(path)) return null;
            return JsonSerializer.Deserialize<SidecarFile>(File.ReadAllText(path), Options);
        }
        catch (Exception)
        {
            // Unreadable snapshot (corrupt JSON, IO): ignore it, keep scanning.
            return null;
        }
    }

    private static SidecarFile? CloneAndSanitize(SidecarFile source, string label)
    {
        var clone = JsonSerializer.Deserialize<SidecarFile>(
            JsonSerializer.Serialize(source, Options), Options);
        if (clone is null) return null;
        clone.Vessels ??= [];
        clone.Plans ??= [];
        int dropped = clone.Vessels.RemoveAll(v => !IsSane(v));
        if (dropped > 0)
            ModLog.Warn($"sidecar {label}: dropped {dropped} unreadable vessel entries");
        return clone;
    }

    private static bool WithinEpochTolerance(double candidate, double requested) =>
        double.IsFinite(candidate) && double.IsFinite(requested)
        && Math.Abs(candidate - requested) < 1.0;

    private static bool GenerationMatches(long? candidate, long? requested) =>
        !requested.HasValue || candidate == requested;

    private static bool IsFinite(Vector3d v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);

    private static bool IsSane(SidecarVessel? v) =>
        v is not null
        && !string.IsNullOrEmpty(v.Id)
        && double.IsFinite(v.EpochSeconds)
        && v.PositionEcl is { Length: 3 } p && p.All(double.IsFinite)
        && v.VelocityEcl is { Length: 3 } w && w.All(double.IsFinite);

    private static void PruneSnapshots(string? directory = null)
    {
        // Stable named-save files are canonical for the lifetime of their matching
        // stock save. The bounded retention policy applies only to anonymous epoch
        // snapshots, which are deliberately interchangeable nearest-time fallbacks.
        var stale = new DirectoryInfo(directory ?? Dir).GetFiles("whiskerdynamics-epoch-*.json")
            .OrderByDescending(f => f.LastWriteTimeUtc)
            .ThenByDescending(f => f.Name, StringComparer.Ordinal) // deterministic on timestamp ties
            .Skip(SnapshotKeep);
        foreach (var file in stale)
        {
            try { file.Delete(); }
            catch (Exception) { /* locked/gone: pruning is best-effort */ }
        }
    }
}
