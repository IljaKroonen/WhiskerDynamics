using System.Collections.Concurrent;

namespace WhiskerDynamics.Mod.Planning;

public static class BasisReconversionUrgency
{
    private static readonly ConcurrentDictionary<string, long> Urgent =
        new(StringComparer.Ordinal);

    private static long _generation;

    public static void Raise(string vesselId) =>
        Urgent[vesselId] = Interlocked.Increment(ref _generation);

    public static bool IsUrgent(string vesselId) => Urgent.ContainsKey(vesselId);

    public static bool Any => !Urgent.IsEmpty;

    public static long? Observe(string vesselId) =>
        Urgent.TryGetValue(vesselId, out long generation) ? generation : null;

    public static void Clear(string vesselId) => Urgent.TryRemove(vesselId, out _);

    public static void Clear(string vesselId, long generation) =>
        Urgent.TryRemove(new KeyValuePair<string, long>(vesselId, generation));

    public static List<string> Snapshot() => [.. Urgent.Keys];

    public static void Reset() => Urgent.Clear();
}
