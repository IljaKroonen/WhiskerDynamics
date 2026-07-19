using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Soi;

/// <summary>The SOI replacement concept: the body whose point-mass field is
/// strongest at a position. Purely informational — dynamics never consult it, and by
/// design it patches nothing game-side. Stock SOI machinery keeps running for
/// game-state bookkeeping, re-anchored to the rails where it matters: on-rails
/// parenting is decided geometrically over rails positions
/// (<see cref="SoiReparentKernel"/> via the registry), and live cross-parent handoffs
/// are re-converted with rails positions (Patches.SoiHandoffPatch) — the teleport-jump
/// guard remains the backstop for anything else that moves a staged state
/// discontinuously. Consumers: the status panel, save sidecar parent selection, any
/// future frame choice.</summary>
public static class DominantAttractor
{
    /// <summary>Strongest mu/r^2 over every finite-mass modeled source. A body
    /// at exactly zero distance is skipped (division guard), not crowned. Cost: one
    /// folded Gate hold for the complete source set; the status panel refreshes its
    /// providers at most once per 500 ms, while non-panel consumers sample on demand.
    /// Direct-field (mu/r^2) semantics, NOT a local-orbital-parent criterion: Sol
    /// dominates cislunar space beyond r_E ~ 2.6e8 m. Save sidecar parent selection
    /// must use the vessel's actual orbit parent (or a Hill-sphere criterion), not
    /// this label.</summary>
    public static string Compute(RailsService rails, Vector3d absolutePosition, double time)
    {
        string best = "";
        double bestField = -1;
        var sources = rails.VesselGravity.Sources;
        if (!rails.TryGetAbsoluteMany(sources, time, out var states)) return best;
        for (int i = 0; i < sources.Count; i++)
        {
            var body = sources[i];
            var state = states[i];
            double r2 = (absolutePosition - state.Position).LengthSquared();
            if (r2 <= 0) continue;
            double field = body.Mu / r2;
            if (field > bestField) { bestField = field; best = body.Id; }
        }
        return best;
    }

    public static bool TryCompute(RailsService rails, Vector3d absolutePosition, double time,
        out string best)
    {
        best = string.Empty;
        double bestField = -1;
        var sources = rails.VesselGravity.Sources;
        if (!rails.TryGetAbsoluteMany(sources, time, out var states)) return false;
        for (int i = 0; i < sources.Count; i++)
        {
            var body = sources[i];
            double r2 = (absolutePosition - states[i].Position).LengthSquared();
            if (r2 <= 0) continue;
            double field = body.Mu / r2;
            if (field > bestField) { bestField = field; best = body.Id; }
        }
        return true;
    }
}

/// <summary>SOI telemetry: reports when the stock parent disagrees with the
/// dominant attractor for a SUSTAINED period — expected between the field-balance
/// radius and the (larger) SOI radius on every boundary transit, and near Lagrange
/// points where SOI's answer is genuinely arbitrary (e.g. Sun–Earth L1 lies outside
/// Earth's SOI). One line when an episode sustains past the threshold, one closing
/// line when the pair re-agrees; pair changes and sim-time regressions (save loads)
/// re-arm the episode. Pure sim-time state machine, single-caller (the panel provider,
/// render thread) — wall-clock throttling stays at the call site.</summary>
public sealed class DominantAttractorTelemetry
{
    private readonly double _sustainSeconds;
    private string _stock = "";
    private string _dominant = "";
    private double _episodeStart;
    private bool _inEpisode;
    private bool _reported;

    public DominantAttractorTelemetry(double sustainSeconds) => _sustainSeconds = sustainSeconds;

    /// <summary>Session statics sweep hook: a save load can jump sim time
    /// FORWARD, which the regression re-arm cannot see — reset on rebind/load so a
    /// pre-load episode never splices into the post-load one (and a reported episode
    /// never emits an orphan closing line against the new session).</summary>
    public void Reset()
    {
        _inEpisode = false;
        _reported = false;
    }

    /// <summary>Feed one observation; returns a log line when the state machine has
    /// something to say, else null.</summary>
    public string? Observe(string stockParentId, string dominantId, double time)
    {
        if (stockParentId == dominantId)
        {
            string? closing = _reported
                ? $"dominant attractor and stock parent agree again on '{dominantId}' at t={time:F1} s "
                  + $"(disagreed since t={_episodeStart:F1} s)"
                : null;
            _inEpisode = false;
            _reported = false;
            return closing;
        }
        if (!_inEpisode || stockParentId != _stock || dominantId != _dominant || time < _episodeStart)
        {
            _inEpisode = true;
            _reported = false;
            _stock = stockParentId;
            _dominant = dominantId;
            _episodeStart = time;
            return null;
        }
        if (!_reported && time - _episodeStart >= _sustainSeconds)
        {
            _reported = true;
            return $"dominant attractor '{dominantId}' vs stock parent '{stockParentId}': disagreement sustained "
                + $"{time - _episodeStart:F0} s (since t={_episodeStart:F1} s; informational - SOI stays game bookkeeping)";
        }
        return null;
    }
}
