namespace WhiskerDynamics.Mod.Planning;

/// <summary>Engine/mass scalars captured from the stock flight computer at snapshot
/// time (TotalMassPropsBody.Mass and the ActiveEngineThrust/ActiveEngineMassFlowRate
/// performance sums): everything the
/// finite-burn estimate needs, and nothing main-thread-bound — the rails worker only
/// ever sees these three numbers. Zeroed or absent scalars
/// are simply not <see cref="Usable"/> and the fold keeps impulsive burns.</summary>
public readonly record struct EngineScalars(double MassKg, double ExhaustVelocity, double MassFlowRate)
{
    public bool Usable =>
        double.IsFinite(MassKg) && MassKg > 0
        && double.IsFinite(ExhaustVelocity) && ExhaustVelocity > 0
        && double.IsFinite(MassFlowRate) && MassFlowRate > 0;
}

/// <summary>The display fold's finite-burn model: capture-time engine scalars plus
/// the config discretization knobs. Null (or an unusable engine) keeps the fold
/// impulsive — today's exact behavior.</summary>
public sealed record FiniteBurnFold(EngineScalars Engine, double SliceSeconds, int MaxSlices);

/// <summary>Propulsion set used for finite-plan estimates. RCS means the active
/// forward-translation (+body X) jets while the vessel is pointed along the burn.
/// KSA stock maneuver nodes still auto-execute on main engines only.</summary>
public enum PropulsionSource
{
    MainEngines,
    RcsForward,
}

/// <summary>KSA-free reduction of selected forward-RCS nozzle performance into the
/// scalar rocket model. Axial forces add with sign while every firing nozzle's flow
/// adds positively: canted or partially opposing jets consume propellant without
/// being credited with thrust they do not produce along the commanded axis.</summary>
public static class RcsPerformanceKernel
{
    public static EngineScalars FromSelectedJets(double massKg,
        IEnumerable<(double AxialForceNewtons, double MassFlowRate)> jets)
    {
        double force = 0.0;
        double flow = 0.0;
        foreach (var jet in jets)
        {
            if (!double.IsFinite(jet.AxialForceNewtons)
                || !double.IsFinite(jet.MassFlowRate)
                || jet.MassFlowRate < 0.0)
                return default;
            if (jet.MassFlowRate == 0.0) continue;
            force += jet.AxialForceNewtons;
            flow += jet.MassFlowRate;
        }
        return double.IsFinite(massKg) && massKg > 0.0
            && double.IsFinite(force) && force > 0.0
            && double.IsFinite(flow) && flow > 0.0
                ? new EngineScalars(massKg, force / flow, flow)
                : default;
    }
}

/// <summary>One planned burn expanded into sub-impulses: strictly increasing slice
/// times spanning the centered burn window, and per-slice delta-v magnitudes whose sum
/// telescopes EXACTLY to the burn's total (each is ve·ln(mᵢ/mᵢ₊₁), so the sum is
/// ve·ln(m₀/m_K) — the rocket equation for the whole burn, by construction).</summary>
public sealed record FiniteBurnExpansion(
    double[] Times, double[] Magnitudes, double DurationSeconds, double IgnitionSeconds);

/// <summary>The flight computer's physical centered-thrust window. This remains
/// distinct from <see cref="FiniteBurnExpansion"/>: a short burn may deliberately
/// use one numerical impulse at its node while still occupying a nonzero physical
/// interval for lead, overlap, horizon, collision, and terminal-state rules.</summary>
public readonly record struct FiniteBurnWindow(
    double IgnitionSeconds, double CutoffSeconds, double DurationSeconds);

/// <summary>A safely representable finite command: its physical window plus an
/// optional multi-slice numerical expansion. Null expansion means the configured
/// discretization represents the command by one impulse at the node; it does not
/// collapse <see cref="Window"/> to the node.</summary>
public readonly record struct FiniteBurnCommand(
    FiniteBurnWindow Window, FiniteBurnExpansion? Expansion);

/// <summary>Finite-burn estimation (KSA-free, offline-tested): predicts the stock
/// flight computer's execution of a planned burn instead of an instantaneous kick.
/// The executor's semantics are deterministic and fully determined by three scalars
/// (the stock flight-computer contract):
///   - duration is the rocket equation at full throttle — propellant =
///     m·(1 − e^(−Δv/vₑ)), duration = propellant / ṁ (UpdateBurnTarget, :750-756;
///     Auto mode commands throttle 1, ComputeBurnControl :703);
///   - the burn is CENTERED on the node: IgnitionTime = ImpulsiveInstant − duration/2
///     (:762);
///   - steering is a fixed inertial direction: the FC points along DeltaVToGoCci =
///     target − accumulated, and the accumulator integrates thrust-only velocity
///     changes (BurnTarget.cs:22; DeltaVAccumCci += DeltaVelocityCci,
///     FlightComputer.cs:299, which
///     excludes gravity), so thrust along to-go keeps the accumulator collinear with
///     the target — the direction never moves in the error-free estimate.
/// The estimate is therefore a discretization choice, not a model choice: K
/// sub-impulses at slice midpoints along the fixed direction, magnitudes from the
/// per-slice rocket equation under linear mass depletion, converge to the executed
/// arc. Attitude slew/wobble and manual throttle are deliberately not modeled.</summary>
public static class FiniteBurnKernel
{
    /// <summary>Full-throttle burn duration, seconds — the FC's own formula
    /// (FlightComputer.cs:750-756). 0 for a non-positive delta-v.</summary>
    public static double BurnDurationSeconds(double dvMagnitude, EngineScalars engine)
    {
        if (!engine.Usable || !(dvMagnitude > 0)) return 0.0;
        double propellant = engine.MassKg * (1.0 - Math.Exp(-dvMagnitude / engine.ExhaustVelocity));
        return propellant / engine.MassFlowRate;
    }

    /// <summary>Vessel mass after the burn: m·e^(−Δv/vₑ) — feeds the NEXT burn's
    /// duration in a chained plan (each burn's propellant leaves the ship).</summary>
    public static double MassAfterBurn(double dvMagnitude, EngineScalars engine)
        => !engine.Usable || !(dvMagnitude > 0)
            ? engine.MassKg
            : engine.MassKg * Math.Exp(-dvMagnitude / engine.ExhaustVelocity);

    /// <summary>Builds the FC's physical centered window independently of the
    /// configured numerical slice count. False for inputs that cannot define a
    /// finite physical command.</summary>
    public static bool TryGetPhysicalWindow(double nodeTime, double dvMagnitude,
        EngineScalars engine, out FiniteBurnWindow window)
    {
        window = default;
        if (!double.IsFinite(nodeTime) || !double.IsFinite(dvMagnitude)
            || dvMagnitude < 0.0 || !engine.Usable)
            return false;
        double duration = BurnDurationSeconds(dvMagnitude, engine);
        double ignition = nodeTime - duration / 2.0;
        double cutoff = ignition + duration;
        if (!double.IsFinite(duration) || duration < 0.0
            || !double.IsFinite(ignition) || !double.IsFinite(cutoff))
            return false;
        window = new FiniteBurnWindow(ignition, cutoff, duration);
        return true;
    }

    /// <summary>Resolves both halves of a finite command without conflating them:
    /// the physical window always comes from the FC duration, while the numerical
    /// representation is an impulse for K=1 or a validated midpoint expansion for
    /// K&gt;1. False means an active finite model cannot represent the command safely.</summary>
    public static bool TryResolveCommand(double nodeTime, double dvMagnitude,
        EngineScalars engine, double sliceSeconds, int maxSlices,
        out FiniteBurnCommand command)
    {
        command = default;
        if (!double.IsFinite(sliceSeconds) || !(sliceSeconds > 0.0)
            || !TryGetPhysicalWindow(nodeTime, dvMagnitude, engine, out var window))
            return false;
        int sliceCount = SliceCount(window.DurationSeconds, sliceSeconds, maxSlices);
        if (sliceCount <= 1)
        {
            command = new FiniteBurnCommand(window, null);
            return true;
        }
        var expansion = Expand(nodeTime, dvMagnitude, engine, sliceSeconds, maxSlices);
        if (expansion is null
            || expansion.IgnitionSeconds != window.IgnitionSeconds
            || expansion.DurationSeconds != window.DurationSeconds
            || expansion.IgnitionSeconds + expansion.DurationSeconds != window.CutoffSeconds)
            return false;
        command = new FiniteBurnCommand(window, expansion);
        return true;
    }

    /// <summary>Slice count for a burn of <paramref name="durationSeconds"/>: one
    /// slice per <paramref name="sliceSeconds"/> of thrust (ceiling), capped at
    /// <paramref name="maxSlices"/>. 1 (= today's impulse, exactly) for short burns,
    /// a disabled config (sliceSeconds &lt;= 0), or a degenerate duration.</summary>
    public static int SliceCount(double durationSeconds, double sliceSeconds, int maxSlices)
    {
        if (!(sliceSeconds > 0) || !(durationSeconds > 0)) return 1;
        double slices = Math.Ceiling(durationSeconds / sliceSeconds);
        return (int)Math.Clamp(slices, 1, Math.Max(1, maxSlices));
    }

    /// <summary>Expands one burn into sub-impulses over the centered window
    /// [node − T/2, node + T/2]: K equal-time slices, each slice's impulse at its
    /// midpoint with magnitude ve·ln(mᵢ/mᵢ₊₁) under linear mass depletion — so the
    /// magnitudes GROW through the burn (the ship lightens) and their sum telescopes
    /// exactly to <paramref name="dvMagnitude"/>. Null when there is nothing to
    /// expand (K &lt;= 1, degenerate engine/delta-v): the caller keeps the impulse.
    /// Window-fit against neighbors is the CALLER's rule — this kernel only shapes
    /// one burn.</summary>
    public static FiniteBurnExpansion? Expand(double nodeTime, double dvMagnitude,
        EngineScalars engine, double sliceSeconds, int maxSlices)
    {
        double duration = BurnDurationSeconds(dvMagnitude, engine);
        int k = SliceCount(duration, sliceSeconds, maxSlices);
        if (k <= 1) return null;

        double ignition = nodeTime - duration / 2.0;
        double sliceDt = duration / k;
        var times = new double[k];
        var magnitudes = new double[k];
        for (int i = 0; i < k; i++)
        {
            times[i] = ignition + (i + 0.5) * sliceDt;
            double massAtStart = engine.MassKg - engine.MassFlowRate * (i * sliceDt);
            double massAtEnd = engine.MassKg - engine.MassFlowRate * ((i + 1) * sliceDt);
            magnitudes[i] = engine.ExhaustVelocity * Math.Log(massAtStart / massAtEnd);
            // Tank-emptying guard: at dv/ve large enough that e^(−dv/ve) underflows,
            // the linearly-depleted end mass reaches 0 (or a few ulps negative) and
            // the log blows up — a NaN/Inf impulse would silently poison every
            // sampled position downstream. Keep the impulse instead.
            if (!double.IsFinite(magnitudes[i])) return null;
        }
        return new FiniteBurnExpansion(times, magnitudes, duration, ignition);
    }
}
