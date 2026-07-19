namespace WhiskerDynamics.Mod.Planning.Periapsis;

/// <summary>The optimizer's typed choice for one candidate burn. Rejections are
/// deliberately distinct from an accepted impulse so an unmodellable or unfit
/// finite arc cannot silently change the objective physics.</summary>
internal enum PeriapsisFiniteAdmissionKind
{
    Impulsive,
    Finite,
    RejectWindowStart,
    RejectHorizon,
    RejectUnmodelable,
}

/// <summary>KSA-free admission policy for the optimizer's candidate burn.</summary>
internal readonly record struct PeriapsisFiniteAdmission
{
    private const string WindowFailure =
        "rejected: finite burn overlaps the preceding burn";
    private const string HorizonFailure =
        "rejected: finite burn extends beyond the prediction horizon - extend the plan length";
    private const string UnmodelableFailure =
        "rejected: finite burn duration could not be modeled safely";

    private readonly FiniteBurnExpansion? _expansion;

    private PeriapsisFiniteAdmission(PeriapsisFiniteAdmissionKind kind,
        FiniteBurnExpansion? expansion, double? modelStartSeconds,
        double? modelEndSeconds, string? failure)
    {
        Kind = kind;
        _expansion = expansion;
        ModelStartSeconds = modelStartSeconds;
        ModelEndSeconds = modelEndSeconds;
        Failure = failure;
    }

    internal PeriapsisFiniteAdmissionKind Kind { get; }
    /// <summary>The accepted physical execution window. For a discretized finite
    /// burn this is ignition through cutoff even when K=1 is represented by the
    /// objective as one impulse. Null on every rejection.</summary>
    internal double? ModelStartSeconds { get; }
    internal double? ModelEndSeconds { get; }
    internal string? Failure { get; }

    /// <summary>Returns true only for an accepted dispatch. A true result with a
    /// null expansion is the intentional impulse case; every rejection returns
    /// false and clears <paramref name="expansion"/>.</summary>
    internal bool TryGetAcceptedExpansion(out FiniteBurnExpansion? expansion)
    {
        if (Kind == PeriapsisFiniteAdmissionKind.Impulsive
            && ModelStartSeconds is not null && ModelEndSeconds is not null)
        {
            expansion = null;
            return true;
        }
        if (Kind == PeriapsisFiniteAdmissionKind.Finite && _expansion is not null
            && ModelStartSeconds is not null && ModelEndSeconds is not null)
        {
            expansion = _expansion;
            return true;
        }

        expansion = null;
        return false;
    }

    /// <summary>Admits a candidate only when its selected physics model can be
    /// represented safely and its entire centered thrust arc fits the available
    /// window. The preceding bound is exclusive; the horizon is inclusive.</summary>
    internal static PeriapsisFiniteAdmission Decide(double nodeTime, double magnitude,
        FiniteBurnFold? model, EngineScalars chainedEngine,
        double exclusiveEarliestIgnition, double inclusiveHorizon)
    {
        if (!double.IsFinite(nodeTime)
            || !double.IsFinite(magnitude)
            || magnitude < 0.0
            || !double.IsFinite(exclusiveEarliestIgnition)
            || !double.IsFinite(inclusiveHorizon))
            return RejectUnmodelable();

        // These are intentional physics choices, so their physical window is the
        // node itself. An active, nonzero finite model must account for the FC's
        // centered thrust window even when its configured K is only one.
        if (model is null || magnitude == 0.0)
            return AcceptedImpulse(nodeTime, nodeTime);

        if (!double.IsFinite(model.SliceSeconds)
            || !FiniteBurnKernel.TryGetPhysicalWindow(
                nodeTime, magnitude, chainedEngine, out var window))
            return RejectUnmodelable();

        double ignition = window.IgnitionSeconds;
        double cutoff = window.CutoffSeconds;

        // Window admission is about the physical burn the FC will fly, not the
        // numerical representation chosen for the objective. Enforce it before K.
        if (ignition <= exclusiveEarliestIgnition)
            return new(PeriapsisFiniteAdmissionKind.RejectWindowStart,
                null, null, null, WindowFailure);
        if (cutoff > inclusiveHorizon)
            return new(PeriapsisFiniteAdmissionKind.RejectHorizon,
                null, null, null, HorizonFailure);

        int sliceCount = FiniteBurnKernel.SliceCount(
            window.DurationSeconds, model.SliceSeconds, model.MaxSlices);
        if (sliceCount <= 1)
            return AcceptedImpulse(ignition, cutoff);

        FiniteBurnExpansion? expansion = FiniteBurnKernel.Expand(nodeTime, magnitude,
            chainedEngine, model.SliceSeconds, model.MaxSlices);
        if (!IsUsable(expansion))
            return RejectUnmodelable();

        // The accepted expansion and the pre-admitted physical window must be the
        // same model. Fail closed if the kernel ever stops preserving that identity.
        if (expansion!.IgnitionSeconds != ignition
            || expansion.DurationSeconds != window.DurationSeconds
            || expansion.IgnitionSeconds + expansion.DurationSeconds != cutoff)
            return RejectUnmodelable();

        return new(PeriapsisFiniteAdmissionKind.Finite,
            expansion, ignition, cutoff, null);
    }

    private static bool IsUsable(FiniteBurnExpansion? expansion)
    {
        if (expansion is null
            || !double.IsFinite(expansion.DurationSeconds)
            || expansion.DurationSeconds <= 0.0
            || !double.IsFinite(expansion.IgnitionSeconds)
            || expansion.Times.Length == 0
            || expansion.Times.Length != expansion.Magnitudes.Length)
            return false;

        for (int i = 0; i < expansion.Times.Length; i++)
        {
            if (!double.IsFinite(expansion.Times[i])
                || !double.IsFinite(expansion.Magnitudes[i])
                || expansion.Magnitudes[i] < 0.0)
                return false;
        }
        return true;
    }

    private static PeriapsisFiniteAdmission RejectUnmodelable()
        => new(PeriapsisFiniteAdmissionKind.RejectUnmodelable,
            null, null, null, UnmodelableFailure);

    private static PeriapsisFiniteAdmission AcceptedImpulse(
        double modelStartSeconds, double modelEndSeconds)
        => new(PeriapsisFiniteAdmissionKind.Impulsive,
            null, modelStartSeconds, modelEndSeconds, null);
}
