using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Planning.Periapsis;

/// <summary>The exact fixed-burn state handed to the candidate objective after a
/// strict fold. Failure preserves the state after the last completely applied burn;
/// the caller discards its partially mutated local predictor and aborts the solve.</summary>
internal readonly record struct PeriapsisStrictFoldResult(
    bool Success,
    int AppliedBurns,
    EngineScalars EngineAtTarget,
    double LastBoundSeconds,
    string? Failure);

/// <summary>KSA-free orchestration for the optimizer's fixed burns. Conversion,
/// finite admission, predictor mutation, and mass/window commitment are one ordered
/// operation per burn, so the candidate engine/bound can never describe different
/// physics from the local predictor. Unlike the display fold, a rejected finite arc
/// is never replaced by an impulse.</summary>
internal static class PeriapsisStrictFold
{
    internal static PeriapsisStrictFoldResult Fold(
        IReadOnlyList<(double Time, Vector3d DvVlf, string BasisParentId)> burns,
        double exclusiveStartSeconds,
        double inclusiveHorizonSeconds,
        FiniteBurnFold? model,
        Func<int, Vector3d?> convertToEcliptic,
        Action<double, Vector3d> applyImpulse)
    {
        double massKg = model?.Engine.MassKg ?? 0.0;
        double lastBound = exclusiveStartSeconds;
        int appliedBurns = 0;
        var appliedNodes = new HashSet<double>();

        EngineScalars EngineAtTarget() => model is { } finite
            ? finite.Engine with { MassKg = massKg }
            : default;

        PeriapsisStrictFoldResult Reject(int index, double time, string reason) =>
            new(false, appliedBurns, EngineAtTarget(), lastBound,
                $"fixed burn {index + 1} at t={time:F1} s {reason}");

        for (int i = 0; i < burns.Count; i++)
        {
            var burn = burns[i];
            if (!OverlayKernel.BurnInWindow(
                    burn.Time, exclusiveStartSeconds, inclusiveHorizonSeconds)
                || BurnIdentityPolicy.ContainsBurn(appliedNodes, burn.Time))
                continue;

            Vector3d? converted;
            try
            {
                // Conversion observes every earlier successful mutation and runs
                // exactly once for this participating node.
                converted = convertToEcliptic(i);
            }
            catch (Exception e)
            {
                return Reject(i, burn.Time,
                    $"could not convert its VLF delta-v safely: {e.Message}");
            }
            if (converted is not { } deltaVEcl || !IsFinite(deltaVEcl))
                return Reject(i, burn.Time, "has a degenerate VLF basis");

            double authoredMagnitude = burn.DvVlf.Length();
            var engineAtBurn = model is { } finite
                ? finite.Engine with { MassKg = massKg }
                : default;
            var admission = PeriapsisFiniteAdmission.Decide(
                burn.Time, authoredMagnitude, model, engineAtBurn,
                lastBound, inclusiveHorizonSeconds);
            if (!admission.TryGetAcceptedExpansion(out var expansion)
                || admission.ModelStartSeconds is not { } modelStart
                || admission.ModelEndSeconds is not { } modelEnd
                || !double.IsFinite(modelStart)
                || !double.IsFinite(modelEnd))
            {
                return Reject(i, burn.Time,
                    admission.Failure ?? "failed finite-burn admission");
            }

            try
            {
                switch (admission.Kind)
                {
                    case PeriapsisFiniteAdmissionKind.Impulsive:
                        // No model, zero dv, and intentional K=1 discretization all
                        // use one objective impulse. Their physical window still
                        // comes from admission and advances the next-burn bound.
                        applyImpulse(burn.Time, deltaVEcl);
                        break;
                    case PeriapsisFiniteAdmissionKind.Finite:
                        if (expansion is null)
                            return Reject(i, burn.Time,
                                "was admitted without a finite expansion");
                        double convertedMagnitude = deltaVEcl.Length();
                        if (!(convertedMagnitude > 0.0)
                            || !double.IsFinite(convertedMagnitude))
                            return Reject(i, burn.Time,
                                "has a degenerate finite-burn direction");
                        var direction = deltaVEcl * (1.0 / convertedMagnitude);
                        for (int s = 0; s < expansion.Times.Length; s++)
                            applyImpulse(expansion.Times[s],
                                direction * expansion.Magnitudes[s]);
                        break;
                    case PeriapsisFiniteAdmissionKind.RejectWindowStart:
                    case PeriapsisFiniteAdmissionKind.RejectHorizon:
                    case PeriapsisFiniteAdmissionKind.RejectUnmodelable:
                    default:
                        return Reject(i, burn.Time,
                            admission.Failure ?? "failed finite-burn admission");
                }
            }
            catch (Exception e)
            {
                return Reject(i, burn.Time,
                    $"could not apply its admitted model safely: {e.Message}");
            }

            // Commit orchestration state only after every admitted impulse landed.
            appliedNodes.Add(burn.Time);
            appliedBurns++;
            if (model is not null)
                massKg = FiniteBurnKernel.MassAfterBurn(
                    authoredMagnitude, engineAtBurn);
            lastBound = Math.Max(lastBound, modelEnd);
        }

        return new(true, appliedBurns, EngineAtTarget(), lastBound, null);
    }

    private static bool IsFinite(Vector3d value) =>
        double.IsFinite(value.X)
        && double.IsFinite(value.Y)
        && double.IsFinite(value.Z);
}
