namespace WhiskerDynamics.Mod.Vessels;

/// <summary>The single KSA-free rule for deciding whether a tracked predictor is the
/// authoritative trajectory of a committed live vehicle.</summary>
public static class PredictorAuthorityPolicy
{
    public enum Reason
    {
        Authoritative,
        MissingEntry,
        EntryReplaced,
        BoundVehicleUnavailable,
        VehicleReplaced,
        ReseedPending,
        NotFreefall,
        PredictorUnavailable,
        PredictorReplaced,
    }

    public readonly record struct State(
        bool EntryPresent,
        bool SameEntry,
        bool BoundVehicleAvailable,
        bool SameVehicle,
        bool ReseedPending,
        bool CommittedFreefall,
        bool PredictorAvailable,
        bool SamePredictor);

    public static Reason Classify(State state)
    {
        if (!state.EntryPresent) return Reason.MissingEntry;
        if (!state.SameEntry) return Reason.EntryReplaced;
        if (!state.BoundVehicleAvailable) return Reason.BoundVehicleUnavailable;
        if (!state.SameVehicle) return Reason.VehicleReplaced;
        if (state.ReseedPending) return Reason.ReseedPending;
        if (!state.CommittedFreefall) return Reason.NotFreefall;
        if (!state.PredictorAvailable) return Reason.PredictorUnavailable;
        if (!state.SamePredictor) return Reason.PredictorReplaced;
        return Reason.Authoritative;
    }

    public static bool IsAuthoritative(Reason reason) => reason == Reason.Authoritative;

    public static string Describe(Reason reason) => reason switch
    {
        Reason.Authoritative => "n-body rails authority is current",
        Reason.MissingEntry => "the vessel is not tracked",
        Reason.EntryReplaced => "the tracked vessel entry was replaced",
        Reason.BoundVehicleUnavailable => "the bound vehicle is no longer available",
        Reason.VehicleReplaced => "the vehicle instance was replaced",
        Reason.ReseedPending => "the predictor is waiting for a post-live reseed",
        Reason.NotFreefall => "live physics currently owns the vessel",
        Reason.PredictorUnavailable => "the predictor is unavailable",
        Reason.PredictorReplaced => "the predictor lineage was reseeded",
        _ => "n-body rails authority is unavailable",
    };
}
