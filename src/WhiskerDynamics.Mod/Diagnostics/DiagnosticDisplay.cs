namespace WhiskerDynamics.Mod.Diagnostics;

/// <summary>Session-local visual diagnostics. These switches affect presentation
/// only; they never alter propagation, plan authority, or interaction routing.</summary>
internal static class DiagnosticDisplay
{
    private static int _showStockPatchedConics;

    internal static bool ShowStockPatchedConics
    {
        get => System.Threading.Volatile.Read(ref _showStockPatchedConics) != 0;
        set => System.Threading.Volatile.Write(
            ref _showStockPatchedConics, value ? 1 : 0);
    }

    internal static void ResetSessionStatics() =>
        System.Threading.Volatile.Write(ref _showStockPatchedConics, 0);
}
