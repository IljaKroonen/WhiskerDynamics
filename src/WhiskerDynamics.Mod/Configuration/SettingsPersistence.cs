using Tomlet;

namespace WhiskerDynamics.Mod.Configuration;

/// <summary>Writes the live ModConfig back to the SAME whiskerdynamics.toml LoadOrCreate read
/// (full-file rewrite through the same Tomlet mapping, so what we write is exactly what
/// the next boot loads). Two accepted consequences of the full rewrite: hand-added TOML
/// comments are lost, and keys the current build no longer defines are dropped (they
/// are ignored on load anyway). KSA-free so the round-trip is offline-testable.</summary>
public static class SettingsPersistence
{
    /// <summary>Containment contract: never throws. A failed write reports the message
    /// for the panel to show (and the caller to log, throttled) — the in-memory config
    /// keeps applying for the session either way.</summary>
    public static bool TrySave(ModConfig config, string path, out string error) =>
        TrySave(config, path, hooks: null, out error);

    internal static bool TrySave(ModConfig config, string path, AtomicTextFileHooks? hooks,
        out string error)
    {
        try
        {
            AtomicTextFile.WriteAllText(path, TomletMain.TomlStringFrom(config), hooks);
            error = "";
            return true;
        }
        catch (Exception e)
        {
            error = e.Message;
            return false;
        }
    }
}
