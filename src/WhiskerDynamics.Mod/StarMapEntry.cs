using StarMap.API;

namespace WhiskerDynamics.Mod;

/// <summary>
/// StarMap Mod Loader entry point.
/// StarMap discovers this class by the <c>[StarMapMod]</c> attribute, instantiates it once
/// via <c>Activator.CreateInstance</c> (public parameterless ctor required), and invokes
/// <c>[StarMapBeforeMain]</c> before <c>KSA.Program.Main</c> — strictly before the first
/// simulation tick.
/// </summary>
[StarMapMod]
public class StarMapEntry
{
    [StarMapBeforeMain]
    public void BeforeMain()
    {
        // CWD is the *game install dir* at this point (StarMap sets it before Init) —
        // derive everything from the assembly location, never from relative paths.
        string location = typeof(StarMapEntry).Assembly.Location;
        string? modDir = string.IsNullOrEmpty(location) ? null : Path.GetDirectoryName(location);
        if (string.IsNullOrEmpty(modDir))
        {
            // Nowhere sane to init from; note it best-effort and bail — never throw
            // into the loader.
            AppendErrorBestEffort(Environment.CurrentDirectory,
                $"assembly location unusable ('{location}'); skipping EarlyInit");
            return;
        }
        try
        {
            ModMain.EarlyInit(modDir);
        }
        catch (Exception e)
        {
            AppendErrorBestEffort(modDir, e.ToString());
        }
    }

    private static void AppendErrorBestEffort(string dir, string message)
    {
        try
        {
            File.AppendAllText(Path.Combine(dir, "whiskerdynamics-entry-error.log"),
                $"{DateTime.Now:O} {message}{Environment.NewLine}");
        }
        catch
        {
            // nothing left to do — never throw into the loader
        }
    }
}
