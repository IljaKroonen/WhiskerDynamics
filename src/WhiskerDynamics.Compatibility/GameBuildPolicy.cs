using System.Reflection;

namespace WhiskerDynamics.Compatibility;

/// <summary>The KSA build this artifact was compiled and verified against.</summary>
public static class GameBuildPolicy
{
    private const string MetadataKey = "VerifiedKsaBuild";

    public static string VerifiedBuild { get; } =
        typeof(GameBuildPolicy).Assembly
            .GetCustomAttributes<AssemblyMetadataAttribute>()
            .Single(attribute => attribute.Key == MetadataKey)
            .Value
        ?? throw new InvalidOperationException($"Missing {MetadataKey} assembly metadata.");

    public static bool IsVerified(string gameBuild) =>
        string.Equals(gameBuild, VerifiedBuild, StringComparison.Ordinal);
}
