namespace WhiskerDynamics.Core;

public sealed class CelestialBody
{
    public required string Id { get; init; }
    /// <summary>Standard gravitational parameter G·M, m³/s².</summary>
    public required double Mu { get; init; }
    /// <summary>Mean radius, metres.</summary>
    public double MeanRadius { get; init; }
    /// <summary>Game sphere-of-influence radius, metres.</summary>
    public double SphereOfInfluence { get; set; } = double.NaN;
    /// <summary>Optional extended-body gravity field.  Vessel dynamics consume this;
    /// celestial rails deliberately remain point-mass.</summary>
    public Geopotential? Geopotential { get; init; }
    public CelestialBody? Parent { get; set; }
    /// <summary>Parent-relative Keplerian orbit. Null only for the root body.</summary>
    public OrbitalElements? Orbit { get; set; }
}
