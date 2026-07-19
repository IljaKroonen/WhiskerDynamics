namespace WhiskerDynamics.Core;

/// <summary>Reports which modeled celestial trajectories mutually backreact. Bodies
/// outside the backbone are still numerical ephemeris tracks.</summary>
public interface IBackboneEphemerides : IEphemerides
{
    bool IsBackbone(CelestialBody body);

    /// <summary>True when <paramref name="body"/>'s numerical track includes the
    /// point-mass gravity of <paramref name="source"/>. Restricted tracks feel the
    /// backbone and their massive restricted ancestors, but not restricted peers.</summary>
    bool FeelsGravityFrom(CelestialBody body, CelestialBody source);
}
