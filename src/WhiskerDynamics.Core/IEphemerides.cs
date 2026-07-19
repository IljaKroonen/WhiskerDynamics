namespace WhiskerDynamics.Core;

/// <summary>Source of celestial body states at any time, in the shared inertial frame.</summary>
public interface IEphemerides
{
    IReadOnlyList<CelestialBody> Bodies { get; }
    CelestialBody this[string id] { get; }
    StateVector GetState(CelestialBody body, double time);
}
