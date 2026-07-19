namespace WhiskerDynamics.Core;

/// <summary>Keplerian elements in SI: metres, radians, seconds (time-at-periapsis on the game clock).</summary>
public readonly record struct OrbitalElements(
    double SemiMajorAxis,
    double Eccentricity,
    double Inclination,
    double LongitudeOfAscendingNode,
    double ArgumentOfPeriapsis,
    double TimeAtPeriapsis);
