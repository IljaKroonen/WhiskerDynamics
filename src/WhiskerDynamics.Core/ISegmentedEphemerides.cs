namespace WhiskerDynamics.Core;

/// <summary>Internal segment surface consumed by GravityModel's SIMD cache. Both the
/// mutable rails store and immutable prediction snapshots implement it.</summary>
internal interface ISegmentedEphemerides : IBackboneEphemerides
{
    double StartTime { get; }
    double Horizon { get; }
    int CommitStamp { get; }
    int IntegratedIndexOf(CelestialBody body);
    NBodyEphemerides.BodySegment ResolveBodySegment(int bodyIndex, double time);
    bool InCommittedRegion(int bodyIndex, double time);
    Vector3d BodyPositionAt(int bodyIndex, double time);
    bool TryResolveDenseSegment(double time, out int hi, out double t0, out double dt);
    StateVector DenseNodeState(int nodeIndex, int bodyIndex);
}
