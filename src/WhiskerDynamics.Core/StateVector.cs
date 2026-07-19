namespace WhiskerDynamics.Core;

public readonly record struct StateVector(Vector3d Position, Vector3d Velocity)
{
    internal bool IsFinite() => Position.IsFinite() && Velocity.IsFinite();

    public static StateVector operator +(StateVector a, StateVector b) =>
        new(a.Position + b.Position, a.Velocity + b.Velocity);

    public static StateVector operator -(StateVector a, StateVector b) =>
        new(a.Position - b.Position, a.Velocity - b.Velocity);

    public static StateVector operator *(StateVector a, double s) =>
        new(a.Position * s, a.Velocity * s);
}
