namespace WhiskerDynamics.Core;

public readonly record struct Vector3d(double X, double Y, double Z)
{
    public static readonly Vector3d Zero = new(0, 0, 0);

    public static Vector3d operator +(Vector3d a, Vector3d b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Vector3d operator -(Vector3d a, Vector3d b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Vector3d operator -(Vector3d a) => new(-a.X, -a.Y, -a.Z);
    public static Vector3d operator *(Vector3d a, double s) => new(a.X * s, a.Y * s, a.Z * s);
    public static Vector3d operator *(double s, Vector3d a) => a * s;
    public static Vector3d operator /(Vector3d a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    public double Dot(Vector3d b) => X * b.X + Y * b.Y + Z * b.Z;

    public Vector3d Cross(Vector3d b) =>
        new(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);

    public double LengthSquared() => Dot(this);
    public double Length() => Math.Sqrt(LengthSquared());
    public Vector3d Normalized() => this / Length();

    internal bool IsFinite() =>
        double.IsFinite(X) && double.IsFinite(Y) && double.IsFinite(Z);

    /// <summary>Rodrigues rotation about a unit axis (right-handed, radians).</summary>
    public Vector3d RotateAbout(Vector3d axis, double angle)
    {
        double c = Math.Cos(angle), s = Math.Sin(angle);
        return this * c + axis.Cross(this) * s + axis * (axis.Dot(this) * (1 - c));
    }
}
