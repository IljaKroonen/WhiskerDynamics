namespace WhiskerDynamics.Core;

/// <summary>A fixed coordinate rotation from the game's body-fixed frame into
/// the frame in which a spherical-harmonic model publishes its coefficients.</summary>
public sealed record BodyFixedToModelRotation
{
    public Vector3d ModelXAxisBodyFixed { get; }
    public Vector3d ModelYAxisBodyFixed { get; }
    public Vector3d ModelZAxisBodyFixed { get; }

    public BodyFixedToModelRotation(
        Vector3d modelXAxisBodyFixed,
        Vector3d modelYAxisBodyFixed,
        Vector3d modelZAxisBodyFixed)
    {
        ModelXAxisBodyFixed = modelXAxisBodyFixed;
        ModelYAxisBodyFixed = modelYAxisBodyFixed;
        ModelZAxisBodyFixed = modelZAxisBodyFixed;

        if (!modelXAxisBodyFixed.IsFinite()
            || !modelYAxisBodyFixed.IsFinite()
            || !modelZAxisBodyFixed.IsFinite())
            throw new FormatException(
                "body-fixed-to-model rotation must contain only finite numbers");
        double error = Math.Max(
            Math.Max(Math.Abs(modelXAxisBodyFixed.LengthSquared() - 1),
                Math.Abs(modelYAxisBodyFixed.LengthSquared() - 1)),
            Math.Max(Math.Abs(modelZAxisBodyFixed.LengthSquared() - 1),
                Math.Max(Math.Abs(modelXAxisBodyFixed.Dot(modelYAxisBodyFixed)),
                    Math.Max(Math.Abs(modelXAxisBodyFixed.Dot(modelZAxisBodyFixed)),
                        Math.Abs(modelYAxisBodyFixed.Dot(modelZAxisBodyFixed))))));
        error = Math.Max(error,
            (modelXAxisBodyFixed.Cross(modelYAxisBodyFixed)
                - modelZAxisBodyFixed).Length());
        if (error > 1e-9)
            throw new FormatException(
                "body-fixed-to-model rotation must be a right-handed "
                + $"orthonormal matrix (error {error:E2})");
    }

    public Vector3d ToModelCoordinates(in Vector3d bodyFixed) => new(
        bodyFixed.Dot(ModelXAxisBodyFixed),
        bodyFixed.Dot(ModelYAxisBodyFixed),
        bodyFixed.Dot(ModelZAxisBodyFixed));

    public Vector3d ToBodyFixedCoordinates(in Vector3d model) =>
        ModelXAxisBodyFixed * model.X
        + ModelYAxisBodyFixed * model.Y
        + ModelZAxisBodyFixed * model.Z;
}
