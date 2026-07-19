namespace WhiskerDynamics.Core;

public sealed record IntegratorOptions
{
    public static IntegratorOptions Default { get; } = new();

    public double RelTol { get; init; } = 1e-13;
    /// <summary>Absolute position tolerance, metres.</summary>
    public double AbsTolPos { get; init; } = 1e-6;
    /// <summary>Absolute velocity tolerance, m/s.</summary>
    public double AbsTolVel { get; init; } = 1e-9;
    /// <summary>Initial trial step, seconds.</summary>
    public double InitialStep { get; init; } = 60;
    /// <summary>Maximum step, seconds.</summary>
    public double MaxStep { get; init; } = double.PositiveInfinity;

    internal static IntegratorOptions Validate(IntegratorOptions? options)
    {
        var value = options ?? Default;
        RequirePositiveFinite(value.RelTol, nameof(RelTol), nameof(options));
        RequirePositiveFinite(value.AbsTolPos, nameof(AbsTolPos), nameof(options));
        RequirePositiveFinite(value.AbsTolVel, nameof(AbsTolVel), nameof(options));
        RequirePositiveFinite(value.InitialStep, nameof(InitialStep), nameof(options));
        if (!(value.MaxStep > 0)
            || !double.IsFinite(value.MaxStep) && !double.IsPositiveInfinity(value.MaxStep))
            throw new ArgumentOutOfRangeException(
                nameof(options), value.MaxStep,
                $"{nameof(MaxStep)} must be positive and finite or positive infinity.");
        return value;
    }

    private static void RequirePositiveFinite(
        double value, string optionName, string parameterName)
    {
        if (!(value > 0) || !double.IsFinite(value))
            throw new ArgumentOutOfRangeException(
                parameterName, value, $"{optionName} must be positive and finite.");
    }
}
