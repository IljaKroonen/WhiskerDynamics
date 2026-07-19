using WhiskerDynamics.Core;
using Xunit;

namespace WhiskerDynamics.Core.Tests;

public class IntegratorBoundaryValidationTests
{
    private static readonly StateVector ValidState = new(
        new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

    public static TheoryData<string, double> InvalidOptions
    {
        get
        {
            var data = new TheoryData<string, double>();
            foreach (string name in new[] { "RelTol", "AbsTolPos", "AbsTolVel", "InitialStep" })
            foreach (double value in new[]
                { 0, -1, double.NaN, double.PositiveInfinity, double.NegativeInfinity })
                data.Add(name, value);
            foreach (double value in new[]
                { 0, -1, double.NaN, double.NegativeInfinity })
                data.Add("MaxStep", value);
            return data;
        }
    }

    public static TheoryData<int, double> NonFiniteStateComponents
    {
        get
        {
            var data = new TheoryData<int, double>();
            for (int component = 0; component < 6; component++)
            foreach (double value in new[]
                { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
                data.Add(component, value);
            return data;
        }
    }

    public static TheoryData<int, double> NonFiniteVectorComponents
    {
        get
        {
            var data = new TheoryData<int, double>();
            for (int component = 0; component < 3; component++)
            foreach (double value in new[]
                { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
                data.Add(component, value);
            return data;
        }
    }

    [Theory]
    [MemberData(nameof(InvalidOptions))]
    public void Invalid_options_are_rejected_at_every_integration_boundary(
        string optionName, double invalid)
    {
        var options = WithOption(optionName, invalid);
        var gravity = new GravityModel(new Ephemerides([]));
        var root = new CelestialBody { Id = "Root", Mu = 1 };
        int rhsCalls = 0;
        int acceptedCallbacks = 0;

        ArgumentOutOfRangeException[] failures =
        [
            Assert.Throws<ArgumentOutOfRangeException>(() => DormandPrince54.Propagate(
                (t, state) => { rhsCalls++; return Vector3d.Zero; },
                ValidState, 0, 1, options,
                onAcceptedStep: (t, state) => acceptedCallbacks++)),
            Assert.Throws<ArgumentOutOfRangeException>(() => DormandPrince54.PropagateSystem(
                (t, states) => { rhsCalls++; return new Vector3d[states.Length]; },
                [ValidState], 0, 1, options,
                onAcceptedStep: (t, states, derivatives) => acceptedCallbacks++)),
            Assert.Throws<ArgumentOutOfRangeException>(() => DormandPrince853.Propagate(
                (t, state) => { rhsCalls++; return Vector3d.Zero; },
                ValidState, 0, 1, out _, options,
                onAcceptedStep: (t, state) => acceptedCallbacks++)),
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                new TrajectoryPredictor(gravity, ValidState, 0, options)),
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                new NBodyEphemerides([root], 0, [root.Id], options)),
        ];
        Assert.All(failures, failure =>
        {
            Assert.Equal("options", failure.ParamName);
            Assert.Contains(optionName, failure.Message, StringComparison.Ordinal);
        });
        Assert.Equal(0, rhsCalls);
        Assert.Equal(0, acceptedCallbacks);
    }

    [Fact]
    public void Zero_duration_still_validates_options()
    {
        var options = new IntegratorOptions { InitialStep = 0 };

        Assert.Throws<ArgumentOutOfRangeException>(() => DormandPrince54.Propagate(
            (t, state) => Vector3d.Zero, ValidState, 0, 0, options));
        Assert.Throws<ArgumentOutOfRangeException>(() => DormandPrince54.PropagateSystem(
            (t, states) => new Vector3d[states.Length], [ValidState], 0, 0, options));
        Assert.Throws<ArgumentOutOfRangeException>(() => DormandPrince853.Propagate(
            (t, state) => Vector3d.Zero, ValidState, 0, 0, out _, options));
    }

    [Fact]
    public void Positive_infinity_is_a_valid_maximum_step()
    {
        var options = new IntegratorOptions { MaxStep = double.PositiveInfinity };

        Assert.Equal(ValidState, DormandPrince54.Propagate(
            (t, state) => Vector3d.Zero, ValidState, 0, 0, options));
        Assert.Equal(ValidState, DormandPrince853.Propagate(
            (t, state) => Vector3d.Zero, ValidState, 0, 0, out _, options));
    }

    [Theory]
    [MemberData(nameof(NonFiniteStateComponents))]
    public void Non_finite_initial_states_are_rejected_at_every_state_boundary(
        int component, double invalid)
    {
        var initial = WithStateComponent(component, invalid);
        var gravity = new GravityModel(new Ephemerides([]));
        int rhsCalls = 0;

        Assert.Throws<ArgumentException>(() => DormandPrince54.Propagate(
            (t, state) => { rhsCalls++; return Vector3d.Zero; }, initial, 0, 0));
        Assert.Throws<ArgumentException>(() => DormandPrince54.PropagateSystem(
            (t, states) => { rhsCalls++; return new Vector3d[states.Length]; },
            [ValidState, initial], 0, 0));
        Assert.Throws<ArgumentException>(() => DormandPrince853.Propagate(
            (t, state) => { rhsCalls++; return Vector3d.Zero; }, initial, 0, 0, out _));
        Assert.Throws<ArgumentException>(() =>
            new TrajectoryPredictor(gravity, initial, 0));
        Assert.Equal(0, rhsCalls);
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Dp853_rejects_non_finite_endpoints_before_rhs_work(double invalid)
    {
        int rhsCalls = 0;
        Vector3d Acceleration(double time, StateVector state)
        {
            rhsCalls++;
            return Vector3d.Zero;
        }

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            DormandPrince853.Propagate(Acceleration, ValidState, invalid, 1, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            DormandPrince853.Propagate(Acceleration, ValidState, 0, invalid, out _));
        Assert.Equal(0, rhsCalls);
    }

    [Theory]
    [MemberData(nameof(NonFiniteVectorComponents))]
    public void Predictor_rejects_non_finite_impulse_vectors(int component, double invalid)
    {
        var predictor = new TrajectoryPredictor(
            new GravityModel(new Ephemerides([])), ValidState, 0);
        var deltaV = WithVectorComponent(component, invalid);

        Assert.Throws<ArgumentException>(() => predictor.AddImpulse(0, deltaV));
        predictor.AddImpulse(0, Vector3d.Zero);
        Assert.Equal(ValidState, predictor.StateAt(0));
    }

    private static IntegratorOptions WithOption(string name, double value) => name switch
    {
        "RelTol" => new IntegratorOptions { RelTol = value },
        "AbsTolPos" => new IntegratorOptions { AbsTolPos = value },
        "AbsTolVel" => new IntegratorOptions { AbsTolVel = value },
        "InitialStep" => new IntegratorOptions { InitialStep = value },
        "MaxStep" => new IntegratorOptions { MaxStep = value },
        _ => throw new ArgumentOutOfRangeException(nameof(name)),
    };

    private static StateVector WithStateComponent(int component, double value) => component switch
    {
        0 => new StateVector(new Vector3d(value, 2, 3), ValidState.Velocity),
        1 => new StateVector(new Vector3d(1, value, 3), ValidState.Velocity),
        2 => new StateVector(new Vector3d(1, 2, value), ValidState.Velocity),
        3 => new StateVector(ValidState.Position, new Vector3d(value, 5, 6)),
        4 => new StateVector(ValidState.Position, new Vector3d(4, value, 6)),
        5 => new StateVector(ValidState.Position, new Vector3d(4, 5, value)),
        _ => throw new ArgumentOutOfRangeException(nameof(component)),
    };

    private static Vector3d WithVectorComponent(int component, double value) => component switch
    {
        0 => new Vector3d(value, 2, 3),
        1 => new Vector3d(1, value, 3),
        2 => new Vector3d(1, 2, value),
        _ => throw new ArgumentOutOfRangeException(nameof(component)),
    };
}
