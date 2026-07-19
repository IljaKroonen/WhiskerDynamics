using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Frames;

[CollectionDefinition(nameof(TargetPredictorAuthorityPathCollection),
    DisableParallelization = true)]
public sealed class TargetPredictorAuthorityPathCollection;

[Collection(nameof(TargetPredictorAuthorityPathCollection))]
public sealed class TargetPredictorAuthorityPathTests : IDisposable
{
    private const double Time = 10_000.0;
    private const string ParentId = "Mercury";
    private const string ControlledId = "Controlled";
    private const string TargetId = "Target";

    private static readonly FieldInfo TrackedEntries = typeof(VesselRegistry).GetField(
        "_tracked", BindingFlags.Instance | BindingFlags.NonPublic)!;
    private static readonly FieldInfo VehicleProps = typeof(Vehicle).GetField(
        "_props", BindingFlags.Instance | BindingFlags.NonPublic)!;
    private static readonly FieldInfo ParentCciToCce = typeof(Celestial).GetField(
        "_cci2Cce", BindingFlags.Instance | BindingFlags.NonPublic)!;
    private static readonly PropertyInfo AstronomicalId = typeof(Astronomical).GetProperty(
        nameof(Astronomical.Id), BindingFlags.Instance | BindingFlags.Public)!;
    private static readonly PropertyInfo CelestialMass = typeof(Celestial).GetProperty(
        nameof(Celestial.Mass), BindingFlags.Instance | BindingFlags.Public)!;
    private static readonly PropertyInfo VehicleTarget = typeof(Vehicle).GetProperty(
        nameof(Vehicle.Target), BindingFlags.Instance | BindingFlags.Public)!;
    private static readonly PropertyInfo VehicleFlightPlan = typeof(Vehicle).GetProperty(
        nameof(Vehicle.FlightPlan), BindingFlags.Instance | BindingFlags.Public)!;

    private readonly RailsService _rails;
    private readonly VesselRegistry _vessels;
    private readonly Celestial _parent;
    private readonly Vehicle _controlled;
    private readonly Vehicle _target;
    private readonly Vehicle? _previousControlled;

    public TargetPredictorAuthorityPathTests()
    {
        var config = new ModConfig { RailsAheadDays = 1 };
        var constants = new GameConstants(
            6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        _rails = TestRailsService.FromFixture(config, constants);
        _rails.NoteSimTime(Time);
        Assert.True(SpinWait.SpinUntil(() => _rails.IsReadyAt(Time), 5000),
            "fixture rails did not reach the target sample time");
        _vessels = new VesselRegistry(config, _rails);
        _parent = NewParent();
        _controlled = NewVehicle(ControlledId,
            new double3(8_000_000, 0, 0), new double3(0, 5_000, 0));
        _target = NewVehicle(TargetId,
            new double3(10_000_000, 2_000_000, 0), new double3(-500, 6_000, 0));
        SetTarget(_controlled, _target);
        Track(_controlled, new Vector3d(8_000_000, 0, 0));
        Track(_target, new Vector3d(30_000_000, -4_000_000, 0));
        _previousControlled = KSA.Program.ControlledVehicle;
        KSA.Program.ControlledVehicle = _controlled;
    }

    [Fact]
    public void Target_fixed_sampling_rejects_pending_and_same_id_replaced_targets()
    {
        var spec = new FrameSpec(FrameKind.TargetFixed, ParentId, TargetId);
        string? authoritative = FrameManager.SamplePose(
            _rails, _vessels, spec, default, Time, out var authoritativePose);

        Assert.Null(authoritative);
        Assert.Null(FrameCatalog.ValidatePose(authoritativePose));

        Tracked(TargetId).MarkReseedPending();

        string? pending = FrameManager.SamplePose(
            _rails, _vessels, spec, default, Time, out var pendingPose);

        Assert.Contains("waiting for a post-live reseed", pending);
        Assert.Equal(default, pendingPose);

        var replacement = NewVehicle(TargetId,
            new double3(12_000_000, 0, 0), new double3(0, 5_500, 0));
        Track(replacement, new Vector3d(12_000_000, 0, 0));

        string? replaced = FrameManager.SamplePose(
            _rails, _vessels, spec, default, Time, out var replacedPose);

        Assert.Contains("vehicle instance was replaced", replaced);
        Assert.Equal(default, replacedPose);
    }

    [Fact]
    public void Navigation_target_uses_bounded_live_truth_after_authority_is_revoked()
    {
        var original = new NavigationTarget
        {
            Body2Cci = doubleQuat.Identity,
            BodyRates = new double3(1, 2, 3),
            PositionCci = new double3(-1, -2, -3),
            VelocityCci = new double3(-4, -5, -6),
        };

        Assert.True(NavigationTargetPatch.TryCorrect(
            _vessels, _rails, _target, _parent, Time,
            in original, out var authoritative));
        AssertPredictorTargetState(authoritative);

        Tracked(TargetId).MarkReseedPending();
        Assert.True(NavigationTargetPatch.TryCorrect(
            _vessels, _rails, _target, _parent, Time,
            in original, out var pending));
        AssertLiveTargetState(pending);

        var replacement = NewVehicle(TargetId,
            new double3(20_000_000, 0, 0), new double3(0, 4_000, 0));
        Track(replacement, new Vector3d(20_000_000, 0, 0));

        Assert.True(NavigationTargetPatch.TryCorrect(
            _vessels, _rails, _target, _parent, Time,
            in original, out var replaced));
        AssertLiveTargetState(replaced);
    }

    public void Dispose()
    {
        KSA.Program.ControlledVehicle = _previousControlled;
        _rails.Dispose();
    }

    private Celestial NewParent()
    {
        var parent = (Celestial)RuntimeHelpers.GetUninitializedObject(typeof(PlanetaryBody));
        Set(AstronomicalId, parent, ParentId);
        Set(CelestialMass, parent, 3.3011e23);
        ParentCciToCce.SetValue(parent, doubleQuat.Identity);
        return parent;
    }

    private Vehicle NewVehicle(string id, double3 position, double3 velocity)
    {
        var vehicle = (Vehicle)RuntimeHelpers.GetUninitializedObject(typeof(Vehicle));
        Set(AstronomicalId, vehicle, id);
        VehicleProperties props = default;
        props.Situation = Situation.Freefall;
        VehicleProps.SetValue(vehicle, props);
        var orbit = Orbit.CreateFromStateCci(
            _parent, new SimTime(Time), position, velocity, default);
        Set(VehicleFlightPlan, vehicle, new FlightPlan(orbit, default));
        return vehicle;
    }

    private TrackedVessel Track(Vehicle vehicle, Vector3d relativePosition)
    {
        StateVector parent = _rails.GetAbsolute(ParentId, Time);
        var tracked = new TrackedVessel
        {
            Id = vehicle.Id,
            Rails = _rails,
            Options = new IntegratorOptions { RelTol = 1e-9 },
        };
        tracked.ReseedAbsolute(new StateVector(
            parent.Position + relativePosition,
            parent.Velocity + new Vector3d(0, 1_000, 0)), Time);
        tracked.BindVehicle(vehicle);
        tracked.LastParentId = ParentId;
        Entries()[vehicle.Id] = tracked;
        return tracked;
    }

    private void AssertLiveTargetState(NavigationTarget corrected)
    {
        Assert.Equal(10_000_000, corrected.PositionCci.X, 8);
        Assert.Equal(2_000_000, corrected.PositionCci.Y, 8);
        Assert.Equal(0, corrected.PositionCci.Z, 8);
        Assert.Equal(-500, corrected.VelocityCci.X, 8);
        Assert.Equal(6_000, corrected.VelocityCci.Y, 8);
        Assert.Equal(0, corrected.VelocityCci.Z, 8);
        Assert.Equal(1, corrected.BodyRates.X, 8);
        Assert.Equal(2, corrected.BodyRates.Y, 8);
        Assert.Equal(3, corrected.BodyRates.Z, 8);
    }

    private static void AssertPredictorTargetState(NavigationTarget corrected)
    {
        Assert.Equal(30_000_000, corrected.PositionCci.X, 8);
        Assert.Equal(-4_000_000, corrected.PositionCci.Y, 8);
        Assert.Equal(0, corrected.PositionCci.Z, 8);
        Assert.Equal(0, corrected.VelocityCci.X, 8);
        Assert.Equal(1_000, corrected.VelocityCci.Y, 8);
        Assert.Equal(0, corrected.VelocityCci.Z, 8);
    }

    private TrackedVessel Tracked(string id) => Entries()[id];

    private Dictionary<string, TrackedVessel> Entries() =>
        (Dictionary<string, TrackedVessel>)TrackedEntries.GetValue(_vessels)!;

    private static void SetTarget(Vehicle vehicle, IOrbiter target) =>
        Set(VehicleTarget, vehicle, target);

    private static void Set(PropertyInfo property, object target, object? value) =>
        property.SetValue(target, value);
}
