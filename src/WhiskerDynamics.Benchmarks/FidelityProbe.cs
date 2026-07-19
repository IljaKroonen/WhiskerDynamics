using System.Globalization;
using WhiskerDynamics.Compatibility;
using WhiskerDynamics.Core;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Benchmarks;

/// <summary>
/// Deterministic, game-process-free comparisons for the two approximations that are
/// otherwise hardest to observe: live physics against the rails vessel predictor, and
/// the planned finite-burn fold against continuously applied thrust.
/// </summary>
public static class FidelityProbe
{
    private const double StockOriginResnapMetres = 2_097.152;
    private const double ReferenceAgreementMetres = 0.01;
    private const double ReferenceAgreementMetresPerSecond = 1e-5;
    private const double PositionRoundoffFloorMetres = 1e-4;
    // Repeated parent-relative reconstruction from ~1e11 m barycentric states and
    // tens of thousands of fixed steps reaches this velocity roundoff scale in the
    // high-orbit 0.1 s sweep even while position continues to converge.
    private const double VelocityRoundoffFloorMetresPerSecond = 1e-7;
    // Exact production constructors: vessel rails uses the configured 1e-11 default;
    // planned display lines intentionally use their cheaper fixed 1e-9 tolerance.
    private static readonly IntegratorOptions RailsOptions = new() { RelTol = 1e-11 };
    private static readonly IntegratorOptions DisplayOptions = new() { RelTol = 1e-9 };
    private static readonly IntegratorOptions ReferenceOptions = new()
    {
        RelTol = 2e-13,
        AbsTolPos = 1e-7,
        AbsTolVel = 1e-10,
        InitialStep = 2.0,
        MaxStep = 30.0,
    };

    public static int Run()
    {
        var model = CreateModel(12_000.0);
        Console.WriteLine("WhiskerDynamics headless fidelity probe");
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"fixed-input/no-timing synthetic {model.Ephemerides.Bodies.Count}-body point-mass catalog; SI units; invariant formatting"));
        Console.WriteLine();

        bool physicsPass = RunPhysicsVsRails(model);
        bool burnPass = RunFiniteBurns(model);

        Console.WriteLine();
        Console.WriteLine($"verdict: {(physicsPass && burnPass ? "PASS" : "FAIL")}");
        Console.WriteLine("gates cover finiteness, reference agreement, invariants, and refinement only; practical errors are reported, not hidden behind an invented gameplay tolerance.");
        return physicsPass && burnPass ? 0 : 1;
    }

    private static bool RunPhysicsVsRails(Model model)
    {
        Console.WriteLine("PHYSICS <-> RAILS");
        Console.WriteLine("rails = production TrajectoryPredictor (DP5(4), reltol 1e-11); ref = DOP853 truth (reltol 5e-14), self-checked against 2e-13");
        Console.WriteLine("clean = parent-relative velocity-Verlet with the exact shared differential field");
        Console.WriteLine($"stock = IL-reconstructed verified KSA {GameBuildPolicy.VerifiedBuild} CCI vacuum translational surrogate: moving conic origin, carried central field, old-time second patch sample, and 2097.152 m resnaps");
        Console.WriteLine("scope excludes cache scheduling, J2/mascons/extended gravity, CCF rotation, contacts, drag, buoyancy, thrust, attitude/COM motion, clamps, and SOI changes");
        Console.WriteLine("h(s) is swept because 2 s is the default free-flight maximum, not every actual substep");
        Console.WriteLine();
        Console.WriteLine("scenario               T(s) h(s) refself:m/mps rails-ref:m/mps clean-rails:m/mps stock-rails:m/mps freshcentral:m nofield:m oldtime:m snaps");

        double mu = model.Parent.Mu;
        double leoRadius = 6.771e6;
        double highRadius = 1.0e8;
        var scenarios = new[]
        {
            new PhysicsScenario("leo-coast-3s", 3.0,
                CircularRelativeState(mu, leoRadius)),
            new PhysicsScenario("leo-live-30s", 30.0,
                CircularRelativeState(mu, leoRadius)),
            new PhysicsScenario("leo-forced-live-1h", 3_600.0,
                CircularRelativeState(mu, leoRadius)),
            new PhysicsScenario("high-forced-live-1h", 3_600.0,
                CircularRelativeState(mu, highRadius)),
        };
        double[] steps = [2.0, 0.5, 0.1];
        bool pass = true;

        foreach (var scenario in scenarios)
        {
            StateVector parent0 = model.Ephemerides.GetState(model.Parent, 0.0);
            StateVector absolute0 = parent0 + scenario.InitialRelative;
            var rails = new TrajectoryPredictor(model.CreateGravity(),
                absolute0, 0.0, RailsOptions);
            StateVector railsRelative = rails.StateAt(scenario.Duration)
                - model.Ephemerides.GetState(model.Parent, scenario.Duration);
            GravityModel referenceGravity = model.CreateGravity();
            StateVector referenceAbsolute = DormandPrince853.Propagate(
                (time, state) => referenceGravity.AccelerationAt(state.Position, time),
                absolute0, 0.0, scenario.Duration,
                out _, ReferenceOptions);
            GravityModel fineReferenceGravity = model.CreateGravity();
            StateVector fineReferenceAbsolute = DormandPrince853.Propagate(
                (time, state) => fineReferenceGravity.AccelerationAt(state.Position, time),
                absolute0, 0.0, scenario.Duration, out _,
                ReferenceOptions with
                {
                    RelTol = 5e-14,
                    InitialStep = 0.5,
                    MaxStep = 7.5,
                });
            StateVector referenceRelative = fineReferenceAbsolute
                - model.Ephemerides.GetState(model.Parent, scenario.Duration);
            StateError referenceFloor = Difference(referenceAbsolute, fineReferenceAbsolute);
            StateError railsFloor = Difference(railsRelative, referenceRelative);
            pass &= Finite(referenceFloor) && Finite(railsFloor)
                && referenceFloor.Position <= ReferenceAgreementMetres
                && referenceFloor.Velocity <= ReferenceAgreementMetresPerSecond
                && railsFloor.Position <= ReferenceAgreementMetres
                && railsFloor.Velocity <= ReferenceAgreementMetresPerSecond;

            StateError? coarsestClean = null;
            StateError? coarsestStock = null;
            foreach (double h in steps)
            {
                StateVector clean = IntegrateCleanRelative(model.CreateGravity(), model.Parent,
                    scenario.InitialRelative, scenario.Duration, h, includePerturbation: true);
                StateVector noField = IntegrateCleanRelative(model.CreateGravity(), model.Parent,
                    scenario.InitialRelative, scenario.Duration, h, includePerturbation: false);
                StateVector stockOld = IntegrateStockBuildSurrogate(model.CreateGravity(), model.Parent,
                    scenario.InitialRelative, scenario.Duration, h, secondSampleAtEndpoint: false,
                    freshCentralSamples: false, out int snaps);
                StateVector stockEndpoint = IntegrateStockBuildSurrogate(model.CreateGravity(), model.Parent,
                    scenario.InitialRelative, scenario.Duration, h, secondSampleAtEndpoint: true,
                    freshCentralSamples: false, out _);
                StateVector freshCentral = IntegrateStockBuildSurrogate(
                    model.CreateGravity(), model.Parent, scenario.InitialRelative,
                    scenario.Duration, h, secondSampleAtEndpoint: false,
                    freshCentralSamples: true, out _);

                StateError cleanError = Difference(clean, railsRelative);
                StateError stockError = Difference(stockOld, railsRelative);
                StateError freshCentralError = Difference(freshCentral, railsRelative);
                StateError noFieldError = Difference(noField, railsRelative);
                StateError oldTimeIsolation = Difference(stockOld, stockEndpoint);
                StateError cleanReferenceError = Difference(clean, referenceRelative);
                StateError stockReferenceError = Difference(stockOld, referenceRelative);
                pass &= Finite(cleanError) && Finite(stockError)
                    && Finite(freshCentralError) && Finite(noFieldError)
                    && Finite(oldTimeIsolation) && Finite(cleanReferenceError)
                    && Finite(stockReferenceError);

                Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"{scenario.Name,-22} {scenario.Duration,5:F0} {h,4:F1} "
                    + $"{Pair(referenceFloor),18} {Pair(railsFloor),18} "
                    + $"{Pair(cleanError),19} {Pair(stockError),19} "
                    + $"{freshCentralError.Position,14:E3} {noFieldError.Position,9:E3} "
                    + $"{oldTimeIsolation.Position,9:E3} {snaps,5}"));

                // Convergence is judged against DOP853, not the lower-order rails
                // subject. Compare the 2 s and 0.1 s endpoints; intermediate-step
                // cancellation cannot manufacture a pass. Only a measured DOP
                // self-difference plus a sub-millimetre coordinate-subtraction
                // allowance counts as a plateau.
                coarsestClean ??= cleanReferenceError;
                coarsestStock ??= stockReferenceError;
                if (h == steps[^1])
                {
                    pass &= ImprovesOrStaysAtMeasuredFloor(
                        coarsestClean.Value, cleanReferenceError, referenceFloor);
                    pass &= ImprovesOrStaysAtMeasuredFloor(
                        coarsestStock.Value, stockReferenceError, referenceFloor);
                }
            }
        }

        Console.WriteLine();
        Console.WriteLine("cache note: the live patch uses nonblocking TryVesselPerturbation; async hit/miss timing is deliberately excluded here. Production snapshot equality, 0.9 s interpolation, cold misses, and jerk-triggered refreshes remain gated in RailsServiceThirdBodyTests.");
        return pass;
    }

    private static bool RunFiniteBurns(Model model)
    {
        Console.WriteLine();
        Console.WriteLine("FINITE-BURN PLAN <-> CONTINUOUS THRUST");
        Console.WriteLine("subject = production planned-display path (DP5(4), reltol 1e-9) with OverlayKernel/FiniteBurnKernel midpoint impulses; oracle = independently timed, exactly segmented DOP853 continuous thrust");
        Console.WriteLine("directions are prescribed and fixed inertially in both paths, isolating thrust discretization from VLF guidance feedback");
        Console.WriteLine("single-burn discretization scope: shipping uses configured defaults; refined sensitivity uses 5 s and the global 1024-slice ceiling");
        Console.WriteLine();
        Console.WriteLine("scenario              T(s) Kship dtship Kfine dtfine order cutoff ship:m/mps fine:m/mps downstream ship:m/mps fine:m/mps impulse:m/mps qerr ship/fine:m oracle:m/mps dvmax:mps");

        StateVector parent0 = model.Ephemerides.GetState(model.Parent, 0.0);
        StateVector initial = parent0 + CircularRelativeState(model.Parent.Mu, 6.771e6);
        const double node = 3_000.0;
        GravityModel nodeGravity = model.CreateGravity();
        StateVector coastAtNode = DormandPrince853.Propagate(
            (time, state) => nodeGravity.AccelerationAt(state.Position, time),
            initial, 0.0, node, out _, ReferenceOptions);
        StateVector relativeAtNode = coastAtNode
            - model.Ephemerides.GetState(model.Parent, node);
        Vector3d radial = relativeAtNode.Position.Normalized();
        Vector3d normal = relativeAtNode.Position.Cross(relativeAtNode.Velocity).Normalized();
        Vector3d prograde = relativeAtNode.Velocity.Normalized();
        var shippingDefaults = new ModConfig();
        double shippingSliceSeconds = shippingDefaults.FiniteBurnSliceSeconds;
        int shippingMaxSlices = shippingDefaults.FiniteBurnMaxSlices;
        const double solverFineSliceSeconds = 5.0;
        int solverFineMaxSlices = ModConfig.MaxFiniteBurnMaxSlices;
        var cases = new[]
        {
            new BurnScenario("short-k1-mixed", 20.0,
                new EngineScalars(1_000.0, 3_000.0, 2.0),
                (radial + 2.0 * prograde + 0.5 * normal).Normalized()),
            new BurnScenario("main-300-prograde", 300.0,
                new EngineScalars(1_000.0, 3_000.0, 2.0), prograde),
            new BurnScenario("main-600-normal", 600.0,
                new EngineScalars(1_000.0, 3_000.0, 2.0), normal),
            new BurnScenario("low-thrust-cap", 1_200.0,
                new EngineScalars(1_000.0, 3_000.0, 0.2),
                (radial + prograde + 0.25 * normal).Normalized()),
        };
        bool pass = true;

        foreach (var scenario in cases)
        {
            double duration = StableDuration(scenario.DeltaV, scenario.Engine);
            double ignition = node - duration / 2.0;
            double cutoff = ignition + duration;
            double horizon = cutoff + 1_800.0;
            var oracle = PropagateContinuousBurn(model.CreateGravity(), initial, ignition, cutoff,
                horizon, scenario.Direction, scenario.Engine,
                coastMaxStep: 10.0, burnMaxStep: 1.0);
            var oracleFine = PropagateContinuousBurn(model.CreateGravity(), initial, ignition, cutoff,
                horizon, scenario.Direction, scenario.Engine,
                coastMaxStep: 2.5, burnMaxStep: 0.25);
            StateError oracleFloor = Max(Difference(oracle.Cutoff, oracleFine.Cutoff),
                Difference(oracle.Horizon, oracleFine.Horizon));

            BurnPath shipping = PropagateSlicedBurn(model.CreateGravity(), initial, node, horizon,
                scenario.Direction * scenario.DeltaV, scenario.Engine,
                shippingSliceSeconds, shippingMaxSlices, cutoff);
            BurnPath refined = PropagateSlicedBurn(model.CreateGravity(), initial, node, horizon,
                scenario.Direction * scenario.DeltaV, scenario.Engine,
                solverFineSliceSeconds, solverFineMaxSlices, cutoff);
            BurnPath impulsive = PropagateImpulsiveBurn(model.CreateGravity(), initial, node, horizon,
                scenario.Direction * scenario.DeltaV, cutoff);

            StateError shipCutoff = Difference(shipping.Cutoff, oracleFine.Cutoff);
            StateError fineCutoff = Difference(refined.Cutoff, oracleFine.Cutoff);
            StateError shipHorizon = Difference(shipping.Horizon, oracleFine.Horizon);
            StateError fineHorizon = Difference(refined.Horizon, oracleFine.Horizon);
            StateError impulseHorizon = Difference(impulsive.Horizon, oracleFine.Horizon);
            double shippingDvResidual = shipping.DeliveredDeltaV - scenario.DeltaV;
            double refinedDvResidual = refined.DeliveredDeltaV - scenario.DeltaV;
            double maxDvResidual = Math.Max(Math.Abs(shippingDvResidual),
                Math.Abs(refinedDvResidual));
            double order = double.NaN;
            if (refined.Slices > shipping.Slices)
                order = Math.Log(shipCutoff.Position / fineCutoff.Position)
                    / Math.Log((duration / shipping.Slices) / (duration / refined.Slices));
            StateVector parentAtHorizon = model.Ephemerides.GetState(model.Parent, horizon);
            double oraclePeriapsis = RendezvousKernel.PeriapsisDistance(
                oracleFine.Horizon - parentAtHorizon, model.Parent.Mu);
            double shippingPeriapsisError = Math.Abs(RendezvousKernel.PeriapsisDistance(
                shipping.Horizon - parentAtHorizon, model.Parent.Mu) - oraclePeriapsis);
            double refinedPeriapsisError = Math.Abs(RendezvousKernel.PeriapsisDistance(
                refined.Horizon - parentAtHorizon, model.Parent.Mu) - oraclePeriapsis);
            pass &= Finite(shipCutoff) && Finite(fineCutoff) && Finite(shipHorizon)
                && Finite(fineHorizon) && Finite(impulseHorizon) && Finite(oracleFloor)
                && oracleFloor.Position <= ReferenceAgreementMetres
                && oracleFloor.Velocity <= ReferenceAgreementMetresPerSecond
                && maxDvResidual <= 2e-12 * Math.Max(1.0, scenario.DeltaV)
                && double.IsFinite(shippingPeriapsisError)
                && double.IsFinite(refinedPeriapsisError)
                && shipping.Applied == 1 && refined.Applied == 1
                && shipping.ExpandedAsExpected && refined.ExpandedAsExpected;
            if (refined.Slices > shipping.Slices)
                pass &= fineHorizon.Position < shipHorizon.Position
                    && order is > 1.6 and < 2.4;

            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{scenario.Name,-21} {duration,6:F1} {shipping.Slices,5} {duration / shipping.Slices,6:F1} "
                + $"{refined.Slices,5} {duration / refined.Slices,6:F1} {order,5:F2} "
                + $"{Pair(shipCutoff),18} {Pair(fineCutoff),18} "
                + $"{Pair(shipHorizon),21} {Pair(fineHorizon),18} {Pair(impulseHorizon),18} "
                + $"{shippingPeriapsisError:E3}/{refinedPeriapsisError:E3} "
                + $"{Pair(oracleFloor),18} {maxDvResidual,10:E3}"));
        }

        AnalyticResult analytic = AnalyticZeroGravityCheck();
        pass &= analytic.Finite && analytic.ErrorRatio is > 3.5 and < 4.5
            && analytic.OraclePositionError <= 1e-6
            && analytic.OracleVelocityError <= 1e-9;
        Console.WriteLine();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"analytic zero-g midpoint check: K=8 error {analytic.CoarseError:E6} m, "
            + $"K=16 error {analytic.FineError:E6} m, ratio {analytic.ErrorRatio:F4} (second-order target ~4)"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"analytic zero-g continuous-oracle check: position error {analytic.OraclePositionError:E6} m, "
            + $"velocity error {analytic.OracleVelocityError:E6} m/s"));
        Console.WriteLine("behavior note: K=1 is reported but excluded from convergence gating; overlapping/boundary-clipped burns are policy fallbacks, not continuous-thrust accuracy samples.");
        return pass;
    }

    private static Model CreateModel(double horizon)
    {
        IReadOnlyList<CelestialBody> bodies = BenchmarkCatalog.CreateBodies();
        var ephemerides = new NBodyEphemerides(bodies, 0.0,
            BenchmarkCatalog.BackboneIds,
            new IntegratorOptions { RelTol = BenchmarkCatalog.ShippingRelTol });
        CelestialBody earth = ephemerides["Earth"];
        _ = ephemerides.GetState(earth, horizon + 86_400.0);
        // Vessel gravity includes every finite-mass modeled source, including
        // restricted tracks that do not back-react on the celestial backbone.
        CelestialBody[] sources = ephemerides.Bodies
            .Where(body => body.Mu > 0.0 && double.IsFinite(body.Mu))
            .ToArray();
        return new Model(ephemerides, earth, sources);
    }

    private static StateVector CircularRelativeState(double mu, double radius) =>
        new(new Vector3d(radius, 0, 0), new Vector3d(0, Math.Sqrt(mu / radius), 0));

    private static StateVector IntegrateCleanRelative(GravityModel gravity, CelestialBody parent,
        StateVector initial, double duration, double step, bool includePerturbation)
    {
        double t = 0.0;
        StateVector state = initial;
        while (t < duration)
        {
            double h = Math.Min(step, duration - t);
            Vector3d a0 = Central(parent.Mu, state.Position)
                + (includePerturbation
                    ? gravity.ThirdBodyDeltaAt(parent, state.Position, t)
                    : Vector3d.Zero);
            Vector3d position = state.Position + state.Velocity * h + a0 * (0.5 * h * h);
            Vector3d a1 = Central(parent.Mu, position)
                + (includePerturbation
                    ? gravity.ThirdBodyDeltaAt(parent, position, t + h)
                    : Vector3d.Zero);
            Vector3d velocity = state.Velocity + (a0 + a1) * (0.5 * h);
            state = new StateVector(position, velocity);
            t += h;
        }
        return state;
    }

    private static StateVector IntegrateStockBuildSurrogate(GravityModel gravity,
        CelestialBody parent, StateVector initial, double duration, double step,
        bool secondSampleAtEndpoint, bool freshCentralSamples, out int resnaps)
    {
        double t = 0.0;
        double seedTime = 0.0;
        StateVector seed = initial;
        StateVector origin = initial;
        Vector3d offsetPosition = Vector3d.Zero;
        Vector3d offsetVelocity = Vector3d.Zero;
        Vector3d carriedCentral = Central(parent.Mu, initial.Position);
        resnaps = 0;

        while (t < duration)
        {
            double h = Math.Min(step, duration - t);
            StateVector nextOrigin = Kepler.PropagateUniversal(seed, parent.Mu,
                t + h - seedTime);
            Vector3d oldVesselPosition = origin.Position + offsetPosition;
            Vector3d perturbation0 = gravity.ThirdBodyDeltaAt(parent, oldVesselPosition, t);
            Vector3d firstVesselCentral = freshCentralSamples
                ? Central(parent.Mu, oldVesselPosition)
                : carriedCentral;
            Vector3d a0 = firstVesselCentral
                - Central(parent.Mu, origin.Position) + perturbation0;

            // Verified build ordering: environment recompute precedes translational drift.
            Vector3d nextCarriedCentral = Central(parent.Mu, oldVesselPosition);
            Vector3d nextOffsetPosition = offsetPosition
                + (offsetVelocity + a0 * (0.5 * h)) * h;
            Vector3d eulerVelocity = offsetVelocity + a0 * h;
            Vector3d patchPosition = nextOrigin.Position + nextOffsetPosition;
            double secondTime = secondSampleAtEndpoint ? t + h : t;
            Vector3d perturbation1 = gravity.ThirdBodyDeltaAt(parent, patchPosition, secondTime);
            Vector3d secondVesselCentral = freshCentralSamples
                ? Central(parent.Mu, patchPosition)
                : nextCarriedCentral;
            Vector3d a1 = secondVesselCentral - Central(parent.Mu, nextOrigin.Position)
                + perturbation1;
            Vector3d nextOffsetVelocity = eulerVelocity + (a1 - a0) * (0.5 * h);

            t += h;
            origin = nextOrigin;
            offsetPosition = nextOffsetPosition;
            offsetVelocity = nextOffsetVelocity;
            carriedCentral = nextCarriedCentral;

            if (offsetPosition.Length() > StockOriginResnapMetres)
            {
                StateVector vessel = new(origin.Position + offsetPosition,
                    origin.Velocity + offsetVelocity);
                seedTime = t;
                seed = vessel;
                origin = vessel;
                offsetPosition = Vector3d.Zero;
                offsetVelocity = Vector3d.Zero;
                // SnapOrigin does not recompute the carried environment field.
                resnaps++;
            }
        }
        return new StateVector(origin.Position + offsetPosition,
            origin.Velocity + offsetVelocity);
    }

    private static BurnPath PropagateSlicedBurn(GravityModel gravity, StateVector initial,
        double node, double horizon, Vector3d deltaV, EngineScalars engine,
        double sliceSeconds, int maxSlices, double cutoff)
    {
        var predictor = new TrajectoryPredictor(gravity, initial, 0.0, DisplayOptions);
        int applied = OverlayKernel.FoldBurns(predictor, [node], 0.0, horizon,
            _ => deltaV, _ => { }, new FiniteBurnFold(engine, sliceSeconds, maxSlices),
            out double earliestStart);
        StateVector cutoffState = predictor.StateAt(cutoff);
        StateVector horizonState = predictor.StateAt(horizon);
        FiniteBurnExpansion? expansion = FiniteBurnKernel.Expand(node, deltaV.Length(),
            engine, sliceSeconds, maxSlices);
        int slices = expansion?.Times.Length ?? 1;
        double delivered = expansion?.Magnitudes.Sum() ?? deltaV.Length();
        double expectedStart = expansion?.IgnitionSeconds ?? node;
        bool expandedAsExpected = Math.Abs(earliestStart - expectedStart) <= 1e-9;
        return new BurnPath(cutoffState, horizonState, slices, delivered, applied,
            expandedAsExpected);
    }

    private static BurnPath PropagateImpulsiveBurn(GravityModel gravity, StateVector initial,
        double node, double horizon, Vector3d deltaV, double cutoff)
    {
        var predictor = new TrajectoryPredictor(gravity, initial, 0.0, DisplayOptions);
        predictor.AddImpulse(node, deltaV);
        return new BurnPath(predictor.StateAt(cutoff), predictor.StateAt(horizon),
            1, deltaV.Length(), 1, ExpandedAsExpected: true);
    }

    private static ContinuousPath PropagateContinuousBurn(GravityModel gravity,
        StateVector initial, double ignition, double cutoff, double horizon,
        Vector3d direction, EngineScalars engine, double coastMaxStep, double burnMaxStep)
        => PropagateContinuousBurn(
            (time, state) => gravity.AccelerationAt(state.Position, time),
            initial, ignition, cutoff, horizon, direction, engine,
            coastMaxStep, burnMaxStep);

    private static ContinuousPath PropagateContinuousBurn(
        Func<double, StateVector, Vector3d> gravityAcceleration,
        StateVector initial, double ignition, double cutoff, double horizon,
        Vector3d direction, EngineScalars engine, double coastMaxStep, double burnMaxStep)
    {
        var coastOptions = ReferenceOptions with
        {
            InitialStep = Math.Min(2.0, coastMaxStep),
            MaxStep = coastMaxStep,
        };
        StateVector atIgnition = DormandPrince853.Propagate(
            gravityAcceleration,
            initial, 0.0, ignition, out _, coastOptions);
        var burnOptions = ReferenceOptions with
        {
            InitialStep = Math.Min(0.25, burnMaxStep),
            MaxStep = burnMaxStep,
        };
        Vector3d BurnAcceleration(double time, StateVector state)
        {
            double mass = engine.MassKg - engine.MassFlowRate * (time - ignition);
            double thrustAcceleration = engine.ExhaustVelocity * engine.MassFlowRate / mass;
            return gravityAcceleration(time, state) + direction * thrustAcceleration;
        }
        StateVector atCutoff = DormandPrince853.Propagate(BurnAcceleration,
            atIgnition, ignition, cutoff, out _, burnOptions);
        StateVector atHorizon = DormandPrince853.Propagate(
            gravityAcceleration,
            atCutoff, cutoff, horizon, out _, coastOptions);
        return new ContinuousPath(atCutoff, atHorizon);
    }

    private static AnalyticResult AnalyticZeroGravityCheck()
    {
        const double dv = 300.0;
        var engine = new EngineScalars(1_000.0, 3_000.0, 2.0);
        double duration = StableDuration(dv, engine);
        double finalMass = engine.MassKg - engine.MassFlowRate * duration;
        double logRatio = Math.Log(engine.MassKg / finalMass);
        double exact = engine.ExhaustVelocity / engine.MassFlowRate
            * (engine.MassKg - finalMass * (1.0 + logRatio));
        double ErrorFor(int slices)
        {
            double sliceSetting = duration / (slices - 0.25);
            FiniteBurnExpansion expansion = FiniteBurnKernel.Expand(0.0, dv, engine,
                sliceSetting, slices) ?? throw new InvalidOperationException("analytic expansion collapsed");
            double cutoff = expansion.IgnitionSeconds + expansion.DurationSeconds;
            double displacement = 0.0;
            for (int i = 0; i < expansion.Times.Length; i++)
                displacement += (cutoff - expansion.Times[i]) * expansion.Magnitudes[i];
            return Math.Abs(displacement - exact);
        }
        double coarse = ErrorFor(8);
        double fine = ErrorFor(16);
        ContinuousPath continuous = PropagateContinuousBurn(
            (_, _) => Vector3d.Zero,
            new StateVector(Vector3d.Zero, Vector3d.Zero),
            ignition: 0.0, cutoff: duration, horizon: duration,
            direction: new Vector3d(1, 0, 0), engine,
            coastMaxStep: 1.0, burnMaxStep: 0.25);
        double oraclePositionError = (continuous.Cutoff.Position
            - new Vector3d(exact, 0, 0)).Length();
        double oracleVelocityError = (continuous.Cutoff.Velocity
            - new Vector3d(dv, 0, 0)).Length();
        return new AnalyticResult(coarse, fine, coarse / fine,
            oraclePositionError, oracleVelocityError,
            double.IsFinite(coarse) && double.IsFinite(fine) && fine > 0.0
                && double.IsFinite(oraclePositionError)
                && double.IsFinite(oracleVelocityError));
    }

    private static double StableDuration(double deltaV, EngineScalars engine)
    {
        double x = -deltaV / engine.ExhaustVelocity;
        double expm1 = Math.Abs(x) < 1e-5
            ? x * (1.0 + x * (0.5 + x * (1.0 / 6.0 + x * (1.0 / 24.0 + x / 120.0))))
            : Math.Exp(x) - 1.0;
        return -engine.MassKg * expm1 / engine.MassFlowRate;
    }

    private static Vector3d Central(double mu, Vector3d position)
    {
        double r2 = position.LengthSquared();
        return position * (-mu / (r2 * Math.Sqrt(r2)));
    }

    private static StateError Difference(StateVector actual, StateVector expected) =>
        new((actual.Position - expected.Position).Length(),
            (actual.Velocity - expected.Velocity).Length());

    private static StateError Max(StateError a, StateError b) =>
        new(Math.Max(a.Position, b.Position), Math.Max(a.Velocity, b.Velocity));

    private static bool ImprovesOrStaysAtMeasuredFloor(StateError coarse, StateError fine,
        StateError referenceFloor) =>
        ImprovesOrStaysAtMeasuredFloor(coarse.Position, fine.Position,
            referenceFloor.Position, PositionRoundoffFloorMetres)
        && ImprovesOrStaysAtMeasuredFloor(coarse.Velocity, fine.Velocity,
            referenceFloor.Velocity, VelocityRoundoffFloorMetresPerSecond);

    private static bool ImprovesOrStaysAtMeasuredFloor(double coarse, double fine,
        double referenceFloor, double roundoffFloor)
    {
        double plateau = Math.Max(20.0 * referenceFloor, roundoffFloor);
        return coarse <= plateau ? fine <= 2.0 * plateau : fine < 0.75 * coarse;
    }

    private static bool Finite(StateError error) =>
        double.IsFinite(error.Position) && double.IsFinite(error.Velocity);

    private static string Pair(StateError error) => string.Create(CultureInfo.InvariantCulture,
        $"{error.Position:E3}/{error.Velocity:E3}");

    private sealed record Model(NBodyEphemerides Ephemerides, CelestialBody Parent,
        CelestialBody[] Sources)
    {
        public GravityModel CreateGravity() => new(Ephemerides, Sources);
    }
    private sealed record PhysicsScenario(string Name, double Duration,
        StateVector InitialRelative);
    private sealed record BurnScenario(string Name, double DeltaV, EngineScalars Engine,
        Vector3d Direction);
    private readonly record struct StateError(double Position, double Velocity);
    private readonly record struct BurnPath(StateVector Cutoff, StateVector Horizon,
        int Slices, double DeliveredDeltaV, int Applied, bool ExpandedAsExpected);
    private readonly record struct ContinuousPath(StateVector Cutoff, StateVector Horizon);
    private readonly record struct AnalyticResult(double CoarseError, double FineError,
        double ErrorRatio, double OraclePositionError, double OracleVelocityError,
        bool Finite);
}
