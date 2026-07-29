using System.Diagnostics;
using System.Text.Json;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;
using WhiskerDynamics.GameTesting;
using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.GameTestDriver.Runtime;

[HarmonyPatch(typeof(Program), "OnDrawUiConsole")]
internal static class GameTestScenarioPatch
{
    private static readonly Stopwatch WallClock = new();
    private static GameTestScenario? _scenario;
    private static readonly List<GameTestStepResult> Results = [];
    private static int _stepIndex;
    private static long _stepStartedMs;
    private static int _readyFrames;
    private static bool _stepIssued;
    private static int _baselineBurnCount;
    private static int _burnExecutionStage;
    private static int _burnExecutionBaselineCount;
    private static int _burnsExecuted;
    private static float _burnTargetMagnitude;
    private static bool _burnExecutionWarpEngaged;
    private static GameTestPlayerLunarTransferSolveJob? _playerLunarTransferJob;
    private static GameTestPlayerLunarCorrectionSolveJob? _playerLunarCorrectionJob;
    private static bool _playerLunarCorrectionQueued;
    private static FlightPlanModel? _playerLunarPlan;
    private static GameTestLunarPeriluneProbeJob? _lunarPeriluneProbeJob;
    private static GameTestPlayerLunarCircularizationSolveJob?
        _playerLunarCircularizationJob;
    private static bool _earthSoiCircularizationReproArmed;
    private static bool _earthSoiCircularizationNodeLossLogged;
    private static int _automaticBurnTargetsLostAtSoi;
    private static bool _finished;

    static void Postfix()
    {
        if (_finished || !ModServices.Enabled) return;
        try
        {
            if (_scenario is null)
            {
                string requestPath = Path.Combine(ModMain.ModDir, GameTestProtocol.RequestFileName);
                if (!File.Exists(requestPath)) return;
                _scenario = JsonSerializer.Deserialize<GameTestScenario>(
                    File.ReadAllText(requestPath), GameTestProtocol.JsonOptions)
                    ?? throw new InvalidDataException("scenario request was empty");
                ValidateScenario(_scenario);
                WallClock.Restart();
                _stepStartedMs = Environment.TickCount64;
                ModLog.Info($"game test: starting '{_scenario.Name}' run {_scenario.RunId}");
            }

            if (WallClock.Elapsed.TotalSeconds > _scenario.TimeoutSeconds)
            {
                Fail($"scenario timed out after {_scenario.TimeoutSeconds:F1} wall seconds");
                return;
            }

            if (!Ready()) return;
            if (_stepIndex >= _scenario.Steps.Count)
            {
                Finish(passed: true, error: null);
                return;
            }

            GameTestStep step = _scenario.Steps[_stepIndex];
            double timeout = step.TimeoutSeconds ?? 60.0;
            if (Environment.TickCount64 - _stepStartedMs > timeout * 1000.0)
            {
                Fail($"step {_stepIndex} '{step.Action}' timed out after {timeout:F1} wall seconds");
                return;
            }

            Execute(step);
        }
        catch (Exception e)
        {
            Fail(e.ToString());
        }
    }

    private static bool Ready()
    {
        if (!ModServices.EnsureBound()
            || ModServices.Status != ModStatus.Active
            || Program.ControlledVehicle is not { } controlled
            || ModServices.Vessels?.TryGetLiveVehicle(controlled.Id) is null
            || !ModServices.Vessels.TryCaptureRailsAuthority(
                controlled, out _, out _))
        {
            _readyFrames = 0;
            return false;
        }

        // Allow startup binding to commit.
        return ++_readyFrames >= 2;
    }

    private static void Execute(GameTestStep step)
    {
        switch (step.Action.Trim().ToLowerInvariant())
        {
            case "wait-ready":
                Pass(step, $"mod active; controlled vessel '{Program.ControlledVehicle!.Id}'");
                break;

            case "plan-and-execute-lunar-transfer":
                PlanAndExecuteLunarTransfer(step);
                break;

            case "assert-perilune-altitude-between":
                AssertPeriluneAltitudeBetween(step);
                break;

            case "plan-lunar-circularization-from-earth-soi":
                PlanLunarCircularizationFromEarthSoi(step);
                break;

            case "execute-burns":
                ExecuteBurns(step);
                break;

            case "assert-bad-lunar-circularization":
                AssertBadLunarCircularization(step);
                break;

            case "assert-parent":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                string expected = Required(step.Expected, "expected");
                string actual = (vessel.Orbit.Parent as Astronomical)?.Id ?? "<none>";
                Require(actual == expected,
                    $"expected parent '{expected}', got '{actual}'");
                Pass(step, $"'{vessel.Id}' parent is '{actual}'");
                break;
            }

            default:
                throw new InvalidDataException($"unknown action '{step.Action}'");
        }
    }
    private static void PlanAndExecuteLunarTransfer(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        VesselRegistry registry = ModServices.Vessels
            ?? throw new InvalidOperationException("vessel registry is unavailable");
        double offset = Required(step.OffsetSeconds, "offsetSeconds");
        double duration = Required(step.DurationSeconds, "flightDurationSeconds");
        double targetAltitude = Required(step.TargetRadiusMeters,
            "targetPeriluneAltitudeMeters");
        double progradeDeltaV = Required(step.DeltaVMetersPerSecond,
            "deltaVMetersPerSecond");
        int departureOrbitOffset = step.OrbitOffset ?? 0;
        Require(offset >= PlannerKernel.MinLeadSeconds && duration > 0,
            "lunar transfer needs a future departure and positive prediction duration");
        Require(targetAltitude >= 0,
            "target perilune altitude must be non-negative");
        Require(departureOrbitOffset >= 0,
            "departure orbit offset must be non-negative");

        if (_playerLunarTransferJob is null)
        {
            Require(BurnPlanWriter.Snapshot(vessel).Count == 0,
                "player-style lunar transfer requires an empty maneuver plan");
            Require(FlightPlans.TryGet(vessel.Id) is null,
                "player-style lunar transfer requires no existing mod flight plan");
            RailsService rails = ModServices.Rails
                ?? throw new InvalidOperationException("rails service is unavailable");
            if (!registry.TryCaptureRailsAuthority(
                    vessel, out var authority, out var authorityReason))
                throw new InvalidOperationException(
                    $"'{vessel.Id}' transfer predictor is unavailable: "
                    + PredictorAuthorityPolicy.Describe(authorityReason));
            double now = Universe.GetElapsedSimTime().Seconds();
            if (!authority.Tracked.TryCaptureSolverSeed(
                    authority.Lineage, now, out StateVector seedState))
                return;
            StateVector earthRelative = seedState - rails.GetAbsolute("Earth", now);
            double parkingPeriod = RendezvousKernel.OrbitalPeriod(
                earthRelative, rails.MuOf("Earth"));
            Require(parkingPeriod > 0 && double.IsFinite(parkingPeriod),
                "default rocket is not in a finite Earth parking orbit");
            double departureStart = now + offset
                + departureOrbitOffset * parkingPeriod;
            double predictionEnd = departureStart + parkingPeriod + duration;
            RailsService.PredictionContext? prediction =
                rails.TryCaptureSolverPredictionContext(now, predictionEnd);
            if (prediction is null)
                return;
            _baselineBurnCount = 0;
            _playerLunarTransferJob = new GameTestPlayerLunarTransferSolveJob
            {
                Prediction = prediction,
                CoastSeedState = seedState,
                CoastSeedTime = now,
                DepartureStartTime = departureStart,
                DepartureSearchDuration = parkingPeriod,
                FlightDuration = duration,
                ProgradeDeltaV = progradeDeltaV,
                DesiredPeriluneRadiusMeters = rails.MeanRadiusOf("Luna")
                    + targetAltitude,
                LunaRadiusMeters = rails.MeanRadiusOf("Luna"),
            };
            _playerLunarTransferJob.Start();
            return;
        }
        if (!_playerLunarTransferJob.Done)
            return;
        if (_playerLunarTransferJob.Result is not { } solution)
            throw new InvalidOperationException(
                _playerLunarTransferJob.Failure
                ?? "player-style lunar transfer produced no result");

        if (!_stepIssued)
        {
            string verdict = BurnPlannerPanel.PlanBurnForGameTest(
                registry, vessel, solution.DepartureTime,
                frame: null, components: solution.DeltaVVlf);
            Require(verdict.StartsWith("queued", StringComparison.Ordinal),
                $"lunar transfer burn was not queued: {verdict}");
            _playerLunarPlan = FlightPlans.TryGet(vessel.Id)
                ?? throw new InvalidOperationException(
                    "planner queued TLI without creating a mod flight plan");
            _stepIssued = true;
            ModLog.Info($"game test: player-style TLI queued at "
                + $"t={solution.DepartureTime:F1} s, "
                + $"dv=(3150, 0, 0) m/s, predicted perilune altitude "
                + $"{(solution.PeriluneRadiusMeters
                    - ModServices.Rails!.MeanRadiusOf("Luna")) / 1000:F1} km");
            return;
        }
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vessel);
        if (_burnsExecuted == 0)
        {
            Require(ReferenceEquals(FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
                "the mod flight plan was removed or replaced during TLI");
            if (_burnExecutionStage == 0 && burns.Count <= _baselineBurnCount)
                return;
            ExecuteBurns(step);
            return;
        }

        double correctionDelay = Required(step.CorrectionTimescaleSeconds,
            "correctionTimescaleSeconds");
        double correctionTime = solution.DepartureTime + correctionDelay;
        if (_playerLunarCorrectionJob is null)
        {
            Require(burns.Count == 0,
                "lunar correction requires the departure node to be complete");
            double now = Universe.GetElapsedSimTime().Seconds();
            Require(correctionTime >= now + PlannerKernel.MinLeadSeconds,
                $"correction t={correctionTime:F1} is too close or already past "
                + $"at t={now:F1}");
            RailsService rails = ModServices.Rails
                ?? throw new InvalidOperationException("rails service is unavailable");
            if (!registry.TryCaptureRailsAuthority(
                    vessel, out var authority, out var authorityReason))
                throw new InvalidOperationException(
                    $"'{vessel.Id}' correction predictor is unavailable: "
                    + PredictorAuthorityPolicy.Describe(authorityReason));
            if (!authority.Tracked.TryCaptureSolverSeed(
                    authority.Lineage, now, out StateVector seedState))
                return;
            double predictionEnd = Math.Min(rails.Horizon,
                correctionTime + duration);
            RailsService.PredictionContext? prediction =
                rails.TryCaptureSolverPredictionContext(now, predictionEnd);
            if (prediction is null)
                return;
            _playerLunarCorrectionJob =
                new GameTestPlayerLunarCorrectionSolveJob
                {
                    Prediction = prediction,
                    SeedState = seedState,
                    StartTime = now,
                    BurnTime = correctionTime,
                    EndTime = predictionEnd,
                    DesiredPeriluneRadiusMeters = rails.MeanRadiusOf("Luna")
                        + targetAltitude,
                    LunaRadiusMeters = rails.MeanRadiusOf("Luna"),
                };
            _playerLunarCorrectionJob.Start();
            return;
        }
        if (!_playerLunarCorrectionJob.Done)
            return;
        if (_playerLunarCorrectionJob.Result is not { } correction)
            throw new InvalidOperationException(
                _playerLunarCorrectionJob.Failure
                ?? "player-style lunar correction produced no result");
        if (!_playerLunarCorrectionQueued)
        {
            Require(ReferenceEquals(FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
                "the mod flight plan was removed or replaced after TLI");
            string rebase = BurnPlannerPanel.RebasePlanForGameTest(
                registry, vessel);
            Require(rebase.StartsWith("plan rebased", StringComparison.Ordinal),
                $"could not rebase the player flight plan after TLI: {rebase}");
            string verdict = BurnPlannerPanel.PlanBurnForGameTest(
                registry, vessel, correction.BurnTime,
                frame: null, components: correction.DeltaVVlf);
            Require(verdict.StartsWith("queued", StringComparison.Ordinal),
                $"lunar correction burn was not queued: {verdict}");
            _playerLunarCorrectionQueued = true;
            ModLog.Info($"game test: queued one-hour lunar correction "
                + $"dvVlf=({correction.DeltaVVlf.X:F3}, "
                + $"{correction.DeltaVVlf.Y:F3}, "
                + $"{correction.DeltaVVlf.Z:F3}) m/s, predicted perilune "
                + $"{(correction.PeriluneRadiusMeters
                    - ModServices.Rails!.MeanRadiusOf("Luna")) / 1000:F1} km");
            return;
        }
        burns = BurnPlanWriter.Snapshot(vessel);
        if (_burnsExecuted == 1)
        {
            Require(ReferenceEquals(FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
                "the mod flight plan was removed or replaced during correction");
            if (_burnExecutionStage == 0 && burns.Count == 0)
                return;
            ExecuteBurns(step);
            return;
        }
        Pass(step, $"executed 3150 m/s prograde TLI and "
            + $"{correction.DeltaVVlf.Length():F3} m/s correction at departure +1 h");
    }

    private static void AssertPeriluneAltitudeBetween(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double minimum = Required(step.MinPeriluneAltitudeMeters,
            "minimumPeriluneAltitudeMeters");
        double maximum = Required(step.MaxPeriluneAltitudeMeters,
            "maximumPeriluneAltitudeMeters");
        Require(minimum >= 0 && maximum >= minimum,
            "perilune altitude range is invalid");
        RailsService rails = ModServices.Rails
            ?? throw new InvalidOperationException("rails service is unavailable");
        if (_lunarPeriluneProbeJob is null)
        {
            VesselRegistry registry = ModServices.Vessels
                ?? throw new InvalidOperationException("vessel registry is unavailable");
            if (!registry.TryCaptureRailsAuthority(
                    vessel, out var authority, out var authorityReason))
                throw new InvalidOperationException(
                    $"'{vessel.Id}' perilune predictor is unavailable: "
                    + PredictorAuthorityPolicy.Describe(authorityReason));
            double now = Universe.GetElapsedSimTime().Seconds();
            if (!authority.Tracked.TryCaptureSolverSeed(
                    authority.Lineage, now, out StateVector seedState))
                return;
            double end = Math.Min(rails.Horizon, now + 5 * 86400);
            RailsService.PredictionContext? prediction =
                rails.TryCaptureSolverPredictionContext(now, end);
            if (prediction is null)
                return;
            _lunarPeriluneProbeJob = new GameTestLunarPeriluneProbeJob
            {
                Prediction = prediction,
                SeedState = seedState,
                StartTime = now,
                EndTime = end,
                LunaRadiusMeters = rails.MeanRadiusOf("Luna"),
            };
            _lunarPeriluneProbeJob.Start();
            return;
        }
        if (!_lunarPeriluneProbeJob.Done)
            return;
        if (_lunarPeriluneProbeJob.PeriluneRadiusMeters is not { } radius)
            throw new InvalidOperationException(
                _lunarPeriluneProbeJob.Failure
                ?? "lunar perilune probe produced no result");
        double altitude = radius - rails.MeanRadiusOf("Luna");
        Require(altitude >= minimum && altitude <= maximum,
            $"predicted lunar perilune altitude {altitude:R} m is outside "
            + $"[{minimum:R}, {maximum:R}] m");
        Pass(step, $"predicted lunar perilune altitude "
            + $"{altitude / 1000:F1} km");
    }

    private static void PlanLunarCircularizationFromEarthSoi(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        Require((vessel.Orbit.Parent as Astronomical)?.Id == "Earth",
            "lunar circularization repro must be planned while Earth owns the vessel SOI");
        RailsService rails = ModServices.Rails
            ?? throw new InvalidOperationException("rails service is unavailable");
        VesselRegistry registry = ModServices.Vessels
            ?? throw new InvalidOperationException("vessel registry is unavailable");
        if (_playerLunarCircularizationJob is null)
        {
            Require(BurnPlanWriter.Snapshot(vessel).Count == 0,
                "Earth-SOI circularization planning requires an empty maneuver plan");
            if (!registry.TryCaptureRailsAuthority(
                    vessel, out var authority, out var authorityReason))
                throw new InvalidOperationException(
                    $"'{vessel.Id}' Earth-SOI circularization predictor is unavailable: "
                    + PredictorAuthorityPolicy.Describe(authorityReason));
            double now = Universe.GetElapsedSimTime().Seconds();
            if (!authority.Tracked.TryCaptureSolverSeed(
                    authority.Lineage, now, out StateVector seedState))
                return;
            double end = Math.Min(rails.Horizon, now + 5 * 86400);
            RailsService.PredictionContext? prediction =
                rails.TryCaptureSolverPredictionContext(now, end);
            if (prediction is null)
                return;
            _baselineBurnCount = 0;
            _playerLunarCircularizationJob =
                new GameTestPlayerLunarCircularizationSolveJob
                {
                    Prediction = prediction,
                    SeedState = seedState,
                    StartTime = now,
                    EndTime = end,
                    LunaMu = rails.MuOf("Luna"),
                    LunaRadiusMeters = rails.MeanRadiusOf("Luna"),
                };
            _playerLunarCircularizationJob.Start();
            return;
        }
        if (!_playerLunarCircularizationJob.Done)
            return;
        if (_playerLunarCircularizationJob.Result is not { } solution)
            throw new InvalidOperationException(
                _playerLunarCircularizationJob.Failure
                ?? "Earth-SOI lunar circularization produced no result");
        double nowAfterSolve = Universe.GetElapsedSimTime().Seconds();
        Require(solution.BurnTime >= nowAfterSolve + PlannerKernel.MinLeadSeconds,
            $"predicted lunar circularization t={solution.BurnTime:F1} is too close "
            + $"or already past at t={nowAfterSolve:F1}");
        if (!_stepIssued)
        {
            var frame = new FrameSpec(FrameKind.Inertial, "Luna", null);
            if (FlightPlans.TryGet(vessel.Id) is { Diverged: true })
            {
                string rebase = BurnPlannerPanel.RebasePlanForGameTest(
                    registry, vessel);
                Require(rebase.StartsWith("plan rebased", StringComparison.Ordinal),
                    $"could not rebase before circularization: {rebase}");
            }
            string verdict = BurnPlannerPanel.PlanBurnForGameTest(
                registry, vessel, solution.BurnTime,
                frame, solution.LunaFrameDeltaVPrn);
            Require(verdict.StartsWith("queued", StringComparison.Ordinal),
                $"Earth-SOI lunar circularization was not queued: {verdict}");
            if (_playerLunarPlan is not null)
                Require(ReferenceEquals(
                        FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
                    "circularization replaced the lunar-transfer flight plan");
            _earthSoiCircularizationReproArmed = true;
            _earthSoiCircularizationNodeLossLogged = false;
            _stepIssued = true;
            ModLog.Info($"game test: queued player-style Luna-frame circularization "
                + $"from Earth SOI at t={solution.BurnTime:F1}: authored PRN=("
                + $"{solution.LunaFrameDeltaVPrn.X:F3}, "
                + $"{solution.LunaFrameDeltaVPrn.Y:F3}, "
                + $"{solution.LunaFrameDeltaVPrn.Z:F3}) m/s");
            return;
        }
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vessel);
        if (burns.Count <= _baselineBurnCount)
            return;
        Pass(step, $"planned {solution.LunaFrameDeltaVPrn.Length():F2} m/s "
            + $"Luna-frame circularization at predicted perilune "
            + $"{solution.PeriluneRadiusMeters / 1000:F1} km radius "
            + "while still in Earth SOI");
    }

    private static void AssertBadLunarCircularization(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        Require((vessel.Orbit.Parent as Astronomical)?.Id == "Luna",
            "circularization repro did not reach Luna SOI");
        double minimumEccentricity = Required(step.MinEccentricity,
            "minimumEccentricity");
        Require(minimumEccentricity > 0 && minimumEccentricity < 1,
            "minimum bad-orbit eccentricity must be in (0, 1)");
        StateVector state = VesselRelativeToBody(vessel, "Luna");
        double mu = ModServices.Rails!.MuOf("Luna");
        double radius = state.Position.Length();
        double speedSquared = state.Velocity.Dot(state.Velocity);
        double energy = 0.5 * speedSquared - mu / radius;
        Vector3d angularMomentum = state.Position.Cross(state.Velocity);
        Vector3d eccentricityVector = state.Velocity.Cross(angularMomentum) / mu
            - state.Position / radius;
        double eccentricity = eccentricityVector.Length();
        double perilune = double.NaN;
        double apolune = double.PositiveInfinity;
        if (energy < 0)
        {
            double semiMajorAxis = -mu / (2 * energy);
            perilune = semiMajorAxis * (1 - eccentricity);
            apolune = semiMajorAxis * (1 + eccentricity);
        }
        Require(energy >= 0 || eccentricity >= minimumEccentricity,
            $"expected bad circularization, but orbit is bound with "
            + $"eccentricity {eccentricity:R} below {minimumEccentricity:R}");
        Pass(step, energy >= 0
            ? $"reproduced bad circularization: unbound lunar trajectory, e={eccentricity:F4}"
            : $"reproduced bad circularization: Pe {perilune / 1000:F0} km, "
                + $"Ap {apolune / 1000:F0} km, e={eccentricity:F4}");
    }

    private static StateVector VesselRelativeToBody(Vehicle vessel, string bodyId)
    {
        StateVectors state = vessel.Orbit.StateVectors;
        double time = state.StateTime.Seconds();
        if (vessel.Orbit.Parent is not Astronomical parentBody
            || vessel.Orbit.Parent is not IParentBody parent)
            throw new InvalidOperationException($"'{vessel.Id}' has no astronomical parent");
        StateVector parentAbsolute = ModServices.Rails!.GetAbsolute(parentBody.Id, time);
        StateVector vesselAbsolute = new(
            parentAbsolute.Position + FrameAdapter.CciToEcl(state.PositionCci, parent.GetCci2Cce()),
            parentAbsolute.Velocity + FrameAdapter.CciToEcl(state.VelocityCci, parent.GetCci2Cce()));
        return vesselAbsolute - ModServices.Rails.GetAbsolute(bodyId, time);
    }

    private static Vehicle ResolveVehicle(string? id)
    {
        if (string.IsNullOrWhiteSpace(id))
            return Program.ControlledVehicle
                ?? throw new InvalidOperationException("there is no controlled vessel");
        return ModServices.Vessels?.TryGetLiveVehicle(id)
            ?? throw new InvalidOperationException($"tracked vessel '{id}' was not found");
    }

    private static void Pass(GameTestStep step, string detail)
    {
        double now = Universe.GetElapsedSimTime().Seconds();
        Results.Add(new GameTestStepResult
        {
            Index = _stepIndex,
            Action = step.Action,
            Passed = true,
            Detail = detail,
            SimulationTime = now,
        });
        ModLog.Info($"game test: step {_stepIndex} '{step.Action}' passed: {detail}");
        _stepIndex++;
        _stepStartedMs = Environment.TickCount64;
        _stepIssued = false;
        _baselineBurnCount = 0;
        _playerLunarTransferJob = null;
        _playerLunarCorrectionJob = null;
        _playerLunarCorrectionQueued = false;
        _lunarPeriluneProbeJob = null;
        _playerLunarCircularizationJob = null;
        _burnExecutionStage = 0;
        _burnExecutionBaselineCount = 0;
        _burnsExecuted = 0;
        _burnTargetMagnitude = 0;
        _burnExecutionWarpEngaged = false;
        _automaticBurnTargetsLostAtSoi = 0;
    }

    private static void Fail(string error)
    {
        if (_finished) return;
        try
        {
            Universe.AutoWarpStop(resetSimulationSpeed: false);
            Universe.SetSimulationSpeed(1.0);
        }
        catch { }
        ModLog.Error($"game test failed: {error}");
        if (_scenario is not null && _stepIndex < _scenario.Steps.Count)
        {
            Results.Add(new GameTestStepResult
            {
                Index = _stepIndex,
                Action = _scenario.Steps[_stepIndex].Action,
                Passed = false,
                Detail = error,
                SimulationTime = TrySimulationTime(),
            });
        }
        Finish(passed: false, error);
    }

    private static void Finish(bool passed, string? error)
    {
        if (_finished || _scenario is null) return;
        _finished = true;
        WallClock.Stop();
        var result = new GameTestResult
        {
            RunId = _scenario.RunId,
            Scenario = _scenario.Name,
            Passed = passed,
            Error = error,
            ElapsedWallSeconds = WallClock.Elapsed.TotalSeconds,
            FinalSimulationTime = TrySimulationTime(),
            Steps = [.. Results],
        };
        string path = Path.Combine(ModMain.ModDir, GameTestProtocol.ResultFileName);
        string temporary = path + ".tmp";
        File.WriteAllText(temporary,
            JsonSerializer.Serialize(result, GameTestProtocol.JsonOptions));
        File.Move(temporary, path, overwrite: true);
        ModLog.Info($"game test: '{_scenario.Name}' {(passed ? "passed" : "failed")}");
    }

    private static double? TrySimulationTime()
    {
        try { return Universe.CurrentSystem is null ? null : Universe.GetElapsedSimTime().Seconds(); }
        catch { return null; }
    }

    private static void ExecuteBurns(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        FlightComputer computer = vessel.FlightComputer;
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vessel);

        if (_burnExecutionStage == 0)
        {
            if (burns.Count == 0)
            {
                Require(_burnsExecuted > 0, "there were no burns to execute");
                Universe.SetSimulationSpeed(1.0);
                string detail = _automaticBurnTargetsLostAtSoi > 0
                    ? "reproduced stock circularization failure: built-in Auto Warp "
                        + "unloaded its node and Auto Burn target at Luna SOI"
                    : $"executed {_burnsExecuted} burn(s) through KSA "
                        + "flight computer Auto and built-in Auto Warp";
                Pass(step, detail);
                return;
            }
            if (computer.Burn is not { } target) return;
            _burnExecutionBaselineCount = burns.Count;
            _burnTargetMagnitude = target.DeltaVTargetCci.Length();
            _burnExecutionWarpEngaged = false;
            Require(_burnTargetMagnitude > 0 && float.IsFinite(_burnTargetMagnitude),
                "next burn has no finite delta-v target");
            ModLog.Info($"game test: arming {_burnTargetMagnitude:F2} m/s burn; "
                + $"mode={computer.BurnMode}, engine flow={computer.VehicleConfig.TotalEngineVacuumMassFlowRate:F3} kg/s");
            QueueFlightComputer(vessel, FlightComputerBurnMode.Auto);
            _burnExecutionStage = 1;
            return;
        }

        if (_burnExecutionStage == 1)
        {
            if (burns.Count < _burnExecutionBaselineCount)
            {
                Require(_earthSoiCircularizationReproArmed,
                    "burn plan disappeared while automatic execution was arming");
                LogExpectedCircularizationNodeLoss(computer);
                return;
            }
            if (computer.Burn is not { } target) return;
            if (computer.BurnMode != FlightComputerBurnMode.Auto) return;
            // Auto mode makes KSA compute finite burn duration and the centered
            // ignition time on its next vehicle-control update. Never auto-warp to
            // PositiveInfinity or to a stale Manual-mode ignition estimate.
            double ignition = target.IgnitionTime.Seconds();
            if (!double.IsFinite(ignition)) return;
            double now = Universe.GetElapsedSimTime().Seconds();
            Require(ignition > now,
                $"next burn ignition t={ignition:F1} is no longer in the future (now {now:F1}); give the node more lead time");
            ModLog.Info($"game test: auto burn accepted; ignition t={ignition:F1}, now={now:F1}, "
                + $"remaining={target.DeltaVToGoCci.Length():F2} m/s");
            ToggleFlightComputer(vessel, FlightComputerAction.WarpToNextBurn);
            ModLog.Info("game test: pressed KSA built-in Auto Warp for next burn");
            _burnExecutionStage = 2;
            return;
        }

        if (_burnExecutionStage == 2)
        {
            bool stockNodeMissing = burns.Count < _burnExecutionBaselineCount;
            if (stockNodeMissing)
            {
                Require(_earthSoiCircularizationReproArmed,
                    "burn plan disappeared before automatic burn completion");
                LogExpectedCircularizationNodeLoss(computer);
                if (computer.Burn is null)
                {
                    Universe.AutoWarpStop(resetSimulationSpeed: false);
                    Universe.SetSimulationSpeed(1.0);
                    _automaticBurnTargetsLostAtSoi++;
                    _burnExecutionStage = 4;
                    ModLog.Info("game test: accepting the unloaded circularization "
                        + "Auto Burn target as the expected bad-orbit repro outcome");
                    return;
                }
            }
            if (computer.Burn is not { } target) return;
            if (Universe.IsAutoWarpActive) return;
            if (computer.BurnMode == FlightComputerBurnMode.Auto)
            {
                double secondsToIgnition = target.IgnitionTime.Seconds()
                    - Universe.GetElapsedSimTime().Seconds();
                Require(secondsToIgnition <= 5,
                    $"KSA built-in Auto Warp disengaged {secondsToIgnition:F1} s "
                    + "before ignition");
                if (!_burnExecutionWarpEngaged)
                {
                    double burnWarp = _earthSoiCircularizationReproArmed
                        ? 4.0 : 10.0;
                    Universe.SetSimulationSpeed(burnWarp);
                    _burnExecutionWarpEngaged = true;
                    ModLog.Info("game test: built-in Auto Warp finished; "
                        + $"set engine burn warp to {burnWarp:R}x");
                }
                return;
            }

            Universe.SetSimulationSpeed(1.0);

            float remaining = target.DeltaVToGoCci.Length();
            float completionDot = float3.Dot(target.DeltaVToGoCci,
                target.DeltaVTargetCci);
            ModLog.Info($"game test: automatic burn exited mode={computer.BurnMode}; "
                + $"remaining={remaining:F2} m/s, completionDot={completionDot:F3}");
            const float automaticBurnResidualToleranceMps = 0.1f;
            Require(float.IsFinite(remaining) && float.IsFinite(completionDot)
                    && (completionDot <= 0
                        || remaining <= automaticBurnResidualToleranceMps),
                $"automatic burn stopped with {remaining:F2} m/s remaining of {_burnTargetMagnitude:F2} m/s");
            _burnExecutionStage = 3;
            ModLog.Info("game test: Auto cutoff acknowledged; keeping the planned "
                + "burn until the mod's live delta-v witness settles");
            return;
        }

        if (_burnExecutionStage == 3)
        {
            if (burns.Count == 0 && _earthSoiCircularizationReproArmed)
            {
                ModLog.Info("game test: completed circularization Auto Burn target "
                    + "had no stock node left to clean up after the SOI transition");
                _burnExecutionStage = 4;
                return;
            }
            VesselRegistry vessels = ModServices.Vessels
                ?? throw new InvalidOperationException("vessel registry is unavailable");
            string verdict = BurnPlannerPanel.RemoveCompletedBurnForGameTest(
                vessels, vessel);
            if (verdict.StartsWith("waiting:", StringComparison.Ordinal))
                return;
            Require(verdict == "queued",
                $"planner did not remove its completed burn: {verdict}");
            ModLog.Info("game test: mod planner removed its completed burn after "
                + "live delta-v stayed quiet for the full coast interval");
            _burnExecutionStage = 4;
            return;
        }

        if (burns.Count >= _burnExecutionBaselineCount) return;
        _burnsExecuted++;
        _burnExecutionStage = 0;
        _burnExecutionWarpEngaged = false;
        _earthSoiCircularizationReproArmed = false;
        _earthSoiCircularizationNodeLossLogged = false;
    }

    private static void LogExpectedCircularizationNodeLoss(
        FlightComputer computer)
    {
        if (_earthSoiCircularizationNodeLossLogged) return;
        _earthSoiCircularizationNodeLossLogged = true;
        ModLog.Info("game test: stock circularization node absent during the "
            + "Earth-to-Luna SOI Auto Warp transition; active target="
            + $"{(computer.Burn is null ? "none" : "present")}, "
            + $"mode={computer.BurnMode}, autoWarp={Universe.IsAutoWarpActive}, "
            + $"speed={Universe.GetSimulationSpeed():R}x");
    }

    private static void QueueFlightComputer(Vehicle vehicle, Enum value) =>
        InputEvents.FlightComputerInputBuffer.Add(new InputEvents.FlightComputerInputData
        {
            Vehicle = vehicle,
            Toggle = false,
            EnumValue = value,
        });

    private static void ToggleFlightComputer(Vehicle vehicle, Enum value) =>
        InputEvents.FlightComputerInputBuffer.Add(new InputEvents.FlightComputerInputData
        {
            Vehicle = vehicle,
            Toggle = true,
            EnumValue = value,
        });

    private static void ValidateScenario(GameTestScenario scenario)
    {
        Require(!string.IsNullOrWhiteSpace(scenario.RunId), "runId is required");
        Require(!string.IsNullOrWhiteSpace(scenario.Name), "name is required");
        Require(scenario.TimeoutSeconds > 0 && double.IsFinite(scenario.TimeoutSeconds),
            "timeoutSeconds must be finite and positive");
        Require(scenario.Steps.Count > 0, "at least one step is required");
    }

    private static string Required(string? value, string name) =>
        !string.IsNullOrWhiteSpace(value) ? value
            : throw new InvalidDataException($"{name} is required");

    private static double Required(double? value, string name) =>
        value is { } number && double.IsFinite(number) ? number
            : throw new InvalidDataException($"{name} must be finite");

    private static void Require(bool condition, string message)
    {
        if (!condition) throw new InvalidOperationException(message);
    }
}
