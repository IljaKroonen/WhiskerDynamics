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
    private static int _autoStagesActivated;
    private static int _autoStageSettleFrames;
    private static GameTestPlayerLunarCircularizationSolveJob?
        _playerLunarCircularizationJob;
    private static int _lunarCircularizationPlanStage;
    private static int _lunarCircularizationPlanSettleFrames;
    private static double? _lunarOrbitStartTime;
    private static double _lunarOrbitPeriod;
    private static StateVector _lunarOrbitStartState;
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

            case "auto-stage":
                AutoStage(step);
                break;

            case "plan-lunar-circularization-from-earth-soi":
                PlanLunarCircularizationFromEarthSoi(step);
                break;

            case "execute-burns":
                ExecuteBurns(step);
                break;

            case "complete-lunar-orbit":
                CompleteLunarOrbit(step);
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

    private static void AutoStage(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        int count = step.StageCount ?? 1;
        Require(count > 0, "auto-stage count must be positive");
        if (_autoStagesActivated < count)
        {
            if (_autoStageSettleFrames++ < 2)
                return;
            // ActivateNextSequence is a silent no-op when no unactivated sequence
            // remains; probe first so staging nothing fails loudly.
            Require(vessel.Parts.SequenceList.GetNextSequenceNumber() >= 0,
                $"no unactivated staging sequence remains for stage "
                + $"{_autoStagesActivated + 1}/{count}");
            int before = vessel.Parts.SequenceList.ActiveSequence;
            vessel.Parts.SequenceList.ActivateNextSequence(vessel);
            _autoStagesActivated++;
            _autoStageSettleFrames = 0;
            ModLog.Info($"game test: activated staging sequence "
                + $"{_autoStagesActivated}/{count} (active sequence {before} -> "
                + $"{vessel.Parts.SequenceList.ActiveSequence})");
            return;
        }
        if (_autoStageSettleFrames++ < 4)
            return;
        int activeCores = 0;
        foreach (RocketCore core in vessel.Parts.RocketCores.Modules)
            if (core.Controller is EngineController { IsActive: true })
                activeCores++;
        // Ignition can take a while to report an active controller; poll within
        // the step timeout instead of failing one-shot.
        if (activeCores == 0)
            return;
        Pass(step, $"activated {count} staging sequences; "
            + $"active main-engine cores {activeCores}, active engine flow "
            + $"{vessel.FlightComputer.ActiveEngineMassFlowRate:F3} kg/s");
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
            // Null marks the plan ExecuteBurns already deleted on completion.
            Require(_playerLunarPlan is null || ReferenceEquals(
                    FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
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
            Require(FlightPlans.TryGet(vessel.Id) is null,
                "the completed TLI flight plan was not deleted");
            string verdict = BurnPlannerPanel.PlanBurnForGameTest(
                registry, vessel, correction.BurnTime,
                frame: null, components: correction.DeltaVVlf);
            Require(verdict.StartsWith("queued", StringComparison.Ordinal),
                $"lunar correction burn was not queued: {verdict}");
            _playerLunarPlan = FlightPlans.TryGet(vessel.Id)
                ?? throw new InvalidOperationException(
                    "planner queued the correction without creating a new flight plan");
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
            Require(_playerLunarPlan is null || ReferenceEquals(
                    FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
                "the mod flight plan was removed or replaced during correction");
            if (_burnExecutionStage == 0 && burns.Count == 0)
                return;
            ExecuteBurns(step);
            return;
        }
        Pass(step, $"executed 3150 m/s prograde TLI and "
            + $"{correction.DeltaVVlf.Length():F3} m/s correction at departure +1 h");
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
        double minimumAltitude = Required(step.MinPeriluneAltitudeMeters,
            "minimumPeriluneAltitudeMeters");
        double maximumAltitude = Required(step.MaxPeriluneAltitudeMeters,
            "maximumPeriluneAltitudeMeters");
        Require(minimumAltitude <= maximumAltitude,
            "minimum lunar perilune altitude exceeds maximum");
        double periluneAltitude =
            solution.PeriluneRadiusMeters - rails.MeanRadiusOf("Luna");
        Require(periluneAltitude >= minimumAltitude
                && periluneAltitude <= maximumAltitude,
            $"predicted lunar perilune altitude {periluneAltitude:R} m is outside "
            + $"[{minimumAltitude:R}, {maximumAltitude:R}] m");
        var frame = new FrameSpec(FrameKind.Inertial, "Luna", null);
        if (_lunarCircularizationPlanStage == 0)
        {
            Require(FlightPlans.TryGet(vessel.Id) is null,
                "the completed transfer flight plan was not deleted");
            string verdict = BurnPlannerPanel.CreatePlanForGameTest(vessel);
            Require(verdict == "plan created",
                $"could not create a fresh circularization plan: {verdict}");
            _playerLunarPlan = FlightPlans.TryGet(vessel.Id)
                ?? throw new InvalidOperationException(
                    "planner did not retain the fresh circularization plan");
            _lunarCircularizationPlanStage = 1;
            _lunarCircularizationPlanSettleFrames = 0;
            ModLog.Info("game test: created a fresh flight plan for LCI after "
                + "deleting the completed transfer plan");
            return;
        }

        if (_lunarCircularizationPlanStage == 1)
        {
            if (_lunarCircularizationPlanSettleFrames++ < 4)
                return;
            string verdict = BurnPlannerPanel.AddPlaceholderBurnForGameTest(
                registry, vessel, frame);
            if (verdict == "rejected: plan diverged; rebase it before adding a burn")
            {
                // A live-physics episode can mark the fresh plan Diverged; recover
                // like a player: rebase once thrust settles, retry next frame.
                string rebase = BurnPlannerPanel.RebasePlanForGameTest(
                    registry, vessel);
                if (rebase != "rejected: rebase is unavailable until thrust stops")
                    ModLog.Info("game test: fresh LCI plan diverged before the "
                        + $"placeholder; rebase: {rebase}");
                return;
            }
            Require(verdict.StartsWith("queued", StringComparison.Ordinal),
                $"Earth-SOI lunar circularization placeholder was not queued: {verdict}");
            Require(ReferenceEquals(
                    FlightPlans.TryGet(vessel.Id), _playerLunarPlan),
                "circularization replaced its fresh flight plan");
            _lunarCircularizationPlanStage = 2;
            _lunarCircularizationPlanSettleFrames = 0;
            ModLog.Info("game test: added the Luna-frame zero-dv placeholder "
                + "through the same path as the planner Add burn button");
            return;
        }

        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vessel);
        if (burns.Count <= _baselineBurnCount)
        {
            // Stage 2 legitimately waits for the queued placeholder to land; once a
            // later stage has edited it, an empty plan means KSA dropped the node.
            Require(_lunarCircularizationPlanStage == 2,
                "the circularization placeholder disappeared while it was being edited");
            return;
        }
        Burn burn = burns[0];
        if (_lunarCircularizationPlanStage == 2)
        {
            if (_lunarCircularizationPlanSettleFrames++ < 4)
                return;
            string verdict = BurnPlannerPanel.MoveBurnForGameTest(
                registry, vessel, burn, solution.BurnTime);
            // Exact-match: reconversion FAILURE statuses also begin with "time applied".
            Require(verdict is "time applied"
                    or "time applied; dv reconverted in authoring frame",
                $"Earth-SOI lunar circularization time edit failed: {verdict}");
            _lunarCircularizationPlanStage = 3;
            _lunarCircularizationPlanSettleFrames = 0;
            ModLog.Info($"game test: moved the Luna-frame placeholder to "
                + $"predicted perilune t={solution.BurnTime:F1}");
            return;
        }

        if (_lunarCircularizationPlanStage == 3)
        {
            if (_lunarCircularizationPlanSettleFrames++ < 4)
                return;
            Require(BurnIdentityPolicy.SameBurn(
                    burn.Time.Seconds(), solution.BurnTime),
                "KSA did not retain the circularization time edit");
            string verdict = BurnPlannerPanel.EditBurnComponentsForGameTest(
                registry, vessel, burn, solution.LunaFrameDeltaVPrn);
            Require(verdict == "applied",
                $"Earth-SOI lunar circularization component edit failed: {verdict}");
            _lunarCircularizationPlanStage = 4;
            _lunarCircularizationPlanSettleFrames = 0;
            ModLog.Info($"game test: entered the pure-retrograde LCI components "
                + $"at t={solution.BurnTime:F1}: authored PRN=("
                + $"{solution.LunaFrameDeltaVPrn.X:F3}, "
                + $"{solution.LunaFrameDeltaVPrn.Y:F3}, "
                + $"{solution.LunaFrameDeltaVPrn.Z:F3}) m/s");
            return;
        }

        if (_lunarCircularizationPlanSettleFrames++ < 4)
            return;
        Require(BurnIdentityPolicy.SameBurn(
                burn.Time.Seconds(), solution.BurnTime),
            "KSA did not retain the circularization node after component entry");
        Pass(step, $"planned {solution.LunaFrameDeltaVPrn.Length():F2} m/s "
            + $"pure retrograde LCI circularization burn at predicted perilune "
            + $"{periluneAltitude / 1000:F1} km altitude "
            + "while still in Earth SOI");
    }

    private readonly record struct LunarOrbitMetrics(
        double Energy, double Eccentricity,
        double PeriluneRadius, double ApoluneRadius, double Period);

    private static void CompleteLunarOrbit(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        Require((vessel.Orbit.Parent as Astronomical)?.Id == "Luna",
            "lunar circularization did not leave the vessel in Luna SOI");
        double maximumEccentricity = Required(step.MaxEccentricity,
            "maximumEccentricity");
        Require(maximumEccentricity > 0 && maximumEccentricity < 1,
            "maximum lunar-orbit eccentricity must be in (0, 1)");

        double now = Universe.GetElapsedSimTime().Seconds();
        if (_lunarOrbitStartTime is null)
        {
            StateVector start = VesselRelativeToBody(vessel, "Luna");
            LunarOrbitMetrics accepted = LunarOrbit(start);
            RequireCircularLunarOrbit(accepted, maximumEccentricity,
                ModServices.Rails!.MeanRadiusOf("Luna"));
            _lunarOrbitStartTime = now;
            _lunarOrbitPeriod = accepted.Period;
            _lunarOrbitStartState = start;
            ModLog.Info($"game test: circular lunar orbit accepted at "
                + $"e={accepted.Eccentricity:F4}; coasting one {accepted.Period:F1} s "
                + "osculating period at up to 1000x");
            return;
        }

        double remaining = _lunarOrbitStartTime.Value + _lunarOrbitPeriod - now;
        double desiredSpeed = remaining switch
        {
            > 300 => 1_000,
            > 30 => 100,
            > 3 => 10,
            _ => 1,
        };
        if (Universe.GetSimulationSpeed() != desiredSpeed)
            Universe.SetSimulationSpeed(desiredSpeed);
        // Osculating metrics legitimately wobble during the n-body coast, so the
        // orbit-shape requirements run only at the two endpoints.
        if (remaining > 0)
            return;

        Universe.SetSimulationSpeed(1.0);
        StateVector state = VesselRelativeToBody(vessel, "Luna");
        double lunaRadius = ModServices.Rails!.MeanRadiusOf("Luna");
        LunarOrbitMetrics orbit = LunarOrbit(state);
        RequireCircularLunarOrbit(orbit, maximumEccentricity, lunaRadius);
        // A hitched wall frame at 1000x can overshoot the one-period target, so
        // judge the phase against the coast time that actually elapsed. Two-body
        // propagation keeps the expectation in the same angular domain as the
        // measurement; a mean-motion expectation would be off by the equation of
        // center (up to ~28.6 deg at e=0.25).
        StateVector expected = Kepler.PropagateUniversal(
            _lunarOrbitStartState, ModServices.Rails!.MuOf("Luna"),
            now - _lunarOrbitStartTime.Value);
        double phaseCosine = Math.Clamp(
            expected.Position.Normalized().Dot(state.Position.Normalized()), -1, 1);
        double phaseErrorDegrees = Math.Acos(phaseCosine) * 180 / Math.PI;
        Require(phaseErrorDegrees <= 30,
            $"one-period lunar coast ended {phaseErrorDegrees:F1} deg from "
            + "its expected orbital phase");
        Pass(step, $"completed one {_lunarOrbitPeriod:F1} s lunar orbit; "
            + $"Pe {(orbit.PeriluneRadius - lunaRadius) / 1000:F1} km, "
            + $"Ap {(orbit.ApoluneRadius - lunaRadius) / 1000:F1} km, "
            + $"e={orbit.Eccentricity:F4}, phase error {phaseErrorDegrees:F1} deg");
    }

    private static void RequireCircularLunarOrbit(
        LunarOrbitMetrics orbit, double maximumEccentricity, double lunaRadius)
    {
        Require(orbit.Energy < 0 && double.IsFinite(orbit.Period),
            $"lunar circularization did not produce a bound orbit; "
            + $"specific energy={orbit.Energy:R} J/kg");
        Require(orbit.PeriluneRadius > lunaRadius,
            $"lunar orbit intersects the surface: Pe radius "
            + $"{orbit.PeriluneRadius:R} m, Luna radius {lunaRadius:R} m");
        Require(orbit.Eccentricity <= maximumEccentricity,
            $"lunar orbit eccentricity {orbit.Eccentricity:R} exceeds "
            + $"{maximumEccentricity:R}");
    }

    private static LunarOrbitMetrics LunarOrbit(in StateVector state)
    {
        double mu = ModServices.Rails!.MuOf("Luna");
        double radius = state.Position.Length();
        double energy = 0.5 * state.Velocity.LengthSquared() - mu / radius;
        Vector3d angularMomentum = state.Position.Cross(state.Velocity);
        Vector3d eccentricityVector = state.Velocity.Cross(angularMomentum) / mu
            - state.Position / radius;
        double eccentricity = eccentricityVector.Length();
        if (!(energy < 0))
            return new LunarOrbitMetrics(
                energy, eccentricity, double.NaN, double.PositiveInfinity,
                double.PositiveInfinity);
        double semiMajorAxis = -mu / (2 * energy);
        return new LunarOrbitMetrics(
            energy, eccentricity,
            semiMajorAxis * (1 - eccentricity),
            semiMajorAxis * (1 + eccentricity),
            RendezvousKernel.OrbitalPeriod(state, mu));
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
        _autoStagesActivated = 0;
        _autoStageSettleFrames = 0;
        _playerLunarCircularizationJob = null;
        _lunarCircularizationPlanStage = 0;
        _lunarCircularizationPlanSettleFrames = 0;
        _burnExecutionStage = 0;
        _burnExecutionBaselineCount = 0;
        _burnsExecuted = 0;
        _burnTargetMagnitude = 0;
        _burnExecutionWarpEngaged = false;
        _lunarOrbitStartTime = null;
        _lunarOrbitPeriod = 0;
        _lunarOrbitStartState = default;
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
                Pass(step, $"executed {_burnsExecuted} burn(s) through KSA "
                    + "flight computer Auto and built-in Auto Warp");
                return;
            }
            if (computer.Burn is not { } target) return;
            _burnExecutionBaselineCount = burns.Count;
            _burnTargetMagnitude = target.DeltaVTargetCci.Length();
            _burnExecutionWarpEngaged = false;
            Require(_burnTargetMagnitude > 0 && float.IsFinite(_burnTargetMagnitude),
                "next burn has no finite delta-v target");
            ModLog.Info($"game test: arming {_burnTargetMagnitude:F2} m/s burn; "
                + $"mode={computer.BurnMode}, engine flow={computer.ActiveEngineMassFlowRate:F3} kg/s");
            QueueFlightComputer(vessel, FlightComputerBurnMode.Auto);
            _burnExecutionStage = 1;
            return;
        }

        if (_burnExecutionStage == 1)
        {
            Require(burns.Count >= _burnExecutionBaselineCount,
                "planned burn disappeared while automatic execution was arming");
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
            // Losing the armed node here is the regression this suite exists to
            // catch — fail it by name instead of the wall timeout. KSA can null the
            // BurnTarget for a frame across an SOI-boundary recalculation, so a
            // null target is tolerated while the node survives.
            if (computer.Burn is not { } target)
            {
                Require(burns.Count >= _burnExecutionBaselineCount,
                    "burn plan disappeared before automatic burn completion");
                return;
            }
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
                    _burnExecutionWarpEngaged = true;
                    if (step.LunarCircularization ?? false)
                    {
                        ModLog.Info("game test: built-in Auto Warp finished for "
                            + "lunar circularization; leaving KSA simulation speed "
                            + $"at {Universe.GetSimulationSpeed():R}x");
                    }
                    else
                    {
                        double burnWarp = _burnTargetMagnitude < 100 ? 1.0 : 10.0;
                        Universe.SetSimulationSpeed(burnWarp);
                        ModLog.Info("game test: built-in Auto Warp finished; "
                            + $"set engine burn warp to {burnWarp:R}x");
                    }
                }
                return;
            }

            Universe.SetSimulationSpeed(1.0);

            float remaining = target.DeltaVToGoCci.Length();
            float completionDot = float3.Dot(target.DeltaVToGoCci,
                target.DeltaVTargetCci);
            ModLog.Info($"game test: automatic burn exited mode={computer.BurnMode}; "
                + $"remaining={remaining:F2} m/s, completionDot={completionDot:F3}");
            // KSA can now cut off a short correction with a sub-m/s residual at
            // accelerated burn warp. That is operationally complete; keep the
            // guard tight enough to reject a materially unfinished burn.
            const float automaticBurnResidualToleranceMps = 0.5f;
            Require(float.IsFinite(remaining) && float.IsFinite(completionDot)
                    && (completionDot <= 0
                        || remaining <= automaticBurnResidualToleranceMps),
                $"automatic burn stopped with {remaining:F2} m/s remaining of {_burnTargetMagnitude:F2} m/s");
            if (step.LunarCircularization ?? false)
            {
                _burnsExecuted++;
                Pass(step, "executed the LCI burn through KSA flight computer "
                    + "Auto and built-in Auto Warp without test-side plan edits");
                return;
            }
            _burnExecutionStage = 3;
            ModLog.Info("game test: Auto cutoff acknowledged; deleting the "
                + "completed plan and burns through the planner");
            return;
        }

        if (_burnExecutionStage == 3)
        {
            string verdict = BurnPlannerPanel.DeletePlanAndBurns(vessel);
            Require(verdict.StartsWith("plan deleted", StringComparison.Ordinal),
                $"planner did not delete its completed flight plan: {verdict}");
            _playerLunarPlan = null;
            ModLog.Info("game test: used the planner's Delete plan and burns "
                + "action after the completed burn");
            _burnExecutionStage = 4;
            return;
        }

        if (_burnExecutionStage == 4)
        {
            if (burns.Count >= _burnExecutionBaselineCount) return;
            _burnsExecuted++;
            _burnExecutionStage = 0;
            _burnExecutionWarpEngaged = false;
            return;
        }

        throw new InvalidOperationException(
            $"unknown burn execution stage {_burnExecutionStage}");
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
