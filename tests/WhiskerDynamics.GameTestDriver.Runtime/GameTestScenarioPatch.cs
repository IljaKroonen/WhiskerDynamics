using System.Diagnostics;
using System.Text.Json;
using Brutal.GlfwApi;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance;
using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;
using WhiskerDynamics.GameTesting;
using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Ui;

namespace WhiskerDynamics.GameTestDriver.Runtime;

/// <summary>
/// Opt-in end-to-end test driver. A request file is written by the external
/// GameTests host; steps execute here on KSA's main UI thread, using the same
/// save, warp, targeting, and maneuver-plan APIs as the game and mod UI.
/// With no request file this patch is a single file-existence check per frame.
/// </summary>
[HarmonyPatch(typeof(Program), "OnDrawUiConsole")]
internal static class GameTestScenarioPatch
{
    private static readonly Stopwatch WallClock = new();
    private static GameTestScenario? _scenario;
    private static readonly List<GameTestStepResult> Results = [];
    private static int _stepIndex;
    private static long _stepStartedMs;
    private static bool _loadIssued;
    private static int _readyFrames;
    private static bool _stepIssued;
    private static int _baselineBurnCount;
    private static double _stepStartedSimTime;
    private static int _burnExecutionStage;
    private static int _burnExecutionBaselineCount;
    private static int _burnsExecuted;
    private static float _burnTargetMagnitude;
    private static bool _burnWarpActive;
    private static int _refillFrames;
    private static float _rcsBurnInitialCompletionDot;
    private static float _rcsBurnInitialResidual;
    private static double _rcsBurnExpectedDuration;
    private static bool _rcsBurnCutoffArmed;
    private static double _rcsBurnCutoffSimTime = double.NaN;
    private static bool _rcsBurnCutoffReached;
    private static Vehicle? _rcsBurnVehicle;
    private static int _rcsBurnReleaseIssued;
    private static GameTestLunarTransferSolveJob? _lunarTransferJob;
    private static double _nrhoEpoch = double.NaN;
    private static double _nrhoTimeUnitSeconds = double.NaN;
    private static double _nrhoMinRadius;
    private static double _nrhoMaxRadius;
    private static double _nrhoCorrectionMagnitude;
    private static double _nrhoPreviousRadius;
    private static bool _nrhoApoluneReached;
    private static bool _nrhoPeriluneApproachSlowed;
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
                ModLog.Info($"game test: starting '{_scenario.Name}' run {_scenario.RunId}");
            }

            if (WallClock.Elapsed.TotalSeconds > _scenario.TimeoutSeconds)
            {
                Fail($"scenario timed out after {_scenario.TimeoutSeconds:F1} wall seconds");
                return;
            }

            if (!_loadIssued)
            {
                _loadIssued = true;
                if (string.IsNullOrWhiteSpace(_scenario.Save))
                    ModLog.Info("game test: using the current default system as the fixture");
                else
                {
                    ModLog.Info($"game test: loading fixture '{_scenario.Save}'");
                    GameSaves.LoadSaveGame(_scenario.Save);
                }
                _stepStartedMs = Environment.TickCount64;
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

        // Let one complete game frame commit after the synchronous save load and rebind.
        return ++_readyFrames >= 2;
    }

    private static void Execute(GameTestStep step)
    {
        string action = step.Action.Trim().ToLowerInvariant();
        switch (action)
        {
            case "wait-ready":
                Pass(step, $"mod active; controlled vessel '{Program.ControlledVehicle!.Id}'");
                break;

            case "assert-mod-active":
                Require(ModServices.Status == ModStatus.Active,
                    $"expected mod Active, got {ModServices.Status}");
                Pass(step, "mod is active");
                break;

            case "set-target":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                Vehicle target = ResolveVehicle(Required(step.Target, "target"));
                vessel.SetTarget(target);
                Pass(step, $"'{vessel.Id}' now targets '{target.Id}'");
                break;
            }

            case "plan-rendezvous":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                string targetId = Required(step.Target, "target");
                double duration = Required(step.DurationSeconds, "planDurationSeconds");
                Require(duration >= 10 * 60.0,
                    "planDurationSeconds must be at least ten minutes");
                Require(vessel.Target?.Id == targetId,
                    $"'{vessel.Id}' must target '{targetId}' before planning rendezvous");

                if (!_stepIssued)
                {
                    string verdict = BurnPlannerPanel.StartRendezvousForGameTest(
                        ModServices.Vessels!, vessel, duration);
                    if (verdict != "queued")
                    {
                        // Rails and the detached prediction snapshot fill asynchronously
                        // after a load. Treat their documented retry states as waiting,
                        // while genuine planner rejections fail immediately.
                        if (verdict.Contains("retry", StringComparison.OrdinalIgnoreCase)
                            || verdict.Contains("preparing", StringComparison.OrdinalIgnoreCase)
                            || verdict.Contains("not yet", StringComparison.OrdinalIgnoreCase)
                            || verdict.Contains("active n-body rails", StringComparison.OrdinalIgnoreCase)
                            || verdict.Contains("coasting on current rails", StringComparison.OrdinalIgnoreCase))
                            return;
                        Require(false, verdict);
                    }
                    _stepIssued = true;
                    return;
                }

                if (BurnPlannerPanel.RendezvousPendingForGameTest(vessel.Id)) return;
                var plan = FlightPlans.TryGet(vessel.Id);
                int burnCount = BurnPlanWriter.Snapshot(vessel).Count;
                Require(plan is not null && burnCount == 2,
                    BurnPlannerPanel.RendezvousStatusForGameTest);
                Pass(step, BurnPlannerPanel.RendezvousStatusForGameTest);
                break;
            }

            case "control-vessel":
            {
                Vehicle vessel = ResolveVehicle(Required(step.Vessel, "vessel"));
                if (!_stepIssued)
                {
                    InputEvents.VehicleResourcesChangeBuffer.Add(
                        new InputEvents.VehicleResourcesChangeData
                        {
                            Vehicle = vessel,
                            Control = true,
                        });
                    _stepIssued = true;
                    return;
                }
                if (Program.ControlledVehicle?.Id != vessel.Id) return;
                Pass(step, $"KSA now controls '{vessel.Id}'");
                break;
            }

            case "refill":
            {
                Vehicle vessel = ResolveVehicle(Required(step.Vessel, "vessel"));
                if (!_stepIssued)
                {
                    vessel.Parts.ResourceGroupList.CalculateStages(reconfigureTankContents: true);
                    _stepIssued = true;
                    _refillFrames = 0;
                    return;
                }
                if ((_refillFrames++ & 1) == 0)
                {
                    InputEvents.VehicleResourcesChangeBuffer.Add(
                        new InputEvents.VehicleResourcesChangeData
                        {
                            Vehicle = vessel,
                            Refill = true,
                        });
                    return;
                }
                ConsumableStatus status = ReadConsumableStatus(vessel);
                if (!status.Ready) return;
                Pass(step, $"KSA refilled '{vessel.Id}' consumables: {status}");
                break;
            }

            case "teleport-nrho-approach":
                TeleportToNrhoApproach(step);
                break;

            case "save-as":
            {
                string saveName = Required(step.Target, "save name");
                string saveDirectory = Path.Combine(GameSaves.SaveFolderPath, saveName);
                Require(!Directory.Exists(saveDirectory),
                    $"save fixture '{saveName}' already exists; remove it explicitly before regenerating fixtures");
                GameSaves.MakeUncompressedSave(saveName);
                string savePath = Path.Combine(saveDirectory, "universe.xml");
                Require(File.Exists(savePath),
                    $"KSA did not create save fixture '{saveName}'");
                Pass(step, $"created save fixture '{saveName}'");
                break;
            }

            case "add-burn":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                if (!_stepIssued)
                {
                    _baselineBurnCount = BurnPlanWriter.Snapshot(vessel).Count;
                    double burnTime = Universe.GetElapsedSimTime().Seconds()
                        + Required(step.OffsetSeconds, "offsetSeconds");
                    string verdict = BurnPlanWriter.TryAdd(vessel, burnTime,
                        PlannerKernel.ComposeVlf(step.Prograde ?? 0, step.Normal ?? 0,
                            step.Outward ?? 0));
                    Require(verdict == "queued", $"burn was not queued: {verdict}");
                    _stepIssued = true;
                    return;
                }
                int count = BurnPlanWriter.Snapshot(vessel).Count;
                if (count <= _baselineBurnCount) return;
                Pass(step, $"stock accepted burn; burn count is {count}");
                break;
            }

            case "plan-lunar-transfer":
                PlanLunarTransfer(step);
                break;

            case "plan-lunar-orbit-insertion":
                PlanLunarOrbitInsertion(step);
                break;

            case "warp-for":
            {
                double duration = Required(step.DurationSeconds, "durationSeconds");
                double speed = Required(step.Speed, "speed");
                Require(duration >= 0, "durationSeconds must be non-negative");
                Require(speed > 0 && double.IsFinite(speed), "speed must be finite and positive");
                if (!_stepIssued)
                {
                    _stepStartedSimTime = Universe.GetElapsedSimTime().Seconds();
                    Universe.SetSimulationSpeed(speed);
                    _stepIssued = true;
                    return;
                }
                double elapsed = Universe.GetElapsedSimTime().Seconds() - _stepStartedSimTime;
                if (elapsed + 1e-6 < duration) return;
                Universe.SetSimulationSpeed(1.0);
                Pass(step, $"advanced {elapsed:F1} simulation seconds at requested {speed:R}x");
                break;
            }

            case "warp-nrho-apolune":
                WarpToNrhoApolune(step);
                break;

            case "assert-nrho-tracking":
                AssertNrhoTracking(step);
                break;

            case "warp-nrho-perilune":
                WarpToNrhoPerilune(step);
                break;

            case "add-nrho-station-keeping-burn":
                AddNrhoStationKeepingBurn(step);
                break;

            case "execute-burns":
                ExecuteBurns(step);
                break;

            case "wait-outbound-lunar-encounter":
                WaitForOutboundLunarEncounter(step);
                break;

            case "assert-bound-lunar-orbit":
                AssertBoundLunarOrbit(step);
                break;

            case "execute-burns-rcs":
                ExecuteBurnsWithRcs(step);
                break;

            case "pause":
                Universe.SetSimulationSpeed(0.0);
                Pass(step, "simulation paused");
                break;

            case "assert-parent":
            case "wait-parent":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                string expected = Required(step.Expected, "expected");
                string actual = (vessel.Orbit.Parent as Astronomical)?.Id ?? "<none>";
                if (action == "wait-parent" && actual != expected)
                {
                    if (!_stepIssued && step.Speed is { } speed)
                    {
                        Require(speed > 0 && double.IsFinite(speed),
                            "speed must be finite and positive");
                        Universe.SetSimulationSpeed(speed);
                        _stepIssued = true;
                    }
                    return;
                }
                if (action == "wait-parent" && _stepIssued)
                    Universe.SetSimulationSpeed(1.0);
                Require(actual == expected, $"expected parent '{expected}', got '{actual}'");
                Pass(step, $"'{vessel.Id}' parent is '{actual}'");
                break;
            }

            case "assert-burn-count":
            case "wait-burn-count":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                int expected = step.Count ?? throw new InvalidDataException("count is required");
                int actual = BurnPlanWriter.Snapshot(vessel).Count;
                if (action == "wait-burn-count" && actual != expected) return;
                Require(actual == expected, $"expected {expected} burns, got {actual}");
                Pass(step, $"'{vessel.Id}' has {actual} burn(s)");
                break;
            }

            case "assert-distance-to-target":
            {
                Vehicle vessel = ResolveVehicle(step.Vessel);
                Vehicle target = ResolveVehicle(step.Target ?? vessel.Target?.Id
                    ?? throw new InvalidDataException("target is required"));
                string? vesselParent = (vessel.Orbit.Parent as Astronomical)?.Id;
                string? targetParent = (target.Orbit.Parent as Astronomical)?.Id;
                Require(vesselParent == targetParent,
                    $"vessels have different parents ('{vesselParent}', '{targetParent}')");
                var delta = vessel.Orbit.StateVectors.PositionCci
                    - target.Orbit.StateVectors.PositionCci;
                double distance = Math.Sqrt(delta.X * delta.X + delta.Y * delta.Y + delta.Z * delta.Z);
                double maximum = Required(step.MaxDistanceMeters, "maxDistanceMeters");
                Require(distance <= maximum,
                    $"distance {distance:R} m exceeds {maximum:R} m");
                Pass(step, $"distance to '{target.Id}' is {distance:F1} m");
                break;
            }

            default:
                throw new InvalidDataException($"unknown action '{step.Action}'");
        }
    }

    private static void PlanLunarTransfer(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double offset = Required(step.OffsetSeconds, "offsetSeconds");
        double duration = Required(step.DurationSeconds, "flightDurationSeconds");
        double targetRadius = Required(step.TargetRadiusMeters,
            "targetLunarRadiusMeters");
        Require(offset >= PlannerKernel.MinLeadSeconds && duration > 0,
            "lunar transfer needs a future departure and positive flight duration");

        if (_lunarTransferJob is null)
        {
            Require(BurnPlanWriter.Snapshot(vessel).Count == 0,
                "lunar transfer planning requires an empty maneuver plan");
            double departure = Universe.GetElapsedSimTime().Seconds() + offset;
            RailsService rails = ModServices.Rails
                ?? throw new InvalidOperationException("rails service is unavailable");
            double lunaRadius = rails.MeanRadiusOf("Luna");
            Require(targetRadius > lunaRadius + 100_000,
                "lunar transfer target must clear Luna by at least 100 km");
            VesselRegistry registry = ModServices.Vessels
                ?? throw new InvalidOperationException("vessel registry is unavailable");
            if (!registry.TryReadAuthoritativePredictorState(
                    vessel, departure, out StateVector departureState,
                    out var authorityReason))
                throw new InvalidOperationException(
                    $"'{vessel.Id}' departure state is unavailable: "
                    + PredictorAuthorityPolicy.Describe(authorityReason));
            double earthMu = rails.MuOf("Earth");
            double departureSearchDuration = RendezvousKernel.OrbitalPeriod(
                departureState - rails.GetAbsolute("Earth", departure), earthMu);
            Require(departureSearchDuration > 0
                    && double.IsFinite(departureSearchDuration),
                "parking orbit has no finite departure-search period");
            double predictionEnd = departure + departureSearchDuration + duration;
            RailsService.PredictionContext? prediction =
                rails.TryCaptureSolverPredictionContext(departure, predictionEnd);
            if (prediction is null)
                return;

            _baselineBurnCount = 0;
            _lunarTransferJob = new GameTestLunarTransferSolveJob
            {
                Prediction = prediction,
                DepartureState = departureState,
                DepartureTime = departure,
                DepartureSearchDuration = departureSearchDuration,
                FlightDuration = duration,
                EarthMu = earthMu,
                TargetLunarRadiusMeters = targetRadius,
                EarthClearanceRadiusMeters = rails.MeanRadiusOf("Earth") + 100_000,
                LunaClearanceRadiusMeters = lunaRadius + 100_000,
            };
            _lunarTransferJob.Start();
            return;
        }

        if (!_lunarTransferJob.Done)
            return;
        if (_lunarTransferJob.Result is not { } solution)
            throw new InvalidOperationException(
                _lunarTransferJob.Failure ?? "lunar transfer solve produced no result");
        if (!_stepIssued)
        {
            string verdict = BurnPlanWriter.TryAdd(vessel, solution.DepartureTime,
                PlannerKernel.ComposeVlf(
                    solution.DeltaVVlf.X, solution.DeltaVVlf.Y,
                    solution.DeltaVVlf.Z));
            Require(verdict == "queued",
                $"lunar transfer burn was not queued: {verdict}");
            _stepIssued = true;
            return;
        }
        int count = BurnPlanWriter.Snapshot(vessel).Count;
        if (count <= _baselineBurnCount)
            return;
        Pass(step, $"queued n-body lunar transfer; predicted closest approach "
            + $"{solution.MissDistanceMeters:F1} m, "
            + $"delta-v {solution.DeltaVVlf.Length():F2} m/s");
    }

    private static void PlanLunarOrbitInsertion(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double targetApolune = Required(step.TargetRadiusMeters,
            "targetApoluneRadiusMeters");
        Require((vessel.Orbit.Parent as Astronomical)?.Id == "Luna",
            "lunar orbit insertion requires Luna SOI authority");

        if (!_stepIssued)
        {
            Require(BurnPlanWriter.Snapshot(vessel).Count == 0,
                "lunar orbit insertion requires an empty maneuver plan");
            double now = Universe.GetElapsedSimTime().Seconds();
            double burnTime = vessel.Orbit.TimeAtPeriapsis.Seconds();
            Require(burnTime >= now + PlannerKernel.MinLeadSeconds,
                $"lunar periapsis t={burnTime:F1} is too close or already past at t={now:F1}");
            StateVectors periapsis = vessel.Orbit.GetStateVectorsAt(new SimTime(burnTime));
            double radius = periapsis.PositionCci.Length();
            double incomingSpeed = periapsis.VelocityCci.Length();
            RailsService rails = ModServices.Rails
                ?? throw new InvalidOperationException("rails service is unavailable");
            double safeRadius = rails.MeanRadiusOf("Luna") + 100_000;
            Require(radius > safeRadius && targetApolune > radius,
                $"lunar capture geometry is invalid: periapsis {radius:R} m, "
                + $"target apolune {targetApolune:R} m");
            double mu = rails.MuOf("Luna");
            double targetSpeed = Math.Sqrt(mu
                * (2 / radius - 2 / (radius + targetApolune)));
            double deltaV = targetSpeed - incomingSpeed;
            Require(deltaV < -1 && deltaV >= -2_000,
                $"lunar insertion delta-v {deltaV:R} m/s is not a capture burn");
            _baselineBurnCount = 0;
            string verdict = BurnPlanWriter.TryAdd(vessel, burnTime,
                PlannerKernel.ComposeVlf(deltaV, 0, 0));
            Require(verdict == "queued",
                $"lunar insertion burn was not queued: {verdict}");
            _stepIssued = true;
            return;
        }

        int count = BurnPlanWriter.Snapshot(vessel).Count;
        if (count <= _baselineBurnCount)
            return;
        Pass(step, $"queued lunar insertion at osculating periapsis; "
            + $"target apolune {targetApolune / 1000:F0} km");
    }

    private static void WaitForOutboundLunarEncounter(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double speed = Required(step.Speed, "warpSpeed");
        Require(speed > 0 && double.IsFinite(speed),
            "warpSpeed must be finite and positive");
        if ((vessel.Orbit.Parent as Astronomical)?.Id != "Luna")
        {
            if (!_stepIssued)
            {
                Universe.SetSimulationSpeed(speed);
                _stepIssued = true;
            }
            return;
        }

        if (_stepIssued)
            Universe.SetSimulationSpeed(1);
        StateVector earthRelative = VesselRelativeToBody(vessel, "Earth");
        StateVector lunaRelative = VesselRelativeToLuna(vessel);
        double earthRadialVelocity = earthRelative.Position.Dot(
            earthRelative.Velocity) / earthRelative.Position.Length();
        double lunaRadialVelocity = lunaRelative.Position.Dot(
            lunaRelative.Velocity) / lunaRelative.Position.Length();
        Require(earthRadialVelocity > 0,
            $"Luna SOI was entered on an Earth-return leg "
            + $"({earthRadialVelocity:R} m/s radial)");
        Require(lunaRadialVelocity < 0,
            $"Luna SOI entry is not approaching Luna "
            + $"({lunaRadialVelocity:R} m/s radial)");
        Pass(step, $"entered Luna SOI outbound from Earth and approaching Luna; "
            + $"radial velocities {earthRadialVelocity:F1}/{lunaRadialVelocity:F1} m/s");
    }

    private static void AssertBoundLunarOrbit(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        Require((vessel.Orbit.Parent as Astronomical)?.Id == "Luna",
            $"expected Luna parent, got '{(vessel.Orbit.Parent as Astronomical)?.Id ?? "<none>"}'");
        double maximumApolune = Required(step.MaxApoluneRadiusMeters,
            "maxApoluneRadiusMeters");
        StateVector state = VesselRelativeToLuna(vessel);
        double mu = ModServices.Rails!.MuOf("Luna");
        double radius = state.Position.Length();
        double energy = state.Velocity.LengthSquared() / 2 - mu / radius;
        Require(energy < 0 && double.IsFinite(energy),
            $"lunar orbit is not bound; specific energy {energy:R} J/kg");
        Vector3d angularMomentum = state.Position.Cross(state.Velocity);
        double eccentricity = (state.Velocity.Cross(angularMomentum) / mu
            - state.Position / radius).Length();
        double semiMajorAxis = -mu / (2 * energy);
        double perilune = semiMajorAxis * (1 - eccentricity);
        double apolune = semiMajorAxis * (1 + eccentricity);
        double safePerilune = ModServices.Rails.MeanRadiusOf("Luna") + 100_000;
        Require(perilune >= safePerilune,
            $"bound lunar orbit intersects Luna: perilune {perilune:R} m, "
            + $"required {safePerilune:R} m");
        Require(apolune <= maximumApolune,
            $"bound lunar orbit apolune {apolune:R} m exceeds {maximumApolune:R} m");
        Pass(step, $"bound lunar orbit: Pe {perilune / 1000:F0} km, "
            + $"Ap {apolune / 1000:F0} km, e={eccentricity:F4}");
    }

    private static void AssertNrhoTracking(GameTestStep step)
    {
        Require(double.IsFinite(_nrhoEpoch) && _nrhoTimeUnitSeconds > 0,
            "the NRHO reference must be initialized before checking tracking");
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double maximumPosition = Required(step.MaxPositionErrorMeters,
            "maxPositionErrorMeters");
        double maximumVelocity = Required(step.MaxVelocityErrorMetersPerSecond,
            "maxVelocityErrorMetersPerSecond");
        Require(maximumPosition > 0 && maximumVelocity > 0,
            "NRHO tracking tolerances must be positive");
        string phase = Required(step.Expected, "phase");
        double normalizedPhase = phase switch
        {
            "perilune" => 0,
            "apolune" => GameTestNrhoKernel.PeriodNormalized / 2,
            _ => throw new InvalidDataException($"unknown NRHO phase '{phase}'"),
        };
        double time = vessel.Orbit.StateVectors.StateTime.Seconds();
        StateVector current = VesselRelativeToLuna(vessel);
        StateVector target = NrhoReferenceRelativeToLuna(time, normalizedPhase);
        double positionError = (target.Position - current.Position).Length();
        double velocityError = (target.Velocity - current.Velocity).Length();
        Require(positionError <= maximumPosition,
            $"NRHO position error {positionError:R} m exceeds {maximumPosition:R} m");
        Require(velocityError <= maximumVelocity,
            $"NRHO velocity error {velocityError:R} m/s exceeds {maximumVelocity:R} m/s");
        Pass(step, $"NRHO reference error {positionError / 1000:F1} km, "
            + $"{velocityError:F3} m/s at {phase}");
    }

    private static void TeleportToNrhoApproach(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double insertionDeltaV = Required(step.DeltaVMetersPerSecond,
            "insertionDeltaVMetersPerSecond");
        Require(insertionDeltaV > 0,
            "insertionDeltaVMetersPerSecond must be positive");

        if (!_stepIssued)
        {
            double now = Universe.GetElapsedSimTime().Seconds();
            ResetNrhoReferenceEpoch(now);

            StateVector target = NrhoReferenceRelativeToLuna(now);
            double targetSpeed = target.Velocity.Length();
            Require(targetSpeed > 0 && double.IsFinite(targetSpeed),
                "NRHO reference has no finite perilune velocity");
            Vector3d approachVelocity = target.Velocity
                + target.Velocity / targetSpeed * insertionDeltaV;

            if (Universe.CurrentSystem?.Get("Luna") is not Astronomical lunaBody
                || lunaBody is not IParentBody luna)
                throw new InvalidOperationException("Luna is not an orbitable body");
            var orbit = Orbit.CreateFromStateCci(luna, new SimTime(now),
                FrameAdapter.EclToCci(target.Position, luna.GetCce2Cci()),
                FrameAdapter.EclToCci(approachVelocity, luna.GetCce2Cci()),
                vessel.Orbit.OrbitLineColor);
            InputEvents.TeleportInputBuffer.Add(new InputEvents.TeleportInputData
            {
                Vehicle = vessel,
                Orbit = orbit,
                Body2Cce = null,
                BodyRates = null,
            });
            _stepIssued = true;
            return;
        }

        if ((vessel.Orbit.Parent as Astronomical)?.Id != "Luna") return;
        double radius = VesselRelativeToLuna(vessel).Position.Length();
        if (!(radius > 0) || radius > 10_000_000) return;
        Pass(step, $"'{vessel.Id}' is on the NRHO approach at lunar radius "
            + $"{radius / 1000:F0} km; insertion is {insertionDeltaV:F2} m/s");
    }

    private static void WarpToNrhoApolune(GameTestStep step)
    {
        Require(double.IsFinite(_nrhoEpoch) && _nrhoTimeUnitSeconds > 0,
            "the NRHO approach must be initialized before warping to apolune");
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double speed = Required(step.Speed, "speed");
        double minApolune = Required(step.MinApoluneRadiusMeters,
            "minApoluneRadiusMeters");
        double maxApolune = Required(step.MaxApoluneRadiusMeters,
            "maxApoluneRadiusMeters");
        Require(speed > 0 && minApolune > 0 && maxApolune >= minApolune,
            "speed and apolune radius corridor must be positive and ordered");

        if (!_stepIssued)
        {
            _stepStartedSimTime = Universe.GetElapsedSimTime().Seconds();
            double radius = VesselRelativeToLuna(vessel).Position.Length();
            _nrhoMaxRadius = radius;
            _nrhoPreviousRadius = radius;
            _nrhoApoluneReached = false;
            Universe.SetSimulationSpeed(speed);
            _stepIssued = true;
            return;
        }

        double currentRadius = VesselRelativeToLuna(vessel).Position.Length();
        if (double.IsFinite(currentRadius))
            _nrhoMaxRadius = Math.Max(_nrhoMaxRadius, currentRadius);
        double elapsed = Universe.GetElapsedSimTime().Seconds() - _stepStartedSimTime;
        if (currentRadius >= minApolune) _nrhoApoluneReached = true;
        bool passedApolune = _nrhoApoluneReached && currentRadius < _nrhoPreviousRadius;
        _nrhoPreviousRadius = currentRadius;
        if (!passedApolune) return;

        Universe.SetSimulationSpeed(1.0);
        Require(_nrhoMaxRadius >= minApolune,
            $"NRHO missed distant lobe: maximum radius {_nrhoMaxRadius:R} m is below {minApolune:R} m");
        Require(_nrhoMaxRadius <= maxApolune,
            $"NRHO escaped distant lobe: maximum radius {_nrhoMaxRadius:R} m "
            + $"exceeds {maxApolune:R} m");
        ResetNrhoReferenceEpoch(Universe.GetElapsedSimTime().Seconds(),
            GameTestNrhoKernel.PeriodNormalized / 2);
        Pass(step, $"reached NRHO apolune after {elapsed / 86400:F3} d at "
            + $"{_nrhoMaxRadius / 1000:F0} km lunar radius");
    }

    private static void WarpToNrhoPerilune(GameTestStep step)
    {
        Require(double.IsFinite(_nrhoEpoch) && _nrhoTimeUnitSeconds > 0,
            "the NRHO reference must be initialized before warping to perilune");
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double speed = Required(step.Speed, "speed");
        double maxPerilune = Required(step.MaxPeriluneRadiusMeters,
            "maxPeriluneRadiusMeters");
        Require(speed > 0 && maxPerilune > 0,
            "speed and maximum perilune radius must be positive");

        if (!_stepIssued)
        {
            _stepStartedSimTime = Universe.GetElapsedSimTime().Seconds();
            double radius = VesselRelativeToLuna(vessel).Position.Length();
            _nrhoMinRadius = radius;
            _nrhoPreviousRadius = radius;
            _nrhoPeriluneApproachSlowed = false;
            Universe.SetSimulationSpeed(speed);
            _stepIssued = true;
            return;
        }

        double currentRadius = VesselRelativeToLuna(vessel).Position.Length();
        if (double.IsFinite(currentRadius))
            _nrhoMinRadius = Math.Min(_nrhoMinRadius, currentRadius);
        if (!_nrhoPeriluneApproachSlowed && currentRadius <= 2 * maxPerilune)
        {
            Universe.SetSimulationSpeed(Math.Min(speed, 1000.0));
            _nrhoPeriluneApproachSlowed = true;
        }
        bool passedPerilune = _nrhoPeriluneApproachSlowed
            && _nrhoPreviousRadius <= maxPerilune
            && currentRadius > _nrhoPreviousRadius;
        _nrhoPreviousRadius = currentRadius;
        if (!passedPerilune) return;

        double elapsed = Universe.GetElapsedSimTime().Seconds() - _stepStartedSimTime;
        Universe.SetSimulationSpeed(1.0);
        double safePerilune = ModServices.Rails!.MeanRadiusOf("Luna") + 100_000;
        Require(_nrhoMinRadius >= safePerilune,
            $"NRHO close passage intersects Luna: minimum radius {_nrhoMinRadius:R} m "
            + $"is below {safePerilune:R} m");
        Require(_nrhoMinRadius <= maxPerilune,
            $"NRHO missed close lunar passage: minimum radius {_nrhoMinRadius:R} m exceeds {maxPerilune:R} m");
        Pass(step, $"returned to NRHO perilune after {elapsed / 86400:F3} d at "
            + $"{_nrhoMinRadius / 1000:F0} km lunar radius");
    }

    private static void AddNrhoStationKeepingBurn(GameTestStep step)
    {
        Require(double.IsFinite(_nrhoEpoch) && _nrhoTimeUnitSeconds > 0,
            "the NRHO approach must be initialized before station keeping");
        Vehicle vessel = ResolveVehicle(step.Vessel);
        double offset = Required(step.OffsetSeconds, "offsetSeconds");
        double timescale = Required(step.CorrectionTimescaleSeconds,
            "correctionTimescaleSeconds");
        double maximum = Required(step.MaxDeltaVMetersPerSecond,
            "maxDeltaVMetersPerSecond");
        Require(offset >= PlannerKernel.MinLeadSeconds,
            "station-keeping burn needs at least the planner minimum lead");
        Require(timescale > 0 && maximum > 0,
            "station-keeping timescale and delta-v bound must be positive");

        if (!_stepIssued)
        {
            _baselineBurnCount = BurnPlanWriter.Snapshot(vessel).Count;
            Require(_baselineBurnCount == 0,
                "station keeping requires an empty maneuver plan");
            double burnTime = Universe.GetElapsedSimTime().Seconds() + offset;
            VesselRegistry registry = ModServices.Vessels
                ?? throw new InvalidOperationException("vessel registry is unavailable");
            StateVector luna = ModServices.Rails!.GetAbsolute("Luna", burnTime);
            if (!registry.TryReadAuthoritativePredictorState(
                    vessel, burnTime, out StateVector absolute, out var authorityReason))
                throw new InvalidOperationException(
                    $"'{vessel.Id}' predictor is unavailable: "
                    + PredictorAuthorityPolicy.Describe(authorityReason));
            StateVector current = absolute - luna;
            StateVector target = NrhoReferenceRelativeToLuna(burnTime);
            Vector3d correction = GameTestNrhoKernel.Feedback(current, target, timescale);
            _nrhoCorrectionMagnitude = correction.Length();
            double positionError = (target.Position - current.Position).Length();
            double velocityError = (target.Velocity - current.Velocity).Length();
            Require(_nrhoCorrectionMagnitude > 1e-6
                    && _nrhoCorrectionMagnitude <= maximum,
                $"NRHO correction {_nrhoCorrectionMagnitude:R} m/s is outside (0, {maximum:R}]; "
                + $"position error {positionError:R} m, velocity error {velocityError:R} m/s, "
                + $"current speed {current.Velocity.Length():R} m/s, "
                + $"reference speed {target.Velocity.Length():R} m/s");
            var vlf = GameTestNrhoKernel.ToVlf(current, correction);
            string verdict = BurnPlanWriter.TryAdd(vessel, burnTime,
                PlannerKernel.ComposeVlf(vlf.Prograde, vlf.Normal, vlf.Outward));
            Require(verdict == "queued",
                $"station-keeping burn was not queued: {verdict}");
            _stepIssued = true;
            return;
        }

        int count = BurnPlanWriter.Snapshot(vessel).Count;
        if (count <= _baselineBurnCount) return;
        Pass(step, $"queued NRHO feedback correction of {_nrhoCorrectionMagnitude:F3} m/s");
    }

    private static StateVector NrhoReferenceRelativeToLuna(double time,
        double? normalizedPhase = null)
    {
        Require(double.IsFinite(_nrhoEpoch) && _nrhoTimeUnitSeconds > 0,
            "NRHO reference epoch is unavailable");
        const double poseStep = 1.0;
        FramePose before = SampleEarthLunaPose(time - poseStep);
        FramePose at = SampleEarthLunaPose(time);
        FramePose after = SampleEarthLunaPose(time + poseStep);
        double normalizedTime = normalizedPhase
            ?? (time - _nrhoEpoch) / _nrhoTimeUnitSeconds;
        StateVector reference = GameTestNrhoKernel.Embed(normalizedTime,
            _nrhoTimeUnitSeconds, before, at, after, poseStep);
        StateVector luna = ModServices.Rails!.GetGameEcl("Luna", time);
        return reference - luna;
    }

    private static void ResetNrhoReferenceEpoch(double time,
        double normalizedPhase = 0)
    {
        const double poseStep = 1.0;
        FramePose before = SampleEarthLunaPose(time - poseStep);
        FramePose after = SampleEarthLunaPose(time + poseStep);
        double angularRate = (after.XAxis - before.XAxis).Length() / (2 * poseStep);
        Require(angularRate > 0 && double.IsFinite(angularRate),
            "Earth-Luna frame has no finite angular rate");
        _nrhoTimeUnitSeconds = 1.0 / angularRate;
        _nrhoEpoch = time - normalizedPhase * _nrhoTimeUnitSeconds;
    }

    private static FramePose SampleEarthLunaPose(double time)
    {
        var frame = new FrameSpec(FrameKind.TwoBodyFixed, "Earth", "Luna");
        string? reason = FrameManager.SampleSpecPose(frame, time, out FramePose pose);
        if (reason is not null)
            throw new InvalidOperationException($"Earth-Luna frame at t={time:F1} s: {reason}");
        return pose;
    }

    private static StateVector VesselRelativeToLuna(Vehicle vessel) =>
        VesselRelativeToBody(vessel, "Luna");

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

    private static ConsumableStatus ReadConsumableStatus(Vehicle vessel)
    {
        var status = new ConsumableStatus();
        var cores = vessel.Parts.RocketCores;
        foreach (RocketCore core in cores.Modules)
        {
            ref readonly RocketCoreState coreState = ref cores.GetState(core);
            bool fueled = core.ComputePropellantAvailable(
                    vessel.Parts.Moles.States, coreState.Throttle > 0f)
                && coreState.IsPropellantAvailable;
            if (core.Controller is EngineController engine && engine.IsActive)
            {
                status.ActiveEngineCores++;
                if (fueled) status.FueledEngineCores++;
            }
            else if (core.Controller is ThrusterController thruster && thruster.IsActive)
            {
                status.ActiveRcsCores++;
                if (fueled) status.FueledRcsCores++;
            }
        }
        return status;
    }

    private struct ConsumableStatus
    {
        internal int ActiveEngineCores;
        internal int FueledEngineCores;
        internal int ActiveRcsCores;
        internal int FueledRcsCores;

        internal readonly bool Ready => ActiveEngineCores > 0
            && FueledEngineCores == ActiveEngineCores
            && ActiveRcsCores > 0
            && FueledRcsCores == ActiveRcsCores;

        public override readonly string ToString() =>
            $"main engines {FueledEngineCores}/{ActiveEngineCores}, "
            + $"RCS {FueledRcsCores}/{ActiveRcsCores}";
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
        _lunarTransferJob = null;
        _stepStartedSimTime = 0;
        _burnExecutionStage = 0;
        _burnExecutionBaselineCount = 0;
        _burnsExecuted = 0;
        _burnTargetMagnitude = 0;
        _burnWarpActive = false;
        _refillFrames = 0;
        _rcsBurnInitialResidual = 0;
        _rcsBurnInitialCompletionDot = 0;
        _rcsBurnExpectedDuration = 0;
        Volatile.Write(ref _rcsBurnCutoffArmed, false);
        _rcsBurnCutoffSimTime = double.NaN;
        Volatile.Write(ref _rcsBurnCutoffReached, false);
        Volatile.Write(ref _rcsBurnVehicle, null);
        Interlocked.Exchange(ref _rcsBurnReleaseIssued, 0);
        _nrhoMinRadius = 0;
        _nrhoMaxRadius = 0;
        _nrhoCorrectionMagnitude = 0;
        _nrhoPreviousRadius = 0;
        _nrhoApoluneReached = false;
        _nrhoPeriluneApproachSlowed = false;
    }

    private static void Fail(string error)
    {
        if (_finished) return;
        try { DisarmRcsBurnCutoff(); }
        catch { }
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

    private static void DisarmRcsBurnCutoff()
    {
        Volatile.Write(ref _rcsBurnCutoffArmed, false);
        Vehicle? vessel = Volatile.Read(ref _rcsBurnVehicle);
        try
        {
            if (vessel is not null)
                vessel.ProcessInput(
                    InputAction.TranslateForward, GlfwKeyAction.Release, default);
        }
        finally
        {
            _rcsBurnCutoffSimTime = double.NaN;
            Volatile.Write(ref _rcsBurnCutoffReached, false);
            Volatile.Write(ref _rcsBurnVehicle, null);
            Interlocked.Exchange(ref _rcsBurnReleaseIssued, 0);
        }
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

    private static void ExecuteBurnsWithRcs(GameTestStep step)
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
                Pass(step, $"executed {_burnsExecuted} burn(s) with forward RCS; main engines remained off");
                return;
            }
            if (computer.Burn is not { } target) return;
            _burnExecutionBaselineCount = burns.Count;
            _burnTargetMagnitude = target.DeltaVTargetCci.Length();
            Require(_burnTargetMagnitude > 0 && float.IsFinite(_burnTargetMagnitude),
                "next burn has no finite delta-v target");
            ConsumableStatus status = ReadConsumableStatus(vessel);
            Require(status.ActiveRcsCores > 0
                    && status.FueledRcsCores == status.ActiveRcsCores,
                $"RCS burn requires fueled active RCS: {status}");
            QueueFlightComputer(vessel, FlightComputerBurnMode.Manual);
            QueueFlightComputer(vessel, VehicleEngine.MainShutdown);
            QueueFlightComputer(vessel, FlightComputerManualThrustMode.Direct);
            QueueFlightComputer(vessel, FlightComputerAttitudeTrackTarget.PositiveDv);
            _refillFrames = 0;
            _burnExecutionStage = 1;
            return;
        }

        if (_burnExecutionStage == 1)
        {
            Require(MaxActiveMainEngineThrottle(vessel) <= 0,
                "main engine fired while RCS was orienting the vessel");
            BurnTarget target = computer.Burn
                ?? throw new InvalidOperationException("RCS burn target disappeared");
            if (RefillDepletedRcs(vessel))
            {
                _refillFrames = 0;
                return;
            }
            if (_refillFrames++ < 2
                || computer.AttitudeMode != FlightComputerAttitudeMode.Auto
                || computer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.PositiveDv
                || !IsRcsAlignedForBurn(vessel, target, computer))
                return;

            QueueFlightComputer(vessel, FlightComputerAttitudeMode.Manual);
            InputEvents.VehicleResourcesChangeBuffer.Add(
                new InputEvents.VehicleResourcesChangeData
                {
                    Vehicle = vessel,
                    Refill = true,
                });
            _refillFrames = 0;
            _burnExecutionStage = 5;
            return;
        }

        if (_burnExecutionStage == 5)
        {
            Require(MaxActiveMainEngineThrottle(vessel) <= 0,
                "main engine fired after RCS orientation");
            Require(computer.AttitudeMode == FlightComputerAttitudeMode.Manual,
                "RCS attitude control was not parked before warp");
            if (_refillFrames++ == 0) return;
            ConsumableStatus status = ReadConsumableStatus(vessel);
            Require(status.ActiveRcsCores > 0
                    && status.FueledRcsCores == status.ActiveRcsCores,
                $"post-orientation RCS refill failed: {status}");

            EngineScalars rcs = TrajectoryOverlay.ReadEngineScalars(
                vessel, PropulsionSource.RcsForward);
            Require(FiniteBurnKernel.TryGetPhysicalWindow(
                    burns[0].Time.Seconds(), _burnTargetMagnitude, rcs,
                    out FiniteBurnWindow window),
                "forward RCS has no finite burn window");
            double now = Universe.GetElapsedSimTime().Seconds();
            _rcsBurnExpectedDuration = window.DurationSeconds;
            Require(window.IgnitionSeconds > now,
                $"RCS ignition t={window.IgnitionSeconds:F1} is no longer in the future (now {now:F1})");
            ModLog.Info($"game test: RCS aligned and replenished for "
                + $"{_burnTargetMagnitude:F3} m/s burn; ignition t={window.IgnitionSeconds:F1}, "
                + $"duration={window.DurationSeconds:F2} s");
            Universe.AutoWarpTo(new SimTime(window.IgnitionSeconds));
            _burnExecutionStage = 2;
            return;
        }

        if (_burnExecutionStage == 2)
        {
            Require(MaxActiveMainEngineThrottle(vessel) <= 0,
                "main engine fired while warping to an RCS burn");
            Require(MaxActiveRcsCoreThrottle(vessel) <= 0,
                "RCS fired while attitude control was parked for warp");
            if (Universe.IsAutoWarpActive) return;
            Universe.SetSimulationSpeed(1.0);
            QueueFlightComputer(vessel, FlightComputerAttitudeTrackTarget.PositiveDv);
            _refillFrames = 0;
            _burnExecutionStage = 7;
            return;
        }

        if (_burnExecutionStage == 7)
        {
            Require(MaxActiveMainEngineThrottle(vessel) <= 0,
                "main engine fired while restoring RCS burn alignment");
            BurnTarget target = computer.Burn
                ?? throw new InvalidOperationException("RCS burn target disappeared");
            if (RefillDepletedRcs(vessel))
            {
                _refillFrames = 0;
                return;
            }
            if (_refillFrames++ < 2
                || computer.AttitudeMode != FlightComputerAttitudeMode.Auto
                || computer.AttitudeTrackTarget != FlightComputerAttitudeTrackTarget.PositiveDv
                || !IsRcsAlignedForBurn(vessel, target, computer))
                return;

            InputEvents.VehicleResourcesChangeBuffer.Add(
                new InputEvents.VehicleResourcesChangeData
                {
                    Vehicle = vessel,
                    Refill = true,
                });
            _refillFrames = 0;
            _burnExecutionStage = 6;
            return;
        }

        if (_burnExecutionStage == 6)
        {
            Require(MaxActiveMainEngineThrottle(vessel) <= 0,
                "main engine fired before forward-RCS burn");
            Require(computer.AttitudeMode == FlightComputerAttitudeMode.Auto,
                "RCS attitude tracking was not active before translation");
            if (_refillFrames++ == 0) return;
            ForwardRcsStatus status = ReadForwardRcsStatus(vessel);
            Require(status.Ready,
                $"pre-burn RCS refill failed: {status}; {ReadConsumableStatus(vessel)}");
            BurnTarget target = computer.Burn
                ?? throw new InvalidOperationException("RCS burn target disappeared");
            Require(IsRcsAlignedForBurn(vessel, target, computer),
                $"vessel lost RCS burn alignment during refill: "
                + $"errors=({computer.ErrorAngles.Y:F4}, {computer.ErrorAngles.Z:F4}) rad");
            _rcsBurnInitialCompletionDot = float3.Dot(
                target.DeltaVToGoCci, target.DeltaVTargetCci);
            _rcsBurnInitialCompletionDot =
                Math.Abs(_rcsBurnInitialCompletionDot);
            _rcsBurnInitialResidual = target.DeltaVToGoCci.Length();
            Require(_rcsBurnInitialCompletionDot > 0,
                "forward-RCS burn was already complete before translation");
            _stepStartedSimTime = Universe.GetElapsedSimTime().Seconds();
            _rcsBurnCutoffSimTime = double.NaN;
            Volatile.Write(ref _rcsBurnCutoffReached, false);
            Volatile.Write(ref _rcsBurnVehicle, vessel);
            Interlocked.Exchange(ref _rcsBurnReleaseIssued, 0);
            Volatile.Write(ref _rcsBurnCutoffArmed, true);
            vessel.ProcessInput(
                InputAction.TranslateForward, GlfwKeyAction.Press, default);
            Require((vessel.GetThrusterFlags() & ThrusterMapFlags.TranslateForward) != 0,
                "stock forward-RCS input did not latch");
            _burnExecutionStage = 3;
            return;
        }

        if (_burnExecutionStage == 3)
        {
            BurnTarget target = computer.Burn
                ?? throw new InvalidOperationException("RCS burn target disappeared");
            Require(MaxActiveMainEngineThrottle(vessel) <= 0,
                "main engine fired during forward-RCS burn");
            float completionDot = float3.Dot(target.DeltaVToGoCci,
                target.DeltaVTargetCci);
            double burnElapsed = Universe.GetElapsedSimTime().Seconds()
                - _stepStartedSimTime;
            if (!Volatile.Read(ref _rcsBurnCutoffReached)) return;
            Require((vessel.GetThrusterFlags() & ThrusterMapFlags.TranslateForward) == 0,
                "forward-RCS input remained latched after the physics cutoff");
            Volatile.Write(ref _rcsBurnCutoffArmed, false);
            Volatile.Write(ref _rcsBurnVehicle, null);
            _rcsBurnCutoffSimTime = double.NaN;
            Volatile.Write(ref _rcsBurnCutoffReached, false);
            Universe.SetSimulationSpeed(1.0);
            float completedResidual = target.DeltaVToGoCci.Length();
            Require(completedResidual < _rcsBurnInitialResidual,
                $"forward-RCS command did not reduce residual "
                + $"{_rcsBurnInitialResidual:F3} -> {completedResidual:F3} m/s");
            Require(completionDot < _rcsBurnInitialCompletionDot,
                $"forward-RCS command did not advance KSA's integrated burn progress: "
                + $"completion dot {_rcsBurnInitialCompletionDot:F4} -> {completionDot:F4}, "
                + DescribeForwardRcsControllers(vessel));
            QueueFlightComputer(vessel, FlightComputerAttitudeMode.Manual);
            InputEvents.VehicleResourcesChangeBuffer.Add(
                new InputEvents.VehicleResourcesChangeData
                {
                    Vehicle = vessel,
                    Refill = true,
                });
            _refillFrames = 0;
            float remaining = target.DeltaVToGoCci.Length();
            Require(remaining <= 0.65f,
                $"forward-RCS burn residual is {remaining:F3} m/s");
            ModLog.Info($"game test: RCS burn completed in {burnElapsed:F2} s; "
                + $"remaining={remaining:F3} m/s, completionDot={completionDot:F4}");
            string verdict = BurnPlanWriter.TryRemove(vessel, burns[0]);
            Require(verdict == "queued", $"completed burn cleanup was not queued: {verdict}");
            _burnExecutionStage = 4;
            return;
        }

        if (burns.Count >= _burnExecutionBaselineCount) return;
        if (_refillFrames++ == 0) return;
        ConsumableStatus replenished = ReadConsumableStatus(vessel);
        Require(replenished.Ready,
            $"post-burn refill failed: {replenished}");
        _burnsExecuted++;
        _burnExecutionStage = 0;
    }

    private static bool RefillDepletedRcs(Vehicle vessel)
    {
        ConsumableStatus status = ReadConsumableStatus(vessel);
        if (status.ActiveRcsCores == 0 || status.FueledRcsCores > 0) return false;
        InputEvents.VehicleResourcesChangeBuffer.Add(
            new InputEvents.VehicleResourcesChangeData
            {
                Vehicle = vessel,
                Refill = true,
            });
        ModLog.Info($"game test: replenishing depleted RCS during attitude slew: {status}");
        return true;
    }

    private static bool IsRcsAlignedForBurn(
        Vehicle vessel, BurnTarget target, FlightComputer computer)
    {
        float magnitude = target.DeltaVToGoCci.Length();
        if (!(magnitude > 0) || !float.IsFinite(magnitude)) return false;
        float3 forwardCci = float3.UnitX.Transform(floatQuat.Pack(vessel.GetBody2Cci()));
        float alignment = float3.Dot(forwardCci, target.DeltaVToGoCci / magnitude);
        float tolerance = Math.Min(computer.AngleDeadband, 0.02f);
        return float.IsFinite(alignment)
            && alignment >= MathF.Cos(tolerance)
            && float.IsFinite(computer.ErrorAngles.Y)
            && float.IsFinite(computer.ErrorAngles.Z)
            && Math.Abs(computer.ErrorAngles.Y) <= tolerance
            && Math.Abs(computer.ErrorAngles.Z) <= tolerance;
    }

    private static float MaxActiveMainEngineThrottle(Vehicle vessel)
    {
        float maximum = 0;
        var cores = vessel.Parts.RocketCores;
        foreach (RocketCore core in cores.Modules)
        {
            if (core.Controller is EngineController engine && engine.IsActive)
                maximum = Math.Max(maximum, cores.GetState(core).Throttle);
        }
        return maximum;
    }

    private static float MaxActiveRcsCoreThrottle(Vehicle vessel)
    {
        float maximum = 0;
        var cores = vessel.Parts.RocketCores;
        foreach (RocketCore core in cores.Modules)
        {
            if (core.Controller is ThrusterController thruster && thruster.IsActive)
                maximum = Math.Max(maximum, cores.GetState(core).Throttle);
        }
        return maximum;
    }

    private static string DescribeForwardRcsControllers(Vehicle vessel)
    {
        if (!vessel.Parts.States.TryGetTypeList<ThrusterController,
                ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>(
                out var states))
            return "RCS controller states unavailable";

        int active = 0;
        int mapped = 0;
        int fueled = 0;
        int mappedFueled = 0;
        int commanded = 0;
        double maximumPulse = 0;
        foreach (ThrusterController thruster in vessel.FlightComputer.VehicleConfig.Thrusters)
        {
            if (!thruster.IsActive) continue;
            active++;
            ref readonly ThrusterControllerState state = ref states.GetState(thruster);
            bool mapsForward = (state.ControlMap & ThrusterMapFlags.TranslateForward) != 0;
            if (mapsForward) mapped++;
            if (state.IsPropellantAvailable) fueled++;
            if (mapsForward && state.IsPropellantAvailable) mappedFueled++;
            if (state.CommandPulseTime > 0)
            {
                commanded++;
                maximumPulse = Math.Max(maximumPulse, state.CommandPulseTime);
            }
        }
        return $"RCS controllers active {active}, forward-mapped {mapped}, "
            + $"forward-fueled {mappedFueled}, fueled total {fueled}, "
            + $"commanded {commanded}, maximum pulse {maximumPulse:R}";
    }

    private static ForwardRcsStatus ReadForwardRcsStatus(Vehicle vessel)
    {
        var status = new ForwardRcsStatus();
        if (!vessel.Parts.States.TryGetTypeList<ThrusterController,
                ThrusterControllerState, ThrusterControllerGlobalState, EmptyStruct>(
                out var states))
            return status;

        var cores = vessel.Parts.RocketCores;
        foreach (ThrusterController thruster in vessel.FlightComputer.VehicleConfig.Thrusters)
        {
            if (!thruster.IsActive) continue;
            ref readonly ThrusterControllerState state = ref states.GetState(thruster);
            if ((state.ControlMap & ThrusterMapFlags.TranslateForward) == 0) continue;
            status.ActiveControllers++;
            if (!state.IsPropellantAvailable) continue;
            foreach (RocketCore core in thruster.Cores)
            {
                ref readonly RocketCoreState coreState = ref cores.GetState(core);
                if (!core.ComputePropellantAvailable(
                        vessel.Parts.Moles.States, coreState.Throttle > 0f)
                    || !coreState.IsPropellantAvailable)
                    continue;
                status.FueledControllers++;
                break;
            }
        }
        return status;
    }

    private struct ForwardRcsStatus
    {
        internal int ActiveControllers;
        internal int FueledControllers;

        internal readonly bool Ready => ActiveControllers > 0
            && FueledControllers == ActiveControllers;

        public override readonly string ToString() =>
            $"forward RCS {FueledControllers}/{ActiveControllers}";
    }

    private static void ExecuteBurns(GameTestStep step)
    {
        Vehicle vessel = ResolveVehicle(step.Vessel);
        FlightComputer computer = vessel.FlightComputer;
        IReadOnlyList<Burn> burns = BurnPlanWriter.Snapshot(vessel);
        double burnWarpSpeed = step.Speed ?? 10.0;
        Require(burnWarpSpeed > 0 && double.IsFinite(burnWarpSpeed),
            "burn warp speed must be finite and positive");

        if (_burnExecutionStage == 0)
        {
            if (burns.Count == 0)
            {
                Require(_burnsExecuted > 0, "there were no burns to execute");
                Universe.SetSimulationSpeed(1.0);
                Pass(step, $"executed {_burnsExecuted} burn(s) through KSA flight computer auto mode "
                    + $"at up to {burnWarpSpeed:R}x");
                return;
            }
            if (computer.Burn is not { } target) return;
            _burnExecutionBaselineCount = burns.Count;
            _burnTargetMagnitude = target.DeltaVTargetCci.Length();
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
            QueueFlightComputer(vessel, FlightComputerAction.WarpToNextBurn);
            _burnExecutionStage = 2;
            return;
        }

        if (_burnExecutionStage == 2)
        {
            if (Universe.IsAutoWarpActive) return;
            if (computer.Burn is not { } target) return;
            if (computer.BurnMode == FlightComputerBurnMode.Auto)
            {
                // Stock auto-warp stops just before ignition. Keep the flight computer
                // in charge of attitude/throttle while advancing the finite burn at the
                // scenario's requested rate; live thrust is still integrated by KSA.
                if (!_burnWarpActive)
                {
                    ModLog.Info($"game test: auto-warp complete; advancing burn at {burnWarpSpeed:R}x");
                    Universe.SetSimulationSpeed(burnWarpSpeed);
                    _burnWarpActive = true;
                }
                return;
            }

            if (_burnWarpActive)
            {
                Universe.SetSimulationSpeed(1.0);
                _burnWarpActive = false;
            }

            float remaining = target.DeltaVToGoCci.Length();
            float completionDot = float3.Dot(target.DeltaVToGoCci,
                target.DeltaVTargetCci);
            ModLog.Info($"game test: automatic burn exited mode={computer.BurnMode}; "
                + $"remaining={remaining:F2} m/s, completionDot={completionDot:F3}");
            Require(float.IsFinite(remaining) && float.IsFinite(completionDot)
                    && completionDot <= 0,
                $"automatic burn stopped with {remaining:F2} m/s remaining of {_burnTargetMagnitude:F2} m/s");
            Require(burns.Count > 0, "completed burn disappeared before cleanup");
            string verdict = BurnPlanWriter.TryRemove(vessel, burns[0]);
            Require(verdict == "queued", $"completed burn cleanup was not queued: {verdict}");
            _burnExecutionStage = 3;
            return;
        }

        if (burns.Count >= _burnExecutionBaselineCount) return;
        _burnsExecuted++;
        _burnExecutionStage = 0;
    }

    internal static void CapRcsBurnCommand(
        FlightComputer computer, SimTime time, ref FlightComputerOutput outputs)
    {
        const double cutoffTolerance = 1e-6;
        if (!Volatile.Read(ref _rcsBurnCutoffArmed)) return;

        double now = time.Seconds();
        double cutoff = _rcsBurnCutoffSimTime;
        var enumerator = outputs.Thrusters
            .GetModulesAndNewStates(computer.VehicleConfig.Thrusters.AsSpan()).GetEnumerator();
        while (enumerator.MoveNext())
        {
            var controller = enumerator.Current;
            if ((controller.State.ControlMap & ThrusterMapFlags.TranslateForward) == 0)
                continue;

            if (double.IsNaN(cutoff))
            {
                if (!double.IsPositiveInfinity(controller.State.CommandPulseTime)) continue;
                cutoff = now + 1.3 * _rcsBurnExpectedDuration;
                _rcsBurnCutoffSimTime = cutoff;
            }

            double remaining = cutoff - now;
            controller.State.CommandPulseTime = Math.Min(
                controller.State.CommandPulseTime,
                remaining > cutoffTolerance ? remaining : 0);
        }

        if (double.IsNaN(cutoff)) return;
        if (cutoff - now > cutoffTolerance) return;
        if (Interlocked.Exchange(ref _rcsBurnReleaseIssued, 1) != 0) return;
        Volatile.Read(ref _rcsBurnVehicle)!.ProcessInput(
            InputAction.TranslateForward, GlfwKeyAction.Release, default);
        Volatile.Write(ref _rcsBurnCutoffReached, true);
    }

    private static void QueueFlightComputer(Vehicle vehicle, Enum value) =>
        InputEvents.FlightComputerInputBuffer.Add(new InputEvents.FlightComputerInputData
        {
            Vehicle = vehicle,
            Toggle = false,
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

[HarmonyPatch(typeof(FlightComputer), nameof(FlightComputer.ComputeControl))]
internal static class GameTestRcsBurnCutoffPatch
{
    [HarmonyPostfix]
    private static void Postfix(
        FlightComputer __instance,
        ref FlightComputerNavigation nav,
        ref FlightComputerOutput outputs) =>
        GameTestScenarioPatch.CapRcsBurnCommand(__instance, nav.Time, ref outputs);
}
