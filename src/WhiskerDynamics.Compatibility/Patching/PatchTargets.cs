using Brutal.Numerics;
using Brutal.Concurrency.Jobs;
using KSA;

namespace WhiskerDynamics.Compatibility.Patching;

internal enum MemberKind { Method, StaticMethod, Field, Property, Constructor }

[Flags]
internal enum PropertyAccessors { None = 0, Getter = 1, Setter = 2 }

internal sealed record TargetSpec(
    string Key, Type DeclaringType, string MemberName, MemberKind Kind,
    Type[]? Parameters = null, Type? ExpectedType = null,
    int? GenericParameterCount = null, int? ParameterCount = null,
    int? OutParameterCount = null, bool? IsStatic = null,
    PropertyAccessors RequiredAccessors = PropertyAccessors.None);

/// <summary>Panel-only registry — deliberately a SEPARATE static class from
/// <see cref="GameplayTargets"/> so the CLR gives each its own independent static
/// initializer: resolving the panel target (KSA.Program members only) can never be
/// poisoned by a vanished gameplay type. ModMain touches this class OUTSIDE its drift
/// guard (if even KSA.Program is gone, EarlyInit itself fails to JIT and the entry
/// shim's fail-closed path is the correct outcome).</summary>
internal static class PanelTargets
{
    public static readonly TargetSpec[] Panel =
    [
        new("Program.OnDrawUiConsole", typeof(Program), "OnDrawUiConsole", MemberKind.Method, [typeof(double)], typeof(void)),
    ];
}

/// <summary>Every gameplay member the mod touches, with the exact shape verified against
/// the inspected game API. Validation failure of any gameplay entry disables all gameplay
/// patches; the panel entry lives in <see cref="PanelTargets"/> so the disabled notice can
/// still render. The FIRST touch of this class must happen INSIDE ModMain's drift-guard try:
/// if a future game build drops a registered type entirely, this class's static initializer
/// throws (TypeInitializationException) and the guard degrades to DisabledIncompatible.</summary>
internal static class GameplayTargets
{
    public static readonly TargetSpec[] Gameplay =
    [
        // Seam 3: celestial rails
        new("CelestialUpdateTask.Run", typeof(CelestialUpdateTask), "Run", MemberKind.Method, Type.EmptyTypes, typeof(void)),
        new("CelestialUpdateTask._readOnlyCelestial", typeof(CelestialUpdateTask), "_readOnlyCelestial", MemberKind.Field, null, typeof(Celestial), IsStatic: false),
        new("CelestialUpdateTask.NewStateVectors", typeof(CelestialUpdateTask), "NewStateVectors", MemberKind.Field, null, typeof(StateVectors?), IsStatic: false),
        new("Orbit.StateVectors", typeof(Orbit), "StateVectors", MemberKind.Property, null, typeof(StateVectors).MakeByRefType(), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),

        // Seam 1: vessel on-rails (the non-inline-marked callers)
        new("VehicleUpdateTask.ApplySingleVehicleMotion", typeof(VehicleUpdateTask), "ApplySingleVehicleMotion", MemberKind.Method,
            [typeof(VehicleUpdateState)], typeof(void)),
        new("VehicleUpdateTask.FullPhysicsUnconstrainedStep", typeof(VehicleUpdateTask), "FullPhysicsUnconstrainedStep", MemberKind.Method,
            [typeof(PhysicsContext).MakeByRefType(), typeof(SimStep).MakeByRefType()], typeof(void)),
        new("VehicleUpdateTask._vehicleStates", typeof(VehicleUpdateTask), "_vehicleStates", MemberKind.Field, null, typeof(List<VehicleUpdateState>), IsStatic: false),
        new("VehicleUpdateTask.SimStep", typeof(VehicleUpdateTask), "SimStep", MemberKind.Field, null, typeof(SimStep), IsStatic: false),
        new("VehicleUpdateTask.Origin", typeof(VehicleUpdateTask), "Origin", MemberKind.Field, null, typeof(BubbleOrigin), IsStatic: false),
        new("VehicleUpdateTask.OriginOrbit", typeof(VehicleUpdateTask), "OriginOrbit", MemberKind.Field, null, typeof(Orbit), IsStatic: false),
        new("Vehicle.UpdateFromTaskResults", typeof(Vehicle), "UpdateFromTaskResults", MemberKind.Method,
            [typeof(VehicleUpdateData).MakeByRefType(), typeof(BubbleOrigin).MakeByRefType(), typeof(Vehicle),
             typeof(ReadOnlySpan<Vehicle>), typeof(double3), typeof(double3)], typeof(void)),

        // Seam 1 staging surfaces (vessel orbital state storage)
        new("VehicleUpdateState.UpdateData", typeof(VehicleUpdateState), "UpdateData", MemberKind.Field, null, typeof(VehicleUpdateData), IsStatic: false),
        new("VehicleUpdateState.CurrentOrbit", typeof(VehicleUpdateState), "CurrentOrbit", MemberKind.Property, null, typeof(Orbit), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("VehicleUpdateState.GetNewStates", typeof(VehicleUpdateState), "GetNewStates", MemberKind.Method, Type.EmptyTypes, typeof(PhysicsStates)),
        new("VehicleUpdateState.SetCurrentOrbit", typeof(VehicleUpdateState), "SetCurrentOrbit", MemberKind.Method,
            [typeof(Orbit), typeof(KeyHash), typeof(bool)], typeof(void)), // the mod calls it: full signature pinned
        new("VehicleUpdateData.NewStateVectors", typeof(VehicleUpdateData), "NewStateVectors", MemberKind.Field, null, typeof(StateVectors?), IsStatic: false),
        new("VehicleUpdateData.NewFlightPlan", typeof(VehicleUpdateData), "NewFlightPlan", MemberKind.Field, null, typeof(FlightPlan), IsStatic: false),
        new("PhysicsStates.UpdateFromAnalytic", typeof(PhysicsStates), "UpdateFromAnalytic", MemberKind.Method,
            [typeof(Orbit), typeof(StateVectors).MakeByRefType(), typeof(doubleQuat), typeof(double3), typeof(Situation)], typeof(void)),
        new("Orbit.UpdatePosition", typeof(Orbit), "UpdatePosition", MemberKind.Method, [typeof(StateVectors)], typeof(void)),
        new("Orbit.CreateFromStateCci", typeof(Orbit), "CreateFromStateCci", MemberKind.StaticMethod,
            [typeof(IParentBody), typeof(SimTime), typeof(double3), typeof(double3), typeof(byte4)], typeof(Orbit)), // the mod calls it: full signature pinned
        new("Orbit.OrbitLineColor", typeof(Orbit), "OrbitLineColor", MemberKind.Field, null, typeof(byte4), IsStatic: false),
        new("BubbleOrigin.CreateFrom(parent,sv)", typeof(BubbleOrigin), "CreateFrom", MemberKind.StaticMethod,
            [typeof(IParentBody), typeof(StateVectors).MakeByRefType()], typeof(BubbleOrigin)),

        // Seam 1 patch-body member touches (registry contract: every game member the
        // patch bodies read or write is validated before use — including Orbit.Parent,
        // GetCce2Cci, the StateVectors ctor/fields, SimTime.Seconds).
        new("VehicleUpdateState.ReadOnlyVehicle", typeof(VehicleUpdateState), "ReadOnlyVehicle", MemberKind.Field, null, typeof(Vehicle), IsStatic: false),
        new("VehicleUpdateState.Id", typeof(VehicleUpdateState), "Id", MemberKind.Property, null, typeof(string), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("VehicleUpdateState.CurrentStateVectors", typeof(VehicleUpdateState), "CurrentStateVectors", MemberKind.Property, null, typeof(StateVectors).MakeByRefType(), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("VehicleUpdateState.CurrentBody2Cce", typeof(VehicleUpdateState), "CurrentBody2Cce", MemberKind.Property, null, typeof(doubleQuat), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("VehicleUpdateState.CurrentBodyRates", typeof(VehicleUpdateState), "CurrentBodyRates", MemberKind.Property, null, typeof(double3), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("PhysicsStates.Origin", typeof(PhysicsStates), "Origin", MemberKind.Field, null, typeof(BubbleOrigin).MakeByRefType(), IsStatic: false),
        new("PhysicsStates.Props", typeof(PhysicsStates), "Props", MemberKind.Field, null, typeof(VehicleProperties).MakeByRefType(), IsStatic: false),
        new("VehicleProperties.Situation", typeof(VehicleProperties), "Situation", MemberKind.Field, null, typeof(Situation), IsStatic: false),
        new("Situation.Freefall", typeof(Situation), "Freefall", MemberKind.Field, null, typeof(Situation), IsStatic: true), // literal; VALUE pinned in ModMain
        new("Astronomical.Id", typeof(Astronomical), "Id", MemberKind.Property, null, typeof(string), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter), // Vehicle.Id / Celestial.Id
        new("Astronomical.Hash", typeof(Astronomical), "Hash", MemberKind.Property, null, typeof(KeyHash), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter), // ReadOnlyVehicle.Hash for SetCurrentOrbit
        new("Vehicle.Props", typeof(Vehicle), "Props", MemberKind.Property, null, typeof(VehicleProperties).MakeByRefType(), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Vehicle.Orbit", typeof(Vehicle), "Orbit", MemberKind.Property, null, typeof(Orbit), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Orbit.Parent", typeof(Orbit), "Parent", MemberKind.Property, null, typeof(IParentBody), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("IParentBody.GetCce2Cci", typeof(IParentBody), "GetCce2Cci", MemberKind.Method, Type.EmptyTypes, typeof(doubleQuat)),
        new("IParentBody.GetCci2Cce", typeof(IParentBody), "GetCci2Cce", MemberKind.Method, Type.EmptyTypes, typeof(doubleQuat)),
        new("FlightPlan.FirstPatch", typeof(FlightPlan), "FirstPatch", MemberKind.Property, null, typeof(PatchedConic), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("PatchedConic.Orbit", typeof(PatchedConic), "Orbit", MemberKind.Field, null, typeof(Orbit), IsStatic: false),
        new("SimStep.PreviousTime", typeof(SimStep), "PreviousTime", MemberKind.Property, null, typeof(SimTime), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("SimStep.NextTime", typeof(SimStep), "NextTime", MemberKind.Property, null, typeof(SimTime), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("SimStep.DeltaTime", typeof(SimStep), "DeltaTime", MemberKind.Property, null, typeof(double), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("SimTime.Seconds()", typeof(SimTime), "Seconds", MemberKind.Method, Type.EmptyTypes, typeof(double)),
        new("BubbleOrigin.Time", typeof(BubbleOrigin), "Time", MemberKind.Field, null, typeof(SimTime), IsStatic: false),
        new("StateVectors..ctor(t,p,v,ta)", typeof(StateVectors), ".ctor", MemberKind.Constructor,
            [typeof(SimTime), typeof(double3), typeof(double3), typeof(TrueAnomaly)]),
        new("StateVectors.StateTime", typeof(StateVectors), "StateTime", MemberKind.Field, null, typeof(SimTime), IsStatic: false),
        new("StateVectors.PositionCci", typeof(StateVectors), "PositionCci", MemberKind.Field, null, typeof(double3), IsStatic: false),
        new("StateVectors.VelocityCci", typeof(StateVectors), "VelocityCci", MemberKind.Field, null, typeof(double3), IsStatic: false),
        new("StateVectors.TrueAnomaly", typeof(StateVectors), "TrueAnomaly", MemberKind.Field, null, typeof(TrueAnomaly), IsStatic: false),

        // Seam 2: live gravity
        new("PhysicsStates.ComputeDerivatives(static)", typeof(PhysicsStates), "ComputeDerivatives", MemberKind.StaticMethod,
            [typeof(BubbleOrigin).MakeByRefType(), typeof(KinematicStates).MakeByRefType(), typeof(VehicleProperties).MakeByRefType(),
             typeof(PhysicsEnvironment).MakeByRefType(), typeof(double), typeof(double), typeof(double3), typeof(double3),
             typeof(ReadOnlySpan<ActiveNozzle>)], typeof(Disturbances)),
        new("PhysicsStates.GetPositionClosestParentBub(static)", typeof(PhysicsStates), "GetPositionClosestParentBub", MemberKind.StaticMethod,
            [typeof(BubbleOrigin).MakeByRefType(), typeof(PhysicsEnvironment).MakeByRefType(), typeof(double3)], typeof(double3)),
        new("Disturbances.AddAccelPhys", typeof(Disturbances), "AddAccelPhys", MemberKind.Method, [typeof(double3)], typeof(void)),
        new("BubbleOrigin.Parent", typeof(BubbleOrigin), "Parent", MemberKind.Field, null, typeof(IParentBody), IsStatic: false),
        new("BubbleOrigin.GetBub2Cce", typeof(BubbleOrigin), "GetBub2Cce", MemberKind.Method, Type.EmptyTypes, typeof(doubleQuat)),
        new("KinematicStates.PositionPhys", typeof(KinematicStates), "PositionPhys", MemberKind.Field, null, typeof(double3), IsStatic: false),

        // SOI seams: the live handoff funnel (SoiHandoffPatch re-anchors cross-parent
        // analytic mirrors to rails; VehicleUpdateTask.cs:1292) and the rails-geometric
        // re-parent surfaces (VesselRegistry.RailsSoiParent mirrors stock's
        // CheckSoiTransitions candidate rules, PhysicsStates.cs:487-519; the re-parent
        // mirrors stock's own rails patch transition SetClosestParent,
        // VehicleUpdateTask.cs:856). ReadOnlyPhysicsStates fields are ref fields
        // (ReadOnlyPhysicsStates.cs:9-15) — byref-typed like the byref properties above.
        new("VehicleUpdateTask.PopulateAnalyticStatesFromKinematicStates", typeof(VehicleUpdateTask),
            "PopulateAnalyticStatesFromKinematicStates", MemberKind.StaticMethod,
            [typeof(VehicleUpdateState), typeof(bool)], typeof(void)),
        new("VehicleUpdateState.GetReadOnlyStates", typeof(VehicleUpdateState), "GetReadOnlyStates",
            MemberKind.Method, Type.EmptyTypes, typeof(ReadOnlyPhysicsStates)),
        new("ReadOnlyPhysicsStates.Environment", typeof(ReadOnlyPhysicsStates), "Environment",
            MemberKind.Field, null, typeof(PhysicsEnvironment).MakeByRefType(), IsStatic: false),
        new("ReadOnlyPhysicsStates.Origin", typeof(ReadOnlyPhysicsStates), "Origin",
            MemberKind.Field, null, typeof(BubbleOrigin).MakeByRefType(), IsStatic: false),
        new("ReadOnlyPhysicsStates.Props", typeof(ReadOnlyPhysicsStates), "Props",
            MemberKind.Field, null, typeof(VehicleProperties).MakeByRefType(), IsStatic: false),
        new("ReadOnlyPhysicsStates.Time", typeof(ReadOnlyPhysicsStates), "Time",
            MemberKind.Property, null, typeof(SimTime), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("SituationEx.IsOnRails", typeof(SituationEx), "IsOnRails",
            MemberKind.StaticMethod, [typeof(Situation)], typeof(bool)),
        new("ReadOnlyPhysicsStates.GetStatesCci", typeof(ReadOnlyPhysicsStates), "GetStatesCci",
            MemberKind.Method,
            [typeof(double3).MakeByRefType(), typeof(double3).MakeByRefType(), typeof(doubleQuat).MakeByRefType()],
            typeof(void)),
        new("PhysicsEnvironment.ClosestParent", typeof(PhysicsEnvironment), "ClosestParent",
            MemberKind.Field, null, typeof(IParentBody), IsStatic: false),
        new("PhysicsStates.SetClosestParent", typeof(PhysicsStates), "SetClosestParent",
            MemberKind.Method, [typeof(IParentBody)], typeof(void)),
        new("IParentBody.SphereOfInfluence", typeof(IParentBody), "SphereOfInfluence",
            MemberKind.Property, null, typeof(double), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("IParentBody.Children", typeof(IParentBody), "Children",
            MemberKind.Property, null, typeof(List<IOrbiter>), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("IOrbiter.Parent", typeof(IOrbiter), "Parent",
            MemberKind.Property, null, typeof(IParentBody), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        // SOI plan authority: suppress conic encounter and escape forecasts for
        // predictor-owned vessels. Stock impacts still wake full physics.
        new("PatchedConic.CheckUpdateEncounter", typeof(PatchedConic), "CheckUpdateEncounter",
            MemberKind.Method,
            [typeof(IOrbiter), typeof(SimTime).MakeByRefType(), typeof(double)], typeof(bool)),
        // Burn-past-impact preservation: everything BurnImpactPreservationPatch
        // consumes — both scoped entry points, the plan-id-to-vehicle lookup, the
        // copy constructor, and the EndTime field the extension writes.
        new("BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly", typeof(BurnPlan),
            "CalculateNewFlightPlansFromFlightComputerOnly", MemberKind.Method,
            [typeof(FlightComputer), typeof(FlightPlan), typeof(KeyHash),
             typeof(List<FlightPlan>)], typeof(void)),
        new("BurnPlan.DeserializeSave", typeof(BurnPlan), "DeserializeSave",
            MemberKind.Method, [typeof(BurnPlanData), typeof(Vehicle)], typeof(void)),
        new("FlightPlan.IdHash", typeof(FlightPlan), "IdHash",
            MemberKind.Property, null, typeof(KeyHash), IsStatic: false,
            RequiredAccessors: PropertyAccessors.Getter),
        new("CelestialSystem.All", typeof(CelestialSystem), "All",
            MemberKind.Property, null, typeof(LookupCollection<Astronomical>),
            IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("LookupCollection<Astronomical>.Get(KeyHash)",
            typeof(LookupCollection<Astronomical>), "Get",
            MemberKind.Method, [typeof(KeyHash)], typeof(Astronomical)),
        new("PatchedConic..ctor(copy)", typeof(PatchedConic), ".ctor",
            MemberKind.Constructor, [typeof(PatchedConic)]),
        new("PatchedConic.EndTime", typeof(PatchedConic), "EndTime",
            MemberKind.Field, null, typeof(SimTime), IsStatic: false),
        new("Vehicle.UpdateTask", typeof(Vehicle), "UpdateTask",
            MemberKind.Field, null, typeof(VehicleUpdateTask), IsStatic: false),
        new("VehicleUpdateTask.NumVehicles", typeof(VehicleUpdateTask), "NumVehicles",
            MemberKind.Property, null, typeof(int), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("FlightPlan.CalculateEscapePatch", typeof(FlightPlan), "CalculateEscapePatch",
            MemberKind.StaticMethod,
            [typeof(PatchedConic), typeof(SimTime), typeof(PatchedConic).MakeByRefType(),
             typeof(bool).MakeByRefType()], typeof(bool)),
        new("VehicleUpdateState.RecalculateFlightPlan", typeof(VehicleUpdateState),
            "RecalculateFlightPlan", MemberKind.Method, [typeof(bool)], typeof(void)),
        new("PatchedConic.StartTime", typeof(PatchedConic), "StartTime",
            MemberKind.Field, null, typeof(SimTime), IsStatic: false),
        new("SimTime.op_Addition(time,double)", typeof(SimTime), "op_Addition",
            MemberKind.StaticMethod, [typeof(SimTime), typeof(double)], typeof(SimTime)),

        // Stock attitude Toward/Away/Antivel target snapshot. The postfix replaces
        // only the stale Kepler position and velocity with rails/predictor truth.
        new("NavigationTarget.Create", typeof(NavigationTarget), "Create",
            MemberKind.StaticMethod,
            [typeof(IOrbiter), typeof(IParentBody), typeof(SimTime)], typeof(NavigationTarget?)),
        new("NavigationTarget.PositionCci", typeof(NavigationTarget), "PositionCci",
            MemberKind.Field, null, typeof(double3), IsStatic: false),
        new("NavigationTarget.VelocityCci", typeof(NavigationTarget), "VelocityCci",
            MemberKind.Field, null, typeof(double3), IsStatic: false),

        // Within-tick burn witness: force ReseedPending when a tick's
        // stock-accumulated DeltaVelocityCci is nonzero.
        new("VehicleUpdateData.NewKinematicMeasurements", typeof(VehicleUpdateData), "NewKinematicMeasurements",
            MemberKind.Field, null, typeof(KinematicMeasurements?), IsStatic: false),
        new("KinematicMeasurements.DeltaVelocityCci", typeof(KinematicMeasurements), "DeltaVelocityCci",
            MemberKind.Field, null, typeof(double3), IsStatic: false),

        // Save + overlay + adapters
        new("Universe.CurrentSystem", typeof(Universe), "CurrentSystem", MemberKind.Property, null, typeof(CelestialSystem), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        new("Universe.GetElapsedSimTime", typeof(Universe), "GetElapsedSimTime", MemberKind.StaticMethod, Type.EmptyTypes, typeof(SimTime)),
        new("Universe.GetSimulationSpeed", typeof(Universe), "GetSimulationSpeed", MemberKind.StaticMethod, Type.EmptyTypes, typeof(double)),
        new("JobSystems.VehicleSolvers", typeof(JobSystems), "VehicleSolvers", MemberKind.Field, null, typeof(JobScheduler), IsStatic: true),
        new("JobScheduler.Wait", typeof(JobScheduler), "Wait", MemberKind.Method, Type.EmptyTypes, typeof(void)),
        // Stable save identity seams. UncompressedSave.Write runs after Populate for
        // UI-new and console-new saves, and is also reached through Overwrite -> Make;
        // its postfix therefore observes a complete stock save. UI and console loads
        // both dispatch through UncompressedSave.Load, whose postfix runs after stock
        // Universe.DeserializeSave. Id is declared on the GameSave base class.
        new("UncompressedSave.Write", typeof(UncompressedSave), "Write", MemberKind.Method,
            Type.EmptyTypes, typeof(void)),
        new("UncompressedSave.Load", typeof(UncompressedSave), "Load", MemberKind.Method,
            Type.EmptyTypes, typeof(void)),
        new("UncompressedSave.MetaData", typeof(UncompressedSave), "MetaData",
            MemberKind.Field, null, typeof(SaveMetaData), IsStatic: false),
        new("SaveMetaData.Updated", typeof(SaveMetaData), "Updated", MemberKind.Field,
            null, typeof(DateTime), IsStatic: false),
        new("GameSave.Id", typeof(GameSave), "Id", MemberKind.Property, null, typeof(string), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("GameSave.UniverseData", typeof(GameSave), "UniverseData", MemberKind.Field,
            null, typeof(UniverseData), IsStatic: false),
        new("UniverseData.GetElapsedSeconds", typeof(UniverseData), "GetElapsedSeconds",
            MemberKind.Method, Type.EmptyTypes, typeof(double)),
        // Save drill (verification scaffolding): the drill patch targets the
        // UI-draw method (also pinned in PanelTargets — duplicated here because a
        // GAMEPLAY patch now applies to it) and fires the game's own save command.
        new("Program.OnDrawUiConsole(drill)", typeof(Program), "OnDrawUiConsole", MemberKind.Method, [typeof(double)], typeof(void)),
        new("GameSaves.MakeUncompressedSave", typeof(GameSaves), "MakeUncompressedSave", MemberKind.StaticMethod, [typeof(string)], typeof(void)),
        new("GameSaves.LoadSaveGame", typeof(GameSaves), "LoadSaveGame", MemberKind.StaticMethod, [typeof(string)], typeof(void)),
        // Warp drill: the public double overload (a SimSpeed overload exists,
        // Universe.cs:574 — parameters disambiguate).
        new("Universe.SetSimulationSpeed(double,bool)", typeof(Universe), "SetSimulationSpeed",
            MemberKind.StaticMethod, [typeof(double), typeof(bool)], typeof(void)),
        new("Orbit.UpdateCachedPoints", typeof(Orbit), "UpdateCachedPoints", MemberKind.Method,
            [typeof(CommunityToolkit.HighPerformance.Buffers.MemoryOwner<OrbitPointCce>)], typeof(void)), // the mod calls it: full signature pinned
        new("FlightComputer.BurnPlan", typeof(FlightComputer), "BurnPlan", MemberKind.Property, null, typeof(BurnPlan), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        // In-game scenario burn executor: mirror the current stock FC lifecycle
        // (Auto mode + WarpToNextBurn targets calculated ignition time).
        new("FlightComputer.Burn", typeof(FlightComputer), "Burn", MemberKind.Field,
            null, typeof(BurnTarget), IsStatic: false),
        new("FlightComputer.BurnMode", typeof(FlightComputer), "BurnMode", MemberKind.Field,
            null, typeof(FlightComputerBurnMode), IsStatic: false),
        new("FlightComputer.ComputeControl(game-test RCS cutoff)", typeof(FlightComputer),
            "ComputeControl", MemberKind.Method,
            [
                typeof(FlightComputerNavigation).MakeByRefType(),
                typeof(ManualControlInputs).MakeByRefType(),
                typeof(FlightComputerOutput).MakeByRefType(),
            ],
            typeof(void)),
        new("FlightComputerNavigation.Time", typeof(FlightComputerNavigation), "Time",
            MemberKind.Field, null, typeof(SimTime), IsStatic: false),
        new("FlightComputerOutput.Thrusters", typeof(FlightComputerOutput), "Thrusters",
            MemberKind.Field, null,
            typeof(ModuleStateful<ThrusterController, ThrusterControllerState,
                ThrusterControllerGlobalState, EmptyStruct>.StateUpdater),
            IsStatic: false),
        new("ThrusterStateUpdater.GetModulesAndNewStates",
            typeof(ModuleStateful<ThrusterController, ThrusterControllerState,
                ThrusterControllerGlobalState, EmptyStruct>.StateUpdater),
            "GetModulesAndNewStates", MemberKind.Method,
            [typeof(ReadOnlySpan<ThrusterController>)],
            typeof(ModuleStateful<ThrusterController, ThrusterControllerState,
                ThrusterControllerGlobalState, EmptyStruct>.StateUpdater
                .ModuleAndNewStateEnumerator)),
        new("ThrusterControllerState.CommandPulseTime", typeof(ThrusterControllerState),
            "CommandPulseTime", MemberKind.Field, null, typeof(double), IsStatic: false),
        new("BurnTarget.IgnitionTime", typeof(BurnTarget), "IgnitionTime", MemberKind.Field,
            null, typeof(SimTime), IsStatic: false),
        new("BurnTarget.DeltaVTargetCci", typeof(BurnTarget), "DeltaVTargetCci", MemberKind.Field,
            null, typeof(float3), IsStatic: false),
        new("BurnTarget.DeltaVToGoCci", typeof(BurnTarget), "DeltaVToGoCci",
            MemberKind.Property, null, typeof(float3), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Universe.IsAutoWarpActive", typeof(Universe), "IsAutoWarpActive",
            MemberKind.Property, null, typeof(bool), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        new("Universe.AutoWarpTime", typeof(Universe), "AutoWarpTime",
            MemberKind.Property, null, typeof(SimTime?), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        new("Universe.AutoWarpStop", typeof(Universe), "AutoWarpStop",
            MemberKind.StaticMethod, [typeof(bool)], typeof(void)),
        new("Burn.Time", typeof(Burn), "Time", MemberKind.Field, null, typeof(SimTime), IsStatic: false),
        new("Burn.DeltaVVlf", typeof(Burn), "DeltaVVlf", MemberKind.Field, null, typeof(double3), IsStatic: false),
        new("Program.ControlledVehicle", typeof(Program), "ControlledVehicle", MemberKind.Field, null, typeof(Vehicle), IsStatic: true),
        new("InputEvents.VehicleResourcesChangeBuffer", typeof(InputEvents),
            "VehicleResourcesChangeBuffer", MemberKind.Field, null,
            typeof(InputEvents.TypedBuffer<InputEvents.VehicleResourcesChangeData>), IsStatic: true),
        new("VehicleResourcesChangeData.Vehicle", typeof(InputEvents.VehicleResourcesChangeData),
            "Vehicle", MemberKind.Field, null, typeof(Vehicle), IsStatic: false),
        new("VehicleResourcesChangeData.Refill", typeof(InputEvents.VehicleResourcesChangeData),
            "Refill", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("VehicleResourcesChangeData.Empty", typeof(InputEvents.VehicleResourcesChangeData),
            "Empty", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("VehicleResourcesChangeData.Control", typeof(InputEvents.VehicleResourcesChangeData),
            "Control", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("PartTree.ResourceGroupList", typeof(PartTree), "ResourceGroupList",
            MemberKind.Field, null, typeof(ResourceGroupList), IsStatic: false),
        new("ResourceGroupList.CalculateStages", typeof(ResourceGroupList), "CalculateStages",
            MemberKind.Method, [typeof(bool)], typeof(void)),
        new("CelestialSystem.Get(string)", typeof(CelestialSystem), "Get", MemberKind.Method,
            [typeof(string)], typeof(Astronomical)),
        new("InputEvents.TeleportInputBuffer", typeof(InputEvents),
            "TeleportInputBuffer", MemberKind.Field, null,
            typeof(InputEvents.TypedBuffer<InputEvents.TeleportInputData>), IsStatic: true),
        new("TypedBuffer<TeleportInputData>.Add",
            typeof(InputEvents.TypedBuffer<InputEvents.TeleportInputData>),
            "Add", MemberKind.Method, [typeof(InputEvents.TeleportInputData)], typeof(void)),
        new("TeleportInputData.Vehicle", typeof(InputEvents.TeleportInputData),
            "Vehicle", MemberKind.Field, null, typeof(Vehicle), IsStatic: false),
        new("TeleportInputData.Orbit", typeof(InputEvents.TeleportInputData),
            "Orbit", MemberKind.Field, null, typeof(Orbit), IsStatic: false),
        new("TeleportInputData.Body2Cce", typeof(InputEvents.TeleportInputData),
            "Body2Cce", MemberKind.Field, null, typeof(doubleQuat?), IsStatic: false),
        new("TeleportInputData.BodyRates", typeof(InputEvents.TeleportInputData),
            "BodyRates", MemberKind.Field, null, typeof(double3?), IsStatic: false),
        new("InputEvents.FlightComputerInputBuffer", typeof(InputEvents),
            "FlightComputerInputBuffer", MemberKind.Field, null,
            typeof(InputEvents.TypedBuffer<InputEvents.FlightComputerInputData>), IsStatic: true),
        new("FlightComputerInputData.Vehicle", typeof(InputEvents.FlightComputerInputData),
            "Vehicle", MemberKind.Field, null, typeof(Vehicle), IsStatic: false),
        new("FlightComputerInputData.Toggle", typeof(InputEvents.FlightComputerInputData),
            "Toggle", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("FlightComputerInputData.EnumValue", typeof(InputEvents.FlightComputerInputData),
            "EnumValue", MemberKind.Field, null, typeof(Enum), IsStatic: false),
        new("Vehicle.Target", typeof(Vehicle), "Target", MemberKind.Property, null, typeof(IOrbiter), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        // Planner delta-v budget: the same stock figure rendered on the
        // navball (Vehicle.UpdateNavBallData reads the active staging sequence's
        // performance total, Parts.PerformanceSequences.FindActiveSequenceDeltaV(),
        // Vehicle.cs:2386).
        new("Vehicle.NavBallData", typeof(Vehicle), "NavBallData", MemberKind.Property,
            null, typeof(NavBallData).MakeByRefType(), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("NavBallData.DeltaV", typeof(NavBallData), "DeltaV",
            MemberKind.Field, null, typeof(double), IsStatic: false),
        // Overlay member touches (registry contract: every game member the mod
        // reads or calls is validated before use).
        new("Orbit.TimeAtPeriapsis", typeof(Orbit), "TimeAtPeriapsis", MemberKind.Property, null, typeof(SimTime), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Vehicle.FlightComputer", typeof(Vehicle), "FlightComputer", MemberKind.Property, null, typeof(FlightComputer), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("BurnPlan.BurnCount", typeof(BurnPlan), "BurnCount", MemberKind.Property, null, typeof(int), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("BurnPlan.TryGetBurn(int,out)", typeof(BurnPlan), "TryGetBurn", MemberKind.Method,
            [typeof(int), typeof(Burn).MakeByRefType()], typeof(bool)),
        new("StateVectors.GetVlf2ParentCci", typeof(StateVectors), "GetVlf2ParentCci", MemberKind.Method,
            Type.EmptyTypes, typeof(doubleQuat?)),
        new("TrueAnomaly.NaN", typeof(TrueAnomaly), "NaN", MemberKind.Property, null, typeof(TrueAnomaly), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        new("OrbitPointCce..ctor(pos,tPe,rem,ta,danger)", typeof(OrbitPointCce), ".ctor", MemberKind.Constructor,
            [typeof(double3), typeof(SimTime), typeof(SimTime), typeof(TrueAnomaly), typeof(bool)]),
        new("SimTime..ctor(seconds)", typeof(SimTime), ".ctor", MemberKind.Constructor, [typeof(double)]),

        // Map pipeline surfaces. Signatures verified in decompiled sources:
        // MapController.cs:124, Viewport.cs:14/366, Camera.cs:94/140,
        // Transform3D.cs:13, IPosition.cs:5. (Vehicle.OnPreRender is deliberately
        // unpatched: the vessel draw-site takeover, VesselLinePatch, covers
        // re-staging — no code touches it.)
        new("MapController.OnFrame", typeof(MapController), "OnFrame", MemberKind.Method,
            [typeof(Viewport), typeof(double)], typeof(void)),
        new("Viewport.Mode", typeof(Viewport), "Mode", MemberKind.Field, null, typeof(CameraMode), IsStatic: false),
        new("Viewport.GetCamera", typeof(Viewport), "GetCamera", MemberKind.Method, Type.EmptyTypes, typeof(Camera)),
        new("Camera.PositionEcl", typeof(Camera), "PositionEcl", MemberKind.Property, null, typeof(double3), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter | PropertyAccessors.Setter),
        new("Camera.Following", typeof(Camera), "Following", MemberKind.Property, null, typeof(IFollowable), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Transform3D.LocalRotation", typeof(Transform3D), "LocalRotation", MemberKind.Field, null, typeof(doubleQuat), IsStatic: false),
        new("IPosition.GetPositionEcl", typeof(IPosition), "GetPositionEcl", MemberKind.Method, Type.EmptyTypes, typeof(double3)),
        // MapFramePatch's follow-coherence telemetry names the followed
        // target (IFollowable inherits Id from IObjectId, so the callvirt token is
        // IObjectId::get_Id — decompiled IObjectId.cs:5, IFollowable.cs:3).
        new("IObjectId.Id", typeof(IObjectId), "Id", MemberKind.Property, null, typeof(string), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("CameraMode.Map", typeof(CameraMode), "Map", MemberKind.Field, null, typeof(CameraMode), IsStatic: true), // literal; VALUE pinned in ModMain

        // Celestial curve staging surfaces (Celestial.cs:64/1761).
        // (Celestial.OnPreRender is deliberately unpatched: the celestial draw-site
        // takeover, CelestialLinePatch, covers it — no code touches it.)
        new("Celestial.Orbit", typeof(Celestial), "Orbit", MemberKind.Property, null, typeof(Orbit), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Celestial.RegenerateOrbitLines", typeof(Celestial), "RegenerateOrbitLines", MemberKind.Method,
            Type.EmptyTypes, typeof(void)),
        // Burn-planner write surface (decompiled evidence:
        // InputEvents.cs:509-517/592/632, Burn.cs:32/34/38/132/170, BurnPlan.cs:223/267/292,
        // FlightPlan.cs:205, Vehicle.cs:418, Orbit.cs:2294).
        new("InputEvents.BurnUpdateBuffer", typeof(InputEvents), "BurnUpdateBuffer",
            MemberKind.Field, null, typeof(InputEvents.TypedBuffer<InputEvents.BurnUpdateData>), IsStatic: true),
        new("TypedBuffer<BurnUpdateData>.Add", typeof(InputEvents.TypedBuffer<InputEvents.BurnUpdateData>),
            "Add", MemberKind.Method, [typeof(InputEvents.BurnUpdateData)], typeof(void)),
        new("BurnUpdateData.FlightComputer", typeof(InputEvents.BurnUpdateData), "FlightComputer",
            MemberKind.Field, null, typeof(FlightComputer), IsStatic: false),
        new("BurnUpdateData.Burn", typeof(InputEvents.BurnUpdateData), "Burn",
            MemberKind.Field, null, typeof(Burn), IsStatic: false),
        new("BurnUpdateData.AddBurn", typeof(InputEvents.BurnUpdateData), "AddBurn",
            MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("BurnUpdateData.DeleteBurn", typeof(InputEvents.BurnUpdateData), "DeleteBurn",
            MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("Burn.Create(point,t,dv,patch,vehicle)", typeof(Burn), "Create", MemberKind.StaticMethod,
            [typeof(OrbitPointCce), typeof(double), typeof(double3), typeof(PatchedConic), typeof(Vehicle)], typeof(Burn)),
        new("Burn.Update(fc)", typeof(Burn), "Update", MemberKind.Method,
            [typeof(FlightComputer)], typeof(void)),
        new("Burn.Vehicle", typeof(Burn), "Vehicle", MemberKind.Field, null, typeof(Vehicle), IsStatic: false),
        new("BurnPlan.TryGetBurn(SimTime,out)", typeof(BurnPlan), "TryGetBurn", MemberKind.Method,
            [typeof(SimTime), typeof(Burn).MakeByRefType()], typeof(bool)),
        new("BurnPlan.TryGetBurnPatch", typeof(BurnPlan), "TryGetBurnPatch", MemberKind.Method,
            [typeof(Burn)], typeof(PatchedConic)),
        new("BurnPlan.FlightPlansOutOfDate", typeof(BurnPlan), "FlightPlansOutOfDate",
            MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("BurnPlan.TryGetValidTimeLinePatch", typeof(BurnPlan), "TryGetValidTimeLinePatch",
            MemberKind.Method, [typeof(SimTime)], typeof(PatchedConic)),
        new("FlightPlan.TryFindPatch", typeof(FlightPlan), "TryFindPatch", MemberKind.Method,
            [typeof(SimTime)], typeof(PatchedConic)),
        new("Vehicle.FlightPlan", typeof(Vehicle), "FlightPlan", MemberKind.Property, null, typeof(FlightPlan), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Orbit.GetPointAt", typeof(Orbit), "GetPointAt", MemberKind.Method,
            [typeof(SimTime)], typeof(OrbitPointCce)),

        // Honest orbit lines: vessel draw-site takeover (VesselLinePatch).
        // Shapes verified in decompiled sources: FlightPlan.cs:758/74,
        // Orbit.cs:2266, Camera.cs:213, Double3Ex.cs:15, Vehicle.cs:339/343,
        // Astronomical.cs:391, SimTime.cs:10. InactiveColor's Color.Preset is
        // Brutal.Numerics (Color.cs:8); its implicit byte4 operator (Color.cs:33)
        // carries the (byte4) cast — Brutal primitives are engine-stable, not pinned.
        // The two AddLineInstances danger params and the DrawLines danger tail
        // (DangerDisplay, depthPriority, dangerTrackMinTime, dangerColorOverride)
        // all default; the mod's stale-patch-0 mirror deliberately leaves them at
        // their defaults, so no DangerDisplay value is compiled into patch IL.
        new("FlightPlan.AddLineInstances", typeof(FlightPlan), "AddLineInstances", MemberKind.Method,
            [typeof(Viewport), typeof(IOrbiter), typeof(bool), typeof(bool), typeof(TrueAnomaly), typeof(TrueAnomaly),
             typeof(bool), typeof(bool)], typeof(void)),
        new("FlightPlan.InactiveColor", typeof(FlightPlan), "InactiveColor", MemberKind.Field, null, typeof(Color.Preset), IsStatic: true),
        new("Orbit.DrawLines(color)", typeof(Orbit), "DrawLines", MemberKind.Method,
            [typeof(Viewport), typeof(double3), typeof(SimTime), typeof(byte4),
             typeof(TrueAnomaly), typeof(TrueAnomaly), typeof(bool), typeof(bool), typeof(bool),
             typeof(Orbit.DangerDisplay), typeof(bool), typeof(SimTime?), typeof(byte4?)], typeof(void)),
        new("Camera.GetPositionEgo", typeof(Camera), "GetPositionEgo", MemberKind.Method,
            [typeof(IPosition)], typeof(double3)),
        new("Vehicle.ShowOrbit", typeof(Vehicle), "ShowOrbit", MemberKind.Property, null, typeof(bool).MakeByRefType(), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter), // ref bool, Vehicle.cs:339
        new("Vehicle.TargetOfControlledVehicle", typeof(Vehicle), "TargetOfControlledVehicle", MemberKind.Property, null, typeof(bool), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Astronomical.ShouldDrawUiOrLines", typeof(Astronomical), "ShouldDrawUiOrLines", MemberKind.StaticMethod,
            [typeof(IParentBody), typeof(Viewport), typeof(Orbit)], typeof(bool)), // Orbit? erases to Orbit at runtime
        new("Double3Ex.NaN", typeof(Double3Ex), "NaN", MemberKind.Property, null, typeof(double3), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        new("SimTime.Zero", typeof(SimTime), "Zero", MemberKind.Property, null, typeof(SimTime), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        // Instance routing: a burn's post-burn plan is recognized by
        // identity against Burn.FlightPlan — public field, Burn.cs:36.
        new("Burn.FlightPlan", typeof(Burn), "FlightPlan", MemberKind.Field, null, typeof(FlightPlan), IsStatic: false),

        // Honest orbit lines: celestial draw-site takeover (CelestialLinePatch).
        // Shapes verified in decompiled sources: Celestial.cs:68/72/1770
        // (ShowOrbit is a ref bool; TargetOfControlledVehicle is DECLARED on Celestial
        // itself — the Vehicle.TargetOfControlledVehicle entry covers only Vehicle.cs:343,
        // a distinct declaring type); Astronomical.cs:546 (the Astronomical overload of
        // ShouldDrawLines; Orbit? erases to Orbit at runtime); Orbit.cs:2071 (the
        // default-color DrawLines overload).
        new("Celestial.AddLineInstances", typeof(Celestial), "AddLineInstances", MemberKind.Method,
            [typeof(Viewport)], typeof(void)),
        new("Celestial.ShowOrbit", typeof(Celestial), "ShowOrbit", MemberKind.Property, null, typeof(bool).MakeByRefType(), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Celestial.TargetOfControlledVehicle", typeof(Celestial), "TargetOfControlledVehicle", MemberKind.Property, null, typeof(bool), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Astronomical.ShouldDrawLines", typeof(Astronomical), "ShouldDrawLines", MemberKind.StaticMethod,
            [typeof(Astronomical), typeof(Viewport), typeof(Orbit)], typeof(bool)),
        new("Orbit.DrawLines", typeof(Orbit), "DrawLines", MemberKind.Method,
            [typeof(Viewport), typeof(double3), typeof(SimTime), typeof(bool), typeof(bool), typeof(bool)], typeof(void)),

        // Honest-density lines + finite-burn estimation. Shapes
        // verified in decompiled sources: OrbitLinePass.cs:293/275 (the growable
        // vertex append the dense draw feeds — Span parameters, no length limit),
        // Orbit.cs:2243 (IsVisible: the FOV + 5-px cull kept for stock parity),
        // FlightComputer.cs:63/101 (TotalMassPropsBody field, VehicleConfig
        // property), :71-73 (ActiveEngineThrust/ActiveEngineMassFlowRate — the
        // executor's own duration inputs, refreshed each control tick by
        // UpdateActiveEnginePerformance :721-735 and read by the rocket-equation
        // mirror; UpdateBurnTarget consumes them at :750-756), MassProperties.cs:9
        // (Mass field). float3.Pack / double3.IsNaN are Brutal/KSA primitives per
        // the Brutal-stability note above (Double3Ex.NaN is already pinned).
        new("OrbitLinePass.AddLineVertices", typeof(OrbitLinePass), "AddLineVertices", MemberKind.StaticMethod,
            [typeof(Viewport), typeof(Span<float3>), typeof(Span<byte4>)], typeof(void)),
        new("OrbitLinePass.AddLineEnd", typeof(OrbitLinePass), "AddLineEnd", MemberKind.StaticMethod,
            [typeof(Viewport)], typeof(void)),
        new("Orbit.IsVisible", typeof(Orbit), "IsVisible", MemberKind.Method,
            [typeof(Camera)], typeof(bool)),
        // The screen-space emit filter reads the camera's exact
        // pixels-per-angle formula (Camera.cs:712-722) and detects stock cache
        // overwrites via the payload TA (OrbitPointCce.cs:12 — NaN on mod points).
        new("Camera.GetObjectDiameterPixelsFrac", typeof(Camera), "GetObjectDiameterPixelsFrac",
            MemberKind.Method, [typeof(double), typeof(double)], typeof(double)),
        new("Orbit.CachedPoints", typeof(Orbit), "CachedPoints", MemberKind.Property,
            null, typeof(Span<OrbitPointCce>), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Orbit.LineCount", typeof(Orbit), "LineCount", MemberKind.Property, null, typeof(int), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("FlightComputer.TotalMassPropsBody", typeof(FlightComputer), "TotalMassPropsBody",
            MemberKind.Field, null, typeof(MassProperties), IsStatic: false),
        new("FlightComputer.VehicleConfig", typeof(FlightComputer), "VehicleConfig",
            MemberKind.Property, null, typeof(FlightComputer.VehicleConfigInfo), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("FlightComputer.ActiveEngineThrust", typeof(FlightComputer),
            "ActiveEngineThrust", MemberKind.Field, null, typeof(float), IsStatic: false),
        new("FlightComputer.ActiveEngineMassFlowRate", typeof(FlightComputer),
            "ActiveEngineMassFlowRate", MemberKind.Field, null, typeof(float), IsStatic: false),
        new("MassProperties.Mass", typeof(MassProperties), "Mass", MemberKind.Field, null, typeof(float), IsStatic: false),

        // Forward-RCS finite-plan estimate. The committed state list supplies KSA's
        // own forward-map and propellant-availability decisions; core/nozzle vacuum
        // performance is reduced to net +body-X thrust and total selected flow.
        new("Vehicle.Parts", typeof(Vehicle), "Parts", MemberKind.Property, null,
            typeof(PartTree), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("PartTree.States", typeof(PartTree), "States", MemberKind.Field, null, typeof(ModuleStateList), IsStatic: false),
        new("VehicleConfigInfo.Thrusters", typeof(FlightComputer.VehicleConfigInfo),
            "Thrusters", MemberKind.Field, null, typeof(List<ThrusterController>), IsStatic: false),
        new("ModuleStateList.TryGetTypeList", typeof(ModuleStateList), "TryGetTypeList",
            MemberKind.Method, null, typeof(bool), GenericParameterCount: 4,
            ParameterCount: 1, OutParameterCount: 1),
        new("ThrusterStateList.GetState",
            typeof(ModuleStateful<ThrusterController, ThrusterControllerState,
                ThrusterControllerGlobalState, EmptyStruct>.StateList),
            "GetState", MemberKind.Method, [typeof(ThrusterController)], typeof(ThrusterControllerState).MakeByRefType()),
        new("ThrusterController.Cores", typeof(ThrusterController), "Cores",
            MemberKind.Field, null, typeof(RocketCore[]), IsStatic: false),
        new("ThrusterControllerState.ControlMap", typeof(ThrusterControllerState), "ControlMap",
            MemberKind.Field, null, typeof(ThrusterMapFlags), IsStatic: false),
        new("ThrusterControllerState.IsPropellantAvailable", typeof(ThrusterControllerState),
            "IsPropellantAvailable", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("RocketCore.ComputeConditions", typeof(RocketCore), "ComputeConditions",
            MemberKind.Method, [typeof(float)], typeof(RocketCoreConditions)),
        new("RocketCore.Rocket", typeof(RocketCore), "Rocket", MemberKind.Field, null, typeof(Rocket), IsStatic: false),
        new("Rocket.Nozzles", typeof(Rocket), "Nozzles", MemberKind.Field, null, typeof(RocketNozzle[]), IsStatic: false),
        new("RocketNozzle.ComputePerformance(core)", typeof(RocketNozzle), "ComputePerformance",
            MemberKind.Method, [typeof(RocketCoreConditions).MakeByRefType(), typeof(float)], typeof(NozzlePerformance)),
        new("RocketNozzle.ExhaustDirectionAsmb", typeof(RocketNozzle), "ExhaustDirectionAsmb",
            MemberKind.Field, null, typeof(float3), IsStatic: false),
        new("Module<RocketNozzle>.Parent", typeof(Module<RocketNozzle>), "Parent",
            MemberKind.Field, null, typeof(Part), IsStatic: false),
        new("Part.Asmb2VehicleAsmb", typeof(Part), "Asmb2VehicleAsmb",
            MemberKind.Property, null, typeof(doubleQuat), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("NozzlePerformance.GetTotalThrust", typeof(NozzlePerformance), "GetTotalThrust",
            MemberKind.Method, Type.EmptyTypes, typeof(float)),
        new("NozzlePerformance.MassFlowRate", typeof(NozzlePerformance), "MassFlowRate",
            MemberKind.Field, null, typeof(float), IsStatic: false),

        // Conic marker suppression (PatchMarkerPatch). Shapes
        // verified in decompiled sources: PatchedConic.cs:1120 (single DrawUi
        // overload; PatchedConic? params erase to PatchedConic at runtime; the three
        // danger/label bools all default), :82
        // (EndTransition public field), :102 (HoveredMarker public field);
        // Astronomical.cs:15 (Astronomical is a readonly FIELD on the nested readonly
        // struct UiContext); PatchTransition.cs:3 (namespace-level int-backed enum);
        // FlightPlan.cs:64 (Patches public field). Caller census: Vehicle.cs:3477 (own
        // plan), BurnPlan.cs:503 (planned burns' plans), TransferPlanner.cs:991/1000
        // (preview — routed to stock). Burn-scan members (Vehicle.FlightComputer,
        // FlightComputer.BurnPlan, BurnPlan.BurnCount, BurnPlan.TryGetBurn(int,out),
        // Burn.FlightPlan, Vehicle.FlightPlan) are existing entries above.
        new("PatchedConic.DrawUi", typeof(PatchedConic), "DrawUi", MemberKind.Method,
            [typeof(Viewport), typeof(Astronomical.UiContext), typeof(int), typeof(PatchedConic), typeof(PatchedConic),
             typeof(bool), typeof(bool), typeof(bool)], typeof(bool)),
        new("UiContext.Astronomical", typeof(Astronomical.UiContext), "Astronomical", MemberKind.Field, null, typeof(Astronomical), IsStatic: false),
        new("PatchedConic.EndTransition", typeof(PatchedConic), "EndTransition", MemberKind.Field, null, typeof(PatchTransition), IsStatic: false),
        new("PatchedConic.HoveredMarker", typeof(PatchedConic), "HoveredMarker", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("PatchTransition.Final", typeof(PatchTransition), "Final", MemberKind.Field, null, typeof(PatchTransition), IsStatic: true), // literal; VALUE pinned in ModMain
        new("FlightPlan.Patches", typeof(FlightPlan), "Patches", MemberKind.Field, null, typeof(List<PatchedConic>), IsStatic: false),

        // Live celestial catalog: bind-time snapshot of the running system
        // (LiveCatalog). Shapes verified in decompiled sources:
        // CelestialSystem.cs:59 (Count), :61 (Id), :65 (GetIndex(int) — the non-generic
        // overload; a generic GetIndex<T>(int) sibling with the same parameter list
        // exists, so the validator resolves Method specs with genericParameterCount 0),
        // IParentBody.cs:11 (Mass — StellarBody.cs:38 and Celestial.cs:86 both implement;
        // the interface token covers the root and every celestial in one entry),
        // Astronomical.cs:93 (MeanRadius, abstract — override calls bind to this token),
        // Orbit.cs:1966 (GetStateVectorsAt(SimTime): pure DEFINING-conic evaluation —
        // time→anomaly→perifocal state→Orb2ParentCci, never the propagated cache).
        // Already-registered members the reader also touches: Universe.CurrentSystem,
        // Astronomical.Id, Celestial.Orbit, Orbit.Parent, IObjectId.Id,
        // IParentBody.GetCci2Cce, SimTime..ctor(seconds), StateVectors.PositionCci/VelocityCci.
        new("CelestialSystem.Count", typeof(CelestialSystem), "Count", MemberKind.Property, null, typeof(int), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("CelestialSystem.Id", typeof(CelestialSystem), "Id", MemberKind.Property, null, typeof(string), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("CelestialSystem.GetIndex(int)", typeof(CelestialSystem), "GetIndex", MemberKind.Method,
            [typeof(int)], typeof(Astronomical)),
        new("IParentBody.Mass", typeof(IParentBody), "Mass", MemberKind.Property, null, typeof(double), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Astronomical.MeanRadius", typeof(Astronomical), "MeanRadius", MemberKind.Property, null, typeof(double), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Orbit.GetStateVectorsAt(SimTime)", typeof(Orbit), "GetStateVectorsAt", MemberKind.Method,
            [typeof(SimTime)], typeof(StateVectors)),

        // Terrain-aware Surface-frame impact cut.
        new("Celestial.GetTerrainHeightFromDirCcf", typeof(Celestial),
            "GetTerrainHeightFromDirCcf", MemberKind.Method,
            [typeof(double3), typeof(bool)], typeof(double)),
        new("Celestial.MaxTerrainRadius", typeof(Celestial), "MaxTerrainRadius",
            MemberKind.Property, null, typeof(double), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Celestial.MaxTerrainHeightApprox", typeof(Celestial), "MaxTerrainHeightApprox",
            MemberKind.Property, null, typeof(double), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),

        // SOI-independence: stock SOI-sphere suppression (SoiIndicatorPatch). Shapes
        // verified in decompiled sources: GizmoParent.cs:155 (UpdateRenderData
        // signature), :114/:118 (Instances/RenderData public fields); GenericGizmo.cs:196
        // (PassIndex public field), :277 (GetSegmentDataByViewport), :170-172 (nested
        // PerSegmentData struct, Active public field), :20 (Static.GlassBallGizmoRenderData
        // singleton — pinning the field also pins the GlassBallGizmoRenderData type the
        // prefix type-tests against). Draw-site evidence: IOrbiter.cs:296-311 (the SOI
        // sphere, the game's only glass-ball user).
        new("GizmoParent.UpdateRenderData", typeof(GizmoParent), "UpdateRenderData", MemberKind.Method,
            [typeof(Viewport), typeof(int), typeof(int)], typeof(void)),
        new("GizmoParent.Instances", typeof(GizmoParent), "Instances", MemberKind.Field, null, typeof(List<GenericGizmo>), IsStatic: false),
        new("GizmoParent.RenderData", typeof(GizmoParent), "RenderData", MemberKind.Field, null, typeof(IGizmoRenderData), IsStatic: false),
        new("GenericGizmo.PassIndex", typeof(GenericGizmo), "PassIndex", MemberKind.Field, null, typeof(int), IsStatic: false),
        new("GenericGizmo.GetSegmentDataByViewport", typeof(GenericGizmo), "GetSegmentDataByViewport", MemberKind.Method,
            [typeof(Viewport)], typeof(GenericGizmo.PerSegmentData[])),
        new("PerSegmentData.Active", typeof(GenericGizmo.PerSegmentData), "Active", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("GenericGizmo.Static.GlassBallGizmoRenderData", typeof(GenericGizmo.Static), "GlassBallGizmoRenderData",
            MemberKind.Field, null, typeof(GlassBallGizmoRenderData), IsStatic: true),

        // Body-surface frames require the body's spin transform (BodyRotationReader).
        // Shapes verified in decompiled sources: IParentBody.cs:31
        // (GetCcf2Cce(SimTime) — Celestial.cs:560 composes the constant-rate UnitZ spin,
        // Celestial.cs:547, with the constant _cci2Cce tilt, Celestial.cs:585;
        // StellarBody.cs:126 returns Identity) and IParentBody.cs:68 (GetAngularVelocity
        // — Celestial.cs:196 returns the template spin rate rad/s, negative when
        // retrograde per Celestial.cs:612-629; StellarBody.cs:137 returns 0). The
        // SimTime-parameterized overload is disambiguated from the parameterless
        // sibling (IParentBody.cs:27) by the parameter list. Already-registered members
        // the reader also touches: Universe.CurrentSystem, CelestialSystem.Count,
        // CelestialSystem.GetIndex(int), Astronomical.Id, SimTime..ctor(seconds).
        new("IParentBody.GetCcf2Cce(SimTime)", typeof(IParentBody), "GetCcf2Cce", MemberKind.Method,
            [typeof(SimTime)], typeof(doubleQuat)),
        new("IParentBody.GetAngularVelocity", typeof(IParentBody), "GetAngularVelocity", MemberKind.Method,
            Type.EmptyTypes, typeof(double)),

        // Two-line display + ghost-cursor fix + burn-node markers.
        // Shapes verified in decompiled sources: Orbit.cs:2304 (GetNearestPoint — the
        // ONE hit-test funnel under hover, click, and burn-node drag, prefixed by
        // OrbitHoverPatch; PatchedConic?/OrbitPointCce? erase to their underlying
        // types at runtime, the out param is a byref Nullable); Camera.cs:401 (the
        // float2 ScreenToNdc overload), :342 (EgoToScreen(double3, bool)), :171
        // (GetForwardEcl — the ego-depth axis stock's own ignore-behind dots
        // against, :344/:366), :67 (NearPlane property), :272 (EgoToClipDouble —
        // the hover pruner's independent clip-space bound);
        // Viewport.cs:30 (Size public field); BurnPlan.cs:20 (BurnPatchColor static
        // Color.Preset — the user-configurable BurnLineColor setting,
        // GameSettings.cs:764); Burn.cs:115 (PositionCce computed property
        // BurnNodePatch prefixes), :96 (Patch property), :119 (ParentEjectBurn).
        new("Orbit.GetNearestPoint", typeof(Orbit), "GetNearestPoint", MemberKind.Method,
            [typeof(Viewport), typeof(float2), typeof(PatchedConic),
             typeof(OrbitPointCce?).MakeByRefType(), typeof(bool), typeof(float)], typeof(bool)),
        new("Camera.GetForwardEcl", typeof(Camera), "GetForwardEcl", MemberKind.Method,
            Type.EmptyTypes, typeof(double3)),
        new("Camera.NearPlane", typeof(Camera), "NearPlane", MemberKind.Property, null, typeof(float), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Camera.EgoToClipDouble", typeof(Camera), "EgoToClipDouble", MemberKind.Method,
            [typeof(double3)], typeof(double4)),
        // Click-to-place suppression (BurnClickPatch): the hover/click channel's one
        // funnel, Orbit.cs:2440 (CelestialPosition? erases to its underlying type at
        // runtime, the out param is a byref Nullable).
        new("Orbit.GetNearestPosition", typeof(Orbit), "GetNearestPosition", MemberKind.Method,
            [typeof(Viewport), typeof(float2), typeof(PatchedConic),
             typeof(CelestialPosition?).MakeByRefType(), typeof(bool), typeof(float)], typeof(bool)),
        new("Camera.ScreenToNdc(float2,float)", typeof(Camera), "ScreenToNdc", MemberKind.Method,
            [typeof(float2), typeof(float)], typeof(float3)),
        new("Camera.EgoToScreen(double3,bool)", typeof(Camera), "EgoToScreen", MemberKind.Method,
            [typeof(double3), typeof(bool)], typeof(float2)),
        new("Viewport.Size", typeof(Viewport), "Size", MemberKind.Field, null, typeof(int2), IsStatic: false),
        new("BurnPlan.BurnPatchColor", typeof(BurnPlan), "BurnPatchColor", MemberKind.Field, null, typeof(Color.Preset), IsStatic: true),
        new("Burn.PositionCce", typeof(Burn), "PositionCce", MemberKind.Property, null, typeof(double3), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Burn.Patch", typeof(Burn), "Patch", MemberKind.Property, null, typeof(PatchedConic), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        new("Burn.ParentEjectBurn", typeof(Burn), "ParentEjectBurn", MemberKind.Property, null, typeof(bool), IsStatic: false, RequiredAccessors: PropertyAccessors.Getter),
        // Ownership-rule stale fallback + gizmo shrink. Shapes
        // verified in decompiled sources: PatchedConic.cs:66 (HidePatch public bool —
        // DrawStalePatch0 honors stock's own hide flag); Burn.cs:394 (UpdateGizmos,
        // BurnGizmoPatch's postfix target), :78-82 (the three private GenericGizmo
        // fields reached via AccessTools field refs); GenericGizmo.cs:170-192
        // (PerSegmentData PositionEgo/Scale public fields — Active is pinned above
        // with the SOI-indicator entries).
        new("PatchedConic.HidePatch", typeof(PatchedConic), "HidePatch", MemberKind.Field, null, typeof(bool), IsStatic: false),
        new("Burn.UpdateGizmos", typeof(Burn), "UpdateGizmos", MemberKind.Method,
            [typeof(Viewport), typeof(double2)], typeof(void)),
        new("Burn.SphereGizmo", typeof(Burn), "SphereGizmo", MemberKind.Field, null, typeof(GenericGizmo), IsStatic: false),
        new("Burn.ConeGizmo", typeof(Burn), "ConeGizmo", MemberKind.Field, null, typeof(GenericGizmo), IsStatic: false),
        new("Burn.ConeReverseGizmo", typeof(Burn), "ConeReverseGizmo", MemberKind.Field, null, typeof(GenericGizmo), IsStatic: false),
        new("PerSegmentData.PositionEgo", typeof(GenericGizmo.PerSegmentData), "PositionEgo",
            MemberKind.Field, null, typeof(double3), IsStatic: false),
        new("PerSegmentData.Scale", typeof(GenericGizmo.PerSegmentData), "Scale",
            MemberKind.Field, null, typeof(double3), IsStatic: false),

        // Honest line markers (Ui.LineMarkers). Shapes verified in
        // decompiled sources: Program.cs:403 (MainViewport static property);
        // Camera.cs:317 (EclToScreen(double3, bool)); Viewport.cs:28 (Position public
        // float2 field); ImGuiHelper.cs:549 (the draw-list DrawTextOnScreen overload —
        // stock's own map-marker text path, CelestialPosition.cs:43).
        new("Program.MainViewport", typeof(Program), "MainViewport", MemberKind.Property, null, typeof(Viewport), IsStatic: true, RequiredAccessors: PropertyAccessors.Getter),
        new("Camera.EclToScreen(double3,bool)", typeof(Camera), "EclToScreen", MemberKind.Method,
            [typeof(double3), typeof(bool)], typeof(float2)),
        new("Viewport.Position", typeof(Viewport), "Position", MemberKind.Field, null, typeof(float2), IsStatic: false),
        new("ImGuiHelper.DrawTextOnScreen(drawList)", typeof(ImGuiHelper), "DrawTextOnScreen",
            MemberKind.StaticMethod,
            [typeof(Brutal.ImGuiApi.ImDrawListPtr), typeof(float2), typeof(Brutal.ImGuiApi.ImString), typeof(byte4)],
            typeof(void)),

        // Game constants the physics depends on
        new("Constants.GRAVITATIONAL_CONSTANT", typeof(Constants), "GRAVITATIONAL_CONSTANT", MemberKind.Field, null, typeof(double), IsStatic: true),
        new("Constants.SOLAR_MASS", typeof(Constants), "SOLAR_MASS", MemberKind.Field, null, typeof(double), IsStatic: true),
        new("Constants.EARTH_MASS", typeof(Constants), "EARTH_MASS", MemberKind.Field, null, typeof(double), IsStatic: true),
        new("Constants.LUNAR_MASS", typeof(Constants), "LUNAR_MASS", MemberKind.Field, null, typeof(double), IsStatic: true),
        new("Constants.JUPITER_MASS", typeof(Constants), "JUPITER_MASS", MemberKind.Field, null, typeof(double), IsStatic: true),
    ];
}
