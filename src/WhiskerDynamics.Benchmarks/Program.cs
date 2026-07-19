using BenchmarkDotNet.Running;
using WhiskerDynamics.Benchmarks;

// Run everything:   dotnet run -c Release --project src/WhiskerDynamics.Benchmarks -- --filter *
// One area:         ... -- --filter *GravityBenchmarks*
// Lunar fidelity:   ... -- --filter *LunarGravity*Benchmarks*
// One benchmark:    ... -- --filter *PredictLeo30Days*
// List benchmarks:  ... -- --list flat
// DOP853 comparison: ... -- dop853-compare (manual stopwatch harness, not BDN)
// Tolerance tradeoff:  ... -- tolerance-sweep (runtime, work, and endpoint error)
// Dominant status:   ... -- --filter *DominantAttractorStatusBenchmarks*
// Third-body refresh: ... -- --filter *ThirdBodySnapshotBenchmarks*
// Physics/burn fidelity: ... -- fidelity-probe (deterministic, headless numerical comparison)
// All-mutual readiness: ... -- all-mutual-99 (99 bodies, production 37-day window)
// Removed speed-gate stress: ... -- all-mutual-fast-100 (>80 km/s, 37-day window)
if (args is ["fidelity-probe"])
{
    return FidelityProbe.Run();
}
if (args is ["dop853-compare"])
{
    HighOrderComparison.Run();
    return 0;
}
if (args is ["tolerance-sweep"])
{
    IntegratorToleranceSweep.Run();
    return 0;
}
// Validation commands: ... -- help
if (ValidationScenarios.TryRun(args, out int validationExitCode))
    return validationExitCode;

BenchmarkSwitcher.FromAssembly(typeof(BenchmarkCatalog).Assembly).Run(args);
return 0;
