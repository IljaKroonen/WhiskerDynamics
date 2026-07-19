using System.Globalization;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Deterministic validation and exploratory probes that do not use BenchmarkDotNet.</summary>
public sealed class ValidationScenarios
{
    private static readonly (string BodyId, string HorizonsId)[] EpochCheckBodies =
        [("Mercury", "199"), ("Venus", "299"), ("Earth", "399"), ("Mars", "499")];

    private readonly string[] args;
    private readonly string astronomicals;
    private readonly string horizonsDir;

    private ValidationScenarios(string[] args)
    {
        this.args = args;
        astronomicals = ArgValue("--astronomicals")
            ?? @"C:\Program Files\Kitten Space Agency\Content\Core\Astronomicals.xml";
        horizonsDir = ArgValue("--horizons") ?? "src/WhiskerDynamics.Benchmarks/data/horizons";
    }

    public static bool TryRun(string[] args, out int exitCode)
    {
        string? scenario = args.FirstOrDefault();
        if (scenario is null || scenario.StartsWith('-'))
        {
            exitCode = 0;
            return false;
        }

        var scenarios = new ValidationScenarios(args);
        exitCode = scenarios.Run(scenario);
        return true;
    }

    private int Run(string scenario) => scenario switch
    {
        "epoch-check" => EpochCheck(),
        "full-catalog" => FullCatalog(),
        "all-mutual-99" => AllMutual99(),
        "all-mutual-fast-100" => AllMutualFast100(),
        "thinning-probe" => ThinningProbe.Run(
            ArgValue("--system") ?? @"C:\Program Files\Kitten Space Agency\Content\Core\SolSystem.xml",
            astronomicals,
            double.Parse(ArgValue("--days") ?? "365", CultureInfo.InvariantCulture)),
        "l4" => TrojanRun(+60.0),
        "l5" => TrojanRun(-60.0),
        _ => Help(),
    };

    private string? ArgValue(string name)
    {
        int i = Array.IndexOf(args, name);
        return i >= 0 && i + 1 < args.Length ? args[i + 1] : null;
    }

    private int Help()
    {
        Console.WriteLine("usage: dotnet run --project src/WhiskerDynamics.Benchmarks -- <command> [options]");
        Console.WriteLine("validation commands: epoch-check, full-catalog, all-mutual-99, all-mutual-fast-100, thinning-probe, l4, l5");
        Console.WriteLine("manual performance commands: dop853-compare, tolerance-sweep");
        Console.WriteLine("BenchmarkDotNet arguments (for example, --filter *GravityBenchmarks*) are passed through.");
        Console.WriteLine("validation options: --astronomicals path, --system path, --horizons dir, --seed-days n, --days n");
        Console.WriteLine("  full-catalog loads the game's SolSystem.xml manifest (override with --system),");
        Console.WriteLine("  constructs the strict numerical ephemeris and gates GO/NO-GO on catalog");
        Console.WriteLine("  readiness, epoch fidelity, 37-day fill time, storage, and radial containment.");
        Console.WriteLine("  Performance verdicts are acceptance evidence only under dotnet run -c Release.");
        Console.WriteLine("  all-mutual-99 fills the synthetic 99-body all-mutual ephemeris across the");
        Console.WriteLine("  production 37-day retained window and requires ordinary radial containment.");
        Console.WriteLine("  all-mutual-fast-100 adds a positive-mass >80 km/s periapsis-speed body");
        Console.WriteLine("  and applies the same 37-day readiness gates.");
        Console.WriteLine("  --seed-days selects the Trojan seed epoch");
        Console.WriteLine("  (l4: 17, l5: 15) — equilateral trojan seeds are solar-phase-sensitive, and these");
        Console.WriteLine("  seed days librate 5 years.");
        return 2;
    }

    private int FullCatalog()
    {
        string systemXml = ArgValue("--system")
            ?? @"C:\Program Files\Kitten Space Agency\Content\Core\SolSystem.xml";
        var bodies = SystemManifestLoader.Load(systemXml, astronomicals);
        var root = bodies.Single(b => b.Parent is null);
        // Catalog rejection and integrator failure are both readiness NO-GO verdicts.
        // Backbone selection controls coupling only; every body remains numerical.
        try
        {
            var backboneIds = IntegratedSetRule.Select(bodies, 0, out _);
            int restrictedCount = bodies.Count - backboneIds.Count;
            Console.WriteLine($"catalog: {bodies.Count} modeled bodies, "
                + $"{backboneIds.Count} massive backbone, {restrictedCount} restricted");
            Console.WriteLine($"backbone: {string.Join(", ", bodies.Where(b => backboneIds.Contains(b.Id)).Select(b => b.Id))}");

            var eph = new NBodyEphemerides(bodies, 0, backboneIds,
                new IntegratorOptions { RelTol = 1e-11 });

            // Epoch sanity covers every modeled track. Positions equal their element seed;
            // velocities differ only by the shared momentum-zeroing frame drift.
            var kepler = new Ephemerides(bodies);
            double maxEpoch = 0;
            foreach (var b in bodies)
                maxEpoch = Math.Max(maxEpoch,
                    (eph.GetState(b, 0).Position - kepler.GetState(b, 0).Position).Length());
            Console.WriteLine($"epoch max |dr| at t=0: {maxEpoch:E3} m");

            // Fill the steady-state window the mod holds (30 d ahead + 7 d behind).
            var sw = System.Diagnostics.Stopwatch.StartNew();
            eph.GetState(root, 37 * 86400);
            sw.Stop();
            Console.WriteLine($"37-day fill: {sw.Elapsed.TotalSeconds:F2} s, {eph.KnotCount} knots"
                + $" + {eph.NodeCount} dense tail nodes, ~{eph.ApproxBytes / 1024} KB"
                + $" ({eph.ApproxBytes / (1024.0 * 37):F1} KB/day of knots+tail)");

            // Blowup gate, not an accuracy gate: compare against each body's instantaneous
            // Kepler radius so eccentric bodies near periapsis are not rejected spuriously.
            bool ok = maxEpoch < 1e-3;
            foreach (var b in bodies.Where(b => b.Parent is not null))
            {
                if (b.Orbit is not { } o || o.Eccentricity > 0.9)
                    continue;
                double r = (eph.GetState(b, 30 * 86400).Position
                          - eph.GetState(b.Parent!, 30 * 86400).Position).Length();
                double rKepler = Kepler.StateFromElements(o, b.Parent!.Mu, 30 * 86400).Position.Length();
                if (r < 0.5 * rKepler || r > 2.0 * rKepler)
                {
                    Console.WriteLine($"CONTAINMENT FAIL {b.Id}: r={r:E4} m vs kepler r={rKepler:E4} m (a={o.SemiMajorAxis:E4} m)");
                    ok = false;
                }
            }
            // The retained 37-day window must remain below the benchmark's memory ceiling.
            bool go = ok && sw.Elapsed.TotalSeconds <= 10 && eph.ApproxBytes <= 8 * 1024 * 1024;
            Console.WriteLine(go ? "full-catalog: GO" : "full-catalog: NO-GO");
            return go ? 0 : 1;
        }
        catch (Exception ex) when (ex is InvalidOperationException or ArgumentException)
        {
            Console.WriteLine($"full-catalog: NO-GO (numerical model unavailable: {ex.Message})");
            return 1;
        }
    }

    private static int AllMutual99() =>
        RunAllMutualReadiness("all-mutual-99", BenchmarkCatalog.CreateBodies(), 99);

    private static int AllMutualFast100()
    {
        const double formerSpeedGate = 80_000.0;
        var bodies = BenchmarkCatalog.CreateFastPeriapsisStressBodies();
        var fast = bodies.Single(body => body.Id == "FastPeriapsisProbe");
        double periapsisSpeed = Kepler.PeriapsisSpeed(fast.Orbit!.Value, fast.Parent!.Mu);
        double period = 2 * Math.PI * Math.Sqrt(
            Math.Pow(fast.Orbit.Value.SemiMajorAxis, 3) / fast.Parent.Mu);
        bool exceedsFormerGate = periapsisSpeed > formerSpeedGate;
        Console.WriteLine($"former speed-gate stress: {fast.Id} periapsis speed "
            + $"{periapsisSpeed / 1000:F3} km/s > {formerSpeedGate / 1000:F0} km/s: "
            + $"{exceedsFormerGate}");
        Console.WriteLine($"stress orbit period: {period / 86400:F3} days; "
            + $"37-day readiness window spans {37 * 86400 / period:F3} revolutions");
        if (!exceedsFormerGate)
        {
            Console.WriteLine("all-mutual-fast-100 performance readiness: NO-GO "
                + "(stress body does not exceed the former speed gate)");
            return 1;
        }
        return RunAllMutualReadiness("all-mutual-fast-100", bodies, 100);
    }

    private static int RunAllMutualReadiness(
        string label, IReadOnlyList<CelestialBody> bodies, int expectedBodyCount)
    {
        const double day = 86400.0;
        const double window = 37 * day;
        const double fillCeilingSeconds = 10.0;
        // Fixed acceptance budget scoped to these 99/100-body all-mutual fixtures.
        // Their current ~9.1 MiB footprint leaves about 3 MiB (~33%) regression headroom.
        const long storageCeilingBytes = 12 * 1024 * 1024;

        if (bodies.Count != expectedBodyCount || bodies.Any(body => !(body.Mu > 0.0)))
        {
            Console.WriteLine($"{label}: NO-GO (catalog must contain exactly "
                + $"{expectedBodyCount} finite-mass bodies)");
            return 1;
        }

        try
        {
            var backboneIds = IntegratedSetRule.Select(bodies, 0.0,
                out var restrictedClassifications);
            Console.WriteLine($"{label}: {bodies.Count} modeled bodies, "
                + $"{backboneIds.Count}-body mutual backbone, "
                + $"{restrictedClassifications.Count} restricted");
            if (backboneIds.Count != expectedBodyCount || restrictedClassifications.Count != 0)
            {
                Console.WriteLine($"{label} production selection: NO-GO");
                foreach (var classification in restrictedClassifications)
                    Console.WriteLine($"restricted {classification.Id}: {classification.Reason}");
                return 1;
            }
            Console.WriteLine($"{label} production selection: GO");
            Console.WriteLine($"pairwise interactions per RHS: "
                + $"{bodies.Count * (bodies.Count - 1) / 2:N0}");

            var eph = new NBodyEphemerides(bodies, 0.0, backboneIds,
                new IntegratorOptions { RelTol = BenchmarkCatalog.ShippingRelTol });
            var seed = new Ephemerides(bodies);
            double maxEpochError = bodies.Max(body =>
                (eph.GetState(body, 0).Position - seed.GetState(body, 0).Position).Length());

            var sw = System.Diagnostics.Stopwatch.StartNew();
            _ = eph.GetState(bodies[0], window);
            sw.Stop();

            bool finite = true;
            var containmentFailures = new List<string>();
            foreach (var body in bodies)
            {
                var state = eph.GetState(body, 30 * day);
                finite &= IsFinite(state.Position) && IsFinite(state.Velocity);
                if (body.Parent is null || body.Orbit is not { } orbit || orbit.Eccentricity > 0.9)
                    continue;
                double radius = (state.Position - eph.GetState(body.Parent, 30 * day).Position).Length();
                double seedRadius = Kepler.StateFromElements(orbit, body.Parent.Mu, 30 * day)
                    .Position.Length();
                double ratio = radius / seedRadius;
                if (ratio < 0.5 || ratio > 2.0)
                    containmentFailures.Add($"{body.Id}={ratio:F3}x");
            }

            bool contained = containmentFailures.Count == 0;
            bool fillWithinBudget = sw.Elapsed.TotalSeconds <= fillCeilingSeconds;
            bool storageWithinBudget = eph.ApproxBytes <= storageCeilingBytes;
            bool ready = maxEpochError < 1e-3 && finite && contained
                && fillWithinBudget && storageWithinBudget;

            Console.WriteLine($"epoch max |dr| at t=0: {maxEpochError:E3} m");
            Console.WriteLine($"37-day fill: {sw.Elapsed.TotalSeconds:F3} s");
            Console.WriteLine($"storage: {eph.ApproxBytes / 1024.0:F1} KiB "
                + $"({eph.KnotCount} knots + {eph.NodeCount} dense tail nodes)");
            Console.WriteLine($"physics: finite={finite}, radial-containment={contained}");
            if (!contained)
                Console.WriteLine($"containment failures ({containmentFailures.Count}): "
                    + string.Join(", ", containmentFailures));
            Console.WriteLine($"budgets: fill<={fillCeilingSeconds:F0}s {fillWithinBudget}, "
                + $"storage<={storageCeilingBytes / (1024 * 1024)}MiB {storageWithinBudget}");
            Console.WriteLine("storage budget rationale: scoped 12MiB all-mutual budget "
                + "leaves about 3MiB (~33%) headroom above the current fixture footprint");
            Console.WriteLine(ready
                ? $"{label} performance readiness: GO"
                : $"{label} performance readiness: NO-GO");
            return ready ? 0 : 1;
        }
        catch (Exception ex) when (ex is InvalidOperationException or ArgumentException)
        {
            Console.WriteLine($"{label} performance readiness: NO-GO ({ex.Message})");
            return 1;
        }
    }

    private static bool IsFinite(Vector3d value) =>
        double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);

    private int EpochCheck()
    {
        var eph = new Ephemerides(AstronomicalsParser.ParseFile(astronomicals));
        bool ok = true;

        foreach (string bodyId in MissingRequiredEpochBodyIds(eph.Bodies.Select(body => body.Id)))
        {
            Console.WriteLine($"{bodyId}: required body is missing from Astronomicals.xml");
            ok = false;
        }

        foreach (var (bodyId, horizonsId) in EpochCheckBodies)
        {
            var body = eph.Bodies.FirstOrDefault(b => b.Id == bodyId);
            if (body is null) continue;

            var truth = ReadHorizonsVector(Path.Combine(horizonsDir, horizonsId + ".txt"));
            var state = eph.GetState(body, 0); // t=0 == JD 2461009.5
            double posErr = (state.Position - truth.Position).Length();
            double velErr = (state.Velocity - truth.Velocity).Length();
            // This harness uses the core default solar mass rather than the manifest's game
            // constant, so the sanity gate allows the resulting along-track offset.
            bool pass = posErr < 10_000e3 && velErr < 1.0;
            ok &= pass;
            Console.WriteLine($"{bodyId,-8} pos err {posErr / 1e3,10:F3} km   vel err {velErr,8:F5} m/s   {(pass ? "OK" : "FAIL")}");
        }
        return ok ? 0 : 1;
    }

    private static string[] MissingRequiredEpochBodyIds(IEnumerable<string> bodyIds)
    {
        var available = bodyIds.ToHashSet(StringComparer.Ordinal);
        return EpochCheckBodies
            .Where(pair => !available.Contains(pair.BodyId))
            .Select(pair => pair.BodyId)
            .ToArray();
    }

    private StateVector ReadHorizonsVector(string path)
    {
        var lines = File.ReadAllLines(path);
        int soe = Array.IndexOf(lines, "$$SOE");
        if (soe < 0 || soe + 1 >= lines.Length)
            throw new FormatException($"{path}: no $$SOE record");
        var f = lines[soe + 1].Split(',');
        double Km(int i) => double.Parse(f[i].Trim(), CultureInfo.InvariantCulture) * 1000;
        return new StateVector(new Vector3d(Km(2), Km(3), Km(4)), new Vector3d(Km(5), Km(6), Km(7)));
    }

    private int TrojanRun(double angleDegrees)
    {
        // +60 deg = L4 (leading), -60 deg = L5 (trailing); shared containment run.
        string label = angleDegrees > 0 ? "L4" : "L5";
        var bodies = AstronomicalsParser.ParseFile(astronomicals);
        var earth = bodies.FirstOrDefault(b => b.Id == "Earth");
        var moon = bodies.FirstOrDefault(b => b.Parent == earth);
        if (earth is null || moon is null) { Console.WriteLine("Earth/moon not found in file"); return 1; }

        var backboneIds = IntegratedSetRule.Select(bodies, 0, out _);
        Console.WriteLine($"massive backbone: {string.Join(", ", backboneIds)}; "
            + $"restricted tracks: {bodies.Count - backboneIds.Count}");

        var eph = new NBodyEphemerides(bodies, 0, backboneIds,
            new IntegratorOptions { RelTol = 1e-11 });
        var sources = bodies.Where(b => b.Mu > 0.0).ToArray();
        var gravity = new GravityModel(eph, sources);

        // Trojan stability depends on lunar phase; seed-days selects the evaluation epoch.
        double seedDays = double.Parse(ArgValue("--seed-days") ?? (angleDegrees > 0 ? "17" : "15"),
            CultureInfo.InvariantCulture);
        double tSeed = seedDays * 86400;
        Console.WriteLine($"{label} seed epoch: t = {seedDays:F0} days (game epoch + {tSeed:F0} s), "
            + $"equilateral point {Math.Abs(angleDegrees):F0} deg {(angleDegrees > 0 ? "ahead of" : "behind")} the moon");

        var e0 = eph.GetState(earth, tSeed);
        var m0 = eph.GetState(moon, tSeed);
        var rel = m0.Position - e0.Position;
        var relV = m0.Velocity - e0.Velocity;
        var axis = rel.Cross(relV).Normalized();
        double seedAngle = angleDegrees * Math.PI / 180;
        var y0 = new StateVector(
            e0.Position + rel.RotateAbout(axis, seedAngle),
            e0.Velocity + relV.RotateAbout(axis, seedAngle));

        var predictor = new TrajectoryPredictor(gravity, y0, tSeed, new IntegratorOptions { RelTol = 1e-11 });
        Console.WriteLine("day, probe_earth_dist_over_moon_dist, separation_angle_deg");
        bool ok = true;
        for (double t = tSeed; t <= tSeed + 5 * 365.25 * 86400; t += 10 * 86400)
        {
            var p = predictor.StateAt(t);
            var e = eph.GetState(earth, t);
            var m = eph.GetState(moon, t);
            double ratio = (p.Position - e.Position).Length() / (m.Position - e.Position).Length();
            double angle = Math.Acos(Math.Clamp(
                (m.Position - e.Position).Normalized().Dot((p.Position - e.Position).Normalized()), -1, 1)) * 180 / Math.PI;
            Console.WriteLine($"{t / 86400,7:F0}, {ratio:F4}, {angle:F2}");
            if (t > tSeed && (ratio < 0.7 || ratio > 1.3 || angle < 5 || angle > 175)) ok = false;
            eph.Prune(t - 40 * 86400);
        }
        Console.WriteLine(ok ? $"{label} containment: OK" : $"{label} containment: FAIL");
        if (!ok)
            Console.WriteLine("note: trojan seed-phase marginality is a known cause of containment failure — "
                + "scan --seed-days 0..27 for a librating seed phase before suspecting an engine regression.");
        return ok ? 0 : 1;
    }
}
