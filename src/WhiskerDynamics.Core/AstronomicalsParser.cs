using System.Globalization;
using System.Xml.Linq;

namespace WhiskerDynamics.Core;

/// <summary>Parses KSA's Content/Core/Astronomicals.xml into celestial bodies (SI units).</summary>
public static class AstronomicalsParser
{
    public static IReadOnlyList<CelestialBody> ParseFile(string path, MassConstants? constants = null)
    {
        using var stream = File.OpenRead(path);
        return Parse(stream, constants);
    }

    public static IReadOnlyList<CelestialBody> Parse(Stream xml, MassConstants? constants = null)
    {
        var c = constants ?? new MassConstants();
        if (!double.IsFinite(c.G) || c.G <= 0)
            throw new FormatException(
                $"Astronomicals gravitational constant must be finite and positive ({c.G}).");
        var root = XDocument.Load(xml).Root
            ?? throw new FormatException("Astronomicals XML has no root element.");

        // First collect only identity and parent edges. Validate that raw graph before
        // parsing physical fields or linking mutable CelestialBody objects: malformed
        // fallback data must fail with a finite, useful diagnostic before any
        // downstream graph consumer can observe it.
        var entries = new List<(XElement Element, string Id, string? ParentId)>();
        var indexById = new Dictionary<string, int>(StringComparer.Ordinal);
        foreach (var el in root.Elements())
        {
            string? id = el.Attribute("Id")?.Value;
            var massEl = el.Element("Mass");
            if (id is null || massEl is null) continue; // not a celestial body entry

            if (!indexById.TryAdd(id, entries.Count))
                throw new FormatException($"Astronomicals XML contains duplicate body id '{id}'.");
            entries.Add((el, id, el.Attribute("Parent")?.Value));
        }

        var links = new ParentGraphLink[entries.Count];
        for (int i = 0; i < entries.Count; i++)
        {
            var entry = entries[i];
            int parentIndex = -1;
            if (entry.ParentId is { } parentId
                && !indexById.TryGetValue(parentId, out parentIndex))
                throw new FormatException(
                    $"Body '{entry.Id}' references unknown parent '{parentId}'.");
            links[i] = new ParentGraphLink(entry.Id, parentIndex);
        }

        var graph = ParentGraphAnalyzer.Analyze(links);
        if (graph.Cycles.Length > 0)
            throw new FormatException("Astronomicals parent cycle detected: "
                + graph.FormatCycles() + ".");
        if (graph.RootIndices.Length != 1)
        {
            string rootIds = string.Join(", ",
                graph.RootIndices.Select(i => $"'{graph.IdAt(i)}'"));
            throw new FormatException(
                $"Astronomicals XML expected exactly one parentless body, found "
                + $"{graph.RootIndices.Length}"
                + (rootIds.Length == 0 ? "." : $" ({rootIds})."));
        }
        int rootIndex = graph.RootIndices[0];
        int[] unreachable = graph.UnreachableFrom(rootIndex);
        if (unreachable.Length > 0)
            throw new FormatException(
                $"Astronomicals bodies are not reachable from root "
                + $"'{graph.IdAt(rootIndex)}': "
                + string.Join(", ", unreachable.Select(i => $"'{graph.IdAt(i)}'")) + ".");

        var bodies = new List<CelestialBody>(entries.Count);
        var orbitFrames = new Dictionary<string, string?>(StringComparer.Ordinal);
        foreach (var entry in entries)
        {
            var el = entry.Element;
            string id = entry.Id;
            var massEl = el.Element("Mass")!; // entry collection required it

            double massKg = ParseMassKg(massEl, id, c);
            bool isRoot = bodies.Count == rootIndex;
            if (!double.IsFinite(massKg) || massKg < 0 || (isRoot && massKg <= 0))
                throw new FormatException($"Body '{id}': mass must be "
                    + (isRoot ? "finite and positive" : "finite and nonnegative")
                    + $" ({massKg} kg).");
            double mu = c.G * massKg;
            if (!double.IsFinite(mu) || mu < 0 || (massKg != 0 && mu == 0))
                throw new FormatException(
                    $"Body '{id}': mass derives an invalid gravitational parameter ({mu}).");
            double meanRadius = ParseKm(el.Element("MeanRadius")?.Attribute("Km"), 0) * 1000;
            if (!double.IsFinite(meanRadius) || meanRadius < 0)
                throw new FormatException(
                    $"Body '{id}': mean radius must be finite and nonnegative ({meanRadius} m).");

            var orbitEl = el.Element("Orbit");
            var body = new CelestialBody
            {
                Id = id,
                Mu = mu,
                MeanRadius = meanRadius,
                Orbit = ParseOrbit(orbitEl, id),
            };
            bodies.Add(body);
            if (orbitEl is not null) orbitFrames[id] = orbitEl.Attribute("DefinitionFrame")?.Value;
        }

        for (int i = 0; i < bodies.Count; i++)
            if (links[i].ParentIndex >= 0)
                bodies[i].Parent = bodies[links[i].ParentIndex];

        foreach (var b in bodies)
        {
            if (b.Parent is null)
            {
                if (b.Orbit is not null)
                    throw new FormatException($"Root body '{b.Id}' must not have an Orbit element.");
                continue;
            }
            if (b.Orbit is null)
                throw new FormatException($"Body '{b.Id}' has a parent but no Orbit element.");
            ValidateOrbit(b.Id, b.Orbit.Value, b.Parent.Mu);
        }

        // The XML fallback does not carry the game's resolved SOI property. Rebuild
        // the classical Laplace sphere from the defining conic and mass ratio.
        // Hyperbolic/parabolic bodies have no closed parent-owned SOI.
        foreach (var b in bodies)
        {
            b.SphereOfInfluence = b.Parent is null
                ? double.PositiveInfinity
                : b.Orbit is { SemiMajorAxis: > 0 } orbit
                    ? orbit.SemiMajorAxis * Math.Pow(b.Mu / b.Parent.Mu, 0.4)
                    : double.NaN;
        }

        // Frame guard (needs resolved parents): our Kepler evaluation models only the game's
        // Ecliptic definition frame. The game's default when the attribute is absent is
        // Equatorial — the parent's equatorial frame — which coincides with the ecliptic only
        // for children of the root body (the root's equatorial frame IS the ecliptic frame).
        foreach (var b in bodies)
        {
            if (!orbitFrames.TryGetValue(b.Id, out var frame)) continue;
            if (frame == "Ecliptic") continue;
            if (frame is null && b.Parent?.Parent is null) continue;
            throw new FormatException(
                $"Body '{b.Id}': Orbit DefinitionFrame is {(frame is null ? "absent" : $"'{frame}'")} — " +
                "non-root-child orbits require DefinitionFrame=\"Ecliptic\" " +
                "(the game's default Equatorial frame is not supported).");
        }

        return bodies;
    }

    /// <summary>Astronomical unit in metres, as used by the game's DistanceReference.</summary>
    private const double AuMetres = 149597870700.0;

    private static OrbitalElements? ParseOrbit(XElement? orbit, string bodyId)
    {
        if (orbit is null) return null;
        double eccentricity = Required(orbit, "Eccentricity", "Value", bodyId);
        return new OrbitalElements(
            SemiMajorAxis: ParseSemiMajorAxisMetres(orbit, eccentricity, bodyId),
            Eccentricity: eccentricity,
            Inclination: Required(orbit, "Inclination", "Degrees", bodyId) * Math.PI / 180,
            LongitudeOfAscendingNode: Required(orbit, "LongitudeOfAscendingNode", "Degrees", bodyId) * Math.PI / 180,
            ArgumentOfPeriapsis: Required(orbit, "ArgumentOfPeriapsis", "Degrees", bodyId) * Math.PI / 180,
            TimeAtPeriapsis: ParseTimeAtPeriapsisSeconds(orbit.Element("TimeAtPeriapsis"), bodyId));
    }

    private static double ParseSemiMajorAxisMetres(XElement orbit, double eccentricity, string bodyId)
    {
        if (ParseLengthMetres(orbit.Element("SemiMajorAxis")) is { } a) return a;
        // Interstellar comets in the game file give periapsis distance instead: q = a(1 - e),
        // so a = q / (1 - e) — negative for hyperbolic orbits, matching the game's OrbitTemplate.
        if (ParseLengthMetres(orbit.Element("Periapsis")) is { } q) return q / (1 - eccentricity);
        throw new FormatException($"Body '{bodyId}': Orbit has no <SemiMajorAxis> or <Periapsis> with a Km|Au attribute.");
    }

    private static double? ParseLengthMetres(XElement? element)
    {
        if (element?.Attribute("Km")?.Value is { } km) return double.Parse(km, CultureInfo.InvariantCulture) * 1000;
        if (element?.Attribute("Au")?.Value is { } au) return double.Parse(au, CultureInfo.InvariantCulture) * AuMetres;
        return null;
    }

    private static double ParseTimeAtPeriapsisSeconds(XElement? element, string bodyId)
    {
        // Absent element means periapsis at t=0, matching the game's OrbitTemplate default.
        if (element is null) return 0;
        if (element.Attribute("Seconds")?.Value is { } s) return double.Parse(s, CultureInfo.InvariantCulture);
        if (element.Attribute("Days")?.Value is { } d) return double.Parse(d, CultureInfo.InvariantCulture) * 86400.0;
        if (element.Attribute("Months")?.Value is { } mo) return double.Parse(mo, CultureInfo.InvariantCulture) * 2592000.0; // 30-day months, per the game's TimeSpanReference
        throw new FormatException($"Body '{bodyId}': TimeAtPeriapsis has no recognised unit attribute (Seconds|Days|Months).");
    }

    private static double Required(XElement orbit, string element, string attribute, string bodyId)
    {
        string? raw = orbit.Element(element)?.Attribute(attribute)?.Value;
        if (raw is null)
            throw new FormatException($"Body '{bodyId}': Orbit is missing <{element} {attribute}=...>.");
        return double.Parse(raw, CultureInfo.InvariantCulture);
    }

    private static double ParseMassKg(XElement mass, string bodyId, MassConstants c)
    {
        if (mass.Attribute("Kg")?.Value is { } kg) return double.Parse(kg, CultureInfo.InvariantCulture);
        if (mass.Attribute("Yg")?.Value is { } yg) return double.Parse(yg, CultureInfo.InvariantCulture) * 1e21;
        if (mass.Attribute("Suns")?.Value is { } suns) return double.Parse(suns, CultureInfo.InvariantCulture) * c.SolarMassKg;
        // Units below appear in the real game file; conversion factors match the game's own
        // MassReference accepts metric-prefix grams and named-body units.
        // Metric-prefix gram conversions stay literal — they are unit definitions, not physical constants.
        if (mass.Attribute("Eg")?.Value is { } eg) return double.Parse(eg, CultureInfo.InvariantCulture) * 1e15;
        if (mass.Attribute("Pg")?.Value is { } pg) return double.Parse(pg, CultureInfo.InvariantCulture) * 1e12;
        if (mass.Attribute("Tg")?.Value is { } tg) return double.Parse(tg, CultureInfo.InvariantCulture) * 1e9;
        if (mass.Attribute("Earths")?.Value is { } earths) return double.Parse(earths, CultureInfo.InvariantCulture) * c.EarthMassKg;
        if (mass.Attribute("Lunars")?.Value is { } lunars) return double.Parse(lunars, CultureInfo.InvariantCulture) * c.LunarMassKg;
        if (mass.Attribute("Jupiters")?.Value is { } jupiters) return double.Parse(jupiters, CultureInfo.InvariantCulture) * c.JupiterMassKg;
        throw new FormatException($"Body '{bodyId}': Mass has no recognised unit attribute (Kg|Yg|Eg|Pg|Tg|Suns|Earths|Lunars|Jupiters).");
    }

    private static double ParseKm(XAttribute? attr, double fallback) =>
        attr is null ? fallback : double.Parse(attr.Value, CultureInfo.InvariantCulture);

    private static void ValidateOrbit(string bodyId, in OrbitalElements orbit, double parentMu)
    {
        if (!double.IsFinite(parentMu) || parentMu <= 0)
            throw new FormatException(
                $"Body '{bodyId}': parent gravitational parameter must be finite and positive ({parentMu}).");
        double[] values =
        [
            orbit.SemiMajorAxis, orbit.Eccentricity, orbit.Inclination,
            orbit.LongitudeOfAscendingNode, orbit.ArgumentOfPeriapsis,
            orbit.TimeAtPeriapsis,
        ];
        if (values.Any(value => !double.IsFinite(value)))
            throw new FormatException($"Body '{bodyId}': orbit contains a non-finite value.");
        if (orbit.Eccentricity < 0
            || orbit.Inclination < 0 || orbit.Inclination > Math.PI
            || !((orbit.Eccentricity < 1 && orbit.SemiMajorAxis > 0)
                || (orbit.Eccentricity > 1 && orbit.SemiMajorAxis < 0)))
            throw new FormatException(
                $"Body '{bodyId}': physically inconsistent orbit "
                + $"(a={orbit.SemiMajorAxis}, e={orbit.Eccentricity}, i={orbit.Inclination}).");
        double periapsisDistance = Kepler.PeriapsisDistance(orbit);
        double periapsisSpeed = Kepler.PeriapsisSpeed(orbit, parentMu);
        if (!(periapsisDistance > 0) || !double.IsFinite(periapsisDistance)
            || !(periapsisSpeed > 0) || !double.IsFinite(periapsisSpeed))
            throw new FormatException(
                $"Body '{bodyId}': orbit has an invalid physical periapsis.");
        StateVector state;
        try
        {
            state = Kepler.StateFromElements(orbit, parentMu, 0);
        }
        catch (Exception e) when (
            e is NotSupportedException or InvalidOperationException or ArithmeticException)
        {
            throw new FormatException(
                $"Body '{bodyId}': orbit is not physically evaluable: {e.Message}", e);
        }
        if (!IsFinite(state.Position) || !IsFinite(state.Velocity))
            throw new FormatException(
                $"Body '{bodyId}': orbit evaluates to a non-finite state.");
    }

    private static bool IsFinite(in Vector3d value) =>
        double.IsFinite(value.X) && double.IsFinite(value.Y) && double.IsFinite(value.Z);
}
