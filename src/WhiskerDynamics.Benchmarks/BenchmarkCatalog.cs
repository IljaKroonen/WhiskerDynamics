using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Deterministic synthetic solar system at the target catalog scale:
/// 50 finite-mass bodies (Sun, 9 planets incl. Pluto, 23 major moons, 17 dwarf
/// planets/asteroids) plus 49 finite-mass small moons and comets — 99 modeled bodies
/// total. The catalog supports both the legacy 50+49 composite workload and the
/// production all-mutual workload without prescribing any body's future conic.
///
/// The repo does not check in the game's Astronomicals.xml, so this catalog is
/// synthetic: backbone masses (mu) and every track's semi-major axis, eccentricity
/// and inclination are approximately real solar-system values (what drives gravity
/// cost and adaptive step counts), including realistic nonzero mu for restricted rows.
/// Orbit orientation angles and phases are deterministic golden-ratio values. These
/// It is not a bit-identical game catalog.</summary>
public static class BenchmarkCatalog
{
    /// <summary>Relative tolerance used by rails and vessel benchmarks.</summary>
    public const double ShippingRelTol = 1e-11;

    private const bool I = true;  // finite-mass mutual-backbone body
    private const bool K = false; // finite-mass restricted-track body

    // NominalMu preserves realistic catalog scale for every modeled body. Backbone
    // membership controls mutual coupling; it does not erase restricted-body gravity.
    private static readonly (string Id, string? Parent, double NominalMu, double A, double E,
        double IncDeg, bool Backbone)[] Rows =
    [
        ("Sun", null, 1.32712440018e20, 0, 0, 0, I),

        // Planets (+ Pluto)
        ("Mercury", "Sun", 2.20320e13, 5.7909e10, 0.2056, 7.005, I),
        ("Venus", "Sun", 3.24859e14, 1.0821e11, 0.0068, 3.395, I),
        ("Earth", "Sun", 3.986004418e14, 1.49598e11, 0.0167, 0.001, I),
        ("Mars", "Sun", 4.282837e13, 2.2794e11, 0.0934, 1.850, I),
        ("Jupiter", "Sun", 1.26686534e17, 7.7857e11, 0.0489, 1.303, I),
        ("Saturn", "Sun", 3.7931187e16, 1.43353e12, 0.0565, 2.485, I),
        ("Uranus", "Sun", 5.793939e15, 2.87246e12, 0.0457, 0.773, I),
        ("Neptune", "Sun", 6.836529e15, 4.49506e12, 0.0113, 1.770, I),
        ("Pluto", "Sun", 8.710e11, 5.90638e12, 0.2488, 17.16, I),

        // Major moons in the mutual backbone.
        ("Luna", "Earth", 4.9048695e12, 3.8440e8, 0.0549, 5.145, I),
        ("Io", "Jupiter", 5.959916e12, 4.2170e8, 0.0041, 0.050, I),
        ("Europa", "Jupiter", 3.202739e12, 6.7090e8, 0.0094, 0.470, I),
        ("Ganymede", "Jupiter", 9.887834e12, 1.0704e9, 0.0013, 0.200, I),
        ("Callisto", "Jupiter", 7.179289e12, 1.8827e9, 0.0074, 0.192, I),
        ("Mimas", "Saturn", 2.5026e9, 1.8552e8, 0.0196, 1.574, I),
        ("Enceladus", "Saturn", 7.2027e9, 2.3802e8, 0.0047, 0.009, I),
        ("Tethys", "Saturn", 4.12067e10, 2.9466e8, 0.0010, 1.120, I),
        ("Dione", "Saturn", 7.31127e10, 3.7742e8, 0.0022, 0.019, I),
        ("Rhea", "Saturn", 1.53939e11, 5.2707e8, 0.0010, 0.345, I),
        ("Titan", "Saturn", 8.978138e12, 1.22187e9, 0.0288, 0.349, I),
        ("Hyperion", "Saturn", 3.708e8, 1.50093e9, 0.1042, 0.430, I),
        ("Iapetus", "Saturn", 1.20512e11, 3.5613e9, 0.0283, 15.47, I),
        ("Phoebe", "Saturn", 5.531e8, 1.29469e10, 0.1635, 175.98, I),
        ("Miranda", "Uranus", 4.400e9, 1.2939e8, 0.0013, 4.232, I),
        ("Ariel", "Uranus", 8.346e10, 1.9090e8, 0.0012, 0.260, I),
        ("Umbriel", "Uranus", 8.509e10, 2.6600e8, 0.0039, 0.128, I),
        ("Titania", "Uranus", 2.269e11, 4.3591e8, 0.0011, 0.340, I),
        ("Oberon", "Uranus", 2.053e11, 5.8352e8, 0.0014, 0.058, I),
        ("Triton", "Neptune", 1.42789e12, 3.5476e8, 0.0010, 156.87, I),
        ("Proteus", "Neptune", 2.940e9, 1.17647e8, 0.0010, 0.524, I),
        ("Nereid", "Neptune", 2.060e9, 5.5134e9, 0.7507, 7.090, I),
        ("Charon", "Pluto", 1.0587e11, 1.9591e7, 0.0010, 0.080, I),

        // Dwarf planets and heavyweight asteroids in the mutual backbone.
        ("Ceres", "Sun", 6.26325e10, 4.1400e11, 0.0758, 10.59, I),
        ("Vesta", "Sun", 1.729e10, 3.5320e11, 0.0887, 7.140, I),
        ("Pallas", "Sun", 1.360e10, 4.1400e11, 0.2310, 34.90, I),
        ("Hygiea", "Sun", 5.800e9, 4.7000e11, 0.1120, 3.830, I),
        ("Juno", "Sun", 1.820e9, 3.9900e11, 0.2570, 12.99, I),
        ("Psyche", "Sun", 1.530e9, 4.3700e11, 0.1340, 3.100, I),
        ("Interamnia", "Sun", 2.600e9, 4.5800e11, 0.1550, 17.30, I),
        ("Davida", "Sun", 1.800e9, 4.7500e11, 0.1880, 15.90, I),
        ("Eunomia", "Sun", 2.100e9, 3.9500e11, 0.1860, 11.75, I),
        ("Eris", "Sun", 1.108e12, 1.01520e13, 0.4360, 44.04, I),
        ("Haumea", "Sun", 2.674e11, 6.4780e12, 0.1960, 28.20, I),
        ("Makemake", "Sun", 2.070e11, 6.7960e12, 0.1610, 28.98, I),
        ("Quaoar", "Sun", 8.000e10, 6.5370e12, 0.0380, 7.990, I),
        ("Orcus", "Sun", 4.100e10, 5.8960e12, 0.2260, 20.59, I),
        ("Gonggong", "Sun", 1.170e11, 1.00720e13, 0.5030, 30.63, I),
        ("Varuna", "Sun", 2.500e10, 6.4510e12, 0.0560, 17.20, I),
        ("Ixion", "Sun", 2.000e10, 5.9330e12, 0.2450, 19.60, I),

        // Finite-mass restricted tracks (small moons and comets).
        ("Phobos", "Mars", 7.100e5, 9.3760e6, 0.0151, 1.100, K),
        ("Deimos", "Mars", 9.600e4, 2.3460e7, 0.0010, 1.800, K),
        ("Metis", "Jupiter", 8.000e6, 1.2800e8, 0.0012, 0.060, K),
        ("Adrastea", "Jupiter", 5.000e5, 1.2900e8, 0.0018, 0.054, K),
        ("Amalthea", "Jupiter", 1.400e8, 1.8140e8, 0.0032, 0.374, K),
        ("Thebe", "Jupiter", 3.000e7, 2.2190e8, 0.0176, 1.076, K),
        ("Himalia", "Jupiter", 2.800e8, 1.1461e10, 0.1600, 27.50, K),
        ("Elara", "Jupiter", 5.800e7, 1.1740e10, 0.2200, 26.60, K),
        ("Pasiphae", "Jupiter", 2.000e7, 2.3570e10, 0.4100, 151.4, K),
        ("Sinope", "Jupiter", 5.000e6, 2.3700e10, 0.2500, 158.1, K),
        ("Lysithea", "Jupiter", 4.200e6, 1.1720e10, 0.1100, 28.30, K),
        ("Carme", "Jupiter", 8.800e6, 2.3400e10, 0.2500, 164.9, K),
        ("Ananke", "Jupiter", 2.000e6, 2.1280e10, 0.2400, 148.9, K),
        ("Leda", "Jupiter", 7.300e5, 1.1170e10, 0.1600, 27.50, K),
        ("Pan", "Saturn", 3.300e5, 1.3358e8, 0.0010, 0.001, K),
        ("Atlas", "Saturn", 4.400e5, 1.3767e8, 0.0012, 0.003, K),
        ("Prometheus", "Saturn", 1.070e7, 1.3938e8, 0.0022, 0.008, K),
        ("Pandora", "Saturn", 9.200e6, 1.4172e8, 0.0042, 0.050, K),
        ("Epimetheus", "Saturn", 3.510e7, 1.5141e8, 0.0098, 0.335, K),
        ("Janus", "Saturn", 1.266e8, 1.5146e8, 0.0068, 0.165, K),
        ("Methone", "Saturn", 6.000e2, 1.9400e8, 0.0010, 0.007, K),
        ("Pallene", "Saturn", 2.200e3, 2.1200e8, 0.0040, 0.181, K),
        ("Telesto", "Saturn", 4.800e5, 2.9471e8, 0.0010, 1.180, K),
        ("Calypso", "Saturn", 2.500e5, 2.9471e8, 0.0010, 1.500, K),
        ("Helene", "Saturn", 7.600e5, 3.7742e8, 0.0071, 0.213, K),
        ("Polydeuces", "Saturn", 3.000e3, 3.7742e8, 0.0192, 0.177, K),
        ("Cordelia", "Uranus", 3.000e6, 4.9800e7, 0.0010, 0.085, K),
        ("Ophelia", "Uranus", 3.600e6, 5.3800e7, 0.0099, 0.104, K),
        ("Bianca", "Uranus", 6.200e6, 5.9200e7, 0.0009, 0.193, K),
        ("Cressida", "Uranus", 2.290e7, 6.1800e7, 0.0004, 0.006, K),
        ("Desdemona", "Uranus", 1.190e7, 6.2700e7, 0.0001, 0.113, K),
        ("Juliet", "Uranus", 3.720e7, 6.4400e7, 0.0007, 0.065, K),
        ("Portia", "Uranus", 1.120e8, 6.6100e7, 0.0001, 0.059, K),
        ("Rosalind", "Uranus", 1.700e7, 6.9900e7, 0.0001, 0.279, K),
        ("Belinda", "Uranus", 2.400e7, 7.5300e7, 0.0001, 0.031, K),
        ("Puck", "Uranus", 1.930e8, 8.6000e7, 0.0001, 0.319, K),
        ("Naiad", "Neptune", 1.300e7, 4.8200e7, 0.0004, 4.746, K),
        ("Thalassa", "Neptune", 2.500e7, 5.0100e7, 0.0002, 0.209, K),
        ("Despina", "Neptune", 1.400e8, 5.2500e7, 0.0002, 0.064, K),
        ("Galatea", "Neptune", 2.500e8, 6.2000e7, 0.0001, 0.062, K),
        ("Larissa", "Neptune", 3.300e8, 7.3500e7, 0.0014, 0.205, K),
        // Small moons of non-binary dwarf-planet parents. Parameters are approximate:
        // Dysnomia a≈37,300 km; Hiʻiaka a≈49,900 km; Namaka a≈25,700 km; Weywot
        // a≈13,300 km. Unlike Pluto's circumbinary small moons, these are compatible
        // with this catalog's parent-relative conic seed representation.
        ("Dysnomia", "Eris", 2.000e9, 3.7300e7, 0.0040, 0.000, K),
        ("Hiʻiaka", "Haumea", 1.200e9, 4.9900e7, 0.0510, 1.000, K),
        ("Namaka", "Haumea", 1.200e8, 2.5700e7, 0.2490, 13.00, K),
        ("Weywot", "Quaoar", 2.000e8, 1.3300e7, 0.1400, 14.00, K),
        ("Halley", "Sun", 1.500e4, 2.6670e12, 0.9670, 162.3, K),
        ("Encke", "Sun", 6.000e2, 3.3100e11, 0.8480, 11.78, K),
        ("Churyumov", "Sun", 6.662e2, 5.1800e11, 0.6410, 7.040, K),
        ("Tempel1", "Sun", 5.000e3, 4.6800e11, 0.5100, 10.47, K),
    ];

    /// <summary>Ids of the 50 finite-mass mutual-backbone bodies (root included).</summary>
    public static IReadOnlyCollection<string> BackboneIds { get; } =
        Rows.Where(r => r.Backbone).Select(r => r.Id).ToHashSet();

    /// <summary>Ids of all 99 finite-mass bodies for uncapped mutual propagation.</summary>
    public static IReadOnlyCollection<string> AllMutualIds { get; } =
        Rows.Select(r => r.Id).ToHashSet();

    /// <summary>Builds a fresh body list (parents resolved, elements populated).</summary>
    public static IReadOnlyList<CelestialBody> CreateBodies()
    {
        var byId = new Dictionary<string, CelestialBody>();
        var bodies = new List<CelestialBody>(Rows.Length);
        for (int i = 0; i < Rows.Length; i++)
        {
            var row = Rows[i];
            var parent = row.Parent is null ? null : byId[row.Parent];
            OrbitalElements? orbit = null;
            if (parent is not null)
            {
                double period = 2 * Math.PI * Math.Sqrt(row.A * row.A * row.A / parent.Mu);
                orbit = new OrbitalElements(
                    SemiMajorAxis: row.A,
                    Eccentricity: row.E,
                    Inclination: row.IncDeg * Math.PI / 180,
                    LongitudeOfAscendingNode: GoldenAngle(i, 1),
                    ArgumentOfPeriapsis: GoldenAngle(i, 2),
                    TimeAtPeriapsis: -GoldenFraction(i, 3) * period);
            }
            var body = new CelestialBody
            {
                Id = row.Id,
                Mu = row.NominalMu,
                Parent = parent,
                Orbit = orbit,
            };
            byId[row.Id] = body;
            bodies.Add(body);
        }
        return bodies;
    }

    /// <summary>Creates rails ephemerides over the catalog with no horizon extension.</summary>
    public static NBodyEphemerides CreateEphemerides(double relTol = ShippingRelTol) =>
        new(CreateBodies(), 0.0, BackboneIds, new IntegratorOptions { RelTol = relTol });

    /// <summary>Creates the production-shaped all-mutual 99-body ephemeris.</summary>
    public static NBodyEphemerides CreateAllMutualEphemerides(double relTol = ShippingRelTol) =>
        new(CreateBodies(), 0.0, AllMutualIds, new IntegratorOptions { RelTol = relTol });

    /// <summary>Creates the 99-body catalog plus a positive-mass heliocentric body
    /// whose periapsis speed exceeds the removed 80 km/s mutual-backbone gate.</summary>
    public static IReadOnlyList<CelestialBody> CreateFastPeriapsisStressBodies()
    {
        var bodies = CreateBodies().ToList();
        var sun = bodies.Single(body => body.Parent is null);
        bodies.Add(new CelestialBody
        {
            Id = "FastPeriapsisProbe",
            Mu = 1.0e9,
            Parent = sun,
            Orbit = new OrbitalElements(
                SemiMajorAxis: 2.5e10,
                Eccentricity: 0.3,
                Inclination: 0.05,
                LongitudeOfAscendingNode: 0.7,
                ArgumentOfPeriapsis: 1.1,
                TimeAtPeriapsis: 0.0),
        });
        return bodies;
    }

    /// <summary>The vessel-prediction benchmark window: the mod's default 30-day
    /// overlay horizon.</summary>
    public const double VesselHorizonSeconds = 30 * 86400.0;

    /// <summary>Creates the shared vessel cases: pre-extended rails, full-catalog
    /// gravity, and Earth-centered circular LEO and high-orbit states.</summary>
    public static (GravityModel Gravity, StateVector Leo, StateVector HighOrbit) CreateVesselCases()
    {
        var ephemerides = CreateEphemerides();
        var earth = ephemerides["Earth"];
        _ = ephemerides.GetState(earth, VesselHorizonSeconds + 86400); // pre-extend rails past the horizon
        var gravity = new GravityModel(ephemerides,
            ephemerides.Bodies.Where(b => b.Mu > 0.0));
        var e0 = ephemerides.GetState(earth, 0.0);
        return (gravity,
            CircularOrbit(e0, earth.Mu, 6.771e6),  // ~400 km altitude
            CircularOrbit(e0, earth.Mu, 1.0e8));   // ~100,000 km
    }

    /// <summary>Circular prograde orbit riding on a parent state (+X offset, +Y speed).</summary>
    private static StateVector CircularOrbit(StateVector parent, double mu, double radius) =>
        new(parent.Position + new Vector3d(radius, 0, 0),
            parent.Velocity + new Vector3d(0, Math.Sqrt(mu / radius), 0));

    /// <summary>Deterministic pseudo-random angle in [0, 2pi) — golden-ratio
    /// low-discrepancy sequence, stable across runs and platforms.</summary>
    private static double GoldenAngle(int index, int salt) => 2 * Math.PI * GoldenFraction(index, salt);

    private static double GoldenFraction(int index, int salt)
    {
        double x = (index + 0.31 * salt) * 0.6180339887498949;
        return x - Math.Floor(x);
    }
}
