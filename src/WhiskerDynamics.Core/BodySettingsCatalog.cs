using System.Text.Json;
using System.Text.Json.Serialization;

namespace WhiskerDynamics.Core;

/// <summary>A settings-catalog match against an authoritative game-catalog body.
/// Id is required; ParentId optionally narrows the match for catalogs that reuse ids.</summary>
public sealed record BodyMatch(string Id, string? ParentId = null);

/// <summary>Configuration for one supported extended-body gravity model.</summary>
public abstract record ExtendedGravitySettings
{
    internal abstract Geopotential Create(in CatalogBody body);
    public abstract string Description { get; }

    internal static BodyRotation RequireRotation(in CatalogBody body) =>
        body.Rotation ?? throw new FormatException(
            $"body '{body.Id}' matched an extended gravity model but supplied no rotation");
}

public enum SphericalHarmonicNormalization
{
    Unnormalized,
    FullyNormalized,
}

/// <summary>A data-defined spherical-harmonic field. The coefficient catalog may
/// contain degrees above <see cref="MaximumDegree"/> so a body file can retain its
/// source model while choosing a cheaper runtime truncation.</summary>
public sealed record SphericalHarmonicGravitySettings : ExtendedGravitySettings
{
    private readonly SphericalHarmonicCoefficient[] _coefficients;
    private readonly IReadOnlyList<SphericalHarmonicCoefficient> _coefficientView;

    public string Name { get; }
    public double? ReferenceRadiusM { get; }
    public int MaximumDegree { get; }
    public SphericalHarmonicNormalization Normalization { get; }
    public BodyFixedToModelRotation? BodyFixedToModel { get; }
    public IReadOnlyList<SphericalHarmonicCoefficient> Coefficients => _coefficientView;

    public SphericalHarmonicGravitySettings(
        string name,
        double? referenceRadiusM,
        int maximumDegree,
        SphericalHarmonicNormalization normalization,
        IEnumerable<SphericalHarmonicCoefficient> coefficients,
        BodyFixedToModelRotation? bodyFixedToModel = null)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new FormatException("spherical-harmonic model name cannot be empty");
        if (referenceRadiusM is { } radius && (!double.IsFinite(radius) || radius <= 0))
            throw new FormatException("spherical-harmonic reference radius must be finite and positive");
        if (maximumDegree is < 2 or > Geopotential.MaximumDegree)
            throw new FormatException(
                $"spherical-harmonic maximum degree must be in [2, {Geopotential.MaximumDegree}]");
        if (!Enum.IsDefined(normalization))
            throw new FormatException("spherical-harmonic normalization is invalid");
        ArgumentNullException.ThrowIfNull(coefficients);

        _coefficients = coefficients
            .OrderBy(coefficient => coefficient.Degree)
            .ThenBy(coefficient => coefficient.Order)
            .ToArray();
        if (_coefficients.Length == 0)
            throw new FormatException("spherical-harmonic coefficient list cannot be empty");
        var seen = new HashSet<(int Degree, int Order)>();
        foreach (var coefficient in _coefficients)
        {
            if (coefficient.Degree is < 2 or > Geopotential.MaximumDegree
                || coefficient.Order < 0 || coefficient.Order > coefficient.Degree)
                throw new FormatException(
                    $"invalid spherical-harmonic index ({coefficient.Degree}, {coefficient.Order})");
            if (!double.IsFinite(coefficient.Cosine) || !double.IsFinite(coefficient.Sine)
                || coefficient.Order == 0 && coefficient.Sine != 0)
                throw new FormatException(
                    $"invalid spherical-harmonic coefficient ({coefficient.Degree}, "
                    + $"{coefficient.Order})");
            if (!seen.Add((coefficient.Degree, coefficient.Order)))
                throw new FormatException(
                    $"duplicate spherical-harmonic coefficient ({coefficient.Degree}, "
                    + $"{coefficient.Order})");
        }
        if (!_coefficients.Any(coefficient => coefficient.Degree == maximumDegree))
            throw new FormatException(
                $"spherical-harmonic coefficient list does not reach maximum degree {maximumDegree}");

        Name = name.Trim();
        ReferenceRadiusM = referenceRadiusM;
        MaximumDegree = maximumDegree;
        Normalization = normalization;
        BodyFixedToModel = bodyFixedToModel;
        _coefficientView = Array.AsReadOnly(_coefficients);
    }

    internal override Geopotential Create(in CatalogBody body)
    {
        double radius = ReferenceRadiusM ?? body.MeanRadiusM;
        var selected = _coefficients.TakeWhile(
            coefficient => coefficient.Degree <= MaximumDegree);
        return Normalization == SphericalHarmonicNormalization.FullyNormalized
            ? Geopotential.FromFullyNormalized(
                radius, RequireRotation(body), selected, BodyFixedToModel)
            : new Geopotential(
                radius, RequireRotation(body), selected, BodyFixedToModel);
    }

    public override string Description =>
        $"{Name} spherical harmonics degree {MaximumDegree} "
        + $"({Normalization}, {_coefficients.Length} catalog coefficients)";
}

/// <summary>One optional overlay on a body from the live game catalog.</summary>
public sealed record BodySettings(
    BodyMatch Match,
    ExtendedGravitySettings GravityModel,
    string Source = "<memory>");

/// <summary>File-backed body settings, matched onto live game-catalog records. The
/// settings only select an extended gravity model; mass, radius, state, hierarchy,
/// sphere of influence, and rotation continue to come from the game catalog.</summary>
public sealed class BodySettingsCatalog
{
    private const int CurrentSchemaVersion = 1;
    private readonly BodySettings[] _entries;
    private readonly IReadOnlyList<BodySettings> _entryView;

    public static BodySettingsCatalog Empty { get; } = new([]);
    public IReadOnlyList<BodySettings> Entries => _entryView;

    public BodySettingsCatalog(IEnumerable<BodySettings> entries)
    {
        ArgumentNullException.ThrowIfNull(entries);
        _entries = entries.ToArray();
        _entryView = Array.AsReadOnly(_entries);

        foreach (var entry in _entries)
        {
            ArgumentNullException.ThrowIfNull(entry);
            ArgumentNullException.ThrowIfNull(entry.Match);
            ArgumentNullException.ThrowIfNull(entry.GravityModel);
            if (string.IsNullOrWhiteSpace(entry.Match.Id))
                throw new FormatException(
                    $"body settings '{entry.Source}' has an empty match id");
        }

        for (int i = 0; i < _entries.Length; i++)
        for (int j = i + 1; j < _entries.Length; j++)
        {
            var a = _entries[i];
            var b = _entries[j];
            if (a.Match.Id == b.Match.Id
                && (a.Match.ParentId is null || b.Match.ParentId is null
                    || a.Match.ParentId == b.Match.ParentId))
                throw new FormatException(
                    $"body settings '{a.Source}' and '{b.Source}' have overlapping "
                    + $"matches for '{a.Match.Id}'");
        }
    }

    /// <summary>Loads every top-level JSON file in ordinal filename order.</summary>
    public static BodySettingsCatalog LoadDirectory(string directory)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(directory);
        if (!Directory.Exists(directory))
            throw new DirectoryNotFoundException(
                $"body settings directory not found: {directory}");

        var entries = Directory.GetFiles(directory, "*.json", SearchOption.TopDirectoryOnly)
            .Order(StringComparer.Ordinal)
            .Select(LoadFile)
            .ToArray();
        if (entries.Length == 0)
            throw new FormatException(
                $"body settings directory contains no JSON files: {directory}");
        return new BodySettingsCatalog(entries);
    }

    public BodySettings? Match(in CatalogBody body)
    {
        BodySettings? match = null;
        foreach (var entry in _entries)
        {
            if (entry.Match.Id != body.Id
                || entry.Match.ParentId is { } parentId && parentId != body.ParentId)
                continue;
            if (match is not null)
                throw new InvalidOperationException(
                    $"body '{body.Id}' matched multiple settings entries");
            match = entry;
        }
        return match;
    }

    internal Geopotential? CreateGeopotential(in CatalogBody body) =>
        Match(body)?.GravityModel.Create(body);

    private static BodySettings LoadFile(string path)
    {
        try
        {
            var definition = JsonSerializer.Deserialize<FileDefinition>(
                File.ReadAllText(path), JsonOptions)
                ?? throw new FormatException("file contains JSON null");
            string source = Path.GetFileName(path);
            if (definition.SchemaVersion != CurrentSchemaVersion)
                throw new FormatException(
                    $"schema_version must be {CurrentSchemaVersion}");
            if (definition.Match is null)
                throw new FormatException("match is required");
            string id = definition.Match.Id?.Trim() ?? "";
            if (id.Length == 0)
                throw new FormatException("match.id is required");
            string? parentId = definition.Match.ParentId?.Trim();
            if (parentId is { Length: 0 })
                throw new FormatException("match.parent_id cannot be empty");
            if (definition.GravityModel is null)
                throw new FormatException("gravity_model is required");

            return new BodySettings(
                new BodyMatch(id, parentId),
                ParseGravity(definition.GravityModel),
                source);
        }
        catch (Exception e) when (e is JsonException or FormatException
            or IOException or UnauthorizedAccessException)
        {
            throw new FormatException(
                $"invalid body settings file '{path}': {e.Message}", e);
        }
    }

    private static ExtendedGravitySettings ParseGravity(GravityDefinition gravity)
    {
        string model = gravity.Model?.Trim().ToLowerInvariant() ?? "";
        if (model != "spherical_harmonics")
            throw new FormatException(
                $"gravity_model.model '{gravity.Model}' is not supported");

        string name = gravity.Name?.Trim() ?? "";
        if (name.Length == 0)
            throw new FormatException(
                "gravity_model.name is required for model 'spherical_harmonics'");
        if (gravity.MaximumDegree is not { } maximumDegree)
            throw new FormatException(
                "gravity_model.maximum_degree is required for model 'spherical_harmonics'");

        var normalization = gravity.Normalization?.Trim().ToLowerInvariant() switch
        {
            "unnormalized" => SphericalHarmonicNormalization.Unnormalized,
            "fully_normalized" => SphericalHarmonicNormalization.FullyNormalized,
            _ => throw new FormatException(
                "gravity_model.normalization must be 'unnormalized' or "
                + "'fully_normalized'"),
        };
        if (gravity.Coefficients.ValueKind != JsonValueKind.Array)
            throw new FormatException(
                "gravity_model.coefficients must be an array");

        var coefficients = new List<SphericalHarmonicCoefficient>();
        int index = 0;
        foreach (var row in gravity.Coefficients.EnumerateArray())
        {
            if (row.ValueKind != JsonValueKind.Array || row.GetArrayLength() != 4)
                throw new FormatException(
                    $"gravity_model.coefficients[{index}] must be [degree, order, cosine, sine]");

            JsonElement[] values = row.EnumerateArray().ToArray();
            if (!values[0].TryGetInt32(out int degree)
                || !values[1].TryGetInt32(out int order)
                || !values[2].TryGetDouble(out double cosine)
                || !values[3].TryGetDouble(out double sine))
                throw new FormatException(
                    $"gravity_model.coefficients[{index}] contains an invalid number");

            coefficients.Add(new SphericalHarmonicCoefficient(
                degree, order, cosine, sine));
            index++;
        }

        return new SphericalHarmonicGravitySettings(
            name,
            gravity.ReferenceRadiusM,
            maximumDegree,
            normalization,
            coefficients,
            BodyFixedToModelRotationJson.Parse(gravity.BodyFixedToModel));
    }

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
        PropertyNameCaseInsensitive = false,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true,
        UnmappedMemberHandling = JsonUnmappedMemberHandling.Disallow,
    };

    private sealed class FileDefinition
    {
        public int SchemaVersion { get; set; }
        public MatchDefinition? Match { get; set; }
        public GravityDefinition? GravityModel { get; set; }
    }

    private sealed class MatchDefinition
    {
        public string? Id { get; set; }
        public string? ParentId { get; set; }
    }

    private sealed class GravityDefinition
    {
        public string? Model { get; set; }
        public string? Name { get; set; }
        public string? Normalization { get; set; }
        public double? ReferenceRadiusM { get; set; }
        public int? MaximumDegree { get; set; }
        public JsonElement BodyFixedToModel { get; set; }
        public JsonElement Coefficients { get; set; }
    }
}
