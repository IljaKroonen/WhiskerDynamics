using System.Globalization;
using System.IO.Compression;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text;

// ICGEM's archived copy of NGA's EGM2008 release. The content-addressed URL and
// archive checksum pin the exact source from which the shipping subset is derived.
const string SourceUrl = "https://icgem.gfz.de/getmodel/zip/"
    + "c50128797a9cb62e936337c890e4425f03f0461d7329b09a8cc8561504465340/"
    + "EGM2008.zip";
const string SourceSha256 =
    "8CB3521F9650568A80D049BD6DD3668A4191F3AC9D741122CC0321BCC28948E3";
const string ArchiveEntryName = "EGM2008.gfc";
const int ShippingMaximumDegree = 10;
const int RetainedMaximumDegree = 50;

return await Generate(args);

static async Task<int> Generate(string[] args)
{
    try
    {
        if (args.Length > 1) throw new ArgumentException(
            "usage: dotnet run --file scripts/generate-earth-gravity-model.cs "
            + "-- [EGM2008.zip|EGM2008.gfc]");

        await using Stream input = args.Length == 1
            ? await OpenLocalSource(Path.GetFullPath(args[0]))
            : await DownloadVerifiedSource();
        var coefficients = await ReadCoefficients(input);
        string root = Directory.GetParent(ScriptDirectory())!.FullName;
        string output = Path.Combine(root, "src", "WhiskerDynamics.Mod",
            "BodySettings", "Earth.json");
        await WriteJson(output, coefficients);
        Console.WriteLine(
            $"generated {coefficients.Count} EGM2008 coefficients in {output}");
        return 0;
    }
    catch (Exception exception)
    {
        Console.Error.WriteLine(exception.Message);
        return 1;
    }
}

static async Task<Stream> DownloadVerifiedSource()
{
    using var client = new HttpClient();
    byte[] sourceBytes = await client.GetByteArrayAsync(SourceUrl);
    string actualSha256 = Convert.ToHexString(SHA256.HashData(sourceBytes));
    if (!actualSha256.Equals(SourceSha256, StringComparison.Ordinal))
        throw new InvalidDataException(
            $"EGM2008 ZIP SHA-256 mismatch: expected {SourceSha256}, "
            + $"found {actualSha256}.");
    return OpenArchive(new MemoryStream(sourceBytes, writable: false));
}

static async Task<Stream> OpenLocalSource(string path)
{
    if (!path.EndsWith(".zip", StringComparison.OrdinalIgnoreCase))
        return File.OpenRead(path);

    byte[] sourceBytes = await File.ReadAllBytesAsync(path);
    string actualSha256 = Convert.ToHexString(SHA256.HashData(sourceBytes));
    if (!actualSha256.Equals(SourceSha256, StringComparison.Ordinal))
        throw new InvalidDataException(
            $"EGM2008 ZIP SHA-256 mismatch: expected {SourceSha256}, "
            + $"found {actualSha256}.");
    return OpenArchive(new MemoryStream(sourceBytes, writable: false));
}

static Stream OpenArchive(Stream source)
{
    using (source)
    using (var archive = new ZipArchive(source, ZipArchiveMode.Read))
    {
        ZipArchiveEntry entry = archive.GetEntry(ArchiveEntryName)
            ?? throw new InvalidDataException(
                $"EGM2008 ZIP contains no '{ArchiveEntryName}'.");
        var copy = new MemoryStream(checked((int)entry.Length));
        using Stream entryStream = entry.Open();
        entryStream.CopyTo(copy);
        copy.Position = 0;
        return copy;
    }
}

static async Task<List<(int Degree, int Order, double C, double S)>>
    ReadCoefficients(Stream input)
{
    using var reader = new StreamReader(input);
    var header = new Dictionary<string, string>(StringComparer.Ordinal);
    string? line;
    bool headerEnded = false;
    while ((line = await reader.ReadLineAsync()) is not null)
    {
        string[] columns = Split(line);
        if (columns.Length == 0) continue;
        if (columns[0] == "end_of_head")
        {
            headerEnded = true;
            break;
        }
        if (columns.Length >= 2)
            header[columns[0]] = columns[1];
    }
    if (!headerEnded)
        throw new InvalidDataException("EGM2008 ICGEM header has no end_of_head.");

    RequireHeader(header, "product_type", "gravity_field");
    RequireHeader(header, "modelname", "EGM2008");
    RequireHeader(header, "earth_gravity_constant", "0.3986004415E+15");
    RequireHeader(header, "radius", "0.63781363E+07");
    RequireHeader(header, "max_degree", "2190");
    RequireHeader(header, "norm", "fully_normalized");
    RequireHeader(header, "tide_system", "tide_free");

    var result = new List<(int, int, double, double)>();
    int expectedDegree = 2, expectedOrder = 0;
    while ((line = await reader.ReadLineAsync()) is not null)
    {
        string[] columns = Split(line);
        if (columns.Length < 5 || columns[0] != "gfc") continue;
        int degree = Int(columns[1]);
        if (degree < 2) continue;
        if (degree > RetainedMaximumDegree) break;
        int order = Int(columns[2]);
        if (degree != expectedDegree || order != expectedOrder)
            throw new InvalidDataException(
                $"Expected coefficient ({expectedDegree}, {expectedOrder}), "
                + $"found ({degree}, {order}).");
        double cosine = Double(columns[3]), sine = Double(columns[4]);
        if (!double.IsFinite(cosine) || !double.IsFinite(sine)
            || order == 0 && sine != 0)
            throw new InvalidDataException(
                $"Invalid coefficient ({degree}, {order}).");
        result.Add((degree, order, cosine, sine));
        if (++expectedOrder > expectedDegree)
        {
            expectedDegree++;
            expectedOrder = 0;
        }
    }

    int expectedCount =
        (RetainedMaximumDegree + 1) * (RetainedMaximumDegree + 2) / 2 - 3;
    if (result.Count != expectedCount
        || result[^1] is not (RetainedMaximumDegree, RetainedMaximumDegree, _, _))
        throw new InvalidDataException(
            $"Expected {expectedCount} coefficients through "
            + $"{RetainedMaximumDegree}x{RetainedMaximumDegree}, "
            + $"found {result.Count}.");
    return result;
}

static void RequireHeader(
    IReadOnlyDictionary<string, string> header, string key, string expected)
{
    if (!header.TryGetValue(key, out string? actual) || actual != expected)
        throw new InvalidDataException(
            $"Unexpected EGM2008 header {key}: expected '{expected}', "
            + $"found '{actual ?? "<missing>"}'.");
}

static async Task WriteJson(string path,
    List<(int Degree, int Order, double C, double S)> coefficients)
{
    var json = new StringBuilder();
    json.AppendLine("{");
    json.AppendLine("  \"schema_version\": 1,");
    json.AppendLine("  \"match\": {");
    json.AppendLine("    \"id\": \"Earth\",");
    json.AppendLine("    \"parent_id\": \"Sol\"");
    json.AppendLine("  },");
    json.AppendLine("  \"gravity_model\": {");
    json.AppendLine("    \"model\": \"spherical_harmonics\",");
    json.AppendLine("    \"name\": \"EGM2008\",");
    json.AppendLine("    \"normalization\": \"fully_normalized\",");
    json.AppendLine("    \"reference_radius_m\": 6378136.3,");
    json.AppendLine($"    \"maximum_degree\": {ShippingMaximumDegree},");
    json.AppendLine("    \"coefficients\": [");
    for (int i = 0; i < coefficients.Count; i++)
    {
        var coefficient = coefficients[i];
        json.Append("      [").Append(coefficient.Degree).Append(", ")
            .Append(coefficient.Order).Append(", ").Append(Format(coefficient.C))
            .Append(", ").Append(Format(coefficient.S)).Append(']');
        json.AppendLine(i + 1 == coefficients.Count ? "" : ",");
    }
    json.AppendLine("    ]");
    json.AppendLine("  }");
    json.AppendLine("}");

    string temporaryPath = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
    try
    {
        await File.WriteAllTextAsync(
            temporaryPath, json.ToString(), new UTF8Encoding(false));
        File.Move(temporaryPath, path, overwrite: true);
    }
    finally
    {
        File.Delete(temporaryPath);
    }
}

static string ScriptDirectory([CallerFilePath] string sourceFile = "") =>
    Path.GetDirectoryName(sourceFile)!;

static string[] Split(string line) =>
    line.Split((char[]?)null,
        StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);

static double Double(string value) =>
    double.Parse(value.Replace('D', 'E').Replace('d', 'e'),
        NumberStyles.Float, CultureInfo.InvariantCulture);

static int Int(string value) =>
    int.Parse(value, NumberStyles.Integer, CultureInfo.InvariantCulture);

static string Format(double value) => value == 0
    ? "0"
    : value.ToString("e16", CultureInfo.InvariantCulture);
