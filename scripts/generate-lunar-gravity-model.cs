using System.Globalization;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text;

const string SourceUrl = "https://pds-geosciences.wustl.edu/grail/"
    + "grail-l-lgrs-5-rdr-v1/grail_1001/shadr/gggrx_1200a_sha.tab";
const string SourceSha256 =
    "FA04C3DCE9376948AD243F3DF74144E2602F12D183EA4D179604ED0A79DA7DED";
const int ShippingMaximumDegree = 30;

return await Generate(args);

static async Task<int> Generate(string[] args)
{
    try
    {
        if (args.Length > 1) throw new ArgumentException(
            "usage: dotnet run --file scripts/generate-lunar-gravity-model.cs -- [GRGM1200A_SHA.TAB]");
        await using Stream input = args.Length == 1
            ? File.OpenRead(Path.GetFullPath(args[0]))
            : await DownloadVerifiedSource();
        var coefficients = await ReadCoefficients(input);
        string root = Directory.GetParent(ScriptDirectory())!.FullName;
        string output = Path.Combine(root, "src", "WhiskerDynamics.Mod",
            "BodySettings", "Luna.json");
        await WriteJson(output, coefficients);
        Console.WriteLine(
            $"generated {coefficients.Count} GRGM1200A coefficients in {output}");
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
            $"GRGM1200A SHA-256 mismatch: expected {SourceSha256}, "
            + $"found {actualSha256}.");
    return new MemoryStream(sourceBytes, writable: false);
}

static async Task<List<(int Degree, int Order, double C, double S)>>
    ReadCoefficients(Stream input)
{
    using var reader = new StreamReader(input);
    string[] header = Split(await reader.ReadLineAsync()
        ?? throw new InvalidDataException("GRGM1200A SHA file is empty."));
    if (header.Length < 8) throw new InvalidDataException("Invalid GRGM1200A SHA header.");
    double radiusKm = Double(header[0]);
    double sourceMu = Double(header[1]);
    int sourceDegree = Int(header[3]), sourceOrder = Int(header[4]);
    int normalization = Int(header[5]);
    double referenceLongitude = Double(header[6]), referenceLatitude = Double(header[7]);
    if (radiusKm != 1738.0 || sourceMu != 4902.8001224453001
        || sourceDegree != 1200 || sourceOrder != 1200 || normalization != 1
        || referenceLongitude != 0 || referenceLatitude != 0)
        throw new InvalidDataException(
            $"Unexpected header: radius={radiusKm:R}, GM={sourceMu:R}, "
            + $"degree/order={sourceDegree}/{sourceOrder}, normalization={normalization}.");

    var result = new List<(int, int, double, double)>();
    int expectedDegree = 2, expectedOrder = 0;
    string? line;
    while ((line = await reader.ReadLineAsync()) is not null)
    {
        string[] columns = Split(line);
        if (columns.Length < 4) continue;
        int degree = Int(columns[0]);
        if (degree > 50) break;
        if (degree >= 2)
        {
            int order = Int(columns[1]);
            if (degree != expectedDegree || order != expectedOrder)
                throw new InvalidDataException(
                    $"Expected coefficient ({expectedDegree}, {expectedOrder}), "
                    + $"found ({degree}, {order}).");
            double cosine = Double(columns[2]), sine = Double(columns[3]);
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
    }
    if (result.Count != 1323 || result[^1] is not (50, 50, _, _))
        throw new InvalidDataException(
            $"Expected 1323 coefficients through 50x50, found {result.Count}.");
    return result;
}

static async Task WriteJson(string path,
    List<(int Degree, int Order, double C, double S)> coefficients)
{
    var json = new StringBuilder();
    json.AppendLine("{");
    json.AppendLine("  \"schema_version\": 1,");
    json.AppendLine("  \"match\": {");
    json.AppendLine("    \"id\": \"Luna\",");
    json.AppendLine("    \"parent_id\": \"Earth\"");
    json.AppendLine("  },");
    json.AppendLine("  \"gravity_model\": {");
    json.AppendLine("    \"model\": \"spherical_harmonics\",");
    json.AppendLine("    \"name\": \"GRGM1200A\",");
    json.AppendLine("    \"normalization\": \"fully_normalized\",");
    json.AppendLine("    \"reference_radius_m\": 1738000.0,");
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
    line.Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);

static double Double(string value) =>
    double.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);

static int Int(string value) =>
    int.Parse(value, NumberStyles.Integer, CultureInfo.InvariantCulture);

static string Format(double value) => value == 0
    ? "0"
    : value.ToString("e16", CultureInfo.InvariantCulture);
