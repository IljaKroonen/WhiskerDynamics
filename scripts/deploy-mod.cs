using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

return Deploy(args);

static int Deploy(string[] args)
{
    try
    {
        // Read-only probe used by the regression tests. Keeping it in this script
        // ensures the test exercises the exact parser used during deployment.
        if (args is ["--check-manifest", var checkPath])
        {
            var manifest = File.ReadAllText(checkPath);
            return ManifestContainsActiveId(manifest, "WhiskerDynamics") ? 0 : 1;
        }

        if (args is ["--check-manifest-update", var updatePath])
        {
            EnsureWhiskerDynamicsManifestEntry(updatePath);
            return 0;
        }

        if (args is ["--check-deployment-transaction", var checkOutput,
            var checkModToml, var checkDestination])
        {
            DeployPayload(checkOutput, checkModToml, checkDestination);
            return 0;
        }

        if (args is ["--check-deployment-precommit-write", var writeOutput,
            var writeModToml, var writeDestination, var writePath])
        {
            DeployPayload(writeOutput, writeModToml, writeDestination,
                () => File.WriteAllText(writePath, "deployment write probe"));
            return 0;
        }

        if (args is ["--check-deployment-commit", var stagingDirectory,
            var destinationDirectory, var backupDirectory])
        {
            var parentDirectory = Directory.GetParent(destinationDirectory)?.FullName
                ?? throw new InvalidOperationException(
                    $"deployment directory has no parent: {destinationDirectory}");
            CommitPreparedDeployment(new PreparedDeployment(
                stagingDirectory, destinationDirectory, backupDirectory,
                Path.Combine(parentDirectory,
                    $".{Path.GetFileName(destinationDirectory)}.retired-{Guid.NewGuid():N}")));
            return 0;
        }

        bool gameTestDriver;
        string configuration;
        if (args is ["--game-test-driver"])
        {
            gameTestDriver = true;
            configuration = "Release";
        }
        else if (args is ["--game-test-driver", var driverConfiguration])
        {
            gameTestDriver = true;
            configuration = driverConfiguration;
        }
        else if (args.Length <= 1)
        {
            gameTestDriver = false;
            configuration = args.FirstOrDefault() ?? "Release";
        }
        else
        {
            throw new ArgumentException(
                "usage: dotnet run --file scripts/deploy-mod.cs -- "
                + "[Configuration] | --game-test-driver [Configuration]");
        }

        var scriptDirectory = GetScriptDirectory();
        var repoRoot = Directory.GetParent(scriptDirectory)!.FullName;
        var documents = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments);
        var docsDirectory = Path.Combine(documents, "My Games", "Kitten Space Agency");
        string modId = gameTestDriver
            ? "WhiskerDynamics.GameTestDriver"
            : "WhiskerDynamics";
        var modDirectory = Path.Combine(docsDirectory, "mods", modId);

        // Never build/deploy while the game is running (DLLs are file-locked).
        var running = Process.GetProcesses()
            .Where(process => process.ProcessName is "KSA" or "StarMap")
            .ToArray();

        if (running.Length > 0)
        {
            var processIds = string.Join(", ", running.Select(process => process.Id));
            throw new InvalidOperationException(
                $"KSA/StarMap is running (pid {processIds}) - close it before deploying.");
        }

        var projectDirectory = gameTestDriver
            ? Path.Combine(repoRoot, "tests", "WhiskerDynamics.GameTestDriver")
            : Path.Combine(repoRoot, "src", "WhiskerDynamics.Mod");
        var outputDirectory = Path.Combine(
            projectDirectory, "bin", configuration, "net10.0", "publish");
        var preparedDeployment = PrepareDeploymentForFiles(
            outputDirectory, Path.Combine(projectDirectory, "mod.toml"), modDirectory,
            gameTestDriver ? RequiredGameTestDriverPublishFiles() : RequiredPublishFiles(),
            includeBodySettings: !gameTestDriver);
        try
        {
            // Ensure the mod is enabled in the game's manifest. If it already has an entry,
            // preserve the user's existing enabled state.
            var manifestPath = Path.Combine(docsDirectory, "manifest.toml");
            if (gameTestDriver) EnsureGameTestDriverManifestEntry(manifestPath);
            else EnsureWhiskerDynamicsManifestEntry(manifestPath);

            CommitPreparedDeployment(preparedDeployment);
        }
        finally
        {
            CleanupStaging(preparedDeployment.StagingDirectory);
        }

        Console.WriteLine($"deployed to {modDirectory}");
        var starMapPath = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "StarMap",
            "StarMap.exe");
        Console.WriteLine($"launch via StarMap: \"{starMapPath}\"");
        return 0;
    }
    catch (Exception exception)
    {
        Console.Error.WriteLine($"deploy failed: {exception.Message}");
        return 1;
    }
}

static void EnsureWhiskerDynamicsManifestEntry(string manifestPath)
{
    const string manifestId = "WhiskerDynamics";
    const string manifestEntry = "[[mods]]\nid = \"WhiskerDynamics\"\nenabled = true";

    if (!File.Exists(manifestPath))
    {
        var manifestDirectory = Path.GetDirectoryName(manifestPath)
            ?? throw new InvalidOperationException($"manifest path has no parent: {manifestPath}");
        Directory.CreateDirectory(manifestDirectory);
        File.WriteAllText(manifestPath, manifestEntry);
        Console.WriteLine($"created {manifestPath} with Whisker Dynamics enabled");
        return;
    }

    var manifest = File.ReadAllText(manifestPath);
    if (ManifestContainsActiveId(manifest, manifestId))
    {
        Console.WriteLine(
            $"Whisker Dynamics already present in {manifestPath} (enabled state left as-is)");
        return;
    }

    File.AppendAllText(
        manifestPath, $"{Environment.NewLine}{manifestEntry}{Environment.NewLine}");
    Console.WriteLine($"enabled Whisker Dynamics in {manifestPath}");
}

static void EnsureGameTestDriverManifestEntry(string manifestPath)
{
    const string manifestId = "WhiskerDynamics.GameTestDriver";
    var quote = (char)34;
    string manifestEntry = $"[[mods]]\nid = {quote}{manifestId}{quote}\nenabled = true";

    if (!File.Exists(manifestPath))
    {
        var manifestDirectory = Path.GetDirectoryName(manifestPath)
            ?? throw new InvalidOperationException($"manifest path has no parent: {manifestPath}");
        Directory.CreateDirectory(manifestDirectory);
        File.WriteAllText(manifestPath, manifestEntry);
        Console.WriteLine($"created {manifestPath} with game test driver enabled");
        return;
    }

    var manifest = File.ReadAllText(manifestPath);
    if (ManifestContainsActiveId(manifest, manifestId))
    {
        Console.WriteLine(
            $"game test driver already present in {manifestPath} (enabled state left as-is)");
        return;
    }

    File.AppendAllText(
        manifestPath, $"{Environment.NewLine}{manifestEntry}{Environment.NewLine}");
    Console.WriteLine($"enabled game test driver in {manifestPath}");
}

static string[] RequiredPublishFiles() =>
[
    "WhiskerDynamics.Mod.dll",
    "WhiskerDynamics.Mod.deps.json",
    "WhiskerDynamics.Core.dll",
    "WhiskerDynamics.Compatibility.dll",
];

static string[] RequiredGameTestDriverPublishFiles() =>
[
    "WhiskerDynamics.GameTestDriver.dll",
    "WhiskerDynamics.GameTestDriver.deps.json",
    "WhiskerDynamics.GameTestDriver.Runtime.dll",
    "WhiskerDynamics.GameTesting.dll",
];

static IReadOnlyList<DeploymentFile> PreflightDeploymentForFiles(
    string outputDirectory, string modTomlPath, IReadOnlyList<string> requiredFiles,
    bool includeBodySettings)
{
    if (!Directory.Exists(outputDirectory))
        throw new DirectoryNotFoundException(
            $"publish output not found at {outputDirectory}; publish the mod before deploying");

    var files = requiredFiles
        .Select(name => new DeploymentFile(
            Path.Combine(outputDirectory, name), name))
        .Append(new DeploymentFile(modTomlPath, "mod.toml"))
        .ToList();
    var missing = files.Where(file => !File.Exists(file.SourcePath)).ToArray();
    if (missing.Length > 0)
        throw new FileNotFoundException(
            "required deployment files are missing: "
            + string.Join(", ", missing.Select(file => file.SourcePath)));

    if (includeBodySettings)
    {
        string settingsDirectory = Path.Combine(outputDirectory, "body-settings");
        if (!Directory.Exists(settingsDirectory))
            throw new DirectoryNotFoundException(
                $"body settings publish directory not found: {settingsDirectory}");
        string[] settingsFiles = Directory.GetFiles(
            settingsDirectory, "*.json", SearchOption.TopDirectoryOnly);
        if (settingsFiles.Length == 0)
            throw new FileNotFoundException(
                $"body settings publish directory contains no JSON files: {settingsDirectory}");
        files.AddRange(settingsFiles
            .Order(StringComparer.Ordinal)
            .Select(path => new DeploymentFile(path,
                Path.Combine("body-settings", Path.GetFileName(path)))));
    }

    files.AddRange(Directory.GetFiles(outputDirectory, "*.pdb")
        .Select(path => new DeploymentFile(path, Path.GetFileName(path))));
    return files;
}

static PreparedDeployment PrepareDeployment(
    string outputDirectory, string modTomlPath, string destinationDirectory)
    => PrepareDeploymentForFiles(
        outputDirectory, modTomlPath, destinationDirectory, RequiredPublishFiles(),
        includeBodySettings: true);

static PreparedDeployment PrepareDeploymentForFiles(
    string outputDirectory, string modTomlPath, string destinationDirectory,
    IReadOnlyList<string> requiredFiles, bool includeBodySettings)
{
    var files = PreflightDeploymentForFiles(
        outputDirectory, modTomlPath, requiredFiles, includeBodySettings);
    var parentDirectory = Directory.GetParent(destinationDirectory)?.FullName
        ?? throw new InvalidOperationException(
            $"deployment directory has no parent: {destinationDirectory}");
    var deploymentName = Path.GetFileName(destinationDirectory);
    var transactionId = Guid.NewGuid().ToString("N");
    var stagingDirectory = Path.Combine(
        parentDirectory, $".{deploymentName}.staging-{transactionId}");
    var backupDirectory = Path.Combine(parentDirectory, $".{deploymentName}.backup");
    var retiredDirectory = Path.Combine(
        parentDirectory, $".{deploymentName}.retired-{transactionId}");

    Directory.CreateDirectory(parentDirectory);
    try
    {
        Directory.CreateDirectory(stagingDirectory);
        foreach (var file in files)
        {
            string destinationPath = Path.Combine(stagingDirectory, file.DestinationName);
            Directory.CreateDirectory(Path.GetDirectoryName(destinationPath)
                ?? throw new InvalidOperationException(
                    "deployment file has no destination directory"));
            File.Copy(file.SourcePath, destinationPath);
        }
        return new PreparedDeployment(
            stagingDirectory, destinationDirectory, backupDirectory, retiredDirectory);
    }
    catch
    {
        CleanupStaging(stagingDirectory);
        throw;
    }
}

static void DeployPayload(
    string outputDirectory, string modTomlPath, string destinationDirectory,
    Action? beforeCommit = null)
{
    var prepared = PrepareDeployment(outputDirectory, modTomlPath, destinationDirectory);
    try
    {
        beforeCommit?.Invoke();
        CommitPreparedDeployment(prepared);
    }
    finally
    {
        CleanupStaging(prepared.StagingDirectory);
    }
}

static void CommitPreparedDeployment(PreparedDeployment prepared)
{
    string? displacedDirectory = null;
    if (Directory.Exists(prepared.DestinationDirectory))
    {
        // An existing backup may be the only known-good payload after a failed restore.
        // Keep it until staging is live; displace the current directory separately.
        displacedDirectory = Directory.Exists(prepared.BackupDirectory)
            ? prepared.RetiredDirectory
            : prepared.BackupDirectory;
        Directory.Move(prepared.DestinationDirectory, displacedDirectory);
    }

    try
    {
        Directory.Move(prepared.StagingDirectory, prepared.DestinationDirectory);
    }
    catch (Exception replacementFailure)
    {
        string? restoreDirectory = displacedDirectory;
        if (restoreDirectory is null && Directory.Exists(prepared.BackupDirectory))
            restoreDirectory = prepared.BackupDirectory;
        if (restoreDirectory is null)
            throw;

        try
        {
            Directory.Move(restoreDirectory, prepared.DestinationDirectory);
        }
        catch (Exception restoreFailure)
        {
            throw new AggregateException(
                $"deployment replacement and automatic restore failed; prior deployment "
                + $"remains at {restoreDirectory}",
                replacementFailure, restoreFailure);
        }
        throw new IOException(
            "deployment replacement failed; prior deployment was restored",
            replacementFailure);
    }

    CleanupPriorDeployment(prepared.BackupDirectory);
    CleanupPriorDeployment(prepared.RetiredDirectory);
}

static void CleanupPriorDeployment(string directory)
{
    if (!Directory.Exists(directory)) return;
    try
    {
        Directory.Delete(directory, recursive: true);
    }
    catch (Exception cleanupFailure)
    {
        Console.Error.WriteLine(
            $"warning: deployed successfully but could not remove prior deployment at "
            + $"{directory}: {cleanupFailure.Message}");
    }
}

static void CleanupStaging(string stagingDirectory)
{
    if (Directory.Exists(stagingDirectory))
        Directory.Delete(stagingDirectory, recursive: true);
}

static bool ManifestContainsActiveId(string manifest, string id)
{
    var insideModsEntry = false;
    foreach (var rawLine in manifest.Split('\n'))
    {
        string line = rawLine.TrimEnd('\r');
        if (TryReadTomlTableHeader(line, out bool arrayTable, out string[] tablePath))
        {
            insideModsEntry = arrayTable && tablePath is ["mods"];
            continue;
        }
        if (StartsTomlTableHeader(line))
        {
            insideModsEntry = false;
            continue;
        }
        if (insideModsEntry
            && TryReadTomlAssignment(line, out string key, out string value)
            && key.Equals("id", StringComparison.Ordinal)
            && value.Equals(id, StringComparison.Ordinal))
            return true;
    }

    return false;
}

static bool TryReadTomlTableHeader(
    string line, out bool arrayTable, out string[] tablePath)
{
    arrayTable = false;
    tablePath = [];
    var position = 0;
    SkipTomlWhitespace(line, ref position);
    if (position >= line.Length || line[position++] != '[') return false;
    if (position < line.Length && line[position] == '[')
    {
        arrayTable = true;
        position++;
    }

    var keys = new List<string>();
    while (true)
    {
        SkipTomlWhitespace(line, ref position);
        if (position >= line.Length) return false;
        string key;
        if (line[position] is (char)34 or (char)39)
        {
            if (!TryReadTomlString(line, ref position, out key)) return false;
        }
        else
        {
            var keyStart = position;
            while (position < line.Length && IsTomlBareKeyCharacter(line[position]))
                position++;
            if (position == keyStart) return false;
            key = line[keyStart..position];
        }
        keys.Add(key);
        SkipTomlWhitespace(line, ref position);
        if (position < line.Length && line[position] == '.')
        {
            position++;
            continue;
        }
        break;
    }

    if (position >= line.Length || line[position++] != ']') return false;
    if (arrayTable
        && (position >= line.Length || line[position++] != ']')) return false;
    SkipTomlWhitespace(line, ref position);
    if (position != line.Length && line[position] != '#') return false;
    tablePath = keys.ToArray();
    return true;
}

static bool StartsTomlTableHeader(string line)
{
    var position = 0;
    SkipTomlWhitespace(line, ref position);
    return position < line.Length && line[position] == '[';
}

static bool TryReadTomlAssignment(string line, out string key, out string value)
{
    key = string.Empty;
    value = string.Empty;
    var position = 0;
    SkipTomlWhitespace(line, ref position);
    if (position >= line.Length || line[position] == '#') return false;

    if (line[position] is (char)34 or (char)39)
    {
        if (!TryReadTomlString(line, ref position, out key)) return false;
    }
    else
    {
        var keyStart = position;
        while (position < line.Length && IsTomlBareKeyCharacter(line[position])) position++;
        if (position == keyStart) return false;
        key = line[keyStart..position];
    }

    SkipTomlWhitespace(line, ref position);
    if (position >= line.Length || line[position] != '=') return false;
    position++;
    SkipTomlWhitespace(line, ref position);
    if (!TryReadTomlString(line, ref position, out value)) return false;
    SkipTomlWhitespace(line, ref position);
    return position == line.Length || line[position] == '#';
}

static bool TryReadTomlString(string source, ref int position, out string value)
{
    value = string.Empty;
    if (position >= source.Length) return false;

    char delimiter = source[position++];
    if (delimiter == (char)39)
    {
        var valueStart = position;
        while (position < source.Length && source[position] != delimiter)
        {
            if (source[position] < (char)32 && source[position] != '\t') return false;
            position++;
        }
        if (position >= source.Length) return false;
        value = source[valueStart..position];
        position++;
        return true;
    }
    if (delimiter != (char)34) return false;

    var decoded = new StringBuilder();
    while (position < source.Length)
    {
        char character = source[position++];
        if (character == delimiter)
        {
            value = decoded.ToString();
            return true;
        }
        if (character != (char)92)
        {
            if (character < (char)32 && character != '\t') return false;
            decoded.Append(character);
            continue;
        }
        if (position >= source.Length) return false;

        char escape = source[position++];
        switch (escape)
        {
            case 'b': decoded.Append('\b'); break;
            case 't': decoded.Append('\t'); break;
            case 'n': decoded.Append('\n'); break;
            case 'f': decoded.Append('\f'); break;
            case 'r': decoded.Append('\r'); break;
            case (char)34: decoded.Append((char)34); break;
            case (char)92: decoded.Append((char)92); break;
            case 'u':
                if (!TryReadTomlUnicodeEscape(source, ref position, 4, out string u)) return false;
                decoded.Append(u);
                break;
            case 'U':
                if (!TryReadTomlUnicodeEscape(source, ref position, 8, out string upperU)) return false;
                decoded.Append(upperU);
                break;
            default:
                return false;
        }
    }

    return false;
}

static bool TryReadTomlUnicodeEscape(
    string source, ref int position, int digitCount, out string value)
{
    value = string.Empty;
    if (source.Length - position < digitCount) return false;

    uint scalar = 0;
    for (var i = 0; i < digitCount; i++)
    {
        int digit = HexDigit(source[position + i]);
        if (digit < 0) return false;
        scalar = scalar * 16 + (uint)digit;
    }
    position += digitCount;
    if (scalar > 0x10ffff || scalar is >= 0xd800 and <= 0xdfff) return false;
    value = char.ConvertFromUtf32((int)scalar);
    return true;
}

static int HexDigit(char character) => character switch
{
    >= '0' and <= '9' => character - '0',
    >= 'a' and <= 'f' => character - 'a' + 10,
    >= 'A' and <= 'F' => character - 'A' + 10,
    _ => -1,
};

static bool IsTomlBareKeyCharacter(char character) =>
    character is >= 'A' and <= 'Z'
        or >= 'a' and <= 'z'
        or >= '0' and <= '9'
        or '_' or '-';

static void SkipTomlWhitespace(string source, ref int position)
{
    while (position < source.Length && source[position] is ' ' or '\t') position++;
}

static string GetScriptDirectory([CallerFilePath] string sourcePath = "") =>
    Path.GetDirectoryName(sourcePath)
    ?? throw new InvalidOperationException("could not locate the deployment script");

sealed record DeploymentFile(string SourcePath, string DestinationName);

sealed record PreparedDeployment(
    string StagingDirectory,
    string DestinationDirectory,
    string BackupDirectory,
    string RetiredDirectory);
