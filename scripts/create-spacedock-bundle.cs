using System.Diagnostics;
using System.IO.Compression;
using System.Runtime.CompilerServices;
using System.Text.RegularExpressions;

return CreateSpaceDockBundle(args);

static int CreateSpaceDockBundle(string[] args)
{
    try
    {
        if (args is ["--check-bundle", var checkPublish, var checkModToml,
            var checkLicense, var checkThirdPartyNotices, var checkBundle])
        {
            WriteBundle(
                checkPublish,
                checkModToml,
                checkLicense,
                checkThirdPartyNotices,
                checkBundle);
            return 0;
        }

        if (args.Length is < 1 or > 2)
            throw new ArgumentException(
                "usage: dotnet run --file scripts/create-spacedock-bundle.cs -- "
                + "<Version> [Configuration]");

        var version = args[0];
        if (!Regex.IsMatch(version,
                @"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"
                + @"(?:-[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
                + @"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?$"))
            throw new ArgumentException($"invalid semantic version: {version}");

        var configuration = args.ElementAtOrDefault(1) ?? "Release";
        if (!Regex.IsMatch(configuration, @"^[0-9A-Za-z_.-]+$"))
            throw new ArgumentException($"invalid configuration: {configuration}");

        var scriptDirectory = GetScriptDirectory();
        var repoRoot = Directory.GetParent(scriptDirectory)!.FullName;
        var projectDirectory = Path.Combine(repoRoot, "src", "WhiskerDynamics.Mod");
        var artifactsDirectory = Path.Combine(repoRoot, "artifacts");
        var stagingDirectory = Path.Combine(
            artifactsDirectory, $".spacedock-{Guid.NewGuid():N}");
        var publishDirectory = Path.Combine(stagingDirectory, "publish");
        var bundlePath = Path.Combine(
            artifactsDirectory, $"WhiskerDynamics-{version}.zip");

        Directory.CreateDirectory(artifactsDirectory);
        try
        {
            Publish(projectDirectory, publishDirectory, configuration, version, repoRoot);
            WriteBundle(
                publishDirectory,
                Path.Combine(projectDirectory, "mod.toml"),
                Path.Combine(repoRoot, "LICENSE"),
                Path.Combine(repoRoot, "THIRD-PARTY-NOTICES"),
                bundlePath);
        }
        finally
        {
            if (Directory.Exists(stagingDirectory))
                Directory.Delete(stagingDirectory, recursive: true);
        }

        Console.WriteLine($"SpaceDock bundle: {bundlePath}");
        return 0;
    }
    catch (Exception exception)
    {
        Console.Error.WriteLine($"bundle failed: {exception.Message}");
        return 1;
    }
}

static void Publish(
    string projectDirectory,
    string publishDirectory,
    string configuration,
    string version,
    string workingDirectory)
{
    var startInfo = new ProcessStartInfo("dotnet")
    {
        UseShellExecute = false,
        WorkingDirectory = workingDirectory,
    };
    foreach (var argument in new[]
    {
        "publish",
        projectDirectory,
        "-c", configuration,
        "--output", publishDirectory,
        "--disable-build-servers",
        "-m:1",
        $"-p:Version={version}",
    })
    {
        startInfo.ArgumentList.Add(argument);
    }

    using var publish = Process.Start(startInfo)
        ?? throw new InvalidOperationException("could not start dotnet publish");
    publish.WaitForExit();
    if (publish.ExitCode != 0)
        throw new InvalidOperationException("dotnet publish failed");
}

static string[] RequiredPublishFiles() =>
[
    "WhiskerDynamics.Mod.dll",
    "WhiskerDynamics.Mod.deps.json",
    "WhiskerDynamics.Core.dll",
    "WhiskerDynamics.Compatibility.dll",
];

static string BundleEntryName(string fileName) => $"WhiskerDynamics/{fileName}";

static void WriteBundle(
    string publishDirectory,
    string modTomlPath,
    string licensePath,
    string thirdPartyNoticesPath,
    string bundlePath)
{
    if (!Directory.Exists(publishDirectory))
        throw new DirectoryNotFoundException(
            $"publish output not found at {publishDirectory}");

    string settingsDirectory = Path.Combine(publishDirectory, "body-settings");
    if (!Directory.Exists(settingsDirectory))
        throw new DirectoryNotFoundException(
            $"body settings publish directory not found at {settingsDirectory}");
    string[] settingsFiles = Directory.GetFiles(
        settingsDirectory, "*.json", SearchOption.TopDirectoryOnly);
    if (settingsFiles.Length == 0)
        throw new FileNotFoundException(
            $"body settings publish directory contains no JSON files: {settingsDirectory}");
    var files = RequiredPublishFiles()
        .Select(name => new BundleFile(
            Path.Combine(publishDirectory, name), BundleEntryName(name)))
        .Append(new BundleFile(modTomlPath, BundleEntryName("mod.toml")))
        .Append(new BundleFile(licensePath, BundleEntryName("LICENSE")))
        .Append(new BundleFile(
            thirdPartyNoticesPath, BundleEntryName("THIRD-PARTY-NOTICES")))
        .Concat(settingsFiles.Order(StringComparer.Ordinal)
            .Select(path => new BundleFile(path, BundleEntryName(
                $"body-settings/{Path.GetFileName(path)}"))))
        .Concat(Directory.GetFiles(publishDirectory, "WhiskerDynamics*.pdb")
            .Order(StringComparer.Ordinal)
            .Select(path => new BundleFile(
                path, BundleEntryName(Path.GetFileName(path)))))
        .ToArray();
    var missing = files.Where(file => !File.Exists(file.SourcePath)).ToArray();
    if (missing.Length > 0)
        throw new FileNotFoundException(
            "required bundle files are missing: "
            + string.Join(", ", missing.Select(file => file.SourcePath)));

    var bundleDirectory = Path.GetDirectoryName(Path.GetFullPath(bundlePath))
        ?? throw new InvalidOperationException($"bundle path has no parent: {bundlePath}");
    Directory.CreateDirectory(bundleDirectory);
    var temporaryBundle = Path.Combine(
        bundleDirectory, $".{Path.GetFileName(bundlePath)}.{Guid.NewGuid():N}.tmp");

    try
    {
        using (var archive = ZipFile.Open(temporaryBundle, ZipArchiveMode.Create))
        {
            foreach (var file in files)
            {
                var entry = archive.CreateEntry(file.EntryName, CompressionLevel.Optimal);
                entry.LastWriteTime = new DateTimeOffset(1980, 1, 1, 0, 0, 0, TimeSpan.Zero);
                using var source = File.OpenRead(file.SourcePath);
                using var destination = entry.Open();
                source.CopyTo(destination);
            }
        }

        File.Move(temporaryBundle, bundlePath, overwrite: true);
    }
    finally
    {
        if (File.Exists(temporaryBundle)) File.Delete(temporaryBundle);
    }
}

static string GetScriptDirectory([CallerFilePath] string sourcePath = "") =>
    Path.GetDirectoryName(sourcePath)
    ?? throw new InvalidOperationException("could not locate the bundle script");

sealed record BundleFile(string SourcePath, string EntryName);
