using System.Diagnostics;
using System.IO.Compression;

namespace WhiskerDynamics.Mod.Tests.Deployment;

public class SpaceDockBundleScriptTests
{
    private static readonly string[] RequiredPublishFiles =
    [
        "WhiskerDynamics.Mod.dll",
        "WhiskerDynamics.Mod.deps.json",
        "WhiskerDynamics.Core.dll",
        "WhiskerDynamics.Compatibility.dll",
    ];

    [Fact]
    public async Task Bundle_contains_the_installable_payload_in_a_named_mod_directory()
    {
        var repoRoot = FindRepoRoot();
        var root = Path.Combine(
            Path.GetTempPath(), $"wd-spacedock-{Guid.NewGuid():N}");
        var publishDirectory = Path.Combine(root, "publish");
        var modTomlPath = Path.Combine(root, "mod.toml");
        var licensePath = Path.Combine(root, "LICENSE");
        var thirdPartyNoticesPath = Path.Combine(root, "THIRD-PARTY-NOTICES");
        var bundlePath = Path.Combine(root, "WhiskerDynamics-test.zip");
        string[] pdbFiles =
        [
            "WhiskerDynamics.Mod.pdb",
            "WhiskerDynamics.Core.pdb",
        ];

        try
        {
            Directory.CreateDirectory(publishDirectory);
            foreach (string name in RequiredPublishFiles.Concat(pdbFiles))
                File.WriteAllText(Path.Combine(publishDirectory, name), name);
            File.WriteAllText(modTomlPath, "mod manifest");
            File.WriteAllText(licensePath, "license");
            File.WriteAllText(thirdPartyNoticesPath, "third-party notices");

            var startInfo = new ProcessStartInfo("dotnet")
            {
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                WorkingDirectory = repoRoot,
            };
            foreach (string argument in new[]
            {
                "run",
                "--disable-build-servers",
                "--file",
                Path.Combine(repoRoot, "scripts", "create-spacedock-bundle.cs"),
                "--",
                "--check-bundle",
                publishDirectory,
                modTomlPath,
                licensePath,
                thirdPartyNoticesPath,
                bundlePath,
            })
            {
                startInfo.ArgumentList.Add(argument);
            }

            using var process = Process.Start(startInfo)
                ?? throw new InvalidOperationException("Could not start bundle probe.");
            Task<string> standardOutput = process.StandardOutput.ReadToEndAsync();
            Task<string> standardError = process.StandardError.ReadToEndAsync();
            Assert.True(process.WaitForExit(30_000), "Bundle probe timed out.");
            Assert.Equal(0, process.ExitCode);
            Assert.Empty(await standardError);
            Assert.Empty(await standardOutput);

            using var archive = ZipFile.OpenRead(bundlePath);
            string[] expectedEntries = RequiredPublishFiles
                .Concat(["mod.toml", "LICENSE", "THIRD-PARTY-NOTICES"])
                .Concat(pdbFiles)
                .Select(name => $"WhiskerDynamics/{name}")
                .Order()
                .ToArray();
            Assert.Equal(expectedEntries,
                archive.Entries.Select(entry => entry.FullName).Order().ToArray());
            var noticesEntry = Assert.Single(archive.Entries,
                entry => entry.FullName == "WhiskerDynamics/THIRD-PARTY-NOTICES");
            using (var reader = new StreamReader(noticesEntry.Open()))
                Assert.Equal("third-party notices", await reader.ReadToEndAsync());
            Assert.All(archive.Entries, entry =>
            {
                Assert.StartsWith("WhiskerDynamics/", entry.FullName);
                Assert.DoesNotContain('/', entry.FullName["WhiskerDynamics/".Length..]);
                Assert.DoesNotContain('\\', entry.FullName);
                Assert.Equal(
                    new DateTimeOffset(1980, 1, 1, 0, 0, 0, TimeSpan.Zero),
                    entry.LastWriteTime);
            });
        }
        finally
        {
            if (Directory.Exists(root)) Directory.Delete(root, recursive: true);
        }
    }

    private static string FindRepoRoot()
    {
        for (var directory = new DirectoryInfo(AppContext.BaseDirectory);
             directory is not null;
             directory = directory.Parent)
        {
            if (File.Exists(Path.Combine(
                    directory.FullName, "scripts", "create-spacedock-bundle.cs")))
                return directory.FullName;
        }

        throw new DirectoryNotFoundException("Could not find the repository root.");
    }
}
