using System.Diagnostics;
using System.Text;
using System.Text.RegularExpressions;

namespace WhiskerDynamics.Mod.Tests.Deployment;

public class DeploymentScriptTests
{
    private static readonly string[] RequiredPublishFiles =
    [
        "WhiskerDynamics.Mod.dll",
        "WhiskerDynamics.Mod.deps.json",
        "WhiskerDynamics.Core.dll",
        "WhiskerDynamics.Compatibility.dll",
    ];

    [Fact]
    public void Deployment_copies_the_exact_runtime_file_set_from_publish_output()
    {
        var repoRoot = FindRepoRoot();
        var script = File.ReadAllText(Path.Combine(repoRoot, "scripts", "deploy-mod.cs"));
        var quote = (char)34;
        Assert.Contains(
            $"projectDirectory, {quote}bin{quote}, configuration, "
            + $"{quote}net10.0{quote}, {quote}publish{quote}",
            script);
        Assert.DoesNotContain($"new ProcessStartInfo({quote}dotnet{quote})", script);
        var publishSet = Regex.Match(script,
            @"static string\[\] RequiredPublishFiles\(\)\s*=>\s*\[(.*?)\];",
            RegexOptions.Singleline);
        Assert.True(publishSet.Success, "Could not find the required publish set.");
        string[] copiedFiles = Regex.Matches(
                publishSet.Groups[1].Value, @"""([^""]+)""")
            .Select(match => match.Groups[1].Value)
            .ToArray();

        Assert.Equal(
        [
            "WhiskerDynamics.Mod.dll",
            "WhiskerDynamics.Mod.deps.json",
            "WhiskerDynamics.Core.dll",
            "WhiskerDynamics.Compatibility.dll",
        ], copiedFiles);
    }

    [Fact]
    public void Game_test_driver_has_a_separate_deployment_payload()
    {
        var repoRoot = FindRepoRoot();
        var script = File.ReadAllText(Path.Combine(repoRoot, "scripts", "deploy-mod.cs"));
        var publishSet = Regex.Match(script,
            @"static string\[\] RequiredGameTestDriverPublishFiles\(\)\s*=>\s*\[(.*?)\];",
            RegexOptions.Singleline);
        Assert.True(publishSet.Success, "Could not find the game test driver publish set.");

        string[] copiedFiles = Regex.Matches(
                publishSet.Groups[1].Value, @"""([^""]+)""")
            .Select(match => match.Groups[1].Value)
            .ToArray();

        Assert.Equal(
        [
            "WhiskerDynamics.GameTestDriver.dll",
            "WhiskerDynamics.GameTestDriver.deps.json",
            "WhiskerDynamics.GameTestDriver.Runtime.dll",
            "WhiskerDynamics.GameTesting.dll",
        ], copiedFiles);
    }

    [Fact]
    public void Missing_later_dependency_leaves_prior_deployment_untouched()
    {
        var repoRoot = FindRepoRoot();
        var scriptPath = Path.Combine(repoRoot, "scripts", "deploy-mod.cs");
        var root = Path.Combine(Path.GetTempPath(), $"wd-deploy-{Guid.NewGuid():N}");
        var outputDirectory = Path.Combine(root, "publish");
        var projectDirectory = Path.Combine(root, "project");
        var destinationDirectory = Path.Combine(root, "WhiskerDynamics");
        var modTomlPath = Path.Combine(projectDirectory, "mod.toml");

        try
        {
            Directory.CreateDirectory(outputDirectory);
            Directory.CreateDirectory(projectDirectory);
            Directory.CreateDirectory(destinationDirectory);
            foreach (var name in new[]
            {
                "WhiskerDynamics.Mod.dll",
                "WhiskerDynamics.Mod.deps.json",
                "WhiskerDynamics.Core.dll",
            })
            {
                File.WriteAllText(Path.Combine(outputDirectory, name), $"new {name}");
            }
            File.WriteAllText(modTomlPath, "new manifest");
            var priorPath = Path.Combine(destinationDirectory, "prior-complete.txt");
            File.WriteAllText(priorPath, "prior deployment");

            using var process = Process.Start(CreateDeploymentProbe(
                scriptPath, outputDirectory, modTomlPath, destinationDirectory, repoRoot))
                ?? throw new InvalidOperationException("Could not start deployment transaction probe.");

            Assert.True(process.WaitForExit(30_000), "Deployment transaction probe timed out.");
            string error = process.StandardError.ReadToEnd();
            Assert.Equal(1, process.ExitCode);
            Assert.Contains("WhiskerDynamics.Compatibility.dll", error);
            Assert.Equal("prior deployment", File.ReadAllText(priorPath));
            Assert.Equal(["prior-complete.txt"],
                Directory.GetFiles(destinationDirectory)
                    .Select(path => Path.GetFileName(path)!)
                    .ToArray());
            Assert.Empty(Directory.GetDirectories(root, ".WhiskerDynamics.*"));
        }
        finally
        {
            if (Directory.Exists(root)) Directory.Delete(root, recursive: true);
        }
    }

    [Fact]
    public void Successful_transaction_replaces_the_payload_and_removes_prior_siblings()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var backup = Path.Combine(root, ".WhiskerDynamics.backup");
        try
        {
            var (output, modToml) = CreatePublish(root);
            Directory.CreateDirectory(destination);
            File.WriteAllText(Path.Combine(destination, "old.txt"), "old payload");

            var result = Run(CreateDeploymentProbe(
                ScriptPath(repoRoot), output, modToml, destination, repoRoot));

            Assert.Equal(0, result.ExitCode);
            Assert.False(File.Exists(Path.Combine(destination, "old.txt")));
            Assert.Equal(
                RequiredPublishFiles.Append("mod.toml").Order().ToArray(),
                Directory.GetFiles(destination)
                    .Select(Path.GetFileName).Order().ToArray());
            Assert.Equal(["Earth.json", "Luna.json"],
                Directory.GetFiles(Path.Combine(destination, "body-settings"))
                    .Select(path => Path.GetFileName(path)!).Order().ToArray());
            Assert.False(Directory.Exists(backup));
            Assert.Empty(Directory.GetDirectories(root, ".WhiskerDynamics.*"));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Copy_failure_cleans_staging_and_leaves_live_payload_untouched()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        try
        {
            var (output, modToml) = CreatePublish(root);
            Directory.CreateDirectory(destination);
            var prior = Path.Combine(destination, "prior.txt");
            File.WriteAllText(prior, "prior payload");
            using (new FileStream(
                Path.Combine(output, "WhiskerDynamics.Compatibility.dll"),
                FileMode.Open, FileAccess.ReadWrite, FileShare.None))
            {
                var result = Run(CreateDeploymentProbe(
                    ScriptPath(repoRoot), output, modToml, destination, repoRoot));
                Assert.Equal(1, result.ExitCode);
            }

            Assert.Equal("prior payload", File.ReadAllText(prior));
            Assert.Empty(Directory.GetDirectories(root, ".WhiskerDynamics.*"));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Precommit_write_failure_leaves_live_payload_untouched_and_cleans_staging()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var blockedWritePath = Path.Combine(root, "manifest.toml");
        try
        {
            var (output, modToml) = CreatePublish(root);
            Directory.CreateDirectory(destination);
            Directory.CreateDirectory(blockedWritePath);
            var prior = Path.Combine(destination, "prior.txt");
            File.WriteAllText(prior, "prior payload");

            var result = Run(CreatePrecommitWriteProbe(
                ScriptPath(repoRoot), output, modToml, destination,
                blockedWritePath, repoRoot));

            Assert.Equal(1, result.ExitCode);
            Assert.Equal("prior payload", File.ReadAllText(prior));
            Assert.Empty(Directory.GetDirectories(root, ".WhiskerDynamics.staging-*"));
            Assert.False(Directory.Exists(Path.Combine(root, ".WhiskerDynamics.backup")));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Stage_promotion_failure_restores_the_displaced_live_payload()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var backup = Path.Combine(root, ".WhiskerDynamics.backup");
        var missingStage = Path.Combine(root, ".WhiskerDynamics.staging-missing");
        try
        {
            Directory.CreateDirectory(destination);
            var prior = Path.Combine(destination, "prior.txt");
            File.WriteAllText(prior, "prior payload");

            var result = Run(CreateCommitProbe(
                ScriptPath(repoRoot), missingStage, destination, backup, repoRoot));

            Assert.Equal(1, result.ExitCode);
            Assert.Contains("prior deployment was restored", result.StandardError);
            Assert.Equal("prior payload", File.ReadAllText(prior));
            Assert.False(Directory.Exists(backup));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Failed_promotion_never_discards_an_existing_recovery_backup()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var backup = Path.Combine(root, ".WhiskerDynamics.backup");
        var missingStage = Path.Combine(root, ".WhiskerDynamics.staging-missing");
        try
        {
            Directory.CreateDirectory(destination);
            Directory.CreateDirectory(backup);
            var live = Path.Combine(destination, "live-candidate.txt");
            var knownGood = Path.Combine(backup, "known-good.txt");
            File.WriteAllText(live, "live candidate");
            File.WriteAllText(knownGood, "known good backup");

            var result = Run(CreateCommitProbe(
                ScriptPath(repoRoot), missingStage, destination, backup, repoRoot));

            Assert.Equal(1, result.ExitCode);
            Assert.Equal("live candidate", File.ReadAllText(live));
            Assert.Equal("known good backup", File.ReadAllText(knownGood));
            Assert.Empty(Directory.GetDirectories(root, ".WhiskerDynamics.retired-*"));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Restoration_failure_preserves_the_backup_at_its_recovery_path()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var backup = Path.Combine(root, ".WhiskerDynamics.backup");
        var missingStage = Path.Combine(root, ".WhiskerDynamics.staging-missing");
        try
        {
            Directory.CreateDirectory(root);
            File.WriteAllText(destination, "path collision");
            Directory.CreateDirectory(backup);
            var knownGood = Path.Combine(backup, "known-good.txt");
            File.WriteAllText(knownGood, "known good backup");

            var result = Run(CreateCommitProbe(
                ScriptPath(repoRoot), missingStage, destination, backup, repoRoot));

            Assert.Equal(1, result.ExitCode);
            Assert.Contains("automatic restore failed", result.StandardError);
            Assert.Equal("path collision", File.ReadAllText(destination));
            Assert.Equal("known good backup", File.ReadAllText(knownGood));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Live_rename_failure_leaves_every_input_in_place()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var backup = Path.Combine(root, ".WhiskerDynamics.backup");
        var staging = Path.Combine(root, ".WhiskerDynamics.staging-ready");
        try
        {
            Directory.CreateDirectory(destination);
            Directory.CreateDirectory(staging);
            File.WriteAllText(Path.Combine(destination, "prior.txt"), "prior payload");
            File.WriteAllText(Path.Combine(staging, "new.txt"), "new payload");
            File.WriteAllText(backup, "backup path collision");

            var result = Run(CreateCommitProbe(
                ScriptPath(repoRoot), staging, destination, backup, repoRoot));

            Assert.Equal(1, result.ExitCode);
            Assert.Equal("prior payload",
                File.ReadAllText(Path.Combine(destination, "prior.txt")));
            Assert.Equal("new payload", File.ReadAllText(Path.Combine(staging, "new.txt")));
            Assert.Equal("backup path collision", File.ReadAllText(backup));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Backup_cleanup_failure_keeps_the_successful_live_deployment()
    {
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var destination = Path.Combine(root, "WhiskerDynamics");
        var backup = Path.Combine(root, ".WhiskerDynamics.backup");
        var staging = Path.Combine(root, ".WhiskerDynamics.staging-ready");
        try
        {
            Directory.CreateDirectory(backup);
            Directory.CreateDirectory(staging);
            var lockedBackup = Path.Combine(backup, "prior.txt");
            File.WriteAllText(lockedBackup, "prior payload");
            File.WriteAllText(Path.Combine(staging, "new.txt"), "new payload");

            using (new FileStream(lockedBackup, FileMode.Open,
                FileAccess.Read, FileShare.Read))
            {
                var result = Run(CreateCommitProbe(
                    ScriptPath(repoRoot), staging, destination, backup, repoRoot));
                Assert.Equal(0, result.ExitCode);
                Assert.Contains("could not remove prior deployment", result.StandardError);
            }

            Assert.Equal("new payload", File.ReadAllText(Path.Combine(destination, "new.txt")));
            Assert.Equal("prior payload", File.ReadAllText(lockedBackup));
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Manifest_probe_requires_an_active_exact_mod_id()
    {
        var quote = (char)34;
        var cases = new[]
        {
            ($"[[mods]]\nid = {quote}WhiskerDynamics{quote}\nenabled = false", 0),
            ("[[mods]]\nid = 'WhiskerDynamics'\nenabled = false", 0),
            ($"[[mods]]\nid = {quote}Whisker\\u0044ynamics{quote}\nenabled = true", 0),
            ($"[[mods]]\n# id = {quote}WhiskerDynamics{quote}\nenabled = false", 1),
            ($"[[mods]]\nid = {quote}WhiskerDynamicsPlus{quote}\nenabled = true", 1),
            ($"[[mods]]\nid = {quote}WhiskerDynamics{quote} # keep disabled\nenabled = false", 0),
        };

        var repoRoot = FindRepoRoot();
        var scriptPath = Path.Combine(repoRoot, "scripts", "deploy-mod.cs");
        var manifestPath = Path.Combine(Path.GetTempPath(), $"wd-manifest-{Guid.NewGuid():N}.toml");

        try
        {
            foreach (var (manifest, expectedExitCode) in cases)
            {
                File.WriteAllText(manifestPath, manifest);
                using var process = Process.Start(CreateProbe(scriptPath, manifestPath, repoRoot))
                    ?? throw new InvalidOperationException("Could not start deployment manifest probe.");

                Assert.True(process.WaitForExit(30_000), "Deployment manifest probe timed out.");
                Assert.Equal(expectedExitCode, process.ExitCode);
            }
        }
        finally
        {
            File.Delete(manifestPath);
        }
    }

    [Fact]
    public void Manifest_probe_respects_tables_quoted_keys_and_malformed_assignments()
    {
        (string Manifest, int ExitCode)[] cases =
        [
            ("""
                [[ "m\u006Fds" ]] # quoted array-table key
                "i\u0064" = "Whisker\U00000044ynamics"
                enabled = false
                """, 0),
            ("""
                [[ 'mods' ]]
                'id' = 'WhiskerDynamics' # literal key and value
                enabled = true
                """, 0),
            ("""
                id = "WhiskerDynamics"
                [metadata]
                name = "root assignment"
                """, 1),
            ("""
                [metadata]
                id = "WhiskerDynamics"
                """, 1),
            ("""
                [[other]]
                id = "WhiskerDynamics"
                """, 1),
            ("""
                [[mods.details]]
                id = "WhiskerDynamics"
                """, 1),
            ("""
                [mods]
                id = "WhiskerDynamics"
                """, 1),
            ("""
                [[mods]]
                metadata = { id = "WhiskerDynamics" }
                """, 1),
            ("""
                [[mods]]
                id = "WhiskerDynamics" trailing
                """, 1),
            ("""
                [[mods]]
                id = "Whisker\qDynamics"
                """, 1),
            ("""
                [[mods]]
                id = "Whisker\tDynamics"
                """, 1),
        ];
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var manifestPath = Path.Combine(root, "manifest.toml");

        try
        {
            Directory.CreateDirectory(root);
            foreach (var testCase in cases)
            {
                File.WriteAllText(manifestPath, testCase.Manifest);
                var result = Run(CreateProbe(ScriptPath(repoRoot), manifestPath, repoRoot));
                Assert.Equal(testCase.ExitCode, result.ExitCode);
            }
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Manifest_update_preserves_literal_and_escaped_basic_string_entries()
    {
        var quote = (char)34;
        string[] manifests =
        [
            "[[mods]]\nid = 'WhiskerDynamics'\nenabled = false\n",
            $"[[mods]]\nid = {quote}Whisker\\u0044ynamics{quote}\nenabled = true\n",
        ];
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var manifestPath = Path.Combine(root, "manifest.toml");

        try
        {
            Directory.CreateDirectory(root);
            foreach (string manifest in manifests)
            {
                byte[] original = Encoding.UTF8.GetBytes(manifest);
                File.WriteAllBytes(manifestPath, original);

                var result = Run(CreateManifestUpdateProbe(
                    ScriptPath(repoRoot), manifestPath, repoRoot));

                Assert.Equal(0, result.ExitCode);
                Assert.Equal(original, File.ReadAllBytes(manifestPath));
                Assert.Contains("enabled state left as-is", result.StandardOutput);
            }
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    [Fact]
    public void Manifest_update_appends_only_for_a_real_missing_mod_entry()
    {
        const string unrelated = """
            [metadata]
            id = "WhiskerDynamics"
            enabled = false
            """;
        const string duplicates = """
            [[mods]]
            id = 'WhiskerDynamics'
            enabled = false

            [[mods]]
            id = "WhiskerDynamics"
            enabled = true
            """;
        var repoRoot = FindRepoRoot();
        var root = NewRoot();
        var manifestPath = Path.Combine(root, "manifest.toml");

        try
        {
            Directory.CreateDirectory(root);
            File.WriteAllText(manifestPath, unrelated);
            var append = Run(CreateManifestUpdateProbe(
                ScriptPath(repoRoot), manifestPath, repoRoot));
            string updated = File.ReadAllText(manifestPath);

            Assert.Equal(0, append.ExitCode);
            Assert.StartsWith(unrelated, updated, StringComparison.Ordinal);
            Assert.Equal(1,
                updated.Split("[[mods]]", StringSplitOptions.None).Length - 1);
            Assert.Equal(0, Run(CreateProbe(
                ScriptPath(repoRoot), manifestPath, repoRoot)).ExitCode);

            byte[] duplicateBytes = Encoding.UTF8.GetBytes(duplicates);
            File.WriteAllBytes(manifestPath, duplicateBytes);
            var preserve = Run(CreateManifestUpdateProbe(
                ScriptPath(repoRoot), manifestPath, repoRoot));

            Assert.Equal(0, preserve.ExitCode);
            Assert.Equal(duplicateBytes, File.ReadAllBytes(manifestPath));
            Assert.Contains("enabled state left as-is", preserve.StandardOutput);
        }
        finally
        {
            DeleteRoot(root);
        }
    }

    private static ProcessStartInfo CreateDeploymentProbe(
        string scriptPath,
        string outputDirectory,
        string modTomlPath,
        string destinationDirectory,
        string repoRoot)
    {
        var info = CreateScriptProcess(scriptPath, repoRoot);
        info.ArgumentList.Add("--check-deployment-transaction");
        info.ArgumentList.Add(outputDirectory);
        info.ArgumentList.Add(modTomlPath);
        info.ArgumentList.Add(destinationDirectory);
        return info;
    }

    private static ProcessStartInfo CreatePrecommitWriteProbe(
        string scriptPath,
        string outputDirectory,
        string modTomlPath,
        string destinationDirectory,
        string writePath,
        string repoRoot)
    {
        var info = CreateScriptProcess(scriptPath, repoRoot);
        info.ArgumentList.Add("--check-deployment-precommit-write");
        info.ArgumentList.Add(outputDirectory);
        info.ArgumentList.Add(modTomlPath);
        info.ArgumentList.Add(destinationDirectory);
        info.ArgumentList.Add(writePath);
        return info;
    }

    private static ProcessStartInfo CreateCommitProbe(
        string scriptPath,
        string stagingDirectory,
        string destinationDirectory,
        string backupDirectory,
        string repoRoot)
    {
        var info = CreateScriptProcess(scriptPath, repoRoot);
        info.ArgumentList.Add("--check-deployment-commit");
        info.ArgumentList.Add(stagingDirectory);
        info.ArgumentList.Add(destinationDirectory);
        info.ArgumentList.Add(backupDirectory);
        return info;
    }

    private static ProcessStartInfo CreateProbe(string scriptPath, string manifestPath, string repoRoot)
    {
        var info = CreateScriptProcess(scriptPath, repoRoot);
        info.ArgumentList.Add("--check-manifest");
        info.ArgumentList.Add(manifestPath);
        return info;
    }

    private static ProcessStartInfo CreateManifestUpdateProbe(
        string scriptPath, string manifestPath, string repoRoot)
    {
        var info = CreateScriptProcess(scriptPath, repoRoot);
        info.ArgumentList.Add("--check-manifest-update");
        info.ArgumentList.Add(manifestPath);
        return info;
    }

    private static ProcessStartInfo CreateScriptProcess(string scriptPath, string repoRoot)
    {
        var info = new ProcessStartInfo("dotnet")
        {
            WorkingDirectory = repoRoot,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
        };
        info.ArgumentList.Add("run");
        info.ArgumentList.Add("--file");
        info.ArgumentList.Add(scriptPath);
        info.ArgumentList.Add("--");
        return info;
    }

    private static (int ExitCode, string StandardOutput, string StandardError) Run(
        ProcessStartInfo info)
    {
        using var process = Process.Start(info)
            ?? throw new InvalidOperationException("Could not start deployment probe.");
        Task<string> standardOutput = process.StandardOutput.ReadToEndAsync();
        Task<string> standardError = process.StandardError.ReadToEndAsync();
        Assert.True(process.WaitForExit(30_000), "Deployment probe timed out.");
        return (process.ExitCode,
            standardOutput.GetAwaiter().GetResult(),
            standardError.GetAwaiter().GetResult());
    }

    private static (string OutputDirectory, string ModTomlPath) CreatePublish(string root)
    {
        var output = Path.Combine(root, "publish");
        var project = Path.Combine(root, "project");
        Directory.CreateDirectory(output);
        Directory.CreateDirectory(project);
        foreach (string name in RequiredPublishFiles)
            File.WriteAllText(Path.Combine(output, name), $"new {name}");
        string bodySettings = Path.Combine(output, "body-settings");
        Directory.CreateDirectory(bodySettings);
        File.WriteAllText(Path.Combine(bodySettings, "Earth.json"), "earth settings");
        File.WriteAllText(Path.Combine(bodySettings, "Luna.json"), "luna settings");
        var modToml = Path.Combine(project, "mod.toml");
        File.WriteAllText(modToml, "new manifest");
        return (output, modToml);
    }

    private static string NewRoot() =>
        Path.Combine(Path.GetTempPath(), $"wd-deploy-{Guid.NewGuid():N}");

    private static string ScriptPath(string repoRoot) =>
        Path.Combine(repoRoot, "scripts", "deploy-mod.cs");

    private static void DeleteRoot(string root)
    {
        if (Directory.Exists(root)) Directory.Delete(root, recursive: true);
    }

    private static string FindRepoRoot()
    {
        for (var directory = new DirectoryInfo(AppContext.BaseDirectory);
             directory is not null;
             directory = directory.Parent)
        {
            if (File.Exists(Path.Combine(directory.FullName, "scripts", "deploy-mod.cs")))
            {
                return directory.FullName;
            }
        }

        throw new DirectoryNotFoundException("Could not find the repository root.");
    }
}
