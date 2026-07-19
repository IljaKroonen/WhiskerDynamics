using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml.Linq;

return PublishGitHubRelease(args);

static int PublishGitHubRelease(string[] args)
{
    try
    {
        var options = ParseOptions(args);
        var scriptDirectory = GetScriptDirectory();
        var repoRoot = Directory.GetParent(scriptDirectory)!.FullName;
        var tag = $"v{options.Version}";
        var bundleName = $"WhiskerDynamics-{options.Version}.zip";
        var bundlePath = Path.Combine(repoRoot, "artifacts", bundleName);
        var checksumPath = $"{bundlePath}.sha256";

        Run("git", ["rev-parse", "--is-inside-work-tree"], repoRoot);

        var changes = CaptureSuccessful(
            "git", ["status", "--porcelain=v1"], repoRoot);
        if (changes.Length > 0)
            throw new InvalidOperationException(
                $"the working tree must be clean before releasing:{Environment.NewLine}{changes}");

        var head = CaptureSuccessful("git", ["rev-parse", "HEAD"], repoRoot);
        var branch = CaptureSuccessful(
            "git", ["branch", "--show-current"], repoRoot);
        if (branch.Length == 0)
            throw new InvalidOperationException(
                "releases must be made from a branch, not a detached HEAD");

        var upstream = CaptureSuccessful(
            "git", ["rev-parse", "@{upstream}"], repoRoot);
        if (!string.Equals(head, upstream, StringComparison.Ordinal))
            throw new InvalidOperationException(
                $"HEAD ({head}) does not match the upstream commit ({upstream}); "
                + "push or pull the branch before releasing");

        Run("gh", ["auth", "status"], repoRoot);

        var existingRelease = Capture(
            "gh", ["release", "view", tag, "--json", "tagName"], repoRoot);
        if (existingRelease.ExitCode == 0)
            throw new InvalidOperationException($"GitHub release {tag} already exists");

        var localTagProbe = Capture(
            "git", ["show-ref", "--verify", "--quiet", $"refs/tags/{tag}"], repoRoot);
        if (localTagProbe.ExitCode is not (0 or 1))
            throw CommandFailed("git", localTagProbe);
        var localTagExists = localTagProbe.ExitCode == 0;

        var remoteTagProbe = Capture(
            "git", ["ls-remote", "--tags", options.Remote, $"refs/tags/{tag}"], repoRoot);
        if (remoteTagProbe.ExitCode != 0)
            throw CommandFailed("git", remoteTagProbe);
        var remoteTagLines = remoteTagProbe.StandardOutput
            .Split(['\r', '\n'], StringSplitOptions.RemoveEmptyEntries);
        var remoteTagExists = remoteTagLines.Length > 0;

        if (!localTagExists && remoteTagExists)
        {
            Run(
                "git",
                ["fetch", "--no-tags", options.Remote,
                    $"refs/tags/{tag}:refs/tags/{tag}"],
                repoRoot);
            localTagExists = true;
        }

        if (localTagExists)
        {
            if (remoteTagExists)
            {
                var localTagObject = CaptureSuccessful(
                    "git", ["rev-parse", tag], repoRoot);
                var remoteTagObject = remoteTagLines[0]
                    .Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries)[0];
                if (!string.Equals(
                        remoteTagObject, localTagObject, StringComparison.Ordinal))
                    throw new InvalidOperationException(
                        $"local and remote tag {tag} do not refer to the same tag object");
            }

            var tagCommit = CaptureSuccessful(
                "git", ["rev-list", "-n", "1", tag], repoRoot);
            if (!string.Equals(tagCommit, head, StringComparison.Ordinal))
                throw new InvalidOperationException(
                    $"tag {tag} points to {tagCommit} instead of HEAD ({head})");
        }

        if (!options.SkipTests)
            Run(
                "dotnet",
                ["test", "WhiskerDynamics.slnx", "--configuration", "Release",
                    "--disable-build-servers", "-m:1"],
                repoRoot);

        Run(
            "dotnet",
            ["run", "--file", "scripts/create-spacedock-bundle.cs",
                "--disable-build-servers", "--", options.Version, "Release"],
            repoRoot);

        if (!File.Exists(bundlePath))
            throw new FileNotFoundException(
                $"bundle was not created at {bundlePath}", bundlePath);

        var hash = Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(bundlePath)))
            .ToLowerInvariant();
        File.WriteAllText(
            checksumPath,
            $"{hash}  {bundleName}{Environment.NewLine}",
            Encoding.ASCII);
        Console.WriteLine($"SHA-256: {hash}");

        if (!localTagExists)
            Run(
                "git",
                ["tag", "--annotate", tag, "--message",
                    $"Whisker Dynamics {options.Version}"],
                repoRoot);

        if (!remoteTagExists)
            Run("git", ["push", options.Remote, $"refs/tags/{tag}"], repoRoot);

        var verifiedKsaBuild = XDocument
            .Load(Path.Combine(repoRoot, "Directory.Build.props"))
            .Descendants("VerifiedKsaBuild")
            .Single()
            .Value;
        var releaseArguments = new List<string>
        {
            "release", "create", tag,
            bundlePath,
            checksumPath,
            "--verify-tag",
            "--generate-notes",
            "--notes", $"Compatible with Kitten Space Agency build {verifiedKsaBuild}.",
            "--title", $"Whisker Dynamics {options.Version}",
        };
        if (options.Draft) releaseArguments.Add("--draft");
        if (options.Prerelease || IsPrerelease(options.Version))
            releaseArguments.Add("--prerelease");

        Run("gh", releaseArguments, repoRoot);
        Console.WriteLine(options.Draft
            ? $"Created draft GitHub release {tag}."
            : $"Published GitHub release {tag}.");
        return 0;
    }
    catch (Exception exception)
    {
        Console.Error.WriteLine($"release failed: {exception.Message}");
        return 1;
    }
}

static ReleaseOptions ParseOptions(string[] args)
{
    if (args.Length == 0)
        throw new ArgumentException(
            "usage: dotnet run --file scripts/publish-github-release.cs "
            + "--disable-build-servers -- <Version> "
            + "[--draft] [--prerelease] [--skip-tests] [--remote <Name>]");

    var version = args[0];
    if (!Regex.IsMatch(
            version,
            @"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)"
            + @"(?:-[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
            + @"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?$"))
        throw new ArgumentException($"invalid semantic version: {version}");

    var draft = false;
    var prerelease = false;
    var skipTests = false;
    var remote = "origin";
    for (var index = 1; index < args.Length; index++)
    {
        switch (args[index])
        {
            case "--draft":
                draft = true;
                break;
            case "--prerelease":
                prerelease = true;
                break;
            case "--skip-tests":
                skipTests = true;
                break;
            case "--remote" when index + 1 < args.Length:
                remote = args[++index];
                break;
            default:
                throw new ArgumentException($"unknown or incomplete option: {args[index]}");
        }
    }

    if (!Regex.IsMatch(remote, @"^[0-9A-Za-z._-]+$"))
        throw new ArgumentException($"invalid remote name: {remote}");

    return new ReleaseOptions(version, draft, prerelease, skipTests, remote);
}

static bool IsPrerelease(string version)
{
    var coreAndPrerelease = version.Split('+', 2)[0];
    return coreAndPrerelease.Contains('-');
}

static void Run(
    string command,
    IEnumerable<string> arguments,
    string workingDirectory)
{
    var argumentArray = arguments.ToArray();
    WriteCommand(command, argumentArray);
    using var process = Process.Start(CreateStartInfo(
        command, argumentArray, workingDirectory, capture: false))
        ?? throw new InvalidOperationException($"could not start {command}");
    process.WaitForExit();
    if (process.ExitCode != 0)
        throw new InvalidOperationException(
            $"command failed with exit code {process.ExitCode}: {command}");
}

static string CaptureSuccessful(
    string command,
    IEnumerable<string> arguments,
    string workingDirectory)
{
    var result = Capture(command, arguments, workingDirectory);
    if (result.ExitCode != 0) throw CommandFailed(command, result);
    return result.StandardOutput.Trim();
}

static ProcessResult Capture(
    string command,
    IEnumerable<string> arguments,
    string workingDirectory)
{
    var argumentArray = arguments.ToArray();
    WriteCommand(command, argumentArray);
    using var process = Process.Start(CreateStartInfo(
        command, argumentArray, workingDirectory, capture: true))
        ?? throw new InvalidOperationException($"could not start {command}");
    var standardOutput = process.StandardOutput.ReadToEndAsync();
    var standardError = process.StandardError.ReadToEndAsync();
    process.WaitForExit();
    Task.WaitAll(standardOutput, standardError);
    return new ProcessResult(
        process.ExitCode,
        standardOutput.Result,
        standardError.Result);
}

static ProcessStartInfo CreateStartInfo(
    string command,
    IEnumerable<string> arguments,
    string workingDirectory,
    bool capture)
{
    var startInfo = new ProcessStartInfo(command)
    {
        UseShellExecute = false,
        WorkingDirectory = workingDirectory,
        RedirectStandardOutput = capture,
        RedirectStandardError = capture,
    };
    foreach (var argument in arguments) startInfo.ArgumentList.Add(argument);
    return startInfo;
}

static InvalidOperationException CommandFailed(
    string command,
    ProcessResult result)
{
    var detail = string.Join(
        Environment.NewLine,
        new[] { result.StandardOutput.Trim(), result.StandardError.Trim() }
            .Where(value => value.Length > 0));
    var message = $"command failed with exit code {result.ExitCode}: {command}";
    if (detail.Length > 0) message += $"{Environment.NewLine}{detail}";
    return new InvalidOperationException(message);
}

static void WriteCommand(string command, IEnumerable<string> arguments) =>
    Console.WriteLine($"> {command} {string.Join(" ", arguments.Select(Quote))}");

static string Quote(string argument) =>
    argument.Any(char.IsWhiteSpace) ? $"\"{argument}\"" : argument;

static string GetScriptDirectory([CallerFilePath] string sourcePath = "") =>
    Path.GetDirectoryName(sourcePath)
    ?? throw new InvalidOperationException("could not locate the release script");

sealed record ReleaseOptions(
    string Version,
    bool Draft,
    bool Prerelease,
    bool SkipTests,
    string Remote);

sealed record ProcessResult(
    int ExitCode,
    string StandardOutput,
    string StandardError);
