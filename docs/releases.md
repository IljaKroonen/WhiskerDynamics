# Publishing a GitHub release

Releases are made locally because compiling the mod requires the KSA and
StarMap assemblies installed on the build computer.

## One-time setup

1. Install the [GitHub CLI](https://cli.github.com/).
2. Authenticate it with `gh auth login`, then confirm with `gh auth status`.
3. Make sure the installed KSA build matches `VerifiedKsaBuild` in
   `Directory.Build.props` and that StarMap is installed.

## Publish

Commit and push everything intended for the release. The current branch must
be clean, pushed, and tracking its upstream. From the repository root, run:

```powershell
dotnet run --file .\scripts\publish-github-release.cs `
  -- 0.1.0
```

The command:

1. Runs the solution's Release test suite.
2. Creates the versioned SpaceDock zip and a SHA-256 checksum file.
3. Creates and pushes an annotated `v0.1.0` tag for the current commit.
4. Publishes a GitHub release with generated notes and both files attached.

Semantic versions containing a pre-release suffix, such as `0.2.0-beta.1`, are
automatically marked as GitHub pre-releases. Useful options are:

```powershell
# Create a draft for review instead of publishing immediately.
dotnet run --file .\scripts\publish-github-release.cs `
  -- 0.1.0 --draft

# Skip tests only if this exact commit already passed the Release suite.
dotnet run --file .\scripts\publish-github-release.cs `
  -- 0.1.0 --skip-tests
```

If a network operation fails after the tag is pushed, rerun the same command.
It safely reuses a local or remote tag when that tag still points to the current
commit. It refuses to move an existing tag or replace an existing release.
