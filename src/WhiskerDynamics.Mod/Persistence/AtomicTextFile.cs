using System.Text;

namespace WhiskerDynamics.Mod.Persistence;

/// <summary>Per-call fault seam for the KSA-free atomic text writer. The commit wrapper
/// may call its supplied default action to perform the real atomic replacement, or throw
/// before calling it to model a replacement failure without global test state.</summary>
internal sealed record AtomicTextFileHooks(
    Action<string, string>? AfterTempFlushedAndClosed = null,
    Action<string, string, Action>? Commit = null);

/// <summary>Writes UTF-8 text without exposing a partially-written destination. Data is
/// flushed through the temporary file before one same-directory metadata commit. This
/// does not claim a portable parent-directory fsync: after the commit, the visible file
/// is the complete old or new version according to the host filesystem's rename rules.
/// The rename publishes the temp file's metadata. Ordinary Unix mode bits are copied
/// from an existing destination, but destination-specific Windows ACLs, attributes and
/// alternate data streams, and other Unix metadata are not portably retained.</summary>
internal static class AtomicTextFile
{
    private static readonly Encoding Utf8NoBom = new UTF8Encoding(
        encoderShouldEmitUTF8Identifier: false);

    internal static void WriteAllText(string path, string contents,
        AtomicTextFileHooks? hooks = null)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        ArgumentNullException.ThrowIfNull(contents);

        string destination = Path.GetFullPath(path);
        string? directory = Path.GetDirectoryName(destination);
        if (string.IsNullOrEmpty(directory) || string.IsNullOrEmpty(Path.GetFileName(destination)))
            throw new ArgumentException("Destination must name a file.", nameof(path));
        if (!Directory.Exists(directory))
            throw new DirectoryNotFoundException(
                $"Destination directory does not exist: '{directory}'.");

        // Encoding happens before any filesystem mutation, so even malformed input cannot
        // leave a temporary artifact. GetBytes does not emit the encoding preamble.
        byte[] bytes = Utf8NoBom.GetBytes(contents);
        UnixFileMode? existingUnixMode = null;
        if (!OperatingSystem.IsWindows() && File.Exists(destination))
            existingUnixMode = File.GetUnixFileMode(destination);

        string temp = Path.Combine(directory,
            ".whiskerdynamics-atomic-" + Guid.NewGuid().ToString("N") + ".tmp");
        bool ownsTemp = false;
        try
        {
            using (var stream = new FileStream(temp, new FileStreamOptions
            {
                Mode = FileMode.CreateNew,
                Access = FileAccess.Write,
                Share = FileShare.None,
                BufferSize = 4096,
                Options = FileOptions.WriteThrough | FileOptions.SequentialScan,
            }))
            {
                ownsTemp = true;
                // Overwrite-move publishes the temp's metadata. Retain ordinary Unix mode
                // bits; no portable API preserves every platform-specific metadata kind.
                if (!OperatingSystem.IsWindows() && existingUnixMode is { } mode)
                    File.SetUnixFileMode(temp, mode);
                stream.Write(bytes);
                stream.Flush(flushToDisk: true);
            }

            hooks?.AfterTempFlushedAndClosed?.Invoke(temp, destination);
            Action defaultCommit = () =>
            {
                Commit(temp, destination);
                // Commit consumed the temp name. Transfer ownership before returning to
                // a hook, which may reuse that path and then throw: finally must never
                // delete a new file that happens to occupy our former temp name.
                ownsTemp = false;
            };
            if (hooks?.Commit is { } commit)
                commit(temp, destination, defaultCommit);
            else
                defaultCommit();
        }
        finally
        {
            // Never let cleanup hide the write/commit error, touch destination, or touch
            // a former temp path after a successful commit transferred its ownership.
            try { if (ownsTemp) File.Delete(temp); }
            catch { /* best effort; a crashed process may likewise leave only a temp */ }
        }
    }

    private static void Commit(string temp, string destination)
    {
        if (File.Exists(destination))
        {
            // Both paths share a directory, so overwrite-move is one rename/replace
            // operation rather than the cross-volume copy fallback. In particular, do
            // not use backup-less File.Replace: ReplaceFileW documents failure states
            // that remove the destination while leaving the replacement at its temp name.
            File.Move(temp, destination, overwrite: true);
            return;
        }

        // First publication is a no-overwrite same-directory rename. If another writer
        // wins the creation race, this fails without damaging that complete winner.
        File.Move(temp, destination, overwrite: false);
    }
}
