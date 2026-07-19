using System.Text;
using WhiskerDynamics.Mod;

namespace WhiskerDynamics.Mod.Tests.Persistence;

public sealed class AtomicTextFileTests : IDisposable
{
    private const string InjectedMessage = "injected atomic-write interruption";
    private readonly string _directory = Path.Combine(Path.GetTempPath(),
        "whisker-dynamics-atomic-text-tests-" + Guid.NewGuid().ToString("N"));

    public AtomicTextFileTests() => Directory.CreateDirectory(_directory);

    public void Dispose()
    {
        try { Directory.Delete(_directory, recursive: true); }
        catch { /* best effort */ }
    }

    [Fact]
    public void Create_and_overwrite_publish_exact_utf8_only_after_the_temp_is_closed()
    {
        string path = Path.Combine(_directory, "settings.toml");
        string expectedText = "first = \"meow\"\nemoji = \"cat\"\n";
        int observed = 0;
        var hooks = new AtomicTextFileHooks
        {
            AfterTempFlushedAndClosed = (temp, destination) =>
            {
                Assert.Equal(path, destination);
                Assert.True(File.Exists(temp));
                // FileShare.None proves the writer released its handle before commit.
                using var stream = new FileStream(temp, FileMode.Open, FileAccess.Read,
                    FileShare.None);
                byte[] bytes = new byte[checked((int)stream.Length)];
                stream.ReadExactly(bytes);
                Assert.Equal(Encoding.UTF8.GetBytes(expectedText), bytes);
                observed++;
            },
        };

        AtomicTextFile.WriteAllText(path, expectedText, hooks);

        Assert.Equal(1, observed);
        Assert.Equal(Encoding.UTF8.GetBytes(expectedText), File.ReadAllBytes(path));
        Assert.Equal([path], Directory.GetFiles(_directory));

        expectedText = "second = \"unicode cat: \u732b\"\nwithout_bom = true\n";
        AtomicTextFile.WriteAllText(path, expectedText, hooks);

        Assert.Equal(2, observed);
        Assert.Equal(Encoding.UTF8.GetBytes(expectedText), File.ReadAllBytes(path));
        Assert.Equal([path], Directory.GetFiles(_directory));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Interrupted_overwrite_preserves_byte_exact_destination_and_retry_recovers(
        bool failCommit)
    {
        string path = Path.Combine(_directory, "durable.txt");
        byte[] original = [0, 1, 2, 13, 10, 0xff, 0x7f];
        File.WriteAllBytes(path, original);

        IOException failure = Assert.Throws<IOException>(() =>
            AtomicTextFile.WriteAllText(path, "replacement", FailingHooks(failCommit)));

        Assert.Equal(InjectedMessage, failure.Message);
        Assert.Equal(original, File.ReadAllBytes(path));
        Assert.Equal([path], Directory.GetFiles(_directory));

        AtomicTextFile.WriteAllText(path, "recovered");
        Assert.Equal(Encoding.UTF8.GetBytes("recovered"), File.ReadAllBytes(path));
        Assert.Equal([path], Directory.GetFiles(_directory));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Interrupted_first_create_leaves_no_destination_or_temp(bool failCommit)
    {
        string path = Path.Combine(_directory, "new.txt");

        IOException failure = Assert.Throws<IOException>(() =>
            AtomicTextFile.WriteAllText(path, "never published", FailingHooks(failCommit)));

        Assert.Equal(InjectedMessage, failure.Message);
        Assert.False(File.Exists(path));
        Assert.Empty(Directory.GetFiles(_directory));

        AtomicTextFile.WriteAllText(path, "retry");
        Assert.Equal(Encoding.UTF8.GetBytes("retry"), File.ReadAllBytes(path));
        Assert.Equal([path], Directory.GetFiles(_directory));
    }

    [Fact]
    public void Real_windows_commit_failure_keeps_destination_and_cleans_temp()
    {
        if (!OperatingSystem.IsWindows()) return;

        string path = Path.Combine(_directory, "locked.txt");
        byte[] original = Encoding.UTF8.GetBytes("previous valid contents");
        File.WriteAllBytes(path, original);

        // Deny delete sharing so MoveFileEx cannot replace the destination. The atomic
        // writer must propagate the real OS failure without touching the old bytes.
        using (new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
        {
            Exception failure = Assert.ThrowsAny<Exception>(() =>
                AtomicTextFile.WriteAllText(path, "replacement"));
            Assert.True(failure is IOException or UnauthorizedAccessException,
                $"unexpected failure type: {failure.GetType().FullName}");
        }

        Assert.Equal(original, File.ReadAllBytes(path));
        Assert.Equal([path], Directory.GetFiles(_directory));
    }

    [Fact]
    public void Successful_wrapped_commit_relinquishes_the_consumed_temp_path()
    {
        string path = Path.Combine(_directory, "published.txt");
        string? formerTemp = null;
        var hooks = new AtomicTextFileHooks(Commit: (temp, destination, commit) =>
        {
            formerTemp = temp;
            commit();
            Assert.Equal(path, destination);
            // Deterministically model another actor reusing the consumed GUID path
            // before WriteAllText's finally block runs.
            File.WriteAllText(temp, "unrelated owner");
            throw new IOException(InjectedMessage);
        });

        IOException failure = Assert.Throws<IOException>(() =>
            AtomicTextFile.WriteAllText(path, "published contents", hooks));

        Assert.Equal(InjectedMessage, failure.Message);
        Assert.Equal("published contents", File.ReadAllText(path));
        Assert.NotNull(formerTemp);
        Assert.Equal("unrelated owner", File.ReadAllText(formerTemp!));
        Assert.Equal(2, Directory.GetFiles(_directory).Length);
    }

    private static AtomicTextFileHooks FailingHooks(bool failCommit) => new(
        AfterTempFlushedAndClosed: failCommit
            ? null
            : (_, _) => throw new IOException(InjectedMessage),
        Commit: failCommit
            ? (_, _, _) => throw new IOException(InjectedMessage)
            : null);
}
