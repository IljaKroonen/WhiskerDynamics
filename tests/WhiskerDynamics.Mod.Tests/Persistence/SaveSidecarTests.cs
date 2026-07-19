using WhiskerDynamics.Mod;
using WhiskerDynamics.Mod.Patching;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Tests.Persistence;

/// <summary>Sidecar kernels: serialization (exact bit-level state round-trip),
/// pairing (exact stock-save identity with deterministic anonymous epoch fallback),
/// per-vessel containment (one bad vessel must not lose the sidecar), and the
/// osculating-note caveat pin (the osculating note references the vessel's ACTUAL stock
/// orbit parent, never the direct-field dominant-attractor label — Sol dominates
/// cislunar space, so the label is display data only). Runs over the same fixture
/// install dir the third-body/dominant tests bind (integrated set: Sol + Mercury);
/// TrackedVessel is constructed directly via ReseedAbsolute — no KSA types touched.
/// In the "flightplans-statics" collection with FlightPlanModelTests: Capture reads the
/// static FlightPlans store (plan persistence), which the other class sweeps.</summary>
[Collection("flightplans-statics")]
public sealed class SaveSidecarTests : IDisposable
{
    private readonly string _dir;
    private readonly string _sidecarDir;
    private readonly RailsService _rails;

    public SaveSidecarTests()
    {
        _dir = Path.Combine(Path.GetTempPath(), "whisker-dynamics-sidecar-tests-" + Guid.NewGuid().ToString("N"));
        var xmlDir = Path.Combine(_dir, "Content", "Core");
        Directory.CreateDirectory(xmlDir);
        File.Copy(
            Path.Combine(AppContext.BaseDirectory, "fixtures", "Astronomicals.sample.xml"),
            Path.Combine(xmlDir, "Astronomicals.xml"));
        var config = new ModConfig { RailsAheadDays = 2 };
        var constants = new GameConstants(6.6743e-11, 1.988416e30, 5.972e24, 7.346e22, 1.898e27);
        _rails = TestRailsService.FromFixture(config, constants);
        _rails.NoteSimTime(5000);
        Assert.True(SpinWait.SpinUntil(() => _rails.IsReadyAt(5000), 5000));
        _sidecarDir = Path.Combine(_dir, "sidecar-out");
        SaveSidecar.DirOverride = _sidecarDir;
    }

    public void Dispose()
    {
        SaveSidecar.DirOverride = null;
        _rails.Dispose();
        try { Directory.Delete(_dir, recursive: true); } catch { /* best effort */ }
    }

    /// <summary>Vessel at radius <paramref name="r"/> from Mercury with tangential speed
    /// <paramref name="speedFactor"/> x circular: 1.1 gives e = 0.21 exactly (tangential
    /// periapsis: e = r v^2 / mu - 1); 1.0 gives e ~ 1e-16 (near-circular throw); above
    /// sqrt(2) is hyperbolic.</summary>
    private TrackedVessel MakeVessel(string id, double elapsed, double speedFactor,
        string? parentId = "Mercury", double r = 2.74e6)
    {
        var mercury = _rails.GetAbsolute("Mercury", elapsed);
        double vCirc = Math.Sqrt(_rails.MuOf("Mercury") / r);
        var vessel = new TrackedVessel
        {
            Id = id,
            Rails = _rails,
            Options = new IntegratorOptions { RelTol = 1e-11 },
        };
        vessel.ReseedAbsolute(
            new StateVector(
                mercury.Position + new Vector3d(r, 0, 0),
                mercury.Velocity + new Vector3d(0, speedFactor * vCirc, 0)),
            elapsed);
        vessel.LastParentId = parentId;
        return vessel;
    }

    private string[] SnapshotFiles() => Directory.Exists(_sidecarDir)
        ? Directory.GetFiles(_sidecarDir, "whiskerdynamics-*.json")
            .Where(path => !path.EndsWith("whiskerdynamics-latest.json",
                StringComparison.OrdinalIgnoreCase))
            .ToArray()
        : [];

    private string[] AtomicTempFiles() => Directory.Exists(_sidecarDir)
        ? Directory.GetFiles(_sidecarDir, ".whiskerdynamics-atomic-*.tmp")
        : [];

    private static bool SamePath(string left, string right) =>
        string.Equals(Path.GetFullPath(left), Path.GetFullPath(right),
            OperatingSystem.IsWindows()
                ? StringComparison.OrdinalIgnoreCase
                : StringComparison.Ordinal);

    private static SidecarFile ReadSidecarFile(string path) =>
        Assert.IsType<SidecarFile>(
            System.Text.Json.JsonSerializer.Deserialize<SidecarFile>(
                File.ReadAllText(path)));

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Rails_rejects_non_finite_host_clock_values(double invalid)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => _rails.NoteSimTime(invalid));
    }

    [Fact]
    public void Write_persists_exact_state_and_TryRead_pairs_by_elapsed()
    {
        double elapsed = 1000.0;
        var vessel = MakeVessel("probe", elapsed, 1.1);
        StateVector expected;
        lock (_rails.Gate) expected = vessel.Predictor.StateAt(elapsed);

        SaveSidecar.Write(_rails, [vessel], elapsed, "test-build");

        string snapshot = Assert.Single(SnapshotFiles());
        Assert.StartsWith("whiskerdynamics-epoch-", Path.GetFileName(snapshot));
        Assert.True(File.Exists(Path.Combine(_sidecarDir, "whiskerdynamics-latest.json")));
        var file = SaveSidecar.TryRead(elapsed);
        Assert.NotNull(file);
        Assert.Equal("test-build", file.GameBuild);
        Assert.Equal(elapsed, file.ElapsedSeconds);
        var v = Assert.Single(file.Vessels);
        Assert.Equal("probe", v.Id);
        Assert.Equal(elapsed, v.EpochSeconds);
        // Bit-exact JSON round-trip: System.Text.Json writes shortest round-trippable doubles.
        Assert.Equal(expected.Position.X, v.PositionEcl[0]);
        Assert.Equal(expected.Position.Y, v.PositionEcl[1]);
        Assert.Equal(expected.Position.Z, v.PositionEcl[2]);
        Assert.Equal(expected.Velocity.X, v.VelocityEcl[0]);
        Assert.Equal(expected.Velocity.Y, v.VelocityEcl[1]);
        Assert.Equal(expected.Velocity.Z, v.VelocityEcl[2]);
        Assert.Equal("Mercury", v.DominantAttractor);
        Assert.Contains("osculating around Mercury", v.OsculatingNote);
        Assert.Contains("e=0.210000", v.OsculatingNote);
    }

    [Fact]
    public void Frame_preferences_persist_without_any_exact_state_eligible_vessels()
    {
        FrameManager.ResetSessionStatics();
        try
        {
            Assert.Equal(1, FrameManager.ImportFrameSelections(new SidecarFile
            {
                FrameSelections =
                [
                    new SidecarFrameSelection
                    {
                        VesselId = "landed-probe",
                        Frame = new SidecarFrame
                        {
                            FrameKind = "Surface",
                            PrimaryId = "Mercury",
                        },
                    },
                ],
            }));

            SaveSidecar.Write(_rails, [], 1250.0, "test-build");

            var file = SaveSidecar.TryRead(1250.0);
            Assert.NotNull(file);
            Assert.Empty(file.Vessels);
            var selection = Assert.Single(file.FrameSelections);
            Assert.Equal("landed-probe", selection.VesselId);
            Assert.Equal("Surface", selection.Frame.FrameKind);
            Assert.Equal("Mercury", selection.Frame.PrimaryId);
            Assert.Null(selection.Frame.SecondaryId);
        }
        finally
        {
            FrameManager.ResetSessionStatics();
        }
    }

    [Fact]
    public void Pending_rendezvous_cleanup_marker_round_trips()
    {
        const double elapsed = 1200.0;
        var pending = new SidecarPendingRendezvous
        {
            VesselId = "chaser",
            Burns =
            [
                new SidecarSnapshotBurn { TimeSeconds = 1800, X = 1, Y = -2, Z = 3 },
                new SidecarSnapshotBurn { TimeSeconds = 2400, X = -4, Y = 5, Z = -6 },
            ],
        };
        SaveSidecar.Write(_rails, [MakeVessel("chaser", elapsed, 1.1)], elapsed,
            "test-build", pending);

        var restored = Assert.IsType<SidecarPendingRendezvous>(
            SaveSidecar.TryRead(elapsed)!.PendingRendezvous);
        Assert.Equal("chaser", restored.VesselId);
        Assert.Collection(restored.Burns,
            first =>
            {
                Assert.Equal(1800, first.TimeSeconds);
                Assert.Equal((1d, -2d, 3d), (first.X, first.Y, first.Z));
            },
            second =>
            {
                Assert.Equal(2400, second.TimeSeconds);
                Assert.Equal((-4d, 5d, -6d), (second.X, second.Y, second.Z));
            });
    }

    [Fact]
    public void Pairing_tolerance_is_strictly_under_one_second()
    {
        SaveSidecar.Write(_rails, [MakeVessel("probe", 5000.0, 1.1)], 5000.0, "b");
        Assert.NotNull(SaveSidecar.TryRead(5000.9));
        Assert.NotNull(SaveSidecar.TryRead(4999.1));
        Assert.Null(SaveSidecar.TryRead(5001.0));
        Assert.Null(SaveSidecar.TryRead(4999.0));
    }

    [Fact]
    public void Two_named_saves_inside_one_second_have_distinct_exact_sidecars()
    {
        SaveSidecar.Write(_rails, [MakeVessel("first", 1000.1, 1.1)], 1000.1,
            "first-build", saveIdentity: "save/first");
        SaveSidecar.Write(_rails, [MakeVessel("second", 1000.2, 1.1)], 1000.2,
            "second-build", saveIdentity: "save:second");

        Assert.Equal(2, Directory.GetFiles(_sidecarDir,
            "whiskerdynamics-save-*.json").Length);
        var first = Assert.IsType<SidecarFile>(
            SaveSidecar.TryRead(1000.1, "save/first"));
        var second = Assert.IsType<SidecarFile>(
            SaveSidecar.TryRead(1000.2, "save:second"));
        Assert.Equal("first", Assert.Single(first.Vessels).Id);
        Assert.Equal("second", Assert.Single(second.Vessels).Id);
        Assert.Equal("save/first", first.SaveIdentity);
        Assert.Equal("save:second", second.SaveIdentity);
    }

    [Fact]
    public void Rewriting_one_named_save_replaces_only_that_identity()
    {
        SaveSidecar.Write(_rails, [MakeVessel("old", 1100, 1.1)], 1100,
            "old-build", saveIdentity: "slot");
        SaveSidecar.Write(_rails, [MakeVessel("new", 1100, 1.1)], 1100,
            "new-build", saveIdentity: "slot");

        Assert.Single(Directory.GetFiles(_sidecarDir,
            "whiskerdynamics-save-*.json"));
        var file = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(1100, "slot"));
        Assert.Equal("new-build", file.GameBuild);
        Assert.Equal("new", Assert.Single(file.Vessels).Id);
    }

    [Fact]
    public void Named_generation_mismatch_fails_closed_at_the_same_identity_and_epoch()
    {
        SaveSidecar.Write(_rails, [], 1150, "old-generation",
            saveIdentity: "generation-slot", saveGenerationTicks: 101);
        SaveSidecar.Write(_rails, [], 1150, "anonymous-at-same-epoch");

        Assert.Null(SaveSidecar.TryRead(1150, "generation-slot",
            saveGenerationTicks: 102));
        var old = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(1150,
            "generation-slot", saveGenerationTicks: 101));
        Assert.Equal(101, old.SaveGenerationTicks);
        Assert.Equal("old-generation", old.GameBuild);
    }

    [Fact]
    public void Successful_named_overwrite_replaces_the_persisted_generation()
    {
        SaveSidecar.Write(_rails, [], 1175, "old-generation",
            saveIdentity: "overwrite-slot", saveGenerationTicks: 201);
        SaveSidecar.Write(_rails, [], 1175, "new-generation",
            saveIdentity: "overwrite-slot", saveGenerationTicks: 202);

        Assert.Single(Directory.GetFiles(_sidecarDir,
            "whiskerdynamics-save-*.json"));
        Assert.Null(SaveSidecar.TryRead(1175, "overwrite-slot",
            saveGenerationTicks: 201));
        var current = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(1175,
            "overwrite-slot", saveGenerationTicks: 202));
        Assert.Equal(202, current.SaveGenerationTicks);
        Assert.Equal("new-generation", current.GameBuild);
    }

    [Fact]
    public void Anonymous_writes_at_the_exact_same_epoch_do_not_collide()
    {
        SaveSidecar.Write(_rails, [], 1200.25, "first");
        SaveSidecar.Write(_rails, [], 1200.25, "second");

        string[] snapshots = SnapshotFiles();
        Assert.Equal(2, snapshots.Length);
        Assert.Equal(2, snapshots.Select(Path.GetFileName).Distinct().Count());
        Assert.All(snapshots, path =>
            Assert.StartsWith("whiskerdynamics-epoch-", Path.GetFileName(path)));
    }

    [Fact]
    public void Exact_identity_beats_a_closer_anonymous_epoch_candidate()
    {
        SaveSidecar.Write(_rails, [], 1300.4, "named",
            saveIdentity: "exact-slot");
        SaveSidecar.Write(_rails, [], 1300.0, "anonymous");

        var file = Assert.IsType<SidecarFile>(
            SaveSidecar.TryRead(1300.0, "exact-slot"));
        Assert.Equal("named", file.GameBuild);
        Assert.Equal("exact-slot", file.SaveIdentity);
    }

    [Fact]
    public void A_different_named_identity_is_never_selected_by_epoch()
    {
        SaveSidecar.Write(_rails, [], 1400.4, "alpha", saveIdentity: "alpha");
        SaveSidecar.Write(_rails, [], 1400.01, "closer-anonymous");

        Assert.Null(SaveSidecar.TryRead(1400, "beta"));
        Assert.Equal("closer-anonymous", SaveSidecar.TryRead(1400)!.GameBuild);
    }

    [Fact]
    public void Anonymous_fallback_selects_the_nearest_exact_epoch()
    {
        SaveSidecar.Write(_rails, [], 1500.2, "near");
        SaveSidecar.Write(_rails, [], 1500.8, "far");

        var file = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(1500.4));
        Assert.Equal("near", file.GameBuild);
        Assert.Equal(1500.2, file.ElapsedSeconds);
    }

    [Fact]
    public void Equal_distance_anonymous_tie_uses_newest_then_ordinal_name()
    {
        Directory.CreateDirectory(_sidecarDir);
        string a = Path.Combine(_sidecarDir, "whiskerdynamics-a.json");
        string b = Path.Combine(_sidecarDir, "whiskerdynamics-b.json");
        File.WriteAllText(a,
            """{"GameBuild":"a","ElapsedSeconds":1599.5}""");
        File.WriteAllText(b,
            """{"GameBuild":"b","ElapsedSeconds":1600.5}""");
        DateTime stamp = DateTime.UtcNow.AddMinutes(-1);
        File.SetLastWriteTimeUtc(a, stamp);
        File.SetLastWriteTimeUtc(b, stamp.AddSeconds(1));
        Assert.Equal("b", SaveSidecar.TryRead(1600)!.GameBuild);

        File.SetLastWriteTimeUtc(a, stamp);
        File.SetLastWriteTimeUtc(b, stamp);
        Assert.Equal("a", SaveSidecar.TryRead(1600)!.GameBuild);
    }

    [Fact]
    public void Pending_named_capture_is_available_only_to_its_exact_identity()
    {
        SaveSidecar.WriterBeforeIoForTest = () => Thread.Sleep(300);
        long sequence = 0;
        try
        {
            sequence = SaveSidecar.QueueWrite(_rails, [], 1700, "pending",
                saveIdentity: "pending-slot", saveGenerationTicks: 301);
            Assert.Equal("pending",
                SaveSidecar.TryRead(1700, "pending-slot",
                    saveGenerationTicks: 301)!.GameBuild);
            Assert.Null(SaveSidecar.TryRead(1700, "pending-slot",
                saveGenerationTicks: 302));
            Assert.Null(SaveSidecar.TryRead(1700, "other-slot"));
        }
        finally { SaveSidecar.WriterBeforeIoForTest = null; }
        Assert.True(SaveSidecar.WaitForDurability(sequence, 5000));
    }

    [Fact]
    public void Hyperbolic_state_gets_hyperbolic_elements_note_with_exact_state_intact()
    {
        // Hyperbolic osculating elements must not prevent exact-state persistence.
        double elapsed = 1000.0;
        SaveSidecar.Write(_rails, [MakeVessel("escaper", elapsed, 1.5)], elapsed, "b");
        var v = Assert.Single(SaveSidecar.TryRead(elapsed)!.Vessels);
        Assert.Contains("osculating around Mercury: a=-", v.OsculatingNote);
        Assert.All(v.PositionEcl, x => Assert.True(double.IsFinite(x)));
        Assert.All(v.VelocityEcl, x => Assert.True(double.IsFinite(x)));
    }

    [Fact]
    public void Near_circular_state_gets_contained_note()
    {
        double elapsed = 1000.0;
        SaveSidecar.Write(_rails, [MakeVessel("circ", elapsed, 1.0)], elapsed, "b");
        var v = Assert.Single(SaveSidecar.TryRead(elapsed)!.Vessels);
        Assert.Contains("no elliptic osculating elements around Mercury", v.OsculatingNote);
        Assert.Contains("Near-circular", v.OsculatingNote);
    }

    [Fact]
    public void Note_uses_actual_orbit_parent_not_the_dominant_attractor_label()
    {
        // The direct-field attractor is Sol, but the osculating state remains relative
        // to the vessel's actual orbit parent, Mercury.
        double elapsed = 1000.0;
        var vessel = MakeVessel("tli", elapsed, 1.1, parentId: "Mercury", r: 1.0e9);
        SaveSidecar.Write(_rails, [vessel], elapsed, "b");
        var v = Assert.Single(SaveSidecar.TryRead(elapsed)!.Vessels);
        Assert.Equal("Sol", v.DominantAttractor);
        Assert.Contains("osculating around Mercury", v.OsculatingNote);
    }

    [Fact]
    public void Unknown_parent_falls_back_to_the_dominant_label_for_the_note()
    {
        double elapsed = 1000.0;
        var vessel = MakeVessel("fresh", elapsed, 1.1, parentId: null);
        SaveSidecar.Write(_rails, [vessel], elapsed, "b");
        var v = Assert.Single(SaveSidecar.TryRead(elapsed)!.Vessels);
        Assert.Contains("osculating around Mercury", v.OsculatingNote); // dominant here
    }

    [Fact]
    public void One_bad_vessel_does_not_lose_the_rest()
    {
        // Seeded in the future: StateAt(elapsed) throws (query before start) — the
        // per-vessel containment must keep the good vessel and the file itself.
        double elapsed = 1000.0;
        var bad = MakeVessel("bad", 2000.0, 1.1);
        var good = MakeVessel("good", elapsed, 1.1);
        SaveSidecar.Write(_rails, [bad, good], elapsed, "b");
        var v = Assert.Single(SaveSidecar.TryRead(elapsed)!.Vessels);
        Assert.Equal("good", v.Id);
    }

    [Fact]
    public void Missing_dir_returns_null()
    {
        Assert.Null(SaveSidecar.TryRead(1.0));
    }

    [Fact]
    public void Corrupt_snapshots_and_insane_entries_are_skipped()
    {
        Directory.CreateDirectory(_sidecarDir);
        File.WriteAllText(Path.Combine(_sidecarDir, "whiskerdynamics-777.json"), "{ not json");
        Assert.Null(SaveSidecar.TryRead(777.0));
        // Valid JSON at the right elapsed but a hand-broken vessel entry (2-element
        // position): the entry is dropped, never handed to the restore path.
        File.WriteAllText(Path.Combine(_sidecarDir, "whiskerdynamics-888.json"),
            """{"GameBuild":"x","ElapsedSeconds":888,"Vessels":[{"Id":"z","EpochSeconds":888,"PositionEcl":[1,2],"VelocityEcl":[1,2,3],"DominantAttractor":"","OsculatingNote":""}]}""");
        var file = SaveSidecar.TryRead(888.0);
        Assert.NotNull(file);
        Assert.Empty(file.Vessels);
        // NaN state: same fate.
        File.WriteAllText(Path.Combine(_sidecarDir, "whiskerdynamics-999.json"),
            """{"GameBuild":"x","ElapsedSeconds":999,"Vessels":[{"Id":"z","EpochSeconds":999,"PositionEcl":[1,2,"NaN"],"VelocityEcl":[1,2,3],"DominantAttractor":"","OsculatingNote":""}]}""");
        var nan = SaveSidecar.TryRead(999.0);
        Assert.NotNull(nan);
        Assert.Empty(nan.Vessels);
    }

    [Fact]
    public void Null_vessel_collections_entries_and_ids_are_sanitized()
    {
        Directory.CreateDirectory(_sidecarDir);
        File.WriteAllText(Path.Combine(_sidecarDir, "whiskerdynamics-null-list.json"),
            """{"GameBuild":"null-list","ElapsedSeconds":1001,"Vessels":null,"Plans":null}""");
        var nullList = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(1001));
        Assert.Empty(nullList.Vessels);
        Assert.Empty(nullList.Plans);

        File.WriteAllText(Path.Combine(_sidecarDir, "whiskerdynamics-null-items.json"),
            """{"GameBuild":"null-items","ElapsedSeconds":1002,"Vessels":[null,{"Id":null,"EpochSeconds":1002,"PositionEcl":[1,2,3],"VelocityEcl":[4,5,6]},{"Id":"good","EpochSeconds":1002,"PositionEcl":[1,2,3],"VelocityEcl":[4,5,6]}]}""");
        var mixed = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(1002));
        Assert.Equal("good", Assert.Single(mixed.Vessels).Id);
    }

    [Fact]
    public void Serialized_invalid_plan_records_degrade_to_plain_stock_burn_metadata()
    {
        const double elapsed = 1002.5;
        FlightPlans.ResetSessionStatics();
        try
        {
            Directory.CreateDirectory(_sidecarDir);
            File.WriteAllText(Path.Combine(_sidecarDir,
                    "whiskerdynamics-invalid-plans.json"),
                """
                {
                  "GameBuild": "invalid-plans",
                  "ElapsedSeconds": 1002.5,
                  "Plans": [
                    null,
                    {
                      "VesselId": "",
                      "Plan": { "CreatedAtSeconds": 0, "LengthSeconds": 86400 }
                    },
                    {
                      "VesselId": "invalid-window",
                      "Plan": { "CreatedAtSeconds": 0, "LengthSeconds": 0 }
                    },
                    {
                      "VesselId": "duplicate",
                      "Plan": { "CreatedAtSeconds": 0, "LengthSeconds": 86400 }
                    },
                    {
                      "VesselId": "duplicate",
                      "Plan": { "CreatedAtSeconds": 10, "LengthSeconds": 86400 }
                    },
                    {
                      "VesselId": "plain-stock",
                      "Plan": {
                        "CreatedAtSeconds": 0,
                        "LengthSeconds": 86400,
                        "Burns": [
                          {
                            "TimeSeconds": 100,
                            "FrameKind": "Inertial",
                            "PrimaryId": "Earth",
                            "Basis": "unsupported",
                            "X": 1,
                            "Y": 2,
                            "Z": 3
                          }
                        ]
                      }
                    }
                  ]
                }
                """);

            var file = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(elapsed));
            Assert.Equal(1, FlightPlans.ImportSidecar(file));
            Assert.Null(FlightPlans.TryGet("invalid-window"));
            Assert.Null(FlightPlans.TryGet("duplicate"));
            Assert.Empty(Assert.IsType<FlightPlanModel>(
                FlightPlans.TryGet("plain-stock")).Meta);
        }
        finally
        {
            FlightPlans.ResetSessionStatics();
        }
    }

    [Fact]
    public void Restore_delta_is_measured_at_a_common_epoch_not_across_the_skew()
    {
        // Compare at a common epoch: heliocentric motion during an allowed seed-time
        // skew can exceed the restore-distance bound for the same trajectory.
        double epoch = 1000.0, seedTime = 1020.0; // 20 s skew
        var mercury = _rails.GetAbsolute("Mercury", epoch);
        double r = 2.74e6;
        double vCirc = Math.Sqrt(_rails.MuOf("Mercury") / r);
        var exactAtEpoch = new StateVector(
            mercury.Position + new Vector3d(r, 0, 0),
            mercury.Velocity + new Vector3d(0, 1.1 * vCirc, 0));
        var options = new IntegratorOptions { RelTol = 1e-11 };

        // Seed the same trajectory 20 seconds later.
        StateVector sameTrajectoryAtSeed;
        lock (_rails.Gate)
            sameTrajectoryAtSeed = new TrajectoryPredictor(_rails.VesselGravity, exactAtEpoch, epoch, options)
                .StateAt(seedTime);
        var tracked = new TrackedVessel { Id = "reentry", Rails = _rails, Options = options };
        tracked.ReseedAbsolute(sameTrajectoryAtSeed, seedTime);

        double raw = (exactAtEpoch.Position - sameTrajectoryAtSeed.Position).Length();
        Assert.True(raw > 1e5, $"test geometry must exceed the sanity bound (raw={raw:E2} m)");

        // At the common epoch the same trajectory agrees to integration precision.
        double common = SaveSidecar.RestoreDeltaMeters(_rails, tracked, exactAtEpoch, epoch);
        Assert.True(common < 1.0, $"common-epoch delta must be tiny (got {common:E2} m)");
    }

    [Fact]
    public void Restore_epoch_rewinds_Ksa_save_time_rounding_skew()
    {
        double seedTime = 2.0363;
        double sidecarTime = 2.0363418920569565;
        var state = new StateVector(new Vector3d(100, 200, 300),
            new Vector3d(-20_000, 10_000, 100));

        Assert.True(SaveSidecar.TryNormalizeRestoreEpoch(state, sidecarTime, seedTime,
            out var normalized, out double epoch));
        Assert.Equal(seedTime, epoch);
        Assert.Equal(state.Velocity, normalized.Velocity);
        Assert.Equal(state.Position - state.Velocity * (sidecarTime - seedTime),
            normalized.Position);
    }

    [Fact]
    public void Restore_epoch_rejects_materially_future_sidecar()
    {
        var state = new StateVector(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        Assert.False(SaveSidecar.TryNormalizeRestoreEpoch(state, 10.01, 10.0,
            out _, out _));
    }

    [Fact]
    public void Flight_plan_metadata_round_trips_through_the_sidecar()
    {
        double elapsed = 1000.0;
        string id = "planship";
        try
        {
            var plan = FlightPlans.Create(id, nowSeconds: 900.0, lengthSeconds: 5 * 86400.0);
            plan.SetMeta(new FlightPlanBurnMeta
            {
                TimeSeconds = 4000.0,
                Frame = new FrameSpec(FrameKind.TwoBodyFixed, "Sol", "Mercury"),
                Authored = new Vector3d(2.5, -1.25, 0.5),
                StampMs = 0,
            });
            SaveSidecar.Write(_rails, [MakeVessel(id, elapsed, 1.1)], elapsed, "b");

            var file = SaveSidecar.TryRead(elapsed)!;
            Assert.Single(file.Vessels);
            var persisted = Assert.Single(file.Plans);
            Assert.Equal(id, persisted.VesselId);
            Assert.NotNull(persisted.Plan);
            Assert.Equal(900.0, persisted.Plan.CreatedAtSeconds);
            Assert.Equal(5 * 86400.0, persisted.Plan.LengthSeconds);
            var burn = Assert.Single(persisted.Plan.Burns);
            Assert.Equal(4000.0, burn.TimeSeconds);   // exact: STJ round-trips doubles
            Assert.Equal("TwoBodyFixed", burn.FrameKind);
            Assert.Equal("Sol", burn.PrimaryId);
            Assert.Equal("Mercury", burn.SecondaryId);
            Assert.Equal((2.5, -1.25, 0.5), (burn.X, burn.Y, burn.Z));

            // Load path: the sweep cleared the store; ImportSidecar rebuilds it.
            FlightPlans.ResetSessionStatics();
            Assert.Equal(1, FlightPlans.ImportSidecar(SaveSidecar.TryRead(elapsed)!));
            var restored = FlightPlans.TryGet(id);
            Assert.NotNull(restored);
            var meta = restored.TryGetMetaAt(4000.0);
            Assert.NotNull(meta);
            Assert.Equal(new Vector3d(2.5, -1.25, 0.5), meta.Authored);
        }
        finally
        {
            FlightPlans.Remove(id);
        }
    }

    [Fact]
    public void Ineligible_landed_and_reseed_pending_vessels_keep_plan_metadata()
    {
        const double elapsed = 5000.0;
        string landedId = "landed-plan-" + Guid.NewGuid().ToString("N");
        string pendingId = "pending-plan-" + Guid.NewGuid().ToString("N");
        FlightPlans.ResetSessionStatics();
        try
        {
            var landed = MakeVessel(landedId, elapsed: 1000.0, speedFactor: 1.1);
            landed.LastRefreshTime = 1000.0;
            landed.LastStagedTime = 1000.0;
            var pending = MakeVessel(pendingId, elapsed, 1.1);
            pending.MarkReseedPending();

            var landedPlan = FlightPlans.Create(landedId, 900.0, 3 * 86400.0);
            landedPlan.SetMeta(new FlightPlanBurnMeta
            {
                TimeSeconds = 2000.0,
                Frame = new FrameSpec(FrameKind.Surface, "Mercury", null),
                Authored = new Vector3d(1, 2, 3),
                StampMs = 0,
            });
            FlightPlans.Create(pendingId, 4500.0, 4 * 86400.0)
                .PropulsionSource = PropulsionSource.RcsForward;

            TrackedVessel[] eligible = [landed, pending];
            eligible = eligible.Where(v => VesselLifecycle.SidecarEligible(
                v.ReseedPending, v.Predictor is not null,
                v.SeedTime, Math.Max(v.LastRefreshTime, v.LastStagedTime),
                elapsed, refreshPeriodSeconds: 600.0)).ToArray();
            Assert.Empty(eligible);

            SaveSidecar.Write(_rails, eligible, elapsed, "b");
            var file = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(elapsed));
            Assert.Empty(file.Vessels);
            Assert.Equal([landedId, pendingId],
                file.Plans.Select(record => record.VesselId).ToArray());

            FlightPlans.ResetSessionStatics();
            Assert.Equal(2, FlightPlans.ImportSidecar(file));
            var restoredLanded = Assert.IsType<FlightPlanModel>(FlightPlans.TryGet(landedId));
            Assert.Equal(3 * 86400.0, restoredLanded.LengthSeconds);
            Assert.Equal(new Vector3d(1, 2, 3),
                Assert.IsType<FlightPlanBurnMeta>(
                    restoredLanded.TryGetMetaAt(2000.0)).Authored);
            var restoredPending = Assert.IsType<FlightPlanModel>(FlightPlans.TryGet(pendingId));
            Assert.Equal(4 * 86400.0, restoredPending.LengthSeconds);
            Assert.Equal(PropulsionSource.RcsForward, restoredPending.PropulsionSource);
        }
        finally
        {
            FlightPlans.ResetSessionStatics();
        }
    }

    [Fact]
    public void Snapshot_files_are_pruned_to_a_bound()
    {
        for (int k = 1; k <= SaveSidecar.SnapshotKeep + 10; k++)
            SaveSidecar.Write(_rails, [], k * 10.0, "b");
        int snapshots = Directory.GetFiles(_sidecarDir, "whiskerdynamics-*.json")
            .Count(p => !p.EndsWith("whiskerdynamics-latest.json", StringComparison.OrdinalIgnoreCase));
        Assert.Equal(SaveSidecar.SnapshotKeep, snapshots);
        // The newest snapshot always survives pruning.
        Assert.NotNull(SaveSidecar.TryRead((SaveSidecar.SnapshotKeep + 10) * 10.0));
    }

    [Fact]
    public void Anonymous_retention_never_deletes_named_save_sidecars()
    {
        int namedCount = SaveSidecar.SnapshotKeep + 5;
        for (int k = 0; k < namedCount; k++)
            SaveSidecar.Write(_rails, [], 10_000 + k, "named-" + k,
                saveIdentity: "slot-" + k);
        for (int k = 0; k < SaveSidecar.SnapshotKeep + 10; k++)
            SaveSidecar.Write(_rails, [], 20_000 + k, "anonymous-" + k);

        Assert.Equal(namedCount, Directory.GetFiles(_sidecarDir,
            "whiskerdynamics-save-*.json").Length);
        Assert.Equal(SaveSidecar.SnapshotKeep, Directory.GetFiles(_sidecarDir,
            "whiskerdynamics-epoch-*.json").Length);
        for (int k = 0; k < namedCount; k++)
            Assert.Equal("named-" + k,
                SaveSidecar.TryRead(10_000 + k, "slot-" + k)!.GameBuild);
    }

    [Fact]
    public void Queued_write_flushes_atomically_and_is_immediately_restorable()
    {
        const double elapsed = 2468.25;
        long sequence = SaveSidecar.QueueWrite(
            _rails, [MakeVessel("queued", elapsed, 1.1)], elapsed, "b");
        var file = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(elapsed));
        Assert.Equal("queued", Assert.Single(file.Vessels).Id);
        Assert.True(SaveSidecar.WaitForDurability(sequence, 5000));
        Assert.Empty(AtomicTempFiles());
    }

    [Fact]
    public void Queued_write_captures_plan_metadata_without_an_exact_vessel_state()
    {
        const double elapsed = 2470.0;
        string id = "queued-plan-" + Guid.NewGuid().ToString("N");
        FlightPlans.ResetSessionStatics();
        try
        {
            var plan = FlightPlans.Create(id, 2400.0, 2 * 86400.0);
            plan.SetMeta(new FlightPlanBurnMeta
            {
                TimeSeconds = 3000.0,
                Frame = new FrameSpec(FrameKind.Surface, "Mercury", null),
                Authored = new Vector3d(4, 5, 6),
                StampMs = 0,
            });

            long sequence = SaveSidecar.QueueWrite(
                _rails, [], elapsed, "queued-plan");
            FlightPlans.ResetSessionStatics();

            Assert.True(SaveSidecar.WaitForDurability(sequence, 5000));
            var file = Assert.IsType<SidecarFile>(SaveSidecar.TryRead(elapsed));
            Assert.Empty(file.Vessels);
            Assert.Equal(id, Assert.Single(file.Plans).VesselId);
            Assert.Equal(1, FlightPlans.ImportSidecar(file));
            var restored = Assert.IsType<FlightPlanModel>(FlightPlans.TryGet(id));
            Assert.Equal(2 * 86400.0, restored.LengthSeconds);
            Assert.Equal(new Vector3d(4, 5, 6),
                Assert.IsType<FlightPlanBurnMeta>(
                    restored.TryGetMetaAt(3000.0)).Authored);
        }
        finally
        {
            FlightPlans.ResetSessionStatics();
        }
    }

    [Fact]
    public void Named_commit_failure_and_flush_retry_keep_the_previous_file_and_pending_capture()
    {
        const double elapsed = 2475.0;
        const string identity = "atomic-retry-slot";
        SaveSidecar.Write(_rails, [], elapsed, "previous",
            saveIdentity: identity, saveGenerationTicks: 1);
        string canonical = SaveSidecar.IdentitySnapshotPath(_sidecarDir, identity);
        byte[] previousBytes = File.ReadAllBytes(canonical);

        int canonicalCommitAttempts = 0;
        var hooks = new AtomicTextFileHooks(Commit: (_, destination, commit) =>
        {
            if (SamePath(destination, canonical)
                && Interlocked.Increment(ref canonicalCommitAttempts) <= 2)
                throw new IOException("injected canonical commit failure");
            commit();
        });

        long sequence = SaveSidecar.QueueWriteForTest(
            _rails, [], elapsed, "replacement", hooks,
            saveIdentity: identity, saveGenerationTicks: 2);

        // Flush waits for the worker's first failed attempt, then gives that exact
        // request one retry. Both commit failures happen after a durable temp write
        // but before the canonical destination is touched.
        Assert.False(SaveSidecar.FlushPendingWrites(5000));
        Assert.Equal(2, Volatile.Read(ref canonicalCommitAttempts));
        Assert.False(SaveSidecar.WaitForDurability(sequence, 0));
        Assert.Equal(previousBytes, File.ReadAllBytes(canonical));
        Assert.Empty(AtomicTempFiles());

        // A failed disk attempt must not evict the newer capture from memory merely
        // because the previous canonical file remains valid on disk.
        var pending = Assert.IsType<SidecarFile>(
            SaveSidecar.TryRead(elapsed, identity, saveGenerationTicks: 2));
        Assert.Equal("replacement", pending.GameBuild);

        // A later flush retries the still-pending request. The local schedule now
        // allows the real commit, after which both durability and disk advance.
        Assert.True(SaveSidecar.FlushPendingWrites(5000));
        Assert.True(SaveSidecar.WaitForDurability(sequence, 0));
        Assert.Equal(3, Volatile.Read(ref canonicalCommitAttempts));
        Assert.False(previousBytes.SequenceEqual(File.ReadAllBytes(canonical)));
        Assert.Equal("replacement",
            SaveSidecar.TryRead(elapsed, identity, saveGenerationTicks: 2)!.GameBuild);
        Assert.Empty(AtomicTempFiles());
    }

    [Fact]
    public void Latest_commit_failure_leaves_canonical_first_but_withholds_pair_durability()
    {
        const double elapsed = 2480.0;
        const string identity = "atomic-latest-slot";
        SaveSidecar.Write(_rails, [], elapsed, "previous",
            saveIdentity: identity, saveGenerationTicks: 10);
        string canonical = SaveSidecar.IdentitySnapshotPath(_sidecarDir, identity);
        string latest = Path.Combine(_sidecarDir, "whiskerdynamics-latest.json");
        byte[] previousCanonical = File.ReadAllBytes(canonical);
        byte[] previousLatest = File.ReadAllBytes(latest);

        int latestCommitAttempts = 0;
        var hooks = new AtomicTextFileHooks(Commit: (_, destination, commit) =>
        {
            if (SamePath(destination, latest)
                && Interlocked.Increment(ref latestCommitAttempts) <= 2)
                throw new IOException("injected latest commit failure");
            commit();
        });

        long sequence = SaveSidecar.QueueWriteForTest(
            _rails, [], elapsed, "replacement", hooks,
            saveIdentity: identity, saveGenerationTicks: 11);

        Assert.False(SaveSidecar.FlushPendingWrites(5000));
        Assert.Equal(2, Volatile.Read(ref latestCommitAttempts));
        Assert.False(SaveSidecar.WaitForDurability(sequence, 0));

        // Each attempt commits the authoritative canonical file before reaching the
        // injected latest failure. Latest remains the previous complete convenience
        // copy, and the two-file request is deliberately not reported durable.
        Assert.False(previousCanonical.SequenceEqual(File.ReadAllBytes(canonical)));
        Assert.Equal(previousLatest, File.ReadAllBytes(latest));
        var canonicalOnDisk = ReadSidecarFile(canonical);
        Assert.Equal("replacement", canonicalOnDisk.GameBuild);
        Assert.Equal(11, canonicalOnDisk.SaveGenerationTicks);
        Assert.Equal("replacement",
            SaveSidecar.TryRead(elapsed, identity, saveGenerationTicks: 11)!.GameBuild);
        Assert.Empty(AtomicTempFiles());

        Assert.True(SaveSidecar.FlushPendingWrites(5000));
        Assert.True(SaveSidecar.WaitForDurability(sequence, 0));
        Assert.Equal(3, Volatile.Read(ref latestCommitAttempts));
        Assert.Equal(File.ReadAllBytes(canonical), File.ReadAllBytes(latest));
        Assert.Empty(AtomicTempFiles());
    }

    [Fact]
    public async Task Sidecar_io_gate_keeps_queued_and_synchronous_two_file_transactions_serial()
    {
        const double elapsed = 2490.0;
        const string identity = "atomic-sidecar-gate-slot";
        SaveSidecar.Write(_rails, [], elapsed, "previous",
            saveIdentity: identity, saveGenerationTicks: 20);
        string canonical = SaveSidecar.IdentitySnapshotPath(_sidecarDir, identity);
        string latest = Path.Combine(_sidecarDir, "whiskerdynamics-latest.json");

        using var queuedCanonicalCommitted = new ManualResetEventSlim();
        using var releaseQueued = new ManualResetEventSlim();
        using var synchronousStarted = new ManualResetEventSlim();
        Task? synchronousWrite = null;
        long queuedSequence = 0;
        var hooks = new AtomicTextFileHooks(Commit: (_, destination, commit) =>
        {
            commit();
            if (!SamePath(destination, canonical)) return;
            queuedCanonicalCommitted.Set();
            if (!releaseQueued.Wait(5000))
                throw new TimeoutException("timed out waiting to release queued sidecar write");
        });

        try
        {
            queuedSequence = SaveSidecar.QueueWriteForTest(
                _rails, [], elapsed, "queued-A", hooks,
                saveIdentity: identity, saveGenerationTicks: 21);
            Assert.True(queuedCanonicalCommitted.Wait(5000));

            // A has atomically committed its canonical file and is paused inside the
            // same SidecarIoGate hold, before it can publish latest.
            Assert.Equal("queued-A", ReadSidecarFile(canonical).GameBuild);
            Assert.Equal("previous", ReadSidecarFile(latest).GameBuild);

            synchronousWrite = Task.Run(() =>
            {
                synchronousStarted.Set();
                SaveSidecar.Write(_rails, [], elapsed, "synchronous-B",
                    saveIdentity: identity, saveGenerationTicks: 22);
            });
            Assert.True(synchronousStarted.Wait(5000));

            // B has begun its synchronous call but cannot complete or publish either
            // destination while A remains paused inside the transaction gate.
            await Assert.ThrowsAsync<TimeoutException>(() =>
                synchronousWrite.WaitAsync(TimeSpan.FromMilliseconds(250)));
            Assert.Equal("queued-A", ReadSidecarFile(canonical).GameBuild);
            Assert.Equal("previous", ReadSidecarFile(latest).GameBuild);
            Assert.Empty(AtomicTempFiles());
        }
        finally
        {
            releaseQueued.Set();
            if (synchronousWrite is not null)
            {
                try { await synchronousWrite.WaitAsync(TimeSpan.FromSeconds(5)); }
                catch { /* propagated below on the successful assertion path */ }
            }
        }

        Assert.NotNull(synchronousWrite);
        await synchronousWrite;
        Assert.True(SaveSidecar.WaitForDurability(queuedSequence, 5000));

        // A finishes its canonical/latest pair before releasing the gate; B then
        // publishes its own complete pair. The final disk state cannot be mixed.
        var canonicalFinal = ReadSidecarFile(canonical);
        var latestFinal = ReadSidecarFile(latest);
        Assert.Equal("synchronous-B", canonicalFinal.GameBuild);
        Assert.Equal(22, canonicalFinal.SaveGenerationTicks);
        Assert.Equal("synchronous-B", latestFinal.GameBuild);
        Assert.Equal(22, latestFinal.SaveGenerationTicks);
        Assert.Equal(File.ReadAllBytes(canonical), File.ReadAllBytes(latest));
        Assert.Empty(AtomicTempFiles());
    }

    [Fact]
    public void Queued_write_captures_destination_directory()
    {
        string captured = _sidecarDir;
        string later = Path.Combine(_dir, "later-sidecar-dir");
        SaveSidecar.QueueWrite(_rails, [], 3579.0, "b");
        SaveSidecar.DirOverride = later;
        try
        {
            Assert.True(SaveSidecar.FlushPendingWrites(5000));
            Assert.Single(Directory.GetFiles(captured,
                "whiskerdynamics-epoch-*.json"));
            Assert.False(Directory.Exists(later));
        }
        finally { SaveSidecar.DirOverride = captured; }
    }

    [Fact]
    public void Saturation_supersession_counts_as_completion_for_flush()
    {
        var pending = new SidecarPendingRendezvous
        {
            VesselId = "bulk",
            Burns = Enumerable.Range(0, 10_000)
                .Select(i => new SidecarSnapshotBurn { TimeSeconds = i, X = i }).ToList(),
        };
        long before = SaveSidecar.SupersededWriteCount;
        for (int i = 0; i < 32; i++)
            SaveSidecar.QueueWrite(_rails, [], 6000 + i, "b", pending);

        Assert.True(SaveSidecar.FlushPendingWrites(15_000));
        Assert.True(SaveSidecar.SupersededWriteCount > before);
    }

    [Fact]
    public void Flush_retries_every_distinct_named_identity_after_saturation()
    {
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        int hookCalls = 0;
        SaveSidecar.WriterBeforeIoForTest = () =>
        {
            if (Interlocked.Increment(ref hookCalls) != 1) return;
            entered.Set();
            release.Wait(5000);
        };
        long supersededBefore = SaveSidecar.SupersededWriteCount;
        Task? releaser = null;
        try
        {
            SaveSidecar.QueueWrite(_rails, [], 8000, "blocker",
                saveIdentity: "blocker-slot");
            Assert.True(entered.Wait(5000));
            long first = SaveSidecar.QueueWrite(_rails, [], 8100, "first",
                saveIdentity: "first-slot");
            for (int k = 0; k < 7; k++)
                SaveSidecar.QueueWrite(_rails, [], 8200, "filler-" + k,
                    saveIdentity: "filler-slot-" + k);
            long second = SaveSidecar.QueueWrite(_rails, [], 8300, "second",
                saveIdentity: "second-slot");
            Assert.True(SaveSidecar.SupersededWriteCount > supersededBefore);

            releaser = Task.Run(() =>
            {
                Thread.Sleep(100);
                release.Set();
            });
            Assert.True(SaveSidecar.FlushPendingWrites(15_000));
            Assert.True(SaveSidecar.WaitForDurability(first, 0));
            Assert.True(SaveSidecar.WaitForDurability(second, 0));
        }
        finally
        {
            release.Set();
            releaser?.Wait(5000);
            SaveSidecar.WriterBeforeIoForTest = null;
        }

        Assert.Equal("first", SaveSidecar.TryRead(8100, "first-slot")!.GameBuild);
        Assert.Equal("second", SaveSidecar.TryRead(8300, "second-slot")!.GameBuild);
    }

    [Fact]
    public void Persistent_distinct_named_identity_failure_storm_is_bounded_and_reported()
    {
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        int unavailable = 1;
        int writerCalls = 0;
        SaveSidecar.WriterBeforeIoForTest = () =>
        {
            if (Interlocked.Increment(ref writerCalls) != 1) return;
            entered.Set();
            release.Wait(5000);
        };
        var hooks = new AtomicTextFileHooks(Commit: (_, _, commit) =>
        {
            if (Volatile.Read(ref unavailable) != 0)
                throw new IOException("SIMULATED persistent sidecar storage failure");
            commit();
        });
        long evictionsBefore = SaveSidecar.NamedCaptureEvictionCount;
        int attemptedBefore = SaveSidecar.AttemptedWriteCount;
        int stormSize = SaveSidecar.MaxPendingNamedIdentities + 12;
        try
        {
            SaveSidecar.QueueWriteForTest(_rails, [], 9000, "storm-0", hooks,
                saveIdentity: "storm-slot-0");
            Assert.True(entered.Wait(5000));
            for (int k = 1; k < stormSize; k++)
                SaveSidecar.QueueWriteForTest(_rails, [], 9000 + k,
                    "storm-" + k, hooks, saveIdentity: "storm-slot-" + k);

            Assert.Equal(SaveSidecar.MaxPendingNamedIdentities,
                SaveSidecar.PendingNamedIdentityCount);
            Assert.Equal(stormSize - SaveSidecar.MaxPendingNamedIdentities,
                SaveSidecar.NamedCaptureEvictionCount - evictionsBefore);

            release.Set();
            Assert.True(SpinWait.SpinUntil(
                () => SaveSidecar.PendingAttemptedNamedIdentityCount
                    == SaveSidecar.MaxPendingNamedIdentities, 15_000),
                "retained named captures did not all reach the injected failure");
            Assert.Equal(SaveSidecar.MaxPendingNamedIdentities,
                SaveSidecar.AttemptedWriteCount - attemptedBefore);

            // Recovery makes every retained identity durable. The first flush still
            // returns false to report that the bounded cache evicted older identities;
            // the next flush reports the now-clean current pending set normally.
            Volatile.Write(ref unavailable, 0);
            Assert.False(SaveSidecar.FlushPendingWrites(30_000));
            Assert.Equal(0, SaveSidecar.PendingNamedIdentityCount);
            Assert.True(SaveSidecar.FlushPendingWrites(5000));
        }
        finally
        {
            Volatile.Write(ref unavailable, 0);
            release.Set();
            SaveSidecar.WriterBeforeIoForTest = null;
            SaveSidecar.FlushPendingWrites(30_000);
            SaveSidecar.FlushPendingWrites(30_000);
        }
    }

    [Fact]
    public void Newer_durable_named_capture_retires_saturated_older_memory()
    {
        using var entered = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        SaveSidecar.WriterBeforeIoForTest = () =>
        {
            entered.Set();
            release.Wait(5000);
        };
        long newest = 0;
        long supersededBefore = SaveSidecar.SupersededWriteCount;
        try
        {
            SaveSidecar.QueueWrite(_rails, [], 6800, "version-0",
                saveIdentity: "saturated-slot");
            Assert.True(entered.Wait(5000));
            for (int i = 1; i <= 12; i++)
                newest = SaveSidecar.QueueWrite(_rails, [], 6800,
                    "version-" + i, saveIdentity: "saturated-slot");
            Assert.True(SaveSidecar.SupersededWriteCount > supersededBefore);
            release.Set();
            Assert.True(SaveSidecar.WaitForDurability(newest, 15_000));
            Assert.True(SaveSidecar.FlushPendingWrites(15_000));
        }
        finally
        {
            release.Set();
            SaveSidecar.WriterBeforeIoForTest = null;
        }

        // Both memory selection and the canonical file must remain on the newest
        // generation even after FlushPendingWrites examines superseded requests.
        Assert.Equal("version-12",
            SaveSidecar.TryRead(6800, "saturated-slot")!.GameBuild);
    }

    [Fact]
    public void Named_durability_is_not_published_until_retirement_completes()
    {
        using var retired = new ManualResetEventSlim();
        using var release = new ManualResetEventSlim();
        SaveSidecar.WriterAfterNamedRetirementForTest = () =>
        {
            retired.Set();
            release.Wait(5000);
        };
        long sequence = 0;
        try
        {
            sequence = SaveSidecar.QueueWrite(_rails, [], 6900, "ordered",
                saveIdentity: "retirement-order");
            Assert.True(retired.Wait(5000));
            Assert.False(SaveSidecar.WaitForDurability(sequence, 0));
        }
        finally
        {
            release.Set();
            SaveSidecar.WriterAfterNamedRetirementForTest = null;
        }
        Assert.True(SaveSidecar.WaitForDurability(sequence, 5000));
    }

    [Fact]
    public void Failed_disk_write_is_not_reported_as_durable_and_memory_restore_survives()
    {
        string blocker = Path.Combine(_dir, "not-a-directory");
        File.WriteAllText(blocker, "x");
        string failedDir = Path.Combine(blocker, "sidecar");
        SaveSidecar.DirOverride = failedDir;
        try
        {
            long sequence = SaveSidecar.QueueWrite(_rails, [], 7000, "b");
            Assert.False(SaveSidecar.WaitForDurability(sequence, 100));
            Assert.NotNull(SaveSidecar.TryRead(7000));
            Assert.False(SaveSidecar.FlushPendingWrites(100));
        }
        finally { SaveSidecar.DirOverride = _sidecarDir; }
    }

    [Fact]
    public void Flush_deadline_does_not_run_blocking_io_on_the_caller()
    {
        SaveSidecar.WriterBeforeIoForTest = () => Thread.Sleep(500);
        long sequence = 0;
        try
        {
            sequence = SaveSidecar.QueueWrite(_rails, [], 7100, "b");
            var sw = System.Diagnostics.Stopwatch.StartNew();
            Assert.False(SaveSidecar.FlushPendingWrites(50));
            sw.Stop();
            Assert.True(sw.ElapsedMilliseconds < 300,
                $"bounded flush blocked for {sw.ElapsedMilliseconds} ms");
        }
        finally { SaveSidecar.WriterBeforeIoForTest = null; }

        Assert.True(SaveSidecar.WaitForDurability(sequence, 5000));
    }
}

/// <summary>Pins the registration step: the save patches must stay in the
/// gameplay patch list (applied inside the guarded try only after ALL gameplay targets
/// validate). Reflection-only on mod types — no KSA type is loaded offline.</summary>
public class SaveRegistrationTests
{
    [Fact]
    public void Save_patches_are_registered_as_gameplay_patches()
    {
        Assert.Contains(typeof(SaveSidecarWritePatch), GameplayPatchSet.PatchTypes);
        Assert.Contains(typeof(SaveSidecarRestorePatch), GameplayPatchSet.PatchTypes);
        Assert.Contains(typeof(SaveDrillPatch), GameplayPatchSet.PatchTypes);
    }
}
