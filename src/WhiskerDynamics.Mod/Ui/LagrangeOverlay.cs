using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>Map-only CR3BP effective-potential overlay for the active body-body fixed
/// frame. Geometry is cached in separation-normalized frame coordinates, then the
/// current rotating-pulsating pose re-embeds it each draw. Thus the mesh is generated
/// once per mass ratio while both bodies remain pinned and eccentric separation changes
/// only the displayed scale. This is a display aid, never a force-model input.</summary>
public static class LagrangeOverlay
{
    private sealed record Mesh(PotentialSegment[] Segments, Vector3d[] Points);
    private sealed record MeshBuild(int Generation, Task<Mesh> Task);

    private static readonly object Gate = new();
    private static readonly Dictionary<(long PrimaryMu, long SecondaryMu), MeshBuild> Cache = [];
    private static CancellationTokenSource _buildStop = new();
    private static int _generation;
    private const float PointRadius = 3.5f;
    private const int MaximumDrawSegments = 8192;

    /// <summary>Session-only user choice, exposed only while a body-body fixed frame
    /// is active. Off by default so the established map view remains unchanged.</summary>
    public static bool Enabled { get; set; }

    internal static bool AvailableFor(FrameSpec? frame) =>
        frame is { Kind: FrameKind.TwoBodyFixed };

    public static void Draw()
    {
        if (!Enabled || !ModServices.Enabled || !Program.DrawUI) return;
        try
        {
            if (FrameManager.Active is not { Kind: FrameKind.TwoBodyFixed } spec
                || spec.SecondaryId is null || ModServices.Rails is not { } rails)
                return;
            var viewport = Program.MainViewport;
            if (viewport.Mode != CameraMode.Map) return;
            double now = Universe.GetElapsedSimTime().Seconds();
            if (!FrameManager.TrySamplePose(now, out var pose)) return;

            double primaryMu = rails.MuOf(spec.PrimaryId);
            double secondaryMu = rails.MuOf(spec.SecondaryId);
            var mesh = GetMesh(primaryMu, secondaryMu);
            if (mesh is null) return;
            var camera = viewport.GetCamera();
            var drawList = ImGui.GetBackgroundDrawList();
            int2 size = viewport.Size;
            byte4 lineColor = default;
            lineColor.R = 80; lineColor.G = 185; lineColor.B = 235; lineColor.A = 105;
            foreach (var segment in mesh.Segments)
            {
                float2 a = camera.EclToScreen(FrameAdapter.ToGame(pose.FromFrame(segment.A)));
                float2 b = camera.EclToScreen(FrameAdapter.ToGame(pose.FromFrame(segment.B)));
                // Let ImGui clip lines whose endpoints are both outside: a long contour may cross
                // the viewport even when neither endpoint lies inside it.
                if (!Finite(a) || !Finite(b)) continue;
                drawList.AddLine(viewport.Position + a, viewport.Position + b, lineColor, 1f);
            }

            byte4 pointColor = default;
            pointColor.R = 115; pointColor.G = 220; pointColor.B = 255; pointColor.A = 230;
            for (int i = 0; i < 5; i++)
            {
                float2 screen = camera.EclToScreen(FrameAdapter.ToGame(pose.FromFrame(mesh.Points[i])));
                if (!Visible(screen, size, PointRadius)) continue;
                float2 at = viewport.Position + screen;
                drawList.AddCircleFilled(at, PointRadius, pointColor);
                ImGuiHelper.DrawTextOnScreen(drawList, at + new float2(6f, -6f),
                    $"L{i + 1}", Color.White.AsByte4);
            }
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("Lagrange potential overlay", e);
        }
    }

    internal static void ResetSessionStatics()
    {
        Enabled = false;
        CancellationTokenSource old;
        lock (Gate)
        {
            old = _buildStop;
            _buildStop = new CancellationTokenSource();
            _generation++;
            Cache.Clear();
        }
        old.Cancel();
        old.Dispose();
    }

    private static Mesh? GetMesh(double primaryMu, double secondaryMu)
    {
        var key = (BitConverter.DoubleToInt64Bits(primaryMu), BitConverter.DoubleToInt64Bits(secondaryMu));
        MeshBuild build;
        lock (Gate)
        {
            if (!Cache.TryGetValue(key, out build!))
            {
                int generation = _generation;
                CancellationToken token = _buildStop.Token;
                var task = Task.Run(() => BuildMesh(primaryMu, secondaryMu, token), token);
                Cache[key] = build = new MeshBuild(generation, task);
            }
        }
        if (build.Generation != Volatile.Read(ref _generation) || build.Task.IsCanceled) return null;
        if (build.Task.IsFaulted) throw build.Task.Exception!.GetBaseException();
        return build.Task.IsCompletedSuccessfully ? build.Task.Result : null;
    }

    private static Mesh BuildMesh(double primaryMu, double secondaryMu,
        CancellationToken cancellationToken)
    {
        double ratio = LagrangePotential.MassRatio(primaryMu, secondaryMu);
        var all = new List<PotentialSegment>();
        foreach (double critical in LagrangePotential.CriticalLevels(ratio))
        {
            cancellationToken.ThrowIfCancellationRequested();
            all.AddRange(LagrangePotential.Contour(ratio, critical + 1e-4,
                columns: 160, rows: 136));
        }
        cancellationToken.ThrowIfCancellationRequested();
        var points = NamedPoints(primaryMu, secondaryMu);
        PotentialSegment[] segments;
        if (all.Count <= MaximumDrawSegments)
        {
            segments = [.. all];
        }
        else
        {
            int[] indices = OverlayKernel.DecimateIndices(all.Count, MaximumDrawSegments);
            segments = new PotentialSegment[indices.Length];
            for (int i = 0; i < indices.Length; i++) segments[i] = all[indices[i]];
        }
        return new Mesh(segments, points);
    }

    internal static Vector3d[] NamedPoints(double primaryMu, double secondaryMu)
    {
        var equilibria = LagrangePotential.Equilibria(
            LagrangePotential.MassRatio(primaryMu, secondaryMu));
        var points = new Vector3d[5];
        for (int i = 0; i < points.Length; i++)
        {
            // Standard names take the more massive member as the primary. The frame
            // tree may order a moon before its parent, so mirror the label ordering.
            int mapped = primaryMu >= secondaryMu ? i : i switch
            {
                1 => 2,
                2 => 1,
                3 => 4,
                4 => 3,
                _ => 0,
            };
            points[i] = equilibria[mapped];
        }
        return points;
    }

    private static bool Finite(float2 p) => float.IsFinite(p.X) && float.IsFinite(p.Y);

    private static bool Visible(float2 p, int2 size, float margin = 1f) =>
        Finite(p)
        && p.X >= -margin && p.Y >= -margin
        && p.X <= size.X + margin && p.Y <= size.Y + margin;
}
