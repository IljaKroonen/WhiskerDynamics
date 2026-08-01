using Brutal.Numerics;
using HarmonyLib;
using KSA;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Honest burn-node DRAGGING (BurnClickPatch suppresses the
/// hover-circle/click channel one seam above, so the drag path — Burn's
/// GizmoLerp calling Orbit.GetNearestPoint directly, Burn.cs:999 — is this patch's
/// consumer): stock's nearest-orbit-point math is ANALYTIC for elliptical
/// orbits (Orbit.GetNearestPoint's e&lt;1 ray/plane path, Orbit.cs:2398-2437) and
/// checks none of the visibility gates the draw path honors, so it would hit-test
/// invisible Kepler ellipses: suppressed future-SOI conics, suppressed post-burn
/// conics, and the osculating ellipse the drawn n-body line diverges from — making
/// node drags dead-zone or teleport wherever the conic leaves the drawn line. This
/// prefix routes by patch instance while the controlled vehicle has a FRESH mod line
/// (the same gate as VesselLinePatch):
/// (1) the vehicle's own plan patch 0 — the DRAWN actual line: substitute a hit-test
/// over the BATCH ITSELF (immutable arrays, real-count prefix — never the pooled
/// cached-points buffer, which stock's solvers can swap mid-scan on this worker
/// thread and whose stock-refreshed content is indistinguishable at 2000==2000);
/// (2) any other own-plan patch (suppressed future-SOI conics) — no hover, no click;
/// (3) the FIRST patch of ANY of the vessel's burn plans — all of them lie along the
/// one drawn PLANNED line: substitute against the planned batch with payload times
/// re-anchored to THAT patch's orbit, so clicks place chained burns at the hovered
/// time (stock's derivation reads value.Patch.StartTime + payload arithmetic against
/// that orbit — exact within one period, stock's own wrap rule beyond) and node drags
/// ride the drawn line (the candidate, the incumbent Burn.PositionCce — repositioned
/// by BurnNodePatch — and the drawn geometry all agree);
/// (4) later patches of burn plans (suppressed post-burn SOI conics) — no hover;
/// (5) anything else (TransferPlanner preview, other vessels, celestials) — stock,
/// untouched. ABSENT batch (never published) =&gt; stock everywhere; STALE batch =&gt;
/// ownership rule: suppressed conics stay unhoverable, patch 0
/// reverts to stock's analytic hit-test — matching the stock-style patch-0 line
/// VesselLinePatch.DrawStalePatch0 shows in the same windows. The substitute projects
/// the mouse onto the drawn CHORDS (conservative block search plus the shared exact
/// adjacent-segment rule), so hover stays continuous even across long screen-space
/// chords, then mirrors stock's NDC acceptance gate bit-for-bit
/// (integer aspect division included, Orbit.cs:2383-2395). Hidden lines
/// (vehicle.ShowOrbit off — VesselLinePatch draws nothing) hover nothing.</summary>
[HarmonyPatch(typeof(Orbit), "GetNearestPoint")]
internal static class OrbitHoverPatch
{
    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static bool Prefix(Orbit __instance, Viewport inViewPort, float2 mousePosScreen,
        PatchedConic? patch, ref OrbitPointCce? pointSelected,
        float mouseDistanceScreenPercent, ref bool __result)
    {
        if (!ModServices.Enabled) return true;
        try
        {
            if (patch is null || KSA.Program.ControlledVehicle is not { } vehicle) return true;
            // Ownership, not freshness (matching VesselLinePatch):
            // an owned-but-stale vessel keeps its suppressed conics unhoverable; only
            // patch 0 falls back to stock's analytic hit-test — matching the
            // stock-style patch-0 line DrawStalePatch0 shows in the same windows.
            var actual = OverlayBuffer.Read(vehicle.Id);
            if (actual is null) return true; // never published: stock everywhere (fallback)
            long nowMs = Environment.TickCount64;
            double nowSimSeconds = Universe.GetElapsedSimTime().Seconds();
            bool fresh = OverlayBuffer.ConsumerSamplesUsable(
                vehicle.Id, actual, nowMs, nowSimSeconds);
            bool lineVisible = vehicle.ShowOrbit || vehicle.TargetOfControlledVehicle;

            var ownPatches = vehicle.FlightPlan.Patches;
            int ownIndex = -1;
            for (int i = 0; i < ownPatches.Count; i++)
                if (ReferenceEquals(ownPatches[i], patch)) { ownIndex = i; break; }

            if (ownIndex == 0)
            {
                if (!fresh)
                {
                    // An optimizer lease may keep only the sampled LINE visible.
                    // Its time payload is intentionally still stale, so suppress
                    // clicking rather than delegate to the invisible stock conic.
                    if (OverlayBuffer.LineSamplesUsable(
                            vehicle.Id, actual, planned: false, nowMs, nowSimSeconds)
                        && OverlayBuffer.IsLineLeased(
                            vehicle.Id, actual, planned: false, nowMs))
                    {
                        pointSelected = null;
                        __result = false;
                        return false;
                    }
                    // Stale window: hit-test whatever is DRAWN there. Inertial view
                    // (no counter-pose — includes body-centred inertial frames) —
                    // stock's analytic conic, matching DrawStalePatch0's stock-style
                    // line. Counter-posed view — VesselLinePatch draws NOTHING
                    // (wrong-frame geometry has no honest line mid-burn), so nothing
                    // may hit-test either: the invisible conic is frame-inconsistent
                    // geometry.
                    if (FrameManager.InertialView) return true;
                    pointSelected = null;
                    __result = false;
                    return false;
                }
                __result = lineVisible && TryNearestOnDrawnLine(__instance, actual,
                    reanchorTimes: false, inViewPort, mousePosScreen,
                    mouseDistanceScreenPercent, out pointSelected);
                if (!__result) pointSelected = null;
                NoteActive(vehicle);
                return false;
            }
            if (ownIndex > 0)
            {
                pointSelected = null;
                __result = false; // hidden future-SOI conic: no ghost hover, no ghost click
                return false;
            }

            if (BurnPlanScan.ContainsPatch(vehicle, patch))
            {
                var planned = OverlayBuffer.ReadPlannedFresh(
                    vehicle.Id, nowMs, nowSimSeconds);
                if (planned is not null && lineVisible
                    && BurnPlanScan.IsFirstPatchOfAnyBurnPlan(vehicle, patch))
                {
                    __result = TryNearestOnDrawnLine(__instance, planned, reanchorTimes: true,
                        inViewPort, mousePosScreen, mouseDistanceScreenPercent, out pointSelected);
                    if (!__result) pointSelected = null;
                    return false;
                }
                pointSelected = null;
                __result = false; // hidden post-burn conic (or no planned line): nothing to hover
                return false;
            }

            return true; // TransferPlanner preview / other vessels / future callers: stock
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("orbit hover routing", e);
            return true;
        }
    }

    /// <summary>Substitute hit-test over the batch's authoritative bounded traversal —
    /// the same worker-built ordered subset DenseLineDraw emits before its optional
    /// current-screen significance filter. Projects
    /// under the SAME staging context the line was drawn with
    /// (TrajectoryOverlay.BuildStagingContext / StagingContext.Drawn — one position
    /// rule), finds the nearest point ON the polyline (chord-projected), gates with
    /// stock's NDC rule, and derives the payloads from the dense times (linear in t:
    /// TimeSincePe = t − batch conic anchor, or t − the canvas re-anchor;
    /// RemainingTimeTo = t − SampleT0 — bit-identical to the staged payloads at the
    /// decimated subset's indices). Never touches the orbit's cached-points buffer:
    /// the batch arrays are immutable, so this is safe from the hover job's worker
    /// thread while the render thread restages.</summary>
    private static bool TryNearestOnDrawnLine(Orbit orbit, OverlaySamples samples,
        bool reanchorTimes, Viewport viewport, float2 mousePosScreen,
        float mouseDistanceScreenPercent, out OrbitPointCce? pointSelected)
    {
        pointSelected = null;
        if (!OverlayKernel.ModeMatches(samples.FrameLabel, FrameManager.Active?.Label))
            return false; // the ≤1 s blink: the line is not drawn, nothing should hover
        var ctx = TrajectoryOverlay.BuildStagingContext(samples, orbit, reanchorTimes);
        var camera = viewport.GetCamera();
        var parentEgo = camera.GetPositionEgo(orbit.Parent);

        int total = samples.DenseTimes.Length;
        if (total < 1) return false;
        var times = samples.DenseTimes;
        var positions = samples.DensePositionsCce;
        var frameCoordinates = samples.DenseFrameCoordinates;
        Vector3d Drawn(int i) => ctx.Drawn(positions[i], ctx.Framed ? frameCoordinates![i] : default);
        double clipTime = reanchorTimes
            ? Universe.GetElapsedSimTime().Seconds()
            : double.NegativeInfinity;
        OverlayKernel.FutureClip? clip = reanchorTimes
            ? OverlayKernel.FutureClipAt(times, clipTime)
            : new OverlayKernel.FutureClip(0, 0, 0.0);
        if (clip is null) return false;
        int first = clip.Value.Hi;
        bool boundary = clip.Value.Lo != clip.Value.Hi;
        DecimationMetrics metrics = ctx.Framed
            ? samples.DenseMetrics
            : samples.DenseMetricsCce;
        int[] traversal = metrics.TraversalIndices;
        _ = OverlayKernel.TraversalSuffixStart(
            traversal, first, boundary, out bool explicitFirst);
        HoverPointRef? prefix = boundary
            ? HoverPointRef.ClippedBoundary(
                clip.Value.Lo, clip.Value.Hi, clip.Value.Frac, clipTime)
            : explicitFirst ? new HoverPointRef(first) : null;

        Vector3d[] sourceCoordinates = ctx.Framed ? frameCoordinates! : positions;
        var projector = new CameraProjector(
            sourceCoordinates, in ctx, camera, parentEgo, ctx.Framed);
        if (!HoverHitTestKernel.TryNearest(
                traversal,
                metrics.HoverPlan,
                first,
                prefix,
                new HoverScreenPoint(mousePosScreen.X, mousePosScreen.Y),
                ref projector,
                out HoverHit hit))
            return false;
        var projected = new float2(hit.Projected.X, hit.Projected.Y);

        // Stock's acceptance gate, mirrored bit-for-bit (Orbit.cs:2383-2395) — the
        // integer aspect division included: behavioral parity beats local taste.
        float3 pointNdc = camera.ScreenToNdc(projected, 1f);
        float3 mouseNdc = camera.ScreenToNdc(mousePosScreen, 1f);
        double deltaX = pointNdc.X - mouseNdc.X, deltaY = pointNdc.Y - mouseNdc.Y;
        int2 size = viewport.Size;
        int aspect = size.X / size.Y;
        deltaX *= aspect;
        if (Math.Sqrt(deltaX * deltaX + deltaY * deltaY) > mouseDistanceScreenPercent)
            return false;

        Vector3d Position(HoverPointRef point)
        {
            Vector3d loPosition = Drawn(point.SourceIndex);
            Vector3d hiPosition = point.IsInterpolated
                ? Drawn(point.OtherSourceIndex)
                : default;
            return OverlayKernel.ResolveHoverPosition(
                point, loPosition, hiPosition);
        }
        double Time(HoverPointRef point) =>
            OverlayKernel.ResolveHoverTime(point, times);
        var cce = Position(hit.Lo) * (1.0 - hit.Fraction)
            + Position(hit.Hi) * hit.Fraction;
        double t = Time(hit.Lo) * (1.0 - hit.Fraction)
            + Time(hit.Hi) * hit.Fraction;
        // The batch's conic anchor, recovered from any staged pair (TimesSincePe[k]
        // = Times[k] − TimeAtPe by construction — the arrays are never empty).
        double anchorPe = double.IsNaN(ctx.AnchorPeSeconds)
            ? samples.Times[0] - samples.TimesSincePe[0]
            : ctx.AnchorPeSeconds;
        pointSelected = new OrbitPointCce(
            FrameAdapter.ToGame(cce),
            new SimTime(t - anchorPe),
            new SimTime(t - samples.SampleT0),
            TrueAnomaly.NaN,
            inDangerZone: false);
        return true;
    }

    /// <summary>Per-query value projector. Bounds and fine vertices use the same
    /// current staging context and camera, so no projected cache can survive a view,
    /// frame, SOI-parent, or clip change and become a stale hit surface.</summary>
    private readonly struct CameraProjector : IHoverPointProjector
    {
        private const double DoubleRoundoffScale = 32.0 * 2.2204460492503131e-16;
        private const double MaximumPrunableScreenMagnitude = 16777216.0;

        private readonly struct FloatInterval
        {
            public FloatInterval(float minimum, float maximum)
            {
                Minimum = minimum;
                Maximum = maximum;
            }

            public float Minimum { get; }
            public float Maximum { get; }
        }

        private readonly Vector3d[] _sourceCoordinates;
        private readonly TrajectoryOverlay.StagingContext _context;
        private readonly Camera _camera;
        private readonly double3 _parentEgo;
        private readonly double3 _forward;
        private readonly bool _framed;

        public CameraProjector(Vector3d[] sourceCoordinates,
            in TrajectoryOverlay.StagingContext context,
            Camera camera,
            double3 parentEgo,
            bool framed)
        {
            _sourceCoordinates = sourceCoordinates;
            _context = context;
            _camera = camera;
            _parentEgo = parentEgo;
            _forward = camera.GetForwardEcl();
            _framed = framed;
        }

        public int SourceCount => _sourceCoordinates.Length;
        public double NearPlaneDepth => _camera.NearPlane;

        public HoverProjection Project(HoverPointRef point)
        {
            Vector3d stagedSource = Stage(_sourceCoordinates[point.SourceIndex]);
            Vector3d stagedOther = point.IsInterpolated
                ? Stage(_sourceCoordinates[point.OtherSourceIndex])
                : default;
            Vector3d drawn = OverlayKernel.ResolveHoverPosition(
                point, stagedSource, stagedOther);
            return ProjectDrawn(drawn, ignoreBehind: true);
        }

        // Bounds need finite projections on both sides of the camera plane so the
        // depth extrema can distinguish wholly-behind boxes from crossings. Fine
        // vertices above retain stock's ordinary ignore-behind behavior.
        public HoverProjection ProjectSource(Vector3d sourceCoordinate) =>
            ProjectDrawn(Stage(sourceCoordinate), ignoreBehind: false);

        /// <summary>Bounds the exact FLOAT projection used by <see cref='Project'/>.
        /// Source-box corners first bound the affine staging transform in ego space;
        /// directed float intervals then replay Camera.EgoToClip's documented
        /// multiply/add/divide/viewport order. EgoToClipDouble supplies an independent
        /// clip-W/near-plane sanity check and magnitude-aware outward screen envelope.
        /// Any non-finite, near, sign-ambiguous, or enormous case refines instead.</summary>
        public HoverProjectedBounds ProjectBounds(
            Vector3d minimum, Vector3d maximum)
        {
            if (!Finite(minimum) || !Finite(maximum)
                || minimum.X > maximum.X || minimum.Y > maximum.Y
                || minimum.Z > maximum.Z)
                return Unprunable();

            double minEgoX = double.PositiveInfinity;
            double minEgoY = double.PositiveInfinity;
            double minEgoZ = double.PositiveInfinity;
            double maxEgoX = double.NegativeInfinity;
            double maxEgoY = double.NegativeInfinity;
            double maxEgoZ = double.NegativeInfinity;
            for (int corner = 0; corner < 8; corner++)
            {
                var source = new Vector3d(
                    (corner & 1) == 0 ? minimum.X : maximum.X,
                    (corner & 2) == 0 ? minimum.Y : maximum.Y,
                    (corner & 4) == 0 ? minimum.Z : maximum.Z);
                double3 ego = _parentEgo + FrameAdapter.ToGame(Stage(source));
                if (!Finite(ego)) return Unprunable();
                minEgoX = Math.Min(minEgoX, ego.X);
                minEgoY = Math.Min(minEgoY, ego.Y);
                minEgoZ = Math.Min(minEgoZ, ego.Z);
                maxEgoX = Math.Max(maxEgoX, ego.X);
                maxEgoY = Math.Max(maxEgoY, ego.Y);
                maxEgoZ = Math.Max(maxEgoZ, ego.Z);
            }

            // Staging is affine. Directed expansion protects the component extrema
            // from the double arithmetic that produced them before float packing.
            minEgoX = OutwardLower(minEgoX); minEgoY = OutwardLower(minEgoY);
            minEgoZ = OutwardLower(minEgoZ); maxEgoX = OutwardUpper(maxEgoX);
            maxEgoY = OutwardUpper(maxEgoY); maxEgoZ = OutwardUpper(maxEgoZ);
            if (!TryPackInterval(minEgoX, maxEgoX, out FloatInterval egoX)
                || !TryPackInterval(minEgoY, maxEgoY, out FloatInterval egoY)
                || !TryPackInterval(minEgoZ, maxEgoZ, out FloatInterval egoZ))
                return Unprunable();

            var matrix = _camera.MVP.viewProjection;
            if (!TryClipInterval(matrix.X.X, matrix.Y.X, matrix.Z.X, matrix.W.X,
                    egoX, egoY, egoZ, out FloatInterval clipX)
                || !TryClipInterval(matrix.X.Y, matrix.Y.Y, matrix.Z.Y, matrix.W.Y,
                    egoX, egoY, egoZ, out FloatInterval clipY)
                || !TryClipInterval(matrix.X.W, matrix.Y.W, matrix.Z.W, matrix.W.W,
                    egoX, egoY, egoZ, out FloatInterval clipW))
                return Unprunable();

            Span<double4> doubleClips = stackalloc double4[8];
            double minDepth = double.PositiveInfinity;
            double maxDepth = double.NegativeInfinity;
            double minDoubleW = double.PositiveInfinity;
            double maxDoubleW = double.NegativeInfinity;
            for (int corner = 0; corner < 8; corner++)
            {
                var ego = new double3(
                    (corner & 1) == 0 ? minEgoX : maxEgoX,
                    (corner & 2) == 0 ? minEgoY : maxEgoY,
                    (corner & 4) == 0 ? minEgoZ : maxEgoZ);
                double depth = double3.Dot(ego, _forward);
                double4 clip = _camera.EgoToClipDouble(ego);
                if (!double.IsFinite(depth) || !Finite(clip))
                    return Unprunable();
                doubleClips[corner] = clip;
                minDepth = Math.Min(minDepth, depth);
                maxDepth = Math.Max(maxDepth, depth);
                minDoubleW = Math.Min(minDoubleW, clip.W);
                maxDoubleW = Math.Max(maxDoubleW, clip.W);
            }
            minDepth = OutwardLower(minDepth);
            maxDepth = OutwardUpper(maxDepth);
            minDoubleW = OutwardLower(minDoubleW);
            maxDoubleW = OutwardUpper(maxDoubleW);

            double near = NearPlaneDepth;
            if (!double.IsFinite(near) || near < 0.0) return Unprunable();
            if (maxDepth < 0.0)
            {
                if (_camera.Orthographic
                    || clipW.Maximum < 0.0f && maxDoubleW < 0.0)
                    return new HoverProjectedBounds(
                        HoverBoundsKind.WhollyBehind, 0.0, 0.0, 0.0, 0.0);
                return Unprunable();
            }

            // The game's ignore-behind predicate is depth-based, while perspective
            // division is clip-W based. Both must prove a comfortably front-facing
            // box; disagreement or a near-plane touch is deliberately unprunable.
            if (minDepth <= near
                || clipW.Minimum <= (_camera.Orthographic ? 0.0f : (float)near)
                || minDoubleW <= (_camera.Orthographic ? 0.0 : near))
                return Unprunable();

            if (!TryDivide(clipX, clipW, out FloatInterval ndcX)
                || !TryDivide(clipY, clipW, out FloatInterval ndcY)
                || !TryScreen(ndcX, _camera.FramebufferSize.X,
                    out FloatInterval floatScreenX)
                || !TryScreen(ndcY, _camera.FramebufferSize.Y,
                    out FloatInterval floatScreenY))
                return Unprunable();

            double minDoubleX = double.PositiveInfinity;
            double minDoubleY = double.PositiveInfinity;
            double maxDoubleX = double.NegativeInfinity;
            double maxDoubleY = double.NegativeInfinity;
            int width = _camera.FramebufferSize.X;
            int height = _camera.FramebufferSize.Y;
            if (width <= 0 || height <= 0) return Unprunable();
            for (int corner = 0; corner < doubleClips.Length; corner++)
            {
                double4 clip = doubleClips[corner];
                double screenX = (clip.X / clip.W + 1.0) * 0.5 * width;
                double screenY = (clip.Y / clip.W + 1.0) * 0.5 * height;
                if (!double.IsFinite(screenX) || !double.IsFinite(screenY))
                    return Unprunable();
                minDoubleX = Math.Min(minDoubleX, screenX);
                minDoubleY = Math.Min(minDoubleY, screenY);
                maxDoubleX = Math.Max(maxDoubleX, screenX);
                maxDoubleY = Math.Max(maxDoubleY, screenY);
            }

            double minX = OutwardLower(Math.Min(floatScreenX.Minimum, minDoubleX));
            double minY = OutwardLower(Math.Min(floatScreenY.Minimum, minDoubleY));
            double maxX = OutwardUpper(Math.Max(floatScreenX.Maximum, maxDoubleX));
            double maxY = OutwardUpper(Math.Max(floatScreenY.Maximum, maxDoubleY));
            double magnitude = Math.Max(
                Math.Max(Math.Abs(minX), Math.Abs(maxX)),
                Math.Max(Math.Abs(minY), Math.Abs(maxY)));
            if (!double.IsFinite(magnitude)
                || magnitude > MaximumPrunableScreenMagnitude)
                return Unprunable();
            return new HoverProjectedBounds(
                HoverBoundsKind.Bounded, minX, minY, maxX, maxY);
        }

        private Vector3d Stage(Vector3d sourceCoordinate) =>
            _framed
                ? _context.Drawn(default, sourceCoordinate)
                : _context.Drawn(sourceCoordinate, default);

        private HoverProjection ProjectDrawn(Vector3d drawn, bool ignoreBehind)
        {
            double3 ego = _parentEgo + FrameAdapter.ToGame(drawn);
            double depth = double3.Dot(ego, _forward);
            float2 screen = _camera.EgoToScreen(ego, ignoreBehind);
            return new HoverProjection(
                new HoverScreenPoint(screen.X, screen.Y), depth);
        }

        private static HoverProjectedBounds Unprunable() =>
            new(HoverBoundsKind.Unprunable, 0.0, 0.0, 0.0, 0.0);

        private static bool Finite(Vector3d value) =>
            double.IsFinite(value.X) && double.IsFinite(value.Y)
                && double.IsFinite(value.Z);

        private static bool Finite(double3 value) =>
            double.IsFinite(value.X) && double.IsFinite(value.Y)
                && double.IsFinite(value.Z);

        private static bool Finite(double4 value) =>
            double.IsFinite(value.X) && double.IsFinite(value.Y)
                && double.IsFinite(value.Z) && double.IsFinite(value.W);

        private static double OutwardLower(double value)
        {
            double pad = Math.Max(double.Epsilon,
                Math.Max(1.0, Math.Abs(value)) * DoubleRoundoffScale);
            return Math.BitDecrement(value - pad);
        }

        private static double OutwardUpper(double value)
        {
            double pad = Math.Max(double.Epsilon,
                Math.Max(1.0, Math.Abs(value)) * DoubleRoundoffScale);
            return Math.BitIncrement(value + pad);
        }

        private static bool TryPackInterval(
            double minimum, double maximum, out FloatInterval interval)
        {
            interval = default;
            float lower = (float)minimum;
            float upper = (float)maximum;
            if (!float.IsFinite(lower) || !float.IsFinite(upper)) return false;
            if (lower > minimum) lower = MathF.BitDecrement(lower);
            if (upper < maximum) upper = MathF.BitIncrement(upper);
            lower = MathF.BitDecrement(lower);
            upper = MathF.BitIncrement(upper);
            if (!float.IsFinite(lower) || !float.IsFinite(upper)) return false;
            interval = new FloatInterval(lower, upper);
            return true;
        }

        private static bool TryClipInterval(float xCoefficient, float yCoefficient,
            float zCoefficient, float constant, FloatInterval x, FloatInterval y,
            FloatInterval z, out FloatInterval result)
        {
            result = default;
            if (!TryMultiply(x, xCoefficient, out FloatInterval value)
                || !TryMultiply(y, yCoefficient, out FloatInterval term)
                || !TryAdd(value, term, out value)
                || !TryMultiply(z, zCoefficient, out term)
                || !TryAdd(value, term, out value)
                || !TryAdd(value, new FloatInterval(constant, constant), out value))
                return false;
            result = value;
            return true;
        }

        private static bool TryMultiply(FloatInterval value, float coefficient,
            out FloatInterval result)
        {
            result = default;
            if (!float.IsFinite(coefficient)) return false;
            float a = value.Minimum * coefficient;
            float b = value.Maximum * coefficient;
            if (!float.IsFinite(a) || !float.IsFinite(b)) return false;
            float lower = MathF.Min(a, b);
            float upper = MathF.Max(a, b);
            lower = MathF.BitDecrement(lower);
            upper = MathF.BitIncrement(upper);
            if (!float.IsFinite(lower) || !float.IsFinite(upper)) return false;
            result = new FloatInterval(lower, upper);
            return true;
        }

        private static bool TryAdd(FloatInterval a, FloatInterval b,
            out FloatInterval result)
        {
            result = default;
            float lower = a.Minimum + b.Minimum;
            float upper = a.Maximum + b.Maximum;
            if (!float.IsFinite(lower) || !float.IsFinite(upper)) return false;
            lower = MathF.BitDecrement(lower);
            upper = MathF.BitIncrement(upper);
            if (!float.IsFinite(lower) || !float.IsFinite(upper)) return false;
            result = new FloatInterval(lower, upper);
            return true;
        }

        private static bool TryDivide(FloatInterval numerator,
            FloatInterval denominator, out FloatInterval result)
        {
            result = default;
            if (!(denominator.Minimum > 0.0f)) return false;
            float q0 = numerator.Minimum / denominator.Minimum;
            float q1 = numerator.Minimum / denominator.Maximum;
            float q2 = numerator.Maximum / denominator.Minimum;
            float q3 = numerator.Maximum / denominator.Maximum;
            if (!float.IsFinite(q0) || !float.IsFinite(q1)
                || !float.IsFinite(q2) || !float.IsFinite(q3))
                return false;
            float lower = MathF.Min(MathF.Min(q0, q1), MathF.Min(q2, q3));
            float upper = MathF.Max(MathF.Max(q0, q1), MathF.Max(q2, q3));
            lower = MathF.BitDecrement(lower);
            upper = MathF.BitIncrement(upper);
            if (!float.IsFinite(lower) || !float.IsFinite(upper)) return false;
            result = new FloatInterval(lower, upper);
            return true;
        }

        private static bool TryScreen(
            FloatInterval ndc, int extent, out FloatInterval result)
        {
            result = default;
            if (extent <= 0
                || !TryAdd(ndc, new FloatInterval(1.0f, 1.0f), out var shifted)
                || !TryMultiply(shifted, 0.5f, out var normalized)
                || !TryMultiply(normalized, (float)extent, out result))
                return false;
            return true;
        }
    }

    private static void NoteActive(Vehicle vehicle)
    {
        if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
            ModLog.Info($"orbit nearest-point takeover active (first: '{vehicle.Id}'; node drags "
                + "ride the n-body batches; suppressed conics hit-test nothing)");
    }
}
