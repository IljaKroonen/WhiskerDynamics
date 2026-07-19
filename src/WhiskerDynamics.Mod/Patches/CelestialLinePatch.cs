using HarmonyLib;
using KSA;

namespace WhiskerDynamics.Mod.Patches;

/// <summary>Honest orbit lines: celestial draw-site takeover, prefixing the
/// method that draws the stock ellipse (Celestial.AddLineInstances, Celestial.cs:1770).
/// Line-visibility policy (<see cref="LineVisibility"/>): while the mod is
/// bound, the frame-relative relevance rule replaces stock's distance culls
/// (ShouldDrawLines and the ShouldDrawUiOrLines heuristics behind it) on EVERY route —
/// stock's ShowOrbit/target opt-ins still gate inside the kernel rule.
/// Three-way context routing (<see cref="LineRoute"/>):
/// Draw =&gt; stage the fresh matching-mode arc and draw it with joinEnds:false (a
/// 30-day Saturn stub must not get a chord across the map, Orbit.cs:2231-2234) when
/// the policy shows it, anchored per the batch mode (inertial payloads are
/// t − SampleT0, so currentTimeSincePe is now − SampleT0 — anchoring at zero at high
/// warp would splice the body's current position beside the STALE arc start, a chord;
/// frame payloads are t − now, so their anchor stays SimTime.Zero), then skip the
/// original.
/// Blink =&gt; fresh curve, wrong frame MODE: draw nothing, keep stock suppressed
/// AND keep the restore debt (StagedIds) for the ≤~1 s until the worker's label-change
/// bypass resamples — restoring stock here would flash a frame-wrong ellipse
/// for every body on every frame switch.
/// Stock =&gt; no usable curve (never sampled / stale): hand back any overwritten line
/// once (TryRestoreStock) BEFORE any gate (restore-before-gates: other consumers
/// read the cached points even when the line then hides), then draw the stock
/// conic OURSELVES when the policy shows it (stock's own call minus its distance
/// cull, Celestial.cs:1774) or suppress it entirely when it doesn't. Mod disabled or
/// unbound stays truly full-stock (restore + original), distance hiding included.</summary>
[HarmonyPatch(typeof(Celestial), "AddLineInstances")]
internal static class CelestialLinePatch
{
    private static int _activeLogged;

    internal static void ResetSessionStatics() => System.Threading.Volatile.Write(ref _activeLogged, 0);

    static bool Prefix(Celestial __instance, Viewport inViewport)
    {
        try
        {
            var rails = ModServices.Enabled ? ModServices.Rails : null;
            if (rails is null)
            {
                // Mod disabled/unbound: full stock — its distance hiding included.
                CelestialCurves.TryRestoreStock(__instance);
                return true;
            }
            // Line-visibility policy: frame-relative relevance replaces
            // stock's distance culls on EVERY route — a body the policy hides must
            // not pop back via the stock fallback, and a body it shows must not
            // vanish when the camera zooms out. The one stock-style terminal path:
            // hand back the cache (restore-before-gates, see class doc — other
            // consumers read the cached points even when the line then hides), then
            // draw stock's own call (Celestial.cs:1774) minus its ShouldDrawLines
            // distance cull.
            bool RestoreAndDrawStockConic()
            {
                CelestialCurves.TryRestoreStock(__instance);
                if (LineVisibility.ForCelestial(__instance, inViewport))
                    __instance.Orbit.DrawLines(inViewport,
                        inViewport.GetCamera().GetPositionEgo(__instance),
                        SimTime.Zero);
                return false;
            }
            switch (CelestialCurves.Route(__instance))
            {
                case LineRoute.Stock:
                    return RestoreAndDrawStockConic();
                case LineRoute.Blink:
                    // Frame mode changed since sampling: fresh-but-wrong-mode curve.
                    // Draw nothing, keep stock suppressed, KEEP the restore debt
                    // (StagedIds) — the worker's label-change bypass resamples within
                    // ~1 s. Restoring stock here would flash a frame-wrong
                    // ellipse for every body on every switch.
                    return false;
            }
            // LineRoute.Draw from here.
            // The dense and stock-shaped buffers are both cosmetic for a hidden
            // body. Gate before StageFresh so an invisible framed curve cannot pay a
            // 2000-point re-embed every render frame.
            if (!LineVisibility.ForCelestial(__instance, inViewport))
                return false;
            if (!CelestialCurves.StageFresh(__instance, rails, out var staged))
            {
                // Stage-time surprise (curve pruned since Route, parentless orbit,
                // frame pose gone): same terminal path as Stock.
                return RestoreAndDrawStockConic();
            }
            {
                // Honest-density draw: the dense arc goes through
                // OrbitLinePass directly; the staged 2000-point buffer stays the
                // pick surface. The splice/fade anchor compares ABSOLUTE dense times
                // against "now", which is the payload-space anchor rule by
                // construction in both batch modes (inertial payload t−SampleT0 >
                // now−SampleT0 ⇔ t > now; frame payload t−now > 0 ⇔ t > now), so
                // the warp stale-arc chord stays fixed. The context comes ready from
                // StageFresh — the same pose/parent resolution the pick buffer was
                // staged with this frame.
                // Celestial arcs never hit the framed-batch-drawn-inertial fallback
                // (StageFresh returns false outright when the pose fails), so the
                // one arc array serves both slots.
                var ctx = staged.Context;
                DenseLineDraw.Draw(inViewport, __instance.Orbit, staged.DenseTimes,
                    staged.DenseCoordinates, ctx.Framed ? staged.DenseCoordinates : null,
                    staged.DenseMetrics, staged.DenseMetrics,
                    in ctx, __instance.Orbit.OrbitLineColor,
                    inViewport.GetCamera().GetPositionEgo(__instance), staged.NowSeconds);
            }
            if (System.Threading.Interlocked.CompareExchange(ref _activeLogged, 1, 0) == 0)
                ModLog.Info($"celestial line takeover active (first: '{__instance.Id}'; "
                    + (FrameManager.Active is { } f ? $"frame '{f.Label}'" : "inertial honest arcs") + ")");
            return false;
        }
        catch (Exception e)
        {
            FrameManager.NoteContained("celestial line staging", e);
            // A throw AFTER a successful stage must not leave stock re-drawing the
            // just-staged mod points (with its joinEnds:true closing chord) every
            // frame: hand back before falling back. Once-only (StagedIds.TryRemove),
            // no-op when nothing was staged. Own swallow: this catch IS the
            // containment boundary (prefixes never throw into the game), so a restore
            // failure must not escape it — the original exception is already noted.
            try { CelestialCurves.TryRestoreStock(__instance); } catch { }
            return true;
        }
    }
}
