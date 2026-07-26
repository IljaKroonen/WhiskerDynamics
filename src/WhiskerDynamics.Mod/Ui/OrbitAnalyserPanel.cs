using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>Orbit analyser opened explicitly from FramesPanel.
/// Opening publishes a lock-free worker request; closing stops analysis work on the
/// next overlay rebuild. The render phase only formats immutable worker reports.</summary>
public static class OrbitAnalyserPanel
{
    internal enum AnalysisPhase
    {
        Waiting,
        Propagating,
        Sampling,
        Reducing,
        Complete,
    }

    internal readonly record struct AnalysisProgress(
        int Version, int Pass, double Fraction, AnalysisPhase Phase, bool Running);

    private sealed record ProgressState(
        int Version, int Pass, double Fraction, AnalysisPhase Phase, bool Running);

    private const double SecondsPerDay = 86400.0;
    private const double MaximumIntervalSeconds = 40 * 365.25 * SecondsPerDay;
    private const double MinimumSpanSeconds = 60.0;

    private static readonly (double Step, string Minus, string Plus)[] StartSteps =
        [(3600.0, "-1h", "+1h"), (86400.0, "-1d", "+1d"), (7 * 86400.0, "-7d", "+7d")];
    private static readonly (double Step, string Minus, string Plus)[] SpanSteps =
        [(3600.0, "-1h", "+1h"), (86400.0, "-1d", "+1d"), (30 * 86400.0, "-30d", "+30d")];

    private sealed record Request(bool Open, double StartOffsetSeconds, double SpanSeconds, int Version);
    private static Request _request = new(false, 0.0, 7 * SecondsPerDay, 0);
    private static ProgressState _progress = new(0, 0, 0, AnalysisPhase.Waiting, false);
    private static int _errors;
    private static bool _firstDrawLogged;
    private static OrbitAnalysisReport? _formattedReport;
    private static int _formattedVersion = -1;
    private static OrbitDashboardPresentation? _presentation;
    private static float[] _periapsisPlot = [];
    private static float[] _apoapsisPlot = [];
    private static float[] _eccentricityPlot = [];
    private static float[] _inclinationPlot = [];
    private static string _status = "";

    private static Request ReadRequest() => Volatile.Read(ref _request);

    private static void NotifyPredictionRequest(Request request) =>
        ModServices.Rails?.UpdateAnalysisPredictionRequest(
            request.Version, request.Open);

    internal static void Open()
    {
        while (true)
        {
            var current = ReadRequest();
            if (current.Open) return;
            var next = current with { Open = true, Version = current.Version + 1 };
            if (ReferenceEquals(Interlocked.CompareExchange(ref _request, next, current), current))
            {
                NotifyPredictionRequest(next);
                return;
            }
        }
    }

    internal static void Close()
    {
        while (true)
        {
            var current = ReadRequest();
            if (!current.Open) break;
            var next = current with { Open = false, Version = current.Version + 1 };
            if (ReferenceEquals(Interlocked.CompareExchange(ref _request, next, current), current))
            {
                NotifyPredictionRequest(next);
                break;
            }
        }
        DurationField.ResetSessionStatics();
        OverlayBuffer.StripAnalysis();
        _formattedReport = null;
        _presentation = null;
    }

    /// <summary>One atomic immutable snapshot consumed by overlay-scope capture.</summary>
    internal static bool TryGetRequest(out double startOffsetSeconds,
        out double spanSeconds, out int version)
    {
        var request = ReadRequest();
        startOffsetSeconds = request.StartOffsetSeconds;
        spanSeconds = request.SpanSeconds;
        version = request.Version;
        return request.Open;
    }

    internal static bool RequestMatches(int version)
    {
        var request = ReadRequest();
        return request.Open && request.Version == version;
    }

    internal static int BeginAnalysisPass(int version)
    {
        while (RequestMatches(version))
        {
            var current = Volatile.Read(ref _progress);
            int pass = current.Version == version ? current.Pass + 1 : 1;
            var next = new ProgressState(
                version, pass, 0, AnalysisPhase.Propagating, true);
            if (ReferenceEquals(
                    Interlocked.CompareExchange(ref _progress, next, current), current))
                return pass;
        }
        return 0;
    }

    internal static void ReportAnalysisProgress(
        int version, int pass, double fraction, AnalysisPhase phase)
    {
        if (!RequestMatches(version)) return;
        double clamped = Math.Clamp(fraction, 0, 1);
        while (true)
        {
            var current = Volatile.Read(ref _progress);
            if (current.Version != version || current.Pass != pass || !current.Running)
                return;
            var next = current with
            {
                Fraction = Math.Max(current.Fraction, clamped),
                Phase = phase,
            };
            if (ReferenceEquals(
                    Interlocked.CompareExchange(ref _progress, next, current), current))
                return;
        }
    }

    internal static void CompleteAnalysisPass(int version, int pass)
    {
        if (!RequestMatches(version)) return;
        while (true)
        {
            var current = Volatile.Read(ref _progress);
            if (current.Version != version || current.Pass != pass) return;
            var next = current with
            {
                Fraction = 1,
                Phase = AnalysisPhase.Complete,
                Running = false,
            };
            if (ReferenceEquals(
                    Interlocked.CompareExchange(ref _progress, next, current), current))
                return;
        }
    }

    internal static AnalysisProgress ReadAnalysisProgress(int version)
    {
        var progress = Volatile.Read(ref _progress);
        return progress.Version == version
            ? new(progress.Version, progress.Pass, progress.Fraction,
                progress.Phase, progress.Running)
            : new(version, 0, 0, AnalysisPhase.Waiting, false);
    }

    internal static string WindowTitle(string? referenceBodyId) =>
        string.IsNullOrEmpty(referenceBodyId)
            ? "Orbit Analysis###WhiskerDynamicsOrbitAnalysis"
            : $"Orbit Analysis — {referenceBodyId}###WhiskerDynamicsOrbitAnalysis";

    private static string? CompletedReferenceBodyForTitle()
    {
        var controlled = KSA.Program.ControlledVehicle;
        if (controlled is null) return null;
        var request = ReadRequest();
        var samples = OverlayBuffer.Read(controlled.Id);
        return samples?.AnalysisRequestVersion == request.Version
            ? samples.Analysis?.BodyId
            : null;
    }

    internal static void SetInterval(double startOffsetSeconds, double spanSeconds)
    {
        double clampedStart = Math.Clamp(startOffsetSeconds, 0.0, MaximumIntervalSeconds);
        double clampedSpan = Math.Clamp(spanSeconds, MinimumSpanSeconds, MaximumIntervalSeconds);
        while (true)
        {
            var current = ReadRequest();
            var next = current with
            {
                StartOffsetSeconds = clampedStart,
                SpanSeconds = clampedSpan,
                Version = current.Version + 1,
            };
            if (ReferenceEquals(Interlocked.CompareExchange(ref _request, next, current), current))
            {
                NotifyPredictionRequest(next);
                break;
            }
        }
        _formattedReport = null;
    }
    internal static void ResetSessionStatics()
    {
        var current = ReadRequest();
        var reset = new Request(false, 0.0, 7 * SecondsPerDay,
            current.Version + 1);
        Volatile.Write(ref _request, reset);
        NotifyPredictionRequest(reset);
        DurationField.ResetSessionStatics();
        _errors = 0;
        _firstDrawLogged = false;
        _formattedReport = null;
        _formattedVersion = -1;
        _presentation = null;
        _periapsisPlot = [];
        _apoapsisPlot = [];
        _eccentricityPlot = [];
        _inclinationPlot = [];
        _status = "";
        Volatile.Write(ref _progress,
            new ProgressState(0, 0, 0, AnalysisPhase.Waiting, false));
    }

    public static void Draw()
    {
        if (!ReadRequest().Open || _errors >= 3) return;
        try
        {
            if (!ModServices.Enabled || !ModServices.EnsureBound(out _)) return;
            UiTheme.PrepareWindow(680f, 780f, 620f, 420f);
            bool open = true;
            bool visible = ImGui.Begin(
                WindowTitle(CompletedReferenceBodyForTitle()), ref open);
            try
            {
                if (!open || !visible) return;
                UiTheme.MutedText(
                    "Actual no-burn trajectory and sampled mission metrics.");
                ImGui.SeparatorText("Analysis interval"u8);
                DrawIntervalControls();

                var controlled = KSA.Program.ControlledVehicle;
                if (controlled is null)
                {
                    DrawAnalysisState(0, "no controlled vessel");
                    return;
                }
                TryGetRequest(out double requestedStart, out double requestedSpan,
                    out int requestVersion);
                var progress = ReadAnalysisProgress(requestVersion);
                var samples = OverlayBuffer.Read(controlled.Id);
                if (samples is null || samples.AnalysisRequestVersion != requestVersion)
                {
                    DrawAnalysisState((float)progress.Fraction,
                        $"{ProgressLabel(progress)} | no completed report yet");
                    return;
                }
                var report = samples.Analysis;
                if (report is null)
                {
                    DrawAnalysisState((float)progress.Fraction,
                        progress.Running
                            ? $"{ProgressLabel(progress)} | no completed report yet"
                            : $"{samples.ParentId}: "
                                + $"{samples.AnalysisUnavailableReason ?? "analysis unavailable"}");
                    return;
                }
                if (!ReferenceEquals(report, _formattedReport)
                    || _formattedVersion != requestVersion)
                {
                    _formattedReport = report;
                    _formattedVersion = requestVersion;
                    _presentation = OrbitAnalysisPresentationModel.Create(
                        report, requestedStart, requestedSpan);
                    BuildPlots(report);
                }
                DrawDashboard(_presentation!, progress);
            }
            finally
            {
                ImGui.End();
                if (!open) Close();
            }
            if (!_firstDrawLogged)
            {
                _firstDrawLogged = true;
                ModLog.Info("orbit analyser panel drawn (first frame)");
            }
        }
        catch (Exception e)
        {
            _errors++;
            if (_errors >= 3) Close();
            ModLog.Error($"orbit analyser panel: {e}");
        }
    }

    private static void DrawIntervalControls()
    {
        var request = ReadRequest();
        double start = request.StartOffsetSeconds;
        double span = request.SpanSeconds;
        bool changed = false;
        string? startError = null;
        string? spanError = null;
        if (UiLayout.BeginProperties("##analysis-interval-properties"u8,
                UiTheme.PropertyLabelWidth))
        {
            try
            {
                UiLayout.NextProperty("Start offset");
                changed = DurationField.Row("##analysisstart"u8, "analysis start", 0,
                    ref start, StartSteps, out startError, years: true);
                ImGui.SetItemTooltip("analysis interval start relative to each pass epoch; changing either field restarts the analysis; enter y/d/h/m/s; clamps to [0, 40y]"u8);
                UiLayout.NextProperty("Duration");
                changed |= DurationField.Row("##analysisspan"u8, "analysis span", 0,
                    ref span, SpanSteps, out spanError, years: true);
                ImGui.SetItemTooltip("analysis interval length, independent of the map's Orbit look-ahead; longer spans take proportionally longer; changing either field restarts the analysis; enter y/d/h/m/s; clamps to [1m, 40y] and to rails coverage actually available"u8);
            }
            finally
            {
                ImGui.EndTable();
            }
        }
        if (startError is not null) _status = startError;
        if (spanError is not null) _status = spanError;
        if (changed)
        {
            SetInterval(start, span);
            var applied = ReadRequest();
            _status = $"interval T+{TimeDisplayKernel.FormatDuration(applied.StartOffsetSeconds, years: true)}"
                + $" for {TimeDisplayKernel.FormatDuration(applied.SpanSeconds, years: true)}";
        }
    }

    private static void DrawAnalysisState(float fraction, string state)
    {
        ImGui.SeparatorText("Analysis state"u8);
        ImGui.Text(state);
        ImGui.ProgressBar(fraction, overlay: "");
        if (_status.Length > 0)
            ImGui.TextWrapped($"Last change: {_status}");
    }

    private static string ProgressLabel(AnalysisProgress progress)
    {
        if (progress.Pass == 0) return "waiting to start";
        if (!progress.Running) return $"pass {progress.Pass} complete";
        string phase = progress.Phase switch
        {
            AnalysisPhase.Propagating => "propagating trajectory",
            AnalysisPhase.Sampling => "sampling orbit",
            AnalysisPhase.Reducing => "reducing elements",
            _ => "computing",
        };
        return $"pass {progress.Pass} | {phase} | {progress.Fraction:P0}";
    }

    private static void DrawDashboard(
        OrbitDashboardPresentation presentation, AnalysisProgress progress)
    {
        DrawAnalysisState((float)progress.Fraction,
            $"{ProgressLabel(progress)} | last result covered {presentation.CoveredInterval}");
        ImGui.Text("Report below is from the last completed pass");
        ImGui.Text($"Reference body: {presentation.BodyId}");
        ImGui.Text(presentation.Description);
        ImGui.Text($"Requested {presentation.RequestedInterval}");

        if (presentation.Warning is { } warning)
        {
            ImGui.SeparatorText("Surface warning"u8);
            ImGui.Text(warning.Text.Replace(" | ", "\n"));
        }

        ImGui.SeparatorText("Mission summary"u8);
        DrawSummary(presentation.Summary);
        DrawTrends();

        if (ImGui.CollapsingHeader("Elements"u8, ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.Text("Osculating at interval start | time-weighted mean | sampled range");
            DrawStatisticTable(presentation.Elements);
        }
        if (ImGui.CollapsingHeader("Periods and precession"u8,
                ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.Text("Periods are measured event-to-event on the n-body path");
            DrawValueTable("##periods"u8, presentation.Periods);
            ImGui.Text("Secular rates are linear fits of sampled osculating elements");
            DrawValueTable("##precession"u8, presentation.Precession);
        }
        if (ImGui.CollapsingHeader("Ground-track recurrence"u8))
            DrawValueTable("##groundtrack"u8, presentation.GroundTrack);
        if (ImGui.CollapsingHeader("Data quality and limits"u8,
                ImGuiTreeNodeFlags.DefaultOpen))
        {
            ImGui.Text($"Requested interval: {presentation.RequestedInterval}");
            ImGui.Text($"Actual covered interval: {presentation.CoveredInterval}");
            foreach (string limit in presentation.Limits) ImGui.Text($"- {limit}");
        }
    }

    private static void DrawSummary(IReadOnlyList<OrbitDashboardMetric> metrics)
    {
        if (!ImGui.BeginTable("##missionsummary"u8, 3,
                ImGuiTableFlags.SizingStretchSame | ImGuiTableFlags.BordersInnerV))
            return;
        try
        {
            for (int i = 0; i < metrics.Count; i++)
            {
                if (i % 3 == 0) ImGui.TableNextRow();
                ImGui.TableNextColumn();
                ImGui.Text(metrics[i].Label);
                ImGui.Text(metrics[i].Value);
            }
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    private static void DrawStatisticTable(
        IReadOnlyList<OrbitDashboardStatisticRow> rows)
    {
        if (!ImGui.BeginTable("##elementstats"u8, 4,
                ImGuiTableFlags.RowBg | ImGuiTableFlags.BordersInnerH
                | ImGuiTableFlags.SizingStretchProp))
            return;
        try
        {
            ImGui.TableSetupColumn("Element");
            ImGui.TableSetupColumn("At start");
            ImGui.TableSetupColumn("Mean");
            ImGui.TableSetupColumn("Range");
            ImGui.TableHeadersRow();
            foreach (var row in rows)
            {
                ImGui.TableNextRow();
                ImGui.TableNextColumn();
                ImGui.Text(row.Label);
                ImGui.TableNextColumn();
                UiTheme.RightAlignedText(row.Current);
                ImGui.TableNextColumn();
                UiTheme.RightAlignedText(row.Mean);
                ImGui.TableNextColumn();
                UiTheme.RightAlignedText(row.Range);
            }
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    private static void DrawValueTable(
        ReadOnlySpan<byte> id, IReadOnlyList<OrbitDashboardValueRow> rows)
    {
        if (!ImGui.BeginTable(id, 3,
                ImGuiTableFlags.RowBg | ImGuiTableFlags.BordersInnerH
                | ImGuiTableFlags.SizingStretchProp))
            return;
        try
        {
            ImGui.TableSetupColumn("Measure");
            ImGui.TableSetupColumn("Value");
            ImGui.TableSetupColumn("Evidence");
            foreach (var row in rows)
            {
                ImGui.TableNextRow();
                ImGui.TableNextColumn();
                ImGui.Text(row.Label);
                ImGui.TableNextColumn();
                UiTheme.RightAlignedText(row.Value);
                ImGui.TableNextColumn();
                ImGui.Text(row.Evidence);
            }
        }
        finally
        {
            ImGui.EndTable();
        }
    }

    private static void BuildPlots(OrbitAnalysisReport report)
    {
        _periapsisPlot = PlotSeries(report.Trend,
            point => point.PeriapsisAltitudeMeters / 1000);
        _apoapsisPlot = PlotSeries(report.Trend,
            point => point.ApoapsisAltitudeMeters / 1000);
        _eccentricityPlot = PlotSeries(report.Trend,
            point => point.Eccentricity);
        _inclinationPlot = PlotSeries(report.Trend,
            point => point.InclinationRadians * 180 / Math.PI);
    }

    private static float[] PlotSeries(
        IReadOnlyList<OrbitTrendPoint> trend, Func<OrbitTrendPoint, double?> selector)
    {
        var values = new float[trend.Count];
        for (int i = 0; i < trend.Count; i++)
        {
            double? selected = selector(trend[i]);
            if (selected is not { } value || !double.IsFinite(value)
                || Math.Abs(value) > float.MaxValue)
                return [];
            values[i] = (float)value;
        }
        return values;
    }

    private static void DrawTrends()
    {
        if (!ImGui.CollapsingHeader("Trend plots"u8, ImGuiTreeNodeFlags.DefaultOpen))
            return;
        ImGui.Text("Chronological trend only; numeric summary values remain authoritative");
        DrawPlot("##petrend"u8, "Pe altitude (km)", _periapsisPlot);
        DrawPlot("##aptrend"u8, "Ap altitude (km)", _apoapsisPlot);
        DrawPlot("##etrend"u8, "Eccentricity", _eccentricityPlot);
        DrawPlot("##itrend"u8, "Inclination (deg)", _inclinationPlot);
    }

    private static void DrawPlot(
        ReadOnlySpan<byte> id, string label, ReadOnlySpan<float> values)
    {
        if (values.Length == 0)
        {
            ImGui.Text($"{label}: undefined on part or all of this interval");
            return;
        }
        float width = Math.Max(1, ImGui.GetContentRegionAvail().X);
        ImGui.PlotLines(id, values, overlayText: label,
            graphSize: new float2(width, 72f));
    }

}
