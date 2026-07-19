using System.Globalization;
using System.Text;

namespace WhiskerDynamics.Mod.Ui;

/// <summary>Human-readable duration text for the panels' time fields: compact
/// descending y/d/h/m/s ("3d 4h 5m 6.25s") and the matching parser. Days is the
/// default largest unit — plan windows and burn leads read best that way — while
/// window-scale controls opt into years (365 display days, the same convention the
/// horizon caps use; nothing calendar-aware). The parser accepts "y" everywhere
/// regardless: a user may type "2y" into any duration field. KSA-free by the kernel
/// convention; invariant culture throughout (panel text must not follow the OS
/// locale).</summary>
public static class TimeDisplayKernel
{
    /// <summary>Formatting quantum: everything is carried on an integer millisecond
    /// grid. Rounding the seconds component on its own would render "60s" whenever
    /// sub-millisecond residue rounds up (59.9999 s must carry into "1m").</summary>
    private const long MsPerSecond = 1_000;
    private const long MsPerMinute = 60 * MsPerSecond;
    private const long MsPerHour = 60 * MsPerMinute;
    private const long MsPerDay = 24 * MsPerHour;
    private const long MsPerYear = 365 * MsPerDay; // display years — see class doc

    /// <summary>Magnitudes whose millisecond count does not fit the long grid
    /// (about 292 million years) fall back to plain seconds — no panel value
    /// legitimately gets there, but the formatter must not overflow on one.</summary>
    private const double MaxGridSeconds = long.MaxValue / (double)MsPerSecond;

    /// <summary>Compact descending-unit rendering: "3d 4h 5m 6.25s", "1h 30m",
    /// "45.5s", "0s". Zero components are omitted wherever they fall; zero itself
    /// is "0s". Seconds carry up to three decimals with trailing zeros trimmed.
    /// Negative values render as a single leading "-" before the absolute form.
    /// <paramref name="years"/> promotes whole 365-day years into a leading "y"
    /// component ("2y 30d") — the window-scale controls' vocabulary; day-scale
    /// fields keep the default (days largest).</summary>
    public static string FormatDuration(double seconds, bool years = false)
    {
        if (!double.IsFinite(seconds))
            return seconds.ToString(CultureInfo.InvariantCulture);
        double abs = Math.Abs(seconds);
        if (abs >= MaxGridSeconds)
            return FormattableString.Invariant($"{seconds:0.###}s");
        long ms = (long)Math.Round(abs * MsPerSecond);
        if (ms == 0) return "0s"; // covers -0.0 and sub-half-millisecond negatives too
        var sb = new StringBuilder(24);
        if (seconds < 0) sb.Append('-');
        int prefix = sb.Length; // the sign is not a component: no space after it
        if (years)
        {
            Append(ms / MsPerYear, 'y');
            Append(ms / MsPerDay % 365, 'd');
        }
        else
        {
            Append(ms / MsPerDay, 'd');
        }
        Append(ms / MsPerHour % 24, 'h');
        Append(ms / MsPerMinute % 60, 'm');
        long secMs = ms % MsPerMinute;
        if (secMs > 0)
        {
            if (sb.Length > prefix) sb.Append(' ');
            sb.Append((secMs / (double)MsPerSecond).ToString("0.###", CultureInfo.InvariantCulture))
              .Append('s');
        }
        return sb.ToString();

        void Append(long value, char unit)
        {
            if (value == 0) return;
            if (sb.Length > prefix) sb.Append(' ');
            sb.Append(value).Append(unit);
        }
    }

    /// <summary>Whole-second countdown text for live map markers. Once a larger unit
    /// appears, every lower unit remains present so the label width changes only when
    /// that larger unit rolls over.</summary>
    public static string FormatCountdown(double seconds, bool years = false)
    {
        if (!double.IsFinite(seconds)) return FormatDuration(seconds, years);
        double abs = Math.Abs(seconds);
        if (abs >= MaxGridSeconds) return FormatDuration(seconds, years);
        long totalSeconds = (long)Math.Round(abs, MidpointRounding.AwayFromZero);
        if (totalSeconds == 0) return "0s";

        long totalMinutes = totalSeconds / 60;
        long totalHours = totalMinutes / 60;
        long totalDays = totalHours / 24;
        long totalYears = totalDays / 365;
        long secondsPart = totalSeconds % 60;
        long minutesPart = totalMinutes % 60;
        long hoursPart = totalHours % 24;
        long daysPart = totalDays % 365;
        var sb = new StringBuilder(28);
        if (seconds < 0) sb.Append('-');
        int prefix = sb.Length;

        if (years && totalYears > 0)
        {
            Append(totalYears, 'y');
            Append(daysPart, 'd');
            Append(hoursPart, 'h');
            Append(minutesPart, 'm');
        }
        else if (totalDays > 0)
        {
            Append(totalDays, 'd');
            Append(hoursPart, 'h');
            Append(minutesPart, 'm');
        }
        else if (totalHours > 0)
        {
            Append(totalHours, 'h');
            Append(minutesPart, 'm');
        }
        else if (totalMinutes > 0)
        {
            Append(totalMinutes, 'm');
        }
        Append(secondsPart, 's');
        return sb.ToString();

        void Append(long value, char unit)
        {
            if (sb.Length > prefix) sb.Append(' ');
            sb.Append(value).Append(unit);
        }
    }

    /// <summary>Parses what <see cref="FormatDuration"/> emits, plus the forms a
    /// hand-typing user reaches for: descending unit tokens with optional whitespace
    /// ("3d 4h 5m 6.25s", "3d4h", "90m", "1.5d", "2y 30d" — y = 365 days), each unit
    /// at most once, decimals allowed on any component, units case-insensitive. A
    /// bare number with no unit is SECONDS. One leading "-" negates the whole value;
    /// per-component signs are rejected, as are unknown units, repeated or
    /// out-of-order units, and empty or trailing garbage. Round-trips FormatDuration
    /// within its millisecond grid.</summary>
    public static bool TryParseDuration(string? text, out double seconds)
    {
        seconds = 0;
        if (text is null) return false;
        ReadOnlySpan<char> s = text.AsSpan().Trim();
        if (s.IsEmpty) return false;
        double sign = 1;
        if (s[0] == '-')
        {
            sign = -1;
            s = s[1..].TrimStart();
            if (s.IsEmpty) return false;
        }
        // Bare number (the whole remaining text) means seconds. Unit tokens never
        // reach this branch: any unit letter fails the plain numeric parse.
        if (double.TryParse(s, NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture,
                out double bare))
        {
            if (!double.IsFinite(bare)) return false;
            seconds = sign * bare;
            return true;
        }
        double total = 0;
        int lastRank = -1;
        int i = 0;
        while (i < s.Length)
        {
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
            if (i >= s.Length) break;
            int start = i;
            while (i < s.Length && (char.IsAsciiDigit(s[i]) || s[i] == '.')) i++;
            if (i == start) return false; // unit without a number, or garbage character
            if (!double.TryParse(s[start..i], NumberStyles.AllowDecimalPoint,
                    CultureInfo.InvariantCulture, out double value))
                return false; // e.g. "1.2.3"
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
            if (i >= s.Length) return false; // trailing number without a unit
            (int rank, double scale) = char.ToLowerInvariant(s[i]) switch
            {
                'y' => (0, 365.0 * 86400.0),
                'd' => (1, 86400.0),
                'h' => (2, 3600.0),
                'm' => (3, 60.0),
                's' => (4, 1.0),
                _ => (-1, 0.0),
            };
            if (rank < 0) return false; // unknown unit
            if (rank <= lastRank) return false; // repeated or ascending unit
            lastRank = rank;
            total += value * scale;
            i++;
        }
        if (lastRank < 0 || !double.IsFinite(total)) return false;
        seconds = sign * total;
        return true;
    }
}
