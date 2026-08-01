namespace WhiskerDynamics.Compatibility.Patching;

/// <summary>Enum values compiled into patch IL and therefore part of the game API
/// contract even though reflection-based member validation cannot see their drift.</summary>
internal static class EnumContract
{
    private static readonly Action<List<string>>[] Checks =
    [
        m => Check<KSA.Situation, byte>(nameof(KSA.Situation.Freefall), 1, m),
        m => Check<KSA.CameraMode, int>(nameof(KSA.CameraMode.Map), 2, m),
        m => Check<KSA.PatchTransition, int>(nameof(KSA.PatchTransition.Final), 1, m),
        m => Check<KSA.PatchTransition, int>(nameof(KSA.PatchTransition.Impact), 5, m),
        m => Check<KSA.ThrusterMapFlags, int>(nameof(KSA.ThrusterMapFlags.TranslateForward), 0x80, m),
    ];

    public static int CheckCount => Checks.Length;

    public static bool Validate(out List<string> mismatches)
    {
        mismatches = [];
        foreach (var check in Checks) check(mismatches);
        return mismatches.Count == 0;
    }

    private static void Check<TEnum, TValue>(string memberName, TValue expectedValue, List<string> mismatches)
        where TEnum : struct, Enum
    {
        var field = typeof(TEnum).GetField(memberName);
        object? runtimeValue = field?.GetRawConstantValue();
        if (!Equals(runtimeValue, expectedValue))
            mismatches.Add($"{typeof(TEnum).Name}.{memberName}: game declares {runtimeValue ?? "<missing>"}, "
                + $"compatibility contract expects {expectedValue}");
    }
}
