using System.Reflection;

namespace WhiskerDynamics.Compatibility.Patching;

internal static class PatchValidator
{
    private const BindingFlags Any =
        BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;

    /// <summary>True when every spec matches the loaded game assembly exactly.
    /// Null Parameters means "match by unique name"; null ExpectedType skips the type check
    /// (used where the game type is what we assert exists, e.g. optional-parameter methods).</summary>
    public static bool ValidateAll(IEnumerable<TargetSpec> specs, out List<string> mismatches)
    {
        mismatches = [];
        foreach (var spec in specs)
        {
            switch (spec.Kind)
            {
                case MemberKind.Method:
                case MemberKind.StaticMethod:
                {
                    MethodInfo? method;
                    if (spec.Parameters is null)
                    {
                        var candidates = spec.DeclaringType
                            .GetMethods(Any)
                            .Where(m => m.Name == spec.MemberName
                                        && m.IsStatic == (spec.Kind == MemberKind.StaticMethod)
                                        && (spec.GenericParameterCount is null
                                            || m.GetGenericArguments().Length == spec.GenericParameterCount)
                                        && (spec.ParameterCount is null
                                            || m.GetParameters().Length == spec.ParameterCount)
                                        && (spec.OutParameterCount is null
                                            || m.GetParameters().Count(p => p.IsOut)
                                                == spec.OutParameterCount))
                            .ToList();
                        method = candidates.Count == 1 ? candidates[0] : null;
                        if (candidates.Count > 1) { mismatches.Add($"{spec.Key}: ambiguous ({candidates.Count} overloads)"); continue; }
                    }
                    else
                    {
                        // genericParameterCount 0: every registered method is non-generic,
                        // and some (CelestialSystem.GetIndex(int)) have a generic sibling
                        // with the SAME parameter list — the plain GetMethod(name, flags,
                        // types) overload would throw AmbiguousMatchException on those.
                        method = spec.DeclaringType.GetMethod(spec.MemberName, 0, Any, null, spec.Parameters, null);
                        if (method is not null && method.IsStatic != (spec.Kind == MemberKind.StaticMethod)) method = null;
                    }
                    if (method is null) mismatches.Add($"{spec.Key}: method missing or signature changed");
                    else if (spec.ExpectedType is not null && method.ReturnType != spec.ExpectedType)
                        mismatches.Add($"{spec.Key}: return type {method.ReturnType.Name}, expected {spec.ExpectedType.Name}");
                    break;
                }
                case MemberKind.Field:
                {
                    var field = spec.DeclaringType.GetField(spec.MemberName, Any);
                    if (field is null) mismatches.Add($"{spec.Key}: field missing");
                    else
                    {
                        ValidateStaticness(spec, field.IsStatic, "field", mismatches);
                        if (spec.ExpectedType is not null && field.FieldType != spec.ExpectedType)
                            mismatches.Add($"{spec.Key}: field type {field.FieldType.Name}, expected {spec.ExpectedType.Name}");
                    }
                    break;
                }
                case MemberKind.Property:
                {
                    var candidates = spec.DeclaringType
                        .GetProperties(Any)
                        .Where(property => property.Name == spec.MemberName
                            && property.GetIndexParameters().Length == 0)
                        .ToList();
                    var property = candidates.Count == 1 ? candidates[0] : null;
                    if (candidates.Count > 1)
                    {
                        mismatches.Add(
                            $"{spec.Key}: ambiguous ({candidates.Count} non-indexed properties)");
                        continue;
                    }
                    if (property is null)
                        mismatches.Add($"{spec.Key}: non-indexed property missing");
                    else
                    {
                        MethodInfo? getter = property.GetGetMethod(nonPublic: true);
                        MethodInfo? setter = property.GetSetMethod(nonPublic: true);
                        MethodInfo[] accessors = property.GetAccessors(nonPublic: true);
                        bool? accessorsAreStatic = accessors.Length == 0 ? null
                            : accessors.All(accessor => accessor.IsStatic) ? true
                            : accessors.All(accessor => !accessor.IsStatic) ? false
                            : null;
                        ValidateStaticness(spec, accessorsAreStatic, "property accessor", mismatches);

                        if (spec.RequiredAccessors == PropertyAccessors.None)
                            mismatches.Add($"{spec.Key}: required property accessors are not specified");
                        else
                        {
                            if ((spec.RequiredAccessors & PropertyAccessors.Getter) != 0 && getter is null)
                                mismatches.Add($"{spec.Key}: required property getter missing");
                            if ((spec.RequiredAccessors & PropertyAccessors.Setter) != 0 && setter is null)
                                mismatches.Add($"{spec.Key}: required property setter missing");
                        }

                        if (spec.ExpectedType is not null && property.PropertyType != spec.ExpectedType)
                            mismatches.Add($"{spec.Key}: property type {property.PropertyType.Name}, expected {spec.ExpectedType.Name}");
                    }
                    break;
                }
                case MemberKind.Constructor:
                {
                    if (spec.DeclaringType.GetConstructor(Any, spec.Parameters ?? Type.EmptyTypes) is null)
                        mismatches.Add($"{spec.Key}: constructor missing");
                    break;
                }
            }
        }
        return mismatches.Count == 0;
    }

    private static void ValidateStaticness(
        TargetSpec spec, bool? actualIsStatic, string memberDescription, List<string> mismatches)
    {
        if (spec.IsStatic is null)
        {
            mismatches.Add($"{spec.Key}: {memberDescription} staticness is not specified");
            return;
        }

        if (actualIsStatic != spec.IsStatic)
        {
            string expected = spec.IsStatic.Value ? "static" : "instance";
            mismatches.Add($"{spec.Key}: {memberDescription} staticness changed; expected {expected}");
        }
    }
}
