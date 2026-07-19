using System.Runtime.CompilerServices;
using WhiskerDynamics.Compatibility.Patching;

namespace WhiskerDynamics.Mod.Tests.Patching;

/// <summary>
/// Offline tests for the reflection validator. All specs target the dummy types below,
/// never the game assembly — the validator's job is pure shape-matching, so dummies
/// exercise every code path (present/missing/moved/ambiguous/static-mismatch/byref/
/// nullable) without loading KSA.dll.
/// </summary>
public class PatchValidatorTests
{
    // --- dummy target types -------------------------------------------------------

#pragma warning disable CA1822 // instance semantics are the point of these dummies
    private class Dummy
    {
        public int PublicField;
        private double _privateField = 0;
        public int? NullableStructField = null;
        public static string? StaticField = null;

        public string Name { get; set; } = "";
        private static bool StaticProp { get; set; }
        public int GetOnly => PublicField;
        public int SetOnly { set => PublicField = value; }

        [IndexerName("Indexed")]
        public int this[int index]
        {
            get => index;
            set => PublicField = value;
        }

        public Dummy() { }
        public Dummy(int a, string b) { PublicField = a; Name = b; }

        public void UniqueMethod() { }
        private int PrivateMethod(int x) => x + (int)_privateField;
        public static void StaticMethod(double d) { }
        public void ByRefMethod(ref int x, in double y) { }
        public bool GenericMethod<TFirst, TSecond>(out TSecond value)
        {
            value = default!;
            return true;
        }

        public void Overloaded() { }
        public void Overloaded(int x) { }
    }
#pragma warning restore CA1822

    private static TargetSpec Spec(
        string name, MemberKind kind, Type[]? parameters = null, Type? expected = null,
        bool? isStatic = null, PropertyAccessors requiredAccessors = PropertyAccessors.None)
        => new($"Dummy.{name}", typeof(Dummy), name, kind, parameters, expected,
            IsStatic: isStatic, RequiredAccessors: requiredAccessors);

    private static List<string> Validate(params TargetSpec[] specs)
    {
        PatchValidator.ValidateAll(specs, out var mismatches);
        return mismatches;
    }

    // --- methods ------------------------------------------------------------------

    [Fact]
    public void Method_with_exact_signature_passes()
    {
        Assert.True(PatchValidator.ValidateAll(
            [Spec("UniqueMethod", MemberKind.Method, Type.EmptyTypes, typeof(void))],
            out var mismatches));
        Assert.Empty(mismatches);
    }

    [Fact]
    public void Private_method_is_found()
    {
        Assert.Empty(Validate(
            Spec("PrivateMethod", MemberKind.Method, [typeof(int)], typeof(int))));
    }

    [Fact]
    public void Missing_method_is_reported()
    {
        var mismatches = Validate(Spec("Gone", MemberKind.Method, Type.EmptyTypes, typeof(void)));
        Assert.Single(mismatches);
        Assert.Contains("Dummy.Gone", mismatches[0]);
    }

    [Fact]
    public void Method_with_changed_parameters_is_reported()
    {
        Assert.Single(Validate(
            Spec("PrivateMethod", MemberKind.Method, [typeof(string)], typeof(int))));
    }

    [Fact]
    public void Method_with_changed_return_type_is_reported()
    {
        var mismatches = Validate(
            Spec("PrivateMethod", MemberKind.Method, [typeof(int)], typeof(string)));
        Assert.Single(mismatches);
        Assert.Contains("return type", mismatches[0]);
    }

    [Fact]
    public void Null_expected_type_skips_return_check()
    {
        Assert.Empty(Validate(Spec("PrivateMethod", MemberKind.Method, [typeof(int)])));
    }

    [Fact]
    public void Byref_and_in_parameters_match_via_MakeByRefType()
    {
        Assert.Empty(Validate(Spec("ByRefMethod", MemberKind.Method,
            [typeof(int).MakeByRefType(), typeof(double).MakeByRefType()], typeof(void))));
    }

    [Fact]
    public void Null_parameters_match_unique_name()
    {
        Assert.Empty(Validate(Spec("UniqueMethod", MemberKind.Method, null, typeof(void))));
    }

    [Fact]
    public void Null_parameters_with_overloads_reports_ambiguity()
    {
        var mismatches = Validate(Spec("Overloaded", MemberKind.Method, null, typeof(void)));
        Assert.Single(mismatches);
        Assert.Contains("ambiguous", mismatches[0]);
    }

    [Fact]
    public void Unique_name_can_pin_generic_arity_and_parameter_count()
    {
        var exact = new TargetSpec("Dummy.GenericMethod", typeof(Dummy), "GenericMethod",
            MemberKind.Method, null, typeof(bool), GenericParameterCount: 2,
            ParameterCount: 1, OutParameterCount: 1);
        Assert.Empty(Validate(exact));
        Assert.Single(Validate(exact with { GenericParameterCount = 1 }));
        Assert.Single(Validate(exact with { ParameterCount = 2 }));
        Assert.Single(Validate(exact with { OutParameterCount = 0 }));
    }

    // --- static vs instance -------------------------------------------------------

    [Fact]
    public void Static_method_with_exact_signature_passes()
    {
        Assert.Empty(Validate(
            Spec("StaticMethod", MemberKind.StaticMethod, [typeof(double)], typeof(void))));
    }

    [Fact]
    public void Instance_method_declared_static_is_reported()
    {
        Assert.Single(Validate(
            Spec("UniqueMethod", MemberKind.StaticMethod, Type.EmptyTypes, typeof(void))));
    }

    [Fact]
    public void Static_method_declared_instance_is_reported()
    {
        Assert.Single(Validate(
            Spec("StaticMethod", MemberKind.Method, [typeof(double)], typeof(void))));
    }

    // --- fields -------------------------------------------------------------------

    [Fact]
    public void Field_with_exact_type_passes_public_private_static_and_nullable()
    {
        Assert.Empty(Validate(
            Spec("PublicField", MemberKind.Field, null, typeof(int), isStatic: false),
            Spec("_privateField", MemberKind.Field, null, typeof(double), isStatic: false),
            Spec("StaticField", MemberKind.Field, null, typeof(string), isStatic: true),
            Spec("NullableStructField", MemberKind.Field, null, typeof(int?), isStatic: false)));
    }

    [Fact]
    public void Instance_field_declared_static_is_reported()
    {
        var mismatches = Validate(
            Spec("PublicField", MemberKind.Field, null, typeof(int), isStatic: true));
        Assert.Single(mismatches);
        Assert.Contains("staticness changed", mismatches[0]);
    }

    [Fact]
    public void Static_field_declared_instance_is_reported()
    {
        var mismatches = Validate(
            Spec("StaticField", MemberKind.Field, null, typeof(string), isStatic: false));
        Assert.Single(mismatches);
        Assert.Contains("staticness changed", mismatches[0]);
    }

    [Fact]
    public void Field_without_declared_staticness_is_reported()
    {
        var mismatches = Validate(
            Spec("PublicField", MemberKind.Field, null, typeof(int)));
        Assert.Single(mismatches);
        Assert.Contains("staticness is not specified", mismatches[0]);
    }

    [Fact]
    public void Missing_field_is_reported()
    {
        var mismatches = Validate(
            Spec("GoneField", MemberKind.Field, null, typeof(int), isStatic: false));
        Assert.Single(mismatches);
        Assert.Contains("field missing", mismatches[0]);
    }

    [Fact]
    public void Field_with_changed_type_is_reported()
    {
        var mismatches = Validate(
            Spec("PublicField", MemberKind.Field, null, typeof(long), isStatic: false));
        Assert.Single(mismatches);
        Assert.Contains("field type", mismatches[0]);
    }

    [Fact]
    public void Nullable_struct_field_does_not_match_bare_type()
    {
        Assert.Single(Validate(
            Spec("NullableStructField", MemberKind.Field, null, typeof(int), isStatic: false)));
    }

    // --- properties ---------------------------------------------------------------

    [Fact]
    public void Property_with_exact_type_passes_including_private_static()
    {
        Assert.Empty(Validate(
            Spec("Name", MemberKind.Property, null, typeof(string), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter | PropertyAccessors.Setter),
            Spec("StaticProp", MemberKind.Property, null, typeof(bool), isStatic: true,
                requiredAccessors: PropertyAccessors.Getter | PropertyAccessors.Setter),
            Spec("GetOnly", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter),
            Spec("SetOnly", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Setter)));
    }

    [Fact]
    public void Instance_property_declared_static_is_reported()
    {
        var mismatches = Validate(
            Spec("Name", MemberKind.Property, null, typeof(string), isStatic: true,
                requiredAccessors: PropertyAccessors.Getter));
        Assert.Single(mismatches);
        Assert.Contains("staticness changed", mismatches[0]);
    }

    [Fact]
    public void Static_property_declared_instance_is_reported()
    {
        var mismatches = Validate(
            Spec("StaticProp", MemberKind.Property, null, typeof(bool), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter));
        Assert.Single(mismatches);
        Assert.Contains("staticness changed", mismatches[0]);
    }

    [Fact]
    public void Missing_required_getter_is_reported()
    {
        var mismatches = Validate(
            Spec("SetOnly", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter));
        Assert.Single(mismatches);
        Assert.Contains("getter missing", mismatches[0]);
    }

    [Fact]
    public void Missing_required_setter_is_reported()
    {
        var mismatches = Validate(
            Spec("GetOnly", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Setter));
        Assert.Single(mismatches);
        Assert.Contains("setter missing", mismatches[0]);
    }

    [Fact]
    public void Property_without_required_accessors_is_reported()
    {
        var mismatches = Validate(
            Spec("Name", MemberKind.Property, null, typeof(string), isStatic: false));
        Assert.Single(mismatches);
        Assert.Contains("accessors are not specified", mismatches[0]);
    }

    [Fact]
    public void Indexer_does_not_satisfy_an_ordinary_property_contract()
    {
        var mismatches = Validate(
            Spec("Indexed", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter));
        Assert.Single(mismatches);
        Assert.Contains("non-indexed property missing", mismatches[0]);
    }

    [Fact]
    public void Missing_property_is_reported()
    {
        var mismatches = Validate(
            Spec("GoneProp", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter));
        Assert.Single(mismatches);
        Assert.Contains("property missing", mismatches[0]);
    }

    [Fact]
    public void Property_with_changed_type_is_reported()
    {
        var mismatches = Validate(
            Spec("Name", MemberKind.Property, null, typeof(int), isStatic: false,
                requiredAccessors: PropertyAccessors.Getter));
        Assert.Single(mismatches);
        Assert.Contains("property type", mismatches[0]);
    }

    // --- constructors -------------------------------------------------------------

    [Fact]
    public void Constructor_with_exact_parameters_passes()
    {
        Assert.Empty(Validate(
            Spec(".ctor", MemberKind.Constructor, [typeof(int), typeof(string)]),
            Spec(".ctor", MemberKind.Constructor, Type.EmptyTypes),
            Spec(".ctor", MemberKind.Constructor))); // null parameters = parameterless
    }

    [Fact]
    public void Missing_constructor_is_reported()
    {
        var mismatches = Validate(Spec(".ctor", MemberKind.Constructor, [typeof(Guid)]));
        Assert.Single(mismatches);
        Assert.Contains("constructor missing", mismatches[0]);
    }

    // --- aggregation --------------------------------------------------------------

    [Fact]
    public void One_bad_spec_fails_the_whole_set_and_all_mismatches_are_collected()
    {
        bool ok = PatchValidator.ValidateAll(
            [
                Spec("UniqueMethod", MemberKind.Method, Type.EmptyTypes, typeof(void)),
                Spec("GoneField", MemberKind.Field, null, typeof(int), isStatic: false),
                Spec("GoneProp", MemberKind.Property, null, typeof(int), isStatic: false,
                    requiredAccessors: PropertyAccessors.Getter),
            ],
            out var mismatches);
        Assert.False(ok);
        Assert.Equal(2, mismatches.Count);
    }

    [Fact]
    public void Empty_spec_set_passes()
    {
        Assert.True(PatchValidator.ValidateAll([], out var mismatches));
        Assert.Empty(mismatches);
    }

    // --- type-level drift -----------------------------------------------------------

    /// <summary>Fakes a game type whose member reflection blows up (the shape of
    /// TypeInitializationException-style drift when a whole type vanishes).</summary>
    private sealed class ThrowingType() : System.Reflection.TypeDelegator(typeof(Dummy))
    {
        public override System.Reflection.MethodInfo[] GetMethods(System.Reflection.BindingFlags bindingAttr)
            => throw new TypeInitializationException(typeof(Dummy).FullName, null);
    }

    [Fact]
    public void Type_level_drift_propagates_out_of_the_validator()
    {
        // Pins the containment contract: the validator does NOT swallow type-level
        // failures into the mismatch list — ModMain's validate+apply try/catch is the
        // layer that turns them into DisabledIncompatible.
        var spec = new TargetSpec("Drift.UniqueMethod", new ThrowingType(),
            "UniqueMethod", MemberKind.Method, null, typeof(void));
        Assert.Throws<TypeInitializationException>(
            () => PatchValidator.ValidateAll([spec], out _));
    }

    [Fact]
    public void Panel_and_gameplay_registries_live_in_separate_classes()
    {
        // Pins the isolation boundary: the CLR runs static initializers per CLASS, so the
        // panel registry (touched OUTSIDE ModMain's drift guard) must not share a class
        // with the gameplay registry (whose first touch must be INSIDE the guard, where a
        // vanished game type's TypeInitializationException degrades to DisabledIncompatible).
        // Metadata checks only — reading the field VALUES would run the initializers, which
        // resolve game types that are deliberately absent from the offline test host.
        var asm = typeof(PatchValidator).Assembly;
        var panel = asm.GetType("WhiskerDynamics.Compatibility.Patching.PanelTargets");
        var gameplay = asm.GetType("WhiskerDynamics.Compatibility.Patching.GameplayTargets");
        Assert.NotNull(panel);
        Assert.NotNull(gameplay);
        Assert.NotEqual(panel, gameplay);
        Assert.NotNull(panel.GetField("Panel"));
        Assert.Null(panel.GetField("Gameplay"));
        Assert.NotNull(gameplay.GetField("Gameplay"));
        Assert.Null(gameplay.GetField("Panel"));
    }
}
