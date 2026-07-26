using System.Text.Json;

namespace WhiskerDynamics.Core;

internal static class BodyFixedToModelRotationJson
{
    internal static BodyFixedToModelRotation? Parse(in JsonElement matrix)
    {
        if (matrix.ValueKind == JsonValueKind.Undefined)
            return null;
        if (matrix.ValueKind != JsonValueKind.Array || matrix.GetArrayLength() != 3)
            throw new FormatException(
                "gravity_model.body_fixed_to_model must be a 3x3 numeric matrix");

        JsonElement[] rows = matrix.EnumerateArray().ToArray();
        return new BodyFixedToModelRotation(
            ParseRow(rows[0], 0),
            ParseRow(rows[1], 1),
            ParseRow(rows[2], 2));
    }

    private static Vector3d ParseRow(in JsonElement row, int index)
    {
        if (row.ValueKind != JsonValueKind.Array || row.GetArrayLength() != 3)
            throw new FormatException(
                $"gravity_model.body_fixed_to_model[{index}] must contain 3 numbers");
        JsonElement[] values = row.EnumerateArray().ToArray();
        if (!values[0].TryGetDouble(out double x)
            || !values[1].TryGetDouble(out double y)
            || !values[2].TryGetDouble(out double z))
            throw new FormatException(
                $"gravity_model.body_fixed_to_model[{index}] contains an invalid number");
        return new Vector3d(x, y, z);
    }
}
