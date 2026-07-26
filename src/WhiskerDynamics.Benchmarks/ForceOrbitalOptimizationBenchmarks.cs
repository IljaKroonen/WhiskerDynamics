using BenchmarkDotNet.Attributes;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

[MemoryDiagnoser]
[ShortRunJob]
public class TrigonometryOptimizationBenchmarks
{
    private const int Batch = 1024;
    private double[] _angles = null!;

    [GlobalSetup]
    public void Setup() => _angles = Enumerable.Range(0, Batch)
        .Select(i => -1e6 + i * (2e6 / Batch) + Math.Sin(i * 0.73)).ToArray();

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double SeparateSinCos()
    {
        double sum = 0;
        foreach (double angle in _angles) sum += Math.Sin(angle) + Math.Cos(angle);
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double CombinedSinCos()
    {
        double sum = 0;
        foreach (double angle in _angles)
        {
            var pair = Math.SinCos(angle);
            sum += pair.Sin + pair.Cos;
        }
        return sum;
    }
}

[MemoryDiagnoser]
[ShortRunJob]
public class FrameSurfaceOptimizationBenchmarks
{
    private const int Batch = 256;
    private readonly StateVector _body = new(
        new Vector3d(1.2e11, -4.5e10, 7.8e9), new Vector3d(123, -456, 78));
    private readonly BodyRotation _rotation = new(
        new Vector3d(0.2, -0.3, Math.Sqrt(0.87)),
        new Vector3d(0.978, 0.061, -0.2),
        new Vector3d(0.0, 0.95, 0.31), 7.2921159e-5, 1234);
    private double[] _times = null!;

    [GlobalSetup]
    public void Setup() => _times = Enumerable.Range(0, Batch)
        .Select(i => i * 123456.789 - 8e6).ToArray();

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double RepeatedTrigAndSqrtBaseline()
    {
        double sum = 0;
        foreach (double time in _times)
        {
            var pose = BaselineSurface(_body, _rotation, time);
            sum += pose.XAxis.X + pose.YAxis.Y;
        }
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double SharedTrigProduction()
    {
        double sum = 0;
        foreach (double time in _times)
        {
            var pose = FrameKernel.Surface(_body, _rotation, time);
            sum += pose.XAxis.X + pose.YAxis.Y;
        }
        return sum;
    }

    private static FramePose BaselineSurface(StateVector body, BodyRotation rotation, double time)
    {
        if (rotation.PoleEcl.Length() == 0) throw new ArgumentException();
        double angle = rotation.AngularVelocity * (time - rotation.ReferenceTime);
        return new FramePose(body.Position,
            rotation.XAxisEcl.RotateAbout(rotation.PoleEcl, angle),
            rotation.YAxisEcl.RotateAbout(rotation.PoleEcl, angle), rotation.PoleEcl);
    }
}

[MemoryDiagnoser]
[ShortRunJob]
/// <summary>Representative low-lunar-orbit correction cost at each selectable
/// GRGM1200A truncation, batched across position, altitude, and body rotation.</summary>
public class LunarGravityFidelityBenchmarks
{
    private const int Batch = 32;
    private const double Mu = 4.9028000661637961e12;
    private Geopotential _degree50 = null!;
    private Geopotential _degree40 = null!;
    private Geopotential _degree30 = null!;
    private Geopotential _degree20 = null!;
    private Geopotential _degree10 = null!;
    private (Vector3d Position, double Time)[] _cases = null!;

    [GlobalSetup]
    public void Setup()
    {
        var rotation = new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 2.6616995e-6, 0);
        _degree50 = BenchmarkGravityModels.Lunar(rotation, 50);
        _degree40 = BenchmarkGravityModels.Lunar(rotation, 40);
        _degree30 = BenchmarkGravityModels.Lunar(rotation, 30);
        _degree20 = BenchmarkGravityModels.Lunar(rotation, 20);
        _degree10 = BenchmarkGravityModels.Lunar(rotation, 10);
        _cases = Enumerable.Range(0, Batch).Select(i =>
        {
            double longitude = i * 2 * Math.PI / Batch;
            double latitude = (i % 9 - 4) * 0.14;
            double r = 1_838_000 + (i % 4) * 50_000;
            return (new Vector3d(r * Math.Cos(latitude) * Math.Cos(longitude),
                r * Math.Cos(latitude) * Math.Sin(longitude), r * Math.Sin(latitude)), i * 54321.0);
        }).ToArray();
    }

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double Degree50() => Evaluate(_degree50);

    [Benchmark(OperationsPerInvoke = Batch)]
    public double Degree40() => Evaluate(_degree40);

    [Benchmark(OperationsPerInvoke = Batch)]
    public double Degree30() => Evaluate(_degree30);

    [Benchmark(OperationsPerInvoke = Batch)]
    public double Degree20() => Evaluate(_degree20);

    [Benchmark(OperationsPerInvoke = Batch)]
    public double Degree10() => Evaluate(_degree10);

    private double Evaluate(Geopotential field)
    {
        double sum = 0;
        foreach (var item in _cases)
        {
            var a = field.AccelerationCorrection(item.Position, Mu, item.Time);
            sum += a.X + a.Y + a.Z;
        }
        return sum;
    }
}

[MemoryDiagnoser]
[ShortRunJob]
/// <summary>End-to-end six-hour propagation cost at each selectable truncation,
/// including any fidelity-dependent adaptive integrator work across four low lunar orbits.</summary>
public class LunarGravityPropagationBenchmarks
{
    private const double Mu = 4.9028000661637961e12;
    private const double Horizon = 6 * 3600.0;
    private Geopotential _degree50 = null!;
    private Geopotential _degree40 = null!;
    private Geopotential _degree30 = null!;
    private Geopotential _degree20 = null!;
    private Geopotential _degree10 = null!;
    private StateVector[] _initialStates = null!;

    [GlobalSetup]
    public void Setup()
    {
        var rotation = new BodyRotation(new Vector3d(0, 0, 1), new Vector3d(1, 0, 0),
            new Vector3d(0, 1, 0), 2.6616995e-6, 0);
        _degree50 = BenchmarkGravityModels.Lunar(rotation, 50);
        _degree40 = BenchmarkGravityModels.Lunar(rotation, 40);
        _degree30 = BenchmarkGravityModels.Lunar(rotation, 30);
        _degree20 = BenchmarkGravityModels.Lunar(rotation, 20);
        _degree10 = BenchmarkGravityModels.Lunar(rotation, 10);
        _initialStates =
        [
            CircularState(100_000, 0, 0),
            CircularState(25_000, Math.PI / 2, 0),
            CircularState(50_000, Math.PI / 4, Math.PI / 2),
            CircularState(150_000, 3 * Math.PI / 4, Math.PI / 4),
        ];
    }

    [Benchmark(Baseline = true)]
    public double Degree50() => PropagateAll(_degree50);

    [Benchmark]
    public double Degree40() => PropagateAll(_degree40);

    [Benchmark]
    public double Degree30() => PropagateAll(_degree30);

    [Benchmark]
    public double Degree20() => PropagateAll(_degree20);

    [Benchmark]
    public double Degree10() => PropagateAll(_degree10);

    private double PropagateAll(Geopotential field)
    {
        double checksum = 0;
        foreach (StateVector initial in _initialStates)
        {
            StateVector final = DormandPrince54.Propagate((time, state) =>
            {
                Vector3d position = state.Position;
                double r2 = position.LengthSquared();
                return position * (-Mu / (r2 * Math.Sqrt(r2)))
                    + field.AccelerationCorrection(position, Mu, time);
            }, initial, 0, Horizon,
                new IntegratorOptions { RelTol = 1e-11, MaxStep = 300 });
            checksum += final.Position.X + final.Position.Y + final.Position.Z;
        }
        return checksum;
    }

    private static StateVector CircularState(double altitude, double longitude, double inclination)
    {
        double radius = 1_738_000 + altitude;
        var radial = new Vector3d(Math.Cos(longitude), Math.Sin(longitude), 0);
        var tangent = new Vector3d(-Math.Sin(longitude), Math.Cos(longitude), 0);
        Vector3d velocityDirection = tangent * Math.Cos(inclination)
            + new Vector3d(0, 0, Math.Sin(inclination));
        return new StateVector(radial * radius, velocityDirection * Math.Sqrt(Mu / radius));
    }
}

[MemoryDiagnoser]
[ShortRunJob]
public class VectorNormalizationTradeoffBenchmarks
{
    private const int Batch = 1024;
    private Vector3d[] _vectors = null!;

    [GlobalSetup]
    public void Setup() => _vectors = Enumerable.Range(1, Batch).Select(i => new Vector3d(
        Math.ScaleB(Math.Sin(i * 0.71), i % 101 - 50),
        Math.ScaleB(Math.Cos(i * 1.13), i % 79 - 39),
        Math.ScaleB(Math.Sin(i * 1.91), i % 61 - 30))).ToArray();

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double ComponentDivision()
    {
        double sum = 0;
        foreach (var vector in _vectors)
        {
            var n = vector.Normalized();
            sum += n.X + n.Y + n.Z;
        }
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double ReciprocalMultiplication()
    {
        double sum = 0;
        foreach (var vector in _vectors)
        {
            double inverseLength = 1.0 / vector.Length();
            var n = vector * inverseLength;
            sum += n.X + n.Y + n.Z;
        }
        return sum;
    }
}

[MemoryDiagnoser]
[ShortRunJob]
public class PointMassInverseRootTradeoffBenchmarks
{
    private const int Batch = 1024;
    private Vector3d[] _offsets = null!;
    private double[] _mus = null!;

    [GlobalSetup]
    public void Setup()
    {
        _offsets = Enumerable.Range(1, Batch).Select(i => new Vector3d(
            1e6 + i * 1.7e8, -2e6 + i * i * 3.1e5, 7e7 * Math.Sin(i * 0.17))).ToArray();
        _mus = Enumerable.Range(1, Batch).Select(i => 1e8 + i * 1.1e17).ToArray();
    }

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double ExactSqrtDivision()
    {
        double sum = 0;
        for (int i = 0; i < Batch; i++)
        {
            var d = _offsets[i];
            double r2 = d.LengthSquared();
            double scale = _mus[i] / (r2 * Math.Sqrt(r2));
            sum += d.X * scale + d.Y * scale + d.Z * scale;
        }
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double ReciprocalRadiusCubed()
    {
        double sum = 0;
        for (int i = 0; i < Batch; i++)
        {
            var d = _offsets[i];
            double r2 = d.LengthSquared();
            double inverseRadius = 1.0 / Math.Sqrt(r2);
            double scale = _mus[i] * inverseRadius * inverseRadius * inverseRadius;
            sum += d.X * scale + d.Y * scale + d.Z * scale;
        }
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double EstimatedRootTwoNewtonSteps()
    {
        double sum = 0;
        for (int i = 0; i < Batch; i++)
        {
            var d = _offsets[i];
            double r2 = d.LengthSquared();
            double inverseRadius = Math.ReciprocalSqrtEstimate(r2);
            inverseRadius *= 1.5 - 0.5 * r2 * inverseRadius * inverseRadius;
            inverseRadius *= 1.5 - 0.5 * r2 * inverseRadius * inverseRadius;
            double scale = _mus[i] * inverseRadius * inverseRadius * inverseRadius;
            sum += d.X * scale + d.Y * scale + d.Z * scale;
        }
        return sum;
    }
}

[MemoryDiagnoser]
[ShortRunJob]
public class DotProductFmaTradeoffBenchmarks
{
    private const int Batch = 1024;
    private Vector3d[] _left = null!;
    private Vector3d[] _right = null!;

    [GlobalSetup]
    public void Setup()
    {
        _left = Enumerable.Range(1, Batch).Select(i => new Vector3d(
            Math.Sin(i * 0.71) * 1e12, Math.Cos(i * 1.13) * 1e9,
            Math.Sin(i * 1.91) * 1e6)).ToArray();
        _right = Enumerable.Range(1, Batch).Select(i => new Vector3d(
            Math.Cos(i * 0.37) * 1e7, Math.Sin(i * 1.31) * 1e10,
            Math.Cos(i * 1.73) * 1e13)).ToArray();
    }

    [Benchmark(Baseline = true, OperationsPerInvoke = Batch)]
    public double MultiplyAddBaseline()
    {
        double sum = 0;
        for (int i = 0; i < Batch; i++) sum += _left[i].Dot(_right[i]);
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double NestedFusedMultiplyAdd()
    {
        double sum = 0;
        for (int i = 0; i < Batch; i++)
        {
            var a = _left[i]; var b = _right[i];
            sum += Math.FusedMultiplyAdd(a.X, b.X,
                Math.FusedMultiplyAdd(a.Y, b.Y, a.Z * b.Z));
        }
        return sum;
    }

    [Benchmark(OperationsPerInvoke = Batch)]
    public double ReassociatedMultiplyAdd()
    {
        double sum = 0;
        for (int i = 0; i < Batch; i++)
        {
            var a = _left[i]; var b = _right[i];
            sum += a.X * b.X + (a.Y * b.Y + a.Z * b.Z);
        }
        return sum;
    }
}
