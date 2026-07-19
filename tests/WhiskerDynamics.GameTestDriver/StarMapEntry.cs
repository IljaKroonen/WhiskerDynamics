using System.Reflection;
using System.Runtime.Loader;
using StarMap.API;

namespace WhiskerDynamics.GameTestDriver;

[StarMapMod]
public sealed class StarMapEntry
{
    [StarMapBeforeMain]
    public void BeforeMain()
    {
        try
        {
            string directory = Path.GetDirectoryName(typeof(StarMapEntry).Assembly.Location)!;
            string runtimePath = Path.Combine(
                directory, "WhiskerDynamics.GameTestDriver.Runtime.dll");
            AssemblyLoadContext context = AssemblyLoadContext.GetLoadContext(
                typeof(StarMapEntry).Assembly)!;
            Assembly runtime = context.LoadFromAssemblyPath(runtimePath);
            Type entry = runtime.GetType(
                "WhiskerDynamics.GameTestDriver.Runtime.GameTestDriverMain",
                throwOnError: true)!;
            entry.GetMethod("Load", BindingFlags.Public | BindingFlags.Static)!
                .Invoke(null, null);
        }
        catch (Exception e)
        {
            string directory = Path.GetDirectoryName(typeof(StarMapEntry).Assembly.Location)!;
            File.WriteAllText(Path.Combine(directory, "game-test-driver-error.log"),
                e.ToString());
        }
    }
}
