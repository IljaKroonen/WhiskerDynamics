using System.Globalization;
using System.Xml.Linq;
using WhiskerDynamics.Core;

namespace WhiskerDynamics.Benchmarks;

/// <summary>Loads a game system manifest for benchmarks, resolving library references
/// and parsing the merged bodies through <see cref="AstronomicalsParser"/>. Missing
/// masses become zero and Zg values are converted to kilograms. Missing definition
/// frames are treated as ecliptic, so this loader preserves orbital timescales rather
/// than parent-equatorial orientation.</summary>
public static class SystemManifestLoader
{
    public static IReadOnlyList<CelestialBody> Load(
        string systemXmlPath, string astronomicalsXmlPath, MassConstants? constants = null)
    {
        static bool IsBody(XElement e) =>
            e.Attribute("Id") is not null && (e.Element("Mass") is not null || e.Element("Orbit") is not null);

        var library = XDocument.Load(astronomicalsXmlPath).Root!
            .Elements()
            .Where(IsBody)
            .ToDictionary(e => e.Attribute("Id")!.Value);

        var merged = new XElement("Assets");
        foreach (var el in XDocument.Load(systemXmlPath).Root!.Elements())
        {
            if (el.Name.LocalName == "LoadFromLibrary")
            {
                string id = el.Attribute("Id")?.Value
                    ?? throw new FormatException("LoadFromLibrary without Id");
                if (!library.TryGetValue(id, out var def))
                    throw new FormatException($"LoadFromLibrary '{id}': no body definition (Mass or Orbit) in the library file");
                var clone = new XElement(def);
                clone.SetAttributeValue("Parent", el.Attribute("Parent")?.Value); // null clears (root)
                merged.Add(clone);
            }
            else if (IsBody(el))
            {
                merged.Add(new XElement(el));
            }
        }
        foreach (var body in merged.Elements())
        {
            var mass = body.Element("Mass");
            if (mass is null)
                body.Add(new XElement("Mass", new XAttribute("Kg", "0"))); // massless catalog entry: test particle
            else if (mass.Attribute("Zg") is { } zg)
            {
                mass.SetAttributeValue("Kg", double.Parse(zg.Value, CultureInfo.InvariantCulture) * 1e18);
                zg.Remove();
            }
            var orbit = body.Element("Orbit");
            if (orbit is not null && orbit.Attribute("DefinitionFrame") is null)
                orbit.SetAttributeValue("DefinitionFrame", "Ecliptic");
        }

        using var stream = new MemoryStream();
        new XDocument(merged).Save(stream);
        stream.Position = 0;
        return AstronomicalsParser.Parse(stream, constants);
    }
}
