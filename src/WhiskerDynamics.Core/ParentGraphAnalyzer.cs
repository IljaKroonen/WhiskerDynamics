namespace WhiskerDynamics.Core;

/// <summary>One indexed edge in a parent graph. <see cref="ParentIndex"/> is -1
/// for a root; every other value indexes another link in the same analysis.</summary>
internal readonly record struct ParentGraphLink(string Id, int ParentIndex);

/// <summary>Result of one non-recursive parent-graph pass.</summary>
internal sealed class ParentGraphAnalysis
{
    private readonly ParentGraphLink[] _links;

    internal ParentGraphAnalysis(
        ParentGraphLink[] links, int[] depths, int[] roots, int[][] cycles)
    {
        _links = links;
        Depths = depths;
        RootIndices = roots;
        Cycles = cycles;
    }

    /// <summary>Distance to a root. -1 denotes a node whose chain reaches a cycle.</summary>
    internal int[] Depths { get; }
    internal int[] RootIndices { get; }

    /// <summary>Each entry contains exactly the participants in one cycle, in
    /// child-to-parent traversal order and rotated to begin at its ordinal-smallest
    /// id. Cycles themselves are sorted by that canonical traversal. The closing
    /// repeat is added only when a cycle is formatted.</summary>
    internal int[][] Cycles { get; }

    internal string FormatCycle(int[] cycle)
    {
        var ids = new string[cycle.Length + 1];
        for (int i = 0; i < cycle.Length; i++) ids[i] = $"'{_links[cycle[i]].Id}'";
        ids[^1] = ids[0];
        return string.Join(" -> ", ids);
    }

    /// <summary>Formats every cycle in deterministic canonical order.</summary>
    internal string FormatCycles() => string.Join("; ", Cycles.Select(FormatCycle));

    /// <summary>Returns graph nodes not reachable by child edges from one root,
    /// preserving source order for deterministic diagnostics.</summary>
    internal int[] UnreachableFrom(int rootIndex)
    {
        if ((uint)rootIndex >= (uint)_links.Length)
            throw new ArgumentOutOfRangeException(nameof(rootIndex));

        var children = new List<int>[_links.Length];
        for (int i = 0; i < _links.Length; i++)
        {
            int parent = _links[i].ParentIndex;
            if (parent < 0) continue;
            (children[parent] ??= []).Add(i);
        }

        var reached = new bool[_links.Length];
        var pending = new Queue<int>();
        reached[rootIndex] = true;
        pending.Enqueue(rootIndex);
        while (pending.TryDequeue(out int parent))
        {
            if (children[parent] is not { } childList) continue;
            foreach (int child in childList)
                if (!reached[child])
                {
                    reached[child] = true;
                    pending.Enqueue(child);
                }
        }

        var unreachable = new List<int>();
        for (int i = 0; i < reached.Length; i++)
            if (!reached[i]) unreachable.Add(i);
        return [.. unreachable];
    }

    internal string IdAt(int index) => _links[index].Id;
}

/// <summary>Analyzes functional parent graphs without recursion. Every node is
/// coloured once; the parent walk costs O(V), followed by deterministic cycle
/// ordering, and memory remains O(V). Even a very deep malformed catalog cannot
/// hang or overflow the stack.</summary>
internal static class ParentGraphAnalyzer
{
    internal static ParentGraphAnalysis Analyze(IReadOnlyList<ParentGraphLink> source)
    {
        var links = source.ToArray();
        int count = links.Length;
        for (int i = 0; i < count; i++)
        {
            int parent = links[i].ParentIndex;
            if (parent < -1 || parent >= count)
                throw new ArgumentOutOfRangeException(nameof(source),
                    $"Parent index {parent} for '{links[i].Id}' is outside the graph.");
        }

        // 0 = unseen, 1 = on the current parent path, 2 = completely analysed.
        var state = new byte[count];
        var depths = new int[count];
        Array.Fill(depths, -1);
        var activePosition = new int[count];
        Array.Fill(activePosition, -1);
        var path = new List<int>();
        var cycles = new List<int[]>();

        for (int start = 0; start < count; start++)
        {
            if (state[start] != 0) continue;
            path.Clear();
            int current = start;
            while (current >= 0 && state[current] == 0)
            {
                state[current] = 1;
                activePosition[current] = path.Count;
                path.Add(current);
                current = links[current].ParentIndex;
            }

            bool reachesCycle = false;
            if (current >= 0 && state[current] == 1)
            {
                int cycleStart = activePosition[current];
                var cycle = new int[path.Count - cycleStart];
                path.CopyTo(cycleStart, cycle, 0, cycle.Length);
                cycles.Add(CanonicalCycle(cycle, links));
                reachesCycle = true;
            }
            else if (current >= 0 && depths[current] < 0)
            {
                // This path joins a previously analysed tail into a cycle.
                reachesCycle = true;
            }

            int depth = current < 0 ? -1 : depths[current];
            for (int i = path.Count - 1; i >= 0; i--)
            {
                int node = path[i];
                if (!reachesCycle) depths[node] = ++depth;
                state[node] = 2;
                activePosition[node] = -1;
            }
        }

        var roots = new List<int>();
        for (int i = 0; i < count; i++)
            if (links[i].ParentIndex < 0) roots.Add(i);
        cycles.Sort((left, right) => CompareCycles(left, right, links));
        return new ParentGraphAnalysis(links, depths, [.. roots], [.. cycles]);
    }

    private static int[] CanonicalCycle(int[] cycle, ParentGraphLink[] links)
    {
        int first = 0;
        for (int i = 1; i < cycle.Length; i++)
        {
            int byId = StringComparer.Ordinal.Compare(
                links[cycle[i]].Id, links[cycle[first]].Id);
            if (byId < 0 || (byId == 0 && cycle[i] < cycle[first])) first = i;
        }
        if (first == 0) return cycle;

        var canonical = new int[cycle.Length];
        for (int i = 0; i < cycle.Length; i++)
            canonical[i] = cycle[(first + i) % cycle.Length];
        return canonical;
    }

    private static int CompareCycles(
        int[] left, int[] right, ParentGraphLink[] links)
    {
        int common = Math.Min(left.Length, right.Length);
        for (int i = 0; i < common; i++)
        {
            int byId = StringComparer.Ordinal.Compare(
                links[left[i]].Id, links[right[i]].Id);
            if (byId != 0) return byId;
        }
        int byLength = left.Length.CompareTo(right.Length);
        if (byLength != 0) return byLength;

        // Ids need not be unique for the generic linked-body analyzer. Indices make
        // the ordering total without changing the reference-identity graph semantics.
        for (int i = 0; i < left.Length; i++)
        {
            int byIndex = left[i].CompareTo(right[i]);
            if (byIndex != 0) return byIndex;
        }
        return 0;
    }

    /// <summary>Projects a linked body forest, including any ancestors outside the
    /// supplied list so existing generic Ephemerides forest semantics are preserved.
    /// <paramref name="bodyIndices"/> maps each supplied slot into the analysis.</summary>
    internal static ParentGraphAnalysis AnalyzeBodies(
        IReadOnlyList<CelestialBody> bodies, out int[] bodyIndices)
    {
        var nodes = new List<CelestialBody>(bodies.Count);
        var indices = new Dictionary<CelestialBody, int>(ReferenceEqualityComparer.Instance);
        bodyIndices = new int[bodies.Count];
        for (int i = 0; i < bodies.Count; i++)
        {
            var body = bodies[i];
            if (!indices.TryGetValue(body, out int index))
            {
                index = nodes.Count;
                indices.Add(body, index);
                nodes.Add(body);
            }
            bodyIndices[i] = index;
        }

        // A for loop deliberately observes nodes appended by earlier parent chains.
        for (int i = 0; i < nodes.Count; i++)
            if (nodes[i].Parent is { } parent && !indices.ContainsKey(parent))
            {
                indices.Add(parent, nodes.Count);
                nodes.Add(parent);
            }

        var links = new ParentGraphLink[nodes.Count];
        for (int i = 0; i < nodes.Count; i++)
            links[i] = new ParentGraphLink(nodes[i].Id,
                nodes[i].Parent is { } parent ? indices[parent] : -1);
        return Analyze(links);
    }
}
