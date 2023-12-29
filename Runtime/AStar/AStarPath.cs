using System;
using System.Collections.Generic;


public class AStarPath
{
    private readonly List<AStarPathNode> _neighbours = new List<AStarPathNode>();
    private readonly IBinaryHeap<AStarVector2Float, AStarPathNode> _frontier;
    private readonly HashSet<AStarVector2Float> _ignoredPositions;
    private readonly List<AStarVector2Float> _outputAStarPath;
    private readonly IDictionary<AStarVector2Float, AStarVector2Float> _links;

    public AStarPath(Dictionary<AStarVector2Float, List<AStarVector2Float>> neighboardNodes, int initialCapacity = 0)
    {
        if (initialCapacity < 0)
        {
            return;
        }
        AStarNodeExtensions.GraphNodesWithNeighboards = neighboardNodes;

        _frontier = new BinaryHeap<AStarVector2Float, AStarPathNode>(a => a.Position, initialCapacity);
        _ignoredPositions = new HashSet<AStarVector2Float>(initialCapacity);
        _outputAStarPath = new List<AStarVector2Float>(initialCapacity);
        _links = new Dictionary<AStarVector2Float, AStarVector2Float>(initialCapacity);
    }

    public bool Calculate(AStarVector2Float start, AStarVector2Float target, IReadOnlyCollection<AStarVector2Float> obstacles, out IReadOnlyCollection<AStarVector2Float> aStarPath)
    {
        if (obstacles == null)
        {
            obstacles = Array.Empty<AStarVector2Float>();
        }

        if (!GenerateNodes(start, target, obstacles))
        {
            aStarPath = Array.Empty<AStarVector2Float>();
            return false;
        }

        _outputAStarPath.Clear();
        _outputAStarPath.Add(target);

        while (_links.TryGetValue(target, out target))
        {
            _outputAStarPath.Add(target);
        }

        aStarPath = _outputAStarPath;
        return true;
    }

    private bool GenerateNodes(AStarVector2Float start, AStarVector2Float target, IReadOnlyCollection<AStarVector2Float> obstacles)
    {
        _frontier.Clear();
        _ignoredPositions.Clear();
        _links.Clear();

        _frontier.Enqueue(new AStarPathNode(start, target, 0));
        _ignoredPositions.UnionWith(obstacles);
        while (_frontier.Count > 0)
        {
            AStarPathNode current = _frontier.Dequeue();
            _ignoredPositions.Add(current.Position);

            if (current.Position.Equals(target))
                return true;

            GenerateFrontierNodes(current, target);
        }

        return false;
    }

    private void GenerateFrontierNodes(AStarPathNode parent, AStarVector2Float target)
    {
        _neighbours.Fill(parent, target);

        foreach (AStarPathNode newNode in _neighbours)
        {
            if (_ignoredPositions.Contains(newNode.Position))
            {
                continue;
            }

            if (!_frontier.TryGet(newNode.Position, out AStarPathNode existingNode))
            {
                _frontier.Enqueue(newNode);
                _links[newNode.Position] = parent.Position;
            }
            else if (newNode.TraverseDistance < existingNode.TraverseDistance)
            {
                _frontier.Modify(newNode);
                _links[newNode.Position] = parent.Position;
            }
        }
    }
}
