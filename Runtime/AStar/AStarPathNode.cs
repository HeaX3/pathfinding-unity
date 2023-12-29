using System;

internal readonly struct AStarPathNode : IComparable<AStarPathNode>
{
    public AStarVector2Float Position { get; }
    public double TraverseDistance { get; }

    private readonly double _estimatedTotalCost;
    private readonly double _heuristicDistance;

    public AStarPathNode(AStarVector2Float position, AStarVector2Float target, double traverseDistance)
    {
        Position = position;
        TraverseDistance = traverseDistance;
        _heuristicDistance = (position - target).DistanceEstimate();
        _estimatedTotalCost = traverseDistance + _heuristicDistance;
    }

    public int CompareTo(AStarPathNode other)
        => _estimatedTotalCost.CompareTo(other._estimatedTotalCost);
}