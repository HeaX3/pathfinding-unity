using System;
using System.Linq;
using System.Collections.Generic;

internal static class AStarNodeExtensions
{
    public static Dictionary<AStarVector2Float, List<AStarVector2Float>> GraphNodesWithNeighboards;

    private static List<(AStarVector2Float position, double cost)> _currentNeighbours = new List<(AStarVector2Float position, double cost)>();
    public static void Fill(this List<AStarPathNode> buffer, AStarPathNode parent, AStarVector2Float target)
    {
        _currentNeighbours.Clear();

        if (GraphNodesWithNeighboards.ContainsKey(parent.Position))
        {
            var listNeighborads = GraphNodesWithNeighboards[parent.Position];

            foreach(var next  in listNeighborads)
            {
                var currentCost = Math.Sqrt(Math.Pow((next.X - parent.Position.X),2) + Math.Pow((next.Z - parent.Position.Z), 2));

                _currentNeighbours.Add((next, currentCost));
            }      
        }

        buffer.Clear();
        foreach ((AStarVector2Float position, double cost) in _currentNeighbours)
        {
            AStarVector2Float nodePosition = position;
            double traverseDistance = parent.TraverseDistance + cost;
            buffer.Add( new AStarPathNode(nodePosition, target, traverseDistance));
        }
    }
}