using System;
using System.Linq;
using System.Collections.Generic;

internal static class AStarNodeExtensions
{
    /*    private static readonly (AStarVector2Int position, double cost)[] NeighboursTemplate = {
                (new AStarVector2Int(1, 0), 1),
                (new AStarVector2Int(0, 1), 1),
                (new AStarVector2Int(-1, 0), 1),
                (new AStarVector2Int(0, -1), 1),

                (new AStarVector2Int(1, 1), Math.Sqrt(2)),
                (new AStarVector2Int(1, -1), Math.Sqrt(2)),
                (new AStarVector2Int(-1, 1), Math.Sqrt(2)),
                (new AStarVector2Int(-1, -1), Math.Sqrt(2))
            };*/

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
            AStarVector2Float nodePosition = position;// position + parent.Position;
            double traverseDistance = parent.TraverseDistance + cost;
            buffer.Add( new AStarPathNode(nodePosition, target, traverseDistance));
        }
    }

    public static IList<T> Swap<T>(this IList<T> list, int indexA, int indexB)
    {
        T tmp = list[indexA];
        list[indexA] = list[indexB];
        list[indexB] = tmp;
        return list;
    }
}