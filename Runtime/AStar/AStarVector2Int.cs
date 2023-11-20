using System;


public readonly struct AStarVector2Int : IEquatable<AStarVector2Int>
{
    private static readonly double _sqr2 = Math.Sqrt(2);

    public AStarVector2Int(int x, int y)
    {
        X = x;
        Y = y;
    }

    public int X { get; }
    public int Y { get; }

    public double DistanceEstimate()
    {
        int linearSteps = Math.Abs(Math.Abs(Y) - Math.Abs(X));
        int diagonalSteps = Math.Max(Math.Abs(Y), Math.Abs(X)) - linearSteps;
        return linearSteps + _sqr2 * diagonalSteps;
    }

    public static AStarVector2Int operator +(AStarVector2Int a, AStarVector2Int b) => new AStarVector2Int(a.X + b.X, a.Y + b.Y);
    public static AStarVector2Int operator -(AStarVector2Int a, AStarVector2Int b) => new AStarVector2Int(a.X - b.X, a.Y - b.Y);

    public bool Equals(AStarVector2Int other)
        => X.Equals(other.X) && Y.Equals(other.Y);

    public override string ToString()
        => $"({X}, {Y})";
}
