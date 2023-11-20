using System;



public readonly struct AStarVector2Float : IEquatable<AStarVector2Float>
{
    private static readonly double _sqr2 = Math.Sqrt(2);

    public AStarVector2Float(float x, float y)
    {
        X = x;
        Y = y;
    }

    public float X { get; }
    public float Y { get; }

    public double DistanceEstimate()
    {
        float linearSteps = Math.Abs(Math.Abs(Y) - Math.Abs(X));
        float diagonalSteps = Math.Max(Math.Abs(Y), Math.Abs(X)) - linearSteps;
        return linearSteps + _sqr2 * diagonalSteps;
    }

    public static AStarVector2Float operator +(AStarVector2Float a, AStarVector2Float b) => new AStarVector2Float(a.X + b.X, a.Y + b.Y);
    public static AStarVector2Float operator -(AStarVector2Float a, AStarVector2Float b) => new AStarVector2Float(a.X - b.X, a.Y - b.Y);

    public bool Equals(AStarVector2Float other)
        => ApproximatelyEqualEpsilon(X,other.X) && ApproximatelyEqualEpsilon(Y, other.Y);

    private bool ApproximatelyEqualEpsilon(float a, float b)
    {
        float epsilon = 0.000001f;
        const float floatNormal = (1 << 23) * float.Epsilon;
        float absA = Math.Abs(a);
        float absB = Math.Abs(b);
        float diff = Math.Abs(a - b);

        if (a == b)
        {
            return true;
        }

        if (a == 0.0f || b == 0.0f || diff < floatNormal)
        {
            return diff < (epsilon * floatNormal);
        }

        return diff / Math.Min((absA + absB), float.MaxValue) < epsilon;
    }

    public override string ToString()
        => $"({X}, {Y})";
}
