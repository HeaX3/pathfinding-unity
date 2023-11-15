using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace FunnelAlgorithm
{
    struct FunnelApex
    {
        public int Index { get; private set; }
        public Vector3 Position { get; private set; }
        public bool IsLeft { get; private set; }
        public FunnelApex(int index, Vector3 position, bool isLeft)
        {
            Index = index;
            Position = position;
            IsLeft = isLeft;
        }
    }

    public class FunnelPathfinder
    {

        private Vector3[] _leftVertices3d;
        private Vector3[] _rightVertices3d;
        private Vector3[] _leftVertices2d;
        private Vector3[] _rightVertices2d;
        private FunnelApex _apex;
        private List<int> _leftIndices = new List<int>();
        private List<int> _rightIndices = new List<int>();
        private List<FunnelApex> _apexes = new List<FunnelApex>();

        public FunnelAlgorithmPath FindPath(IEnumerable<Triangle> triangles)
        {
            if (triangles.Count() < 2)
            {
                return null;
            }
                
            if (!CreateVertices3D(triangles))
            {
                return null;
            }
                
            CopyVertices3DToVertices2D();

            ConvertTo2D();

            ConvertToXYPlane();

            _apex = new FunnelApex(0, _leftVertices2d[0], true);

            _leftIndices.Add(1);

            _rightIndices.Add(1);

            while (UpdateFunnel()) { }

            _apexes.Add(_apex);

            _apexes.Add(new FunnelApex(_leftVertices2d.Length - 1, _leftVertices2d.Last(), true));

            var path = MakePath();

            return path;
        }

        private bool CreateVertices3D(IEnumerable<Triangle> triangles)
        {
            var commonEdges = new List<Edge>();
            foreach (var pair in triangles.MakePairs())
            {
                var commonEdge = pair.Left.FindCommonEdge(pair.Right);
                if (commonEdge == null)
                {
                    return false;
                }
                commonEdges.Add(commonEdge);
            }

            _leftVertices3d = new Vector3[commonEdges.Count + 2];
            _rightVertices3d = new Vector3[commonEdges.Count + 2];

            var startPoint = triangles.First().FindOppositeVertex(commonEdges.First()).Value;

            _leftVertices3d[0] = _rightVertices3d[0] = startPoint;

            var i = 1;
            foreach (var commonEdge in commonEdges)
            {
                var nextOrigin = Vector3.zero;

                if (i == 1)
                {
                    _leftVertices3d[i] = commonEdge.A;
                    _rightVertices3d[i] = commonEdge.B;
                    nextOrigin = _leftVertices3d[i];
                }
                else
                {

                    if (commonEdge.A.Equals(_leftVertices3d[i - 1]) || commonEdge.B.Equals(_rightVertices3d[i - 1]))
                    {
                        _leftVertices3d[i] = commonEdge.A;
                        _rightVertices3d[i] = commonEdge.B;

                        nextOrigin = _rightVertices3d[i];
                    }
                    else
                    {
                        _leftVertices3d[i] = commonEdge.B;
                        _rightVertices3d[i] = commonEdge.A;

                        nextOrigin = _leftVertices3d[i];
                    }
                }

                i++;
            }

            var endPoint = triangles.Last().FindOppositeVertex(commonEdges.Last()).Value;
            _leftVertices3d[_leftVertices3d.Length - 1] = _rightVertices3d[_leftVertices3d.Length - 1] = endPoint;
            return true;
        }

        private void CopyVertices3DToVertices2D()
        {
            _leftVertices2d = _leftVertices3d.Select(v => new Vector3(v.x, v.y, v.z)).ToArray();
            _rightVertices2d = _rightVertices3d.Select(v => new Vector3(v.x, v.y, v.z)).ToArray();
        }

        private void ConvertTo2D()
        {
            var origin = _leftVertices2d[0];

            for (int i = 2, count = _leftVertices2d.Length; i < count; i++)
            {
                var isLeftEqual = _leftVertices2d[i].Equals(_leftVertices2d[i - 1]);
                var target = isLeftEqual ? _rightVertices2d[i] : _leftVertices2d[i];

                var originNormal = Vector3.Cross(_leftVertices2d[i - 1] - _rightVertices2d[i - 1], origin - _leftVertices2d[i - 1]).normalized;

                var targetNormal = Vector3.Cross(_rightVertices2d[i - 1] - _leftVertices2d[i - 1], target - _rightVertices2d[i - 1]).normalized;

                var angle = FunnelAlgorithmMathUtility.SignedVectorAngle(originNormal, targetNormal, _leftVertices2d[i - 1] - _rightVertices2d[i - 1]);

                var rotation = Quaternion.AngleAxis(angle, _rightVertices2d[i - 1] - _leftVertices2d[i - 1]);

                var translation = _leftVertices2d[i - 1] - (rotation * _leftVertices2d[i - 1]);

                for (int j = i; j < _leftVertices2d.Length; j++)
                {

                    _leftVertices2d[j] = rotation * _leftVertices2d[j];
                    _leftVertices2d[j] = _leftVertices2d[j] + translation;

                    _rightVertices2d[j] = rotation * _rightVertices2d[j];
                    _rightVertices2d[j] = _rightVertices2d[j] + translation;
                }

                var nextOrigin = isLeftEqual ? _rightVertices2d[i - 1] : _leftVertices2d[i - 1];
                origin = nextOrigin;
            }
        }

        private void ConvertToXYPlane()
        {

            var origin = _leftVertices2d[0];
            var translation = Vector3.zero - origin;
            var normal = Vector3.Cross(_leftVertices2d[1] - origin, _rightVertices2d[1] - origin).normalized;
            var rotation = Quaternion.FromToRotation(normal, new Vector3(0, 0, 1));

            for (int i = 0; i < _leftVertices2d.Length; i++)
            {
                _leftVertices2d[i] = rotation * _leftVertices2d[i];
                _leftVertices2d[i] = _leftVertices2d[i] + translation;
                _leftVertices2d[i] = new Vector3(_leftVertices2d[i].x, _leftVertices2d[i].y);

                _rightVertices2d[i] = rotation * _rightVertices2d[i];
                _rightVertices2d[i] = _rightVertices2d[i] + translation;
                _rightVertices2d[i] = new Vector3(_rightVertices2d[i].x, _rightVertices2d[i].y);
            }
        }

        private FunnelAlgorithmPath MakePath()
        {
            var positions = new List<Vector3>();
            var normals = new List<Vector3>();


            foreach (var pair in _apexes.MakePairs())
            {

                var startIndex = pair.Left.Index;
                var endIndex = pair.Right.Index;

                var startPoint = (pair.Left.IsLeft ? _leftVertices3d : _rightVertices3d)[pair.Left.Index];
                positions.Add(startPoint);

                if (startIndex == 0)
                {
                    normals.Add(Vector3.Cross(_rightVertices3d[1] - _leftVertices3d[1], _rightVertices3d[1] - _leftVertices3d[0]).normalized);
                }
                else
                {
                    var opposite = _rightVertices3d[startIndex].Equals(_rightVertices3d[startIndex - 1]) ? _leftVertices3d[startIndex - 1] : _rightVertices3d[startIndex - 1];
                    normals.Add(Vector3.Cross(_rightVertices3d[startIndex] - _leftVertices3d[startIndex], _rightVertices3d[startIndex] - opposite).normalized);
                }

                foreach (var i in Enumerable.Range(startIndex + 1, endIndex - startIndex))
                {
                    Vector3 intersection;
                    if (FunnelAlgorithmMathUtility.SegmentSegmentIntersection(out intersection, pair.Left.Position, pair.Right.Position, _leftVertices2d[i], _rightVertices2d[i]))
                    {
                        var lerp = Vector3.Distance(intersection, _leftVertices2d[i]) / Vector3.Distance(_rightVertices2d[i], _leftVertices2d[i]);

                        var left3dpos = _leftVertices3d[i];
                        var right3dpos = _rightVertices3d[i];

                        var position = Vector3.MoveTowards(left3dpos, right3dpos, Vector3.Distance(left3dpos, right3dpos) * lerp);

                        var currentOpposite = _rightVertices3d[i].Equals(_rightVertices3d[i - 1]) ? _leftVertices3d[i - 1] : _rightVertices3d[i - 1];
                        var currentNormal = Vector3.Cross(_rightVertices3d[i] - _leftVertices3d[i], _rightVertices3d[i] - currentOpposite);

                        var nextOpposite = _rightVertices3d[i + 1].Equals(_rightVertices3d[i]) ? _leftVertices3d[i] : _rightVertices3d[i];
                        var nextNormal = Vector3.Cross(_rightVertices3d[i + 1] - _leftVertices3d[i + 1], _rightVertices3d[i + 1] - nextOpposite);

                        var normal = new Vector3((currentNormal.x + nextNormal.x) / 2.0f, (currentNormal.y + nextNormal.y) / 2.0f, (currentNormal.z + nextNormal.z) / 2.0f).normalized;

                        positions.Add(position);
                        normals.Add(normal);
                    }
                }
            }

            var endPoint = _leftVertices3d[_leftVertices3d.Length - 1];
            positions.Add(endPoint);

            var endNormal = Vector3.Cross(_leftVertices3d[_leftVertices3d.Length - 2] - _rightVertices3d[_rightVertices3d.Length - 2], _leftVertices3d[_leftVertices3d.Length - 2] - _leftVertices3d.Last()).normalized;
            normals.Add(endNormal);

            return MakePath(positions, normals);
        }

        private FunnelAlgorithmPath MakePath(IEnumerable<Vector3> positions, IEnumerable<Vector3> normals)
        {

            var positionGroups = positions.SplitByEquality();

            var resultNormals = new List<Vector3>();
            int skipCount = 0;
            foreach (var g in positionGroups)
            {
                var resultNormal = FunnelAlgorithmMathUtility.Synthesize(normals.Skip(skipCount).Take(g.Count()));
                resultNormals.Add(resultNormal);
                skipCount += g.Count();
            }

            var resultPositions = positionGroups.Select(g => g.First());

            return new FunnelAlgorithmPath(resultPositions.Reverse<Vector3>(), resultNormals.Reverse<Vector3>());
        }

        private bool UpdateFunnel()
        {
            if (_leftVertices2d.Length - 1 <= _leftIndices.Last())
            {
                return false;
            }
                
            if (Push(_leftVertices2d, _rightVertices2d, ref _leftIndices, ref _rightIndices, true))
            {

                Push(_rightVertices2d, _leftVertices2d, ref _rightIndices, ref _leftIndices, false);
            }

            return true;
        }

        private bool Push(Vector3[] targets, Vector3[] opposites, ref List<int> targetIndices, ref List<int> oppositeIndices, bool isLeft)
        {

            var crossedIndex = IsCrossedOppositeVertices(targets, opposites, targetIndices, oppositeIndices);
            if (crossedIndex > 0 && crossedIndex < targets.Count() - 1)
            {

                _apexes.Add(_apex);

                _apex = new FunnelApex(crossedIndex, opposites[crossedIndex], !isLeft);

                var nextIndex = _apex.Index + 1;
                while (opposites.Length > nextIndex)
                {
                    if (!_apex.Position.Equals(opposites[nextIndex]))
                    {
                        break;
                    }
                        
                    nextIndex++;
                }

                targetIndices = new List<int>() { nextIndex };
                oppositeIndices = new List<int> { nextIndex };

                return false;
            }

            var next = targetIndices.Last() + 1;

            if (IsTightened(targets, opposites, targetIndices, oppositeIndices))
            {
                targetIndices.Clear();
            }

            targetIndices.Add(next);

            return true;
        }

        private bool IsTightened(Vector3[] targets, Vector3[] opposites, List<int> targetIndices, List<int> oppositeIndices)
        {
            var lastVec = (targets[targetIndices.Last() + 1] - _apex.Position).normalized;
            var firstVec = (targets[targetIndices.First()] - _apex.Position).normalized;
            var oppositeVec = (opposites[oppositeIndices.First()] - _apex.Position).normalized;

            return Vector3.Dot(lastVec, oppositeVec) > Vector3.Dot(firstVec, oppositeVec);
        }

        private int IsCrossedOppositeVertices(Vector3[] targets, Vector3[] opposites, List<int> targetIndices, List<int> oppositeIndices)
        {

            var lastVec = targets[targetIndices.Last()] - _apex.Position;

            var nextVec = targets[targetIndices.Last() + 1] - _apex.Position;
            foreach (var i in oppositeIndices)
            {
                var oppositeVec = opposites[i] - _apex.Position;

                if (IsCrossed(oppositeVec, lastVec, nextVec))
                {
                    return i;
                }
                   
            }

            return 0;
        }

        private bool IsCrossed(Vector3 target, Vector3 current, Vector3 next)
        {

            if (target == Vector3.zero || current == Vector3.zero || next == Vector3.zero)
            {
                return false;
            }
                
            var currentCross = Vector3.Cross(target, current).normalized;

            var nextCross = Vector3.Cross(target, next).normalized;

            var dot = Vector3.Dot(currentCross, nextCross);

            return dot <= 0;
        }
    }
}


