using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using Perpetuum.Collections;

namespace Perpetuum.PathFinders
{
    public class AStarFinder : PathFinder
    {
        private const int WIDTH = 2048;
        private const int HEIGHT = 2048;

        private readonly bool[] closed = new bool[WIDTH * HEIGHT];
        private readonly PriorityQueue<Node> open = new PriorityQueue<Node>(500);
        private readonly Heuristic _heuristic;

        public AStarFinder(Heuristic heuristic)
        {
            _heuristic = heuristic;
            Weight = 10;
        }

        public int Weight { get; set; }

        public override Point[] FindPath(Point start, Point end, PathFinderNodePassableHandler handler, CancellationToken cancellationToken)
        {
            if (!handler(end.X, end.Y))
                return new Point[0];

            if (start == end)
                return EmptyPath;

            open.Clear();
            for (var i = 0; i < WIDTH * HEIGHT; i++ )
            {
                closed[i] = false;
            }

            var startNode = new Node(start.X, start.Y);
            open.Enqueue(startNode);
            closed[start.X + start.Y * WIDTH] = true;

            Node node;
            while (open.TryDequeue(out node) && !cancellationToken.IsCancellationRequested)
            {
                if (!OnProcessNode(node))
                    break;

                if (node.X == end.X && node.Y == end.Y)
                    return Backtrace(node);

                foreach (var neighbor in GetNeighbors(node, handler, closed))
                {
                    var newG = node.g + (int)(Weight * (neighbor.X - node.X == 0 || neighbor.Y - node.Y == 0 ? 1 : SQRT2));
                    var newH = _heuristic.Calculate(neighbor.X, neighbor.Y, end.X, end.Y) * Weight;

                    neighbor.g = newG;
                    neighbor.f = newG + newH;
                    neighbor.parent = node;

                    open.Enqueue(neighbor);
                    OnPathFinderDebug(neighbor, PathFinderNodeType.Open);
                }
            }

            return new Point[0];
        }

        private Point[] Backtrace(Node node)
        {
            var stack = new Stack<Point>();

            while (node != null)
            {
                stack.Push(new Point(node.X, node.Y));
                OnPathFinderDebug(node,PathFinderNodeType.Path);
                node = node.parent;
            }

            return stack.ToArray();
        }

        private static readonly sbyte[,] _n = { { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 }, { 1, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 } };

        private IEnumerable<Node> GetNeighbors(Node node, PathFinderNodePassableHandler handler, bool[] closed)
        {
            for (var i = 0; i < 8; i++)
            {
                var nx = node.X + _n[i, 0];
                var ny = node.Y + _n[i, 1];

                if (!handler(nx, ny))
                {
                    continue;
                }

                if (closed[nx + ny * WIDTH])
                {
                    continue;
                }
                closed[nx + ny * WIDTH] = true;

                yield return new Node(nx, ny);
            }
        }

        private class Node : PathFinderNode,IComparable<Node>
        {
            public int g;
            public int f;
            public Node parent;

            public Node(int x, int y) : base(x, y)
            {
            }

            public int CompareTo(Node other)
            {
                return f - other.f;
            }

            public override string ToString()
            {
                return $"{base.ToString()}, G: {g}, F: {f}";
            }
        }
    }
}
