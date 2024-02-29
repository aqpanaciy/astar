using System.Diagnostics;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;

namespace Perpetuum.PathFinders
{
    public class PathFinderNode
    {
        public readonly int X;
        public readonly int Y;

        public PathFinderNode(int x,int y)
        {
            X = x;
            Y = y;
        }
        public override string ToString()
        {
            return string.Format($"Location: {X},{Y}");
        }
    }

    public abstract class PathFinder
    {
        public const float SQRT2 = 1.41f;

        protected static readonly Point[] EmptyPath = new Point[0];

        public delegate bool PathFinderNodePassableHandler(int x, int y);

        public delegate void PathFinderDebugHandler(PathFinderNode node, PathFinderNodeType type);

#if DEBUG
        private PathFinderDebugHandler _pathFinderDebug;
#endif

        //[CanBeNull]
        public Point[] FindPath(Point start, Point end, PathFinderNodePassableHandler handler)
        {
            return FindPath(start, end, handler, CancellationToken.None);
        }

        public Task<Point[]> FindPathAsync(Point start, Point end, PathFinderNodePassableHandler handler)
        {
            return Task.Run(() => FindPath(start, end, handler));
        }

        //[CanBeNull]
        public abstract Point[] FindPath(Point start, Point end, PathFinderNodePassableHandler handler, CancellationToken cancellationToken);

        [Conditional("DEBUG")]
        public void RegisterDebugHandler(PathFinderDebugHandler handler)
        {
#if DEBUG
            _pathFinderDebug += handler;
#endif
        }

        [Conditional("DEBUG")]
        protected void OnPathFinderDebug(PathFinderNode node, PathFinderNodeType type)
        {
#if DEBUG
            _pathFinderDebug?.Invoke(node, type);
#endif
        }

        protected virtual bool OnProcessNode(PathFinderNode node)
        {
            OnPathFinderDebug(node,PathFinderNodeType.Current);
            return true;
        }
    }
}
