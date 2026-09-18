namespace ClassLibraryAstar
{
    /// <summary>
    /// The class encapsulates the A* algorithm for pathfinding on a rectangular map.
    /// </summary>
    public sealed class AStarFinderReusable
    {
        /// <summary>
        /// Map width and height.
        /// </summary>
        public const int HW = 2048;
        /// <summary>
        /// Mask for extracting the X-coordinate from a hash code.
        /// </summary>
        private const int HW_X_MASK = 0x7ff;
        /// <summary>
        /// Offset for obtaining the Y-coordinate from the hash code.
        /// </summary>
        private const int HW_Y_SHIFT = 11;

        /// <summary>
        /// Distance weight along the coordinate axes.
        /// </summary>
        private const int WEIGHT = 10;
        /// <summary>
        /// Weight of the diagonal distance.
        /// </summary>
        private const int WEIGHT_DIAG = (int)(WEIGHT * 1.41f);

        /// <summary>
        /// A function for determining the traversability of points on the map.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns>True if the point is passable.</returns>
        public delegate bool PathFinderNodePassableHandler(int x, int y);

        /// <summary>
        /// Array for path backtracking. Links a point to its parent.
        /// </summary>
        private readonly int[] _array = new int[HW * HW];
        /// <summary>
        /// stamp-technique by Claude.AI
        /// </summary>
        private int _currentStamp = 0;
        /// <summary>
        /// A priority queue for selecting a new point for range analysis.
        /// </summary>
        private readonly PriorityQueue<Node, int> _list = new PriorityQueue<Node, int>(500);

        private readonly Action<int, int>? _onNewPoint;

        public AStarFinderReusable(Action<int, int>? onNewPoint = null)
        {
            _onNewPoint = onNewPoint;
        }

        /// <summary>
        /// The main pathfinding procedure.
        /// </summary>
        /// <param name="startX">Coordinate of the starting point of the path.</param>
        /// <param name="startY">Coordinate of the starting point of the path.</param>
        /// <param name="endX">Coordinate of the path endpoint.</param>
        /// <param name="endY">Coordinate of the path endpoint.</param>
        /// <param name="heuristic">Method for estimating the distance to the end of the route.</param>
        /// <param name="passableHandler">Passability for each point on the map.</param>
        /// <param name="cancellationToken">Pathfinding cancellation token.</param>
        /// <returns></returns>
        public Tuple<int, int>[]? FindPath(int startX, int startY,
            int endX, int endY,
            Heuristic heuristic,
            PathFinderNodePassableHandler passableHandler,
            CancellationToken cancellationToken)
        {
            // Input data validation.
            if (startX < 0 || startX >= HW || startY < 0 || startY >= HW || (startX == 0 && startY == 0))
            {
                return null;
            }
            if (endX < 0 || endX >= HW || endY < 0 || endY >= HW || (endX == 0 && endY == 0))
            {
                return null;
            }

            // The endpoint must be reachable.
            if (!passableHandler(endX, endY))
            {
                return null;
            }

            // If the start and end points coincide, we return an empty path.
            if (startX == endX && startY == endY)
            {
                return Array.Empty<Tuple<int, int>>();
            }

            _currentStamp++;
            if (_currentStamp >= 1024)
            {
                _currentStamp = 1;
                // We clear the array.
                Array.Clear(_array);
            }

            // Start and end nodes.
            var endNode = new Node(endX, endY, 0);
            var startNode = new Node(startX, startY, 0);
            // We place the start of the path into the queue.
            _list.Enqueue(startNode, startNode.G + heuristic.Calculate(startNode.X, startNode.Y, endX, endY) * WEIGHT);
            // The starting point is its own parent.
            _array[startNode.Hash] = (_currentStamp << 22) | startNode.Hash;

            try
            {
                // We select from the queue the node with the minimum path cost from the start and the estimated remaining path cost.
                while (_list.TryDequeue(out Node node, out _) && !cancellationToken.IsCancellationRequested)
                {
                    // If we are at the end of the path, we conclude the search.
                    if (node.Hash == endNode.Hash)
                    {
                        return Backtrace(node, _array);
                    }

                    // For all neighboring nodes.
                    foreach (var neighbor in GetNeighbors(node, passableHandler))
                    {
                        int stamp = (int)((uint)_array[neighbor.Hash] >>> 22);
                        // If the neighboring node has a parent, it means we have already analyzed it.
                        if (stamp == _currentStamp)
                        {
                            continue;
                        }
                        // We add a parent to the node.
                        _array[neighbor.Hash] = (_currentStamp << 22) | node.Hash;
                        _onNewPoint?.Invoke(neighbor.X, neighbor.Y);

                        // We add this node to the queue with a weight equal to the path from the start plus the estimated path to the end.
                        _list.Enqueue(neighbor, neighbor.G + heuristic.Calculate(neighbor.X, neighbor.Y, endX, endY) * WEIGHT);
                    }
                }
            }
            finally
            {
                // We clear the queue.
                _list.Clear();
            }

            // Path not found.
            return null;
        }

        /// <summary>
        /// Reverse path tracing from end to beginning.
        /// </summary>
        /// <param name="node">End node of the path.</param>
        /// <param name="array">An array in which nodes are connected from the start node to the end node.</param>
        /// <returns></returns>
        private static Tuple<int, int>[] Backtrace(Node node, int[] array)
        {
            // Stack for placing nodes.
            var stack = new Stack<Tuple<int, int>>();
            // End-node data.
            var x = node.X;
            var y = node.Y;
            int d = node.Hash;
            int parent = array[d] & 0x3FFFFF;
            // We continue executing as long as the node's parent and the node itself do not coincide.
            while (parent != d)
            {
                // We push the current node onto the stack.
                stack.Push(Tuple.Create(x, y));
                // We take the parent node.
                d = parent;
                // Decoding the coordinates.
                x = d & HW_X_MASK;
                y = d >> HW_Y_SHIFT;
                parent = array[d] & 0x3FFFFF;
            }
            // We place the start node onto the stack.
            stack.Push(Tuple.Create(x, y));

            // We convert the stack into an array, from the start node to the end node.
            return stack.ToArray();
        }

        /// <summary>
        /// Axial displacements for the nearest nodes.
        /// </summary>
        private static readonly int[,] _n = { { -1, -1 }, { 0, -1 }, { 1, -1 }, { -1, 0 }, { 1, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 } };
        /// <summary>
        /// We obtain the coordinates of the nearest nodes.
        /// </summary>
        /// <param name="node">Center node</param>
        /// <param name="passableHandler"></param>
        /// <returns>We return only the nearest traversable nodes.</returns>
        private static IEnumerable<Node> GetNeighbors(Node node, PathFinderNodePassableHandler passableHandler)
        {
            for (var i = 0; i < 8; i++)
            {
                var dx = _n[i, 0];
                var dy = _n[i, 1];

                var nx = node.X + dx;
                var ny = node.Y + dy;

                if (nx < 0 || nx > HW-1 || ny < 0 || ny > HW-1 || (nx == 0 && ny == 0))
                {
                    continue;
                }

                if (passableHandler(nx, ny))
                {
                    // For new nodes, we calculate the path from the starting node.
                    yield return new Node(nx, ny, node.G + (dx == 0 || dy == 0 ? WEIGHT : WEIGHT_DIAG));
                }
            }
        }

        /// <summary>
        /// Node class for pathfinding.
        /// </summary>
        private struct Node
        {
            /// <summary>
            /// The encoded coordinates are unique to each node.
            /// </summary>
            public int Hash;
            /// <summary>
            /// Decoding the coordinate.
            /// </summary>
            public int X => Hash & HW_X_MASK;
            /// <summary>
            /// Decoding the coordinate.
            /// </summary>
            public int Y => Hash >> HW_Y_SHIFT;
            /// <summary>
            /// Distance to the start of the route, taking the weighting factor into account.
            /// </summary>
            public int G;

            public Node(int x, int y, int g)
            {
                Hash = (y << HW_Y_SHIFT) + x;
                G = g;
            }
        }
    }
}
