using System;
using System.Diagnostics;
using System.Drawing;
using System.Xml.Linq;
using Perpetuum.PathFinders;

namespace Astar
{
    internal class Program
    {
#if DEBUG
        private const int WIDTH  = 128;
        private const int HEIGHT = 128;
        private const int SHIFT  =  48;
        private const int AREA   =   2;
        private const int ITER   =   1;
#else
        private const int WIDTH  = 2048;
        private const int HEIGHT = 2048;
        private const int SHIFT  = 1000;
        private const int AREA   =    8;
        private const int ITER   =  100;
#endif

        private const double BLOCK = 0.55;

        private static Bitmap bmp;

        static void Main(string[] args)
        {
            var random = new Random(0);
            bool[,] blocks = new bool[WIDTH, HEIGHT];
            for (var x = 0; x < WIDTH; x++)
            {
                for (var y = 0; y < HEIGHT; y++)
                {
                    blocks[x, y] = random.NextDouble() > BLOCK;
                }
            }

            var start = new Point(WIDTH / 2 - SHIFT, HEIGHT / 2 - SHIFT);
            var end   = new Point(WIDTH / 2 + SHIFT - 1, HEIGHT / 2 + SHIFT - 1);

            for (var x = start.X - AREA; x <= start.X + AREA; x++)
            {
                for (var y = start.Y - AREA; y <= start.Y + AREA; y++)
                {
                    blocks[x, y] = true;
                }
            }
            for (var x = end.X - AREA; x <= end.X + AREA; x++)
            {
                for (var y = end.Y - AREA; y <= end.Y + AREA; y++)
                {
                    blocks[x, y] = true;
                }
            }

            for (var x = 0; x < WIDTH; x++)
            {
                blocks[x, 0] = false;
                blocks[x, HEIGHT - 1] = false;
            }
            for (var y = 0; y < HEIGHT; y++)
            {
                blocks[0, y] = false;
                blocks[WIDTH - 1, y] = false;
            }

            bmp = new Bitmap(WIDTH, HEIGHT);
            for (var x = 0; x < WIDTH; x++)
            {
                for (var y = 0; y < HEIGHT; y++)
                {
                    if (blocks[x, y])
                    {
                        bmp.SetPixel(x, y, Color.Green);
                    }
                    else
                    {
                        bmp.SetPixel(x, y, Color.Yellow);
                    }
                }
            }

            var pathFinder = new AStarFinder(Heuristic.Manhattan);
            pathFinder.RegisterDebugHandler(Handler);
            Point[] path = new Point[0];
            var watch = Stopwatch.StartNew();
            for (var i = 0; i < ITER; i++)
            {
                path = pathFinder.FindPath(start, end, (x, y) => blocks[x, y]);
            }
            watch.Stop();
#if DEBUG
            if (path.Length > 0)
            {
                for (var i = 0; i < path.Length; i++)
                {
                    bmp.SetPixel(path[i].X, path[i].Y, Color.Red);
                    bmp.Save($"./image_{_index++}.png", System.Drawing.Imaging.ImageFormat.Png);
                }
            }
#endif
            bmp.Dispose();

            Console.WriteLine("Elapsed: " + watch.Elapsed.ToString());
            Console.ReadLine();
        }

        private static int _index;

        private static void Handler(PathFinderNode node, PathFinderNodeType type)
        {
            Color color;

            switch (type)
            {
                case PathFinderNodeType.Start:
                    color = Color.Black; break;
                case PathFinderNodeType.End:
                    color = Color.Black; break;
                default:
                    color = Color.Blue; break;
            }

            bmp.SetPixel(node.X, node.Y, color);
            bmp.Save($"./image_{_index++}.png", System.Drawing.Imaging.ImageFormat.Png);
        }

    }
}
