using System;
using System.Diagnostics;
using System.Drawing;
using System.Xml.Linq;
using Perpetuum.PathFinders;

namespace Astar
{
    internal class Program
    {
        private const int WEIGHT = 128;
        private const int HEIGHT = 128;
        private const int SHIFT  =  48;
        private const int AREA   =   2;
#if DEBUG
        private const int ITER   =   1;
#else
        private const int ITER   = 100;
#endif

        private const double BLOCK = 0.55;

        private static Bitmap bmp;

        static void Main(string[] args)
        {
            var random = new Random();
            bool[,] blocks = new bool[WEIGHT, HEIGHT];
            for (var x = 0; x < WEIGHT; x++)
            {
                for (var y = 0; y < HEIGHT; y++)
                {
                    blocks[x, y] = random.NextDouble() > BLOCK;
                }
            }

            var start = new Point(WEIGHT / 2 - SHIFT, HEIGHT / 2 - SHIFT);
            var end   = new Point(WEIGHT / 2 + SHIFT - 1, HEIGHT / 2 + SHIFT - 1);

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

            bmp = new Bitmap(WEIGHT, HEIGHT);
            for (var x = 0; x < WEIGHT; x++)
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

            var pathFinder = new AStarFinder(Heuristic.Manhattan, (x, y) =>
            {
                if (x < 0 || x >= WEIGHT || y < 0 || y >= HEIGHT)
                {
                    return false;
                }

                return blocks[x, y];
            });
            pathFinder.RegisterDebugHandler(Handler);

            Point[] path = new Point[0];
            var watch = Stopwatch.StartNew();
            for (var i = 0; i < ITER; i++)
            {
                path = pathFinder.FindPath(start, end);
            }
            watch.Stop();

            if (path.Length > 0)
            {
                for (var i = 0; i < path.Length; i++)
                {
                    bmp.SetPixel(path[i].X, path[i].Y, Color.Red);
                    bmp.Save($"./image_{_index++}.png", System.Drawing.Imaging.ImageFormat.Png);
                }
            }


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

            bmp.SetPixel(node.Location.X, node.Location.Y, color);
            bmp.Save($"./image_{_index++}.png", System.Drawing.Imaging.ImageFormat.Png);
        }

    }
}
