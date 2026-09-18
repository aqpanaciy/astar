using ClassLibraryAstar;
using SkiaSharp;
using System.Diagnostics;
using System.Reflection;
using System.Xml.Linq;

namespace ProgramAstar
{
    internal class Program
    {
        static int delta = 100;
        static bool[,] obstacle = new bool[AStarFinderReusable.HW, AStarFinderReusable.HW];
        static bool[,] node = new bool[AStarFinderReusable.HW, AStarFinderReusable.HW];

        static SKColor nColor = new SKColor(0x00, 0x00, 0xff);
        static SKColor bColor = new SKColor(0x00, 0xff, 0x00);
        static SKColor oColor = new SKColor(0xff, 0xff, 0x00);
        static SKColor pColor = new SKColor(0xff, 0x00, 0x00);

        static void Main(string[] args)
        {
            var pathFinder = new AStarFinderReusable(NewPoint);
            var random = new Random();

            for (var x = 1; x < AStarFinderReusable.HW - 1; x++)
            {
                for (var y = 1; y < AStarFinderReusable.HW - 1; y++)
                {
                    if (random.NextDouble() > 0.515)
                    {
                        obstacle[x, y] = true;
                    }
                }
            }

            for (var x = delta - delta / 2; x < delta + delta / 2; x++)
            {
                for (var y = delta - delta / 2; y < delta + delta / 2; y++)
                {
                    obstacle[x, y] = true;
                }
            }
            for (var x = AStarFinderReusable.HW - delta - delta / 2; x < AStarFinderReusable.HW - delta + delta / 2; x++)
            {
                for (var y = AStarFinderReusable.HW - delta - delta / 2; y < AStarFinderReusable.HW - delta + delta / 2; y++)
                {
                    obstacle[x, y] = true;
                }
            }

            using (var bitmap = new SKBitmap(AStarFinderReusable.HW, AStarFinderReusable.HW))
            {
                for (var x = 0; x < AStarFinderReusable.HW; x++)
                {
                    for (var y = 0; y < AStarFinderReusable.HW; y++)
                    {
                        if (obstacle[x, y])
                        {
                            bitmap.SetPixel(x, y, bColor);
                        }
                        else
                        {
                            bitmap.SetPixel(x, y, oColor);
                        }
                    }
                }

                var Iter = 10000;
                Tuple<int, int>[]? resultOne = null;
                one = true;
                var watch = Stopwatch.StartNew();
                for (var i = 0; i < Iter; i++)
                {
                    var result = pathFinder.FindPath(delta, delta,
                        AStarFinderReusable.HW - delta, AStarFinderReusable.HW - delta, Heuristic.Manhattan,
                        (x, y) => obstacle[x, y], CancellationToken.None);
                    
                    if (one)
                    {
                        resultOne = result;
                        one = false;
                    }
                }
                watch.Stop();
                Console.WriteLine($"Result: {resultOne?.Length} Elapse: {(double)watch.ElapsedMilliseconds / Iter} ms");

                for (var x = 0; x < AStarFinderReusable.HW; x++)
                {
                    for (var y = 0; y < AStarFinderReusable.HW; y++)
                    {
                        if (node[x, y])
                        {
                            bitmap.SetPixel(x, y, nColor);
                        }
                    }
                }

                if (resultOne != null)
                {
                    for (var i = 0; i < resultOne.Length; i++)
                    {
                        bitmap.SetPixel(resultOne[i].Item1, resultOne[i].Item2, pColor);
                    }
                }

                using (var stream = new FileStream("output.png", FileMode.Create, FileAccess.Write))
                using (var image = SKImage.FromBitmap(bitmap))
                using (var encodedImage = image.Encode(SKEncodedImageFormat.Png, 100))
                {
                    encodedImage.SaveTo(stream);
                }
            }
        }

        static bool one;
        static void NewPoint(int x, int y)
        {
            if (one)
            {
                node[x, y] = true;
            }
        }
    }
}
