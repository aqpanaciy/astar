using ClassLibraryAstar;
using System.Diagnostics;

namespace ProgramAstar
{
    internal class Program
    {
        static void Main(string[] args)
        {
            var number = Environment.ProcessorCount / 2;
            var tasks = new Task[number];

            var watch = Stopwatch.StartNew();
            for (var i = 0; i < number; i++)
            {
                tasks[i] = Task.Run(() =>
                {
                    var finder = new AStarFinderReusable();

                    for (var i = 0; i < 10000; i++)
                    {
                        var _ = finder.FindPath(100, 100, 2047 - 100, 2047 - 100,
                            Heuristic.Manhattan, (x, y) => true, CancellationToken.None);
                    }
                });
            }
            Task.WaitAll(tasks);
            watch.Stop();

            Console.WriteLine($"Time elapsed: {watch.ElapsedMilliseconds} ms");
        }
    }
}
