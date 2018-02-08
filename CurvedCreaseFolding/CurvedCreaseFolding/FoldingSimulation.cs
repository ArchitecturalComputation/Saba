using System;
using System.Linq;
using Rhino.Geometry;
using KangarooSolver;
using KangarooSolver.Goals;
using System.Collections.Generic;
//using System.Diagnostics;

namespace CurvedCreaseFolding
{
    class FoldingSimulation
    {
        public Mesh OutMesh { get; private set; }

        public FoldingSimulation(Mesh mesh, double degree, List<Curve> folds)
        {
            Simulate(mesh, degree, folds);
        }

        public List<Line> foldings = new List<Line> ();

        void Simulate(Mesh mesh, double degree, List<Curve> folds)
        {
            mesh.Weld(Math.PI);
            mesh.Faces.ConvertQuadsToTriangles();
            var simulation = new PhysicalSystem();
            var particles = mesh.TopologyVertices
                .Select(p => new Point3d(p.X, p.Y, p.Z))
                .ToList();

            simulation.SetParticleList(particles);
            var goals = new List<IGoal>();
            var points = mesh.TopologyVertices.ToArray();

            Edges();
            Hinges();
            //Anchors();
            Simulation();

            void Edges()
            {
                for (int i = 0; i < mesh.TopologyEdges.Count; i++)
                {
                    var pair = mesh.TopologyEdges.GetTopologyVertices(i);
                    Vector3d vector = mesh.TopologyVertices[pair.I] - mesh.TopologyVertices[pair.J];
                    double length = vector.Length;
                    var spring = new Spring(pair.I, pair.J, length, 1.0);
                    goals.Add(spring);
                }
            }

            void Hinges()
            {
                double minDist = 0;
                for (int i = 0; i < mesh.TopologyEdges.Count; i++) minDist += mesh.TopologyEdges.EdgeLine(i).Length;
                minDist /= mesh.TopologyEdges.Count;

                for (int i = 0; i < mesh.TopologyEdges.Count; i++)
                {
                    var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(i);

                    double dist = 0.0;
                    var midPoint = (float)0.5 * points[edgeVertices.I] + (float)0.5 * points[edgeVertices.J];
                    foreach (Curve c in folds) c.ClosestPoint(midPoint, out dist, 0.2 * minDist);

                    if (dist == 0.0) continue;
                    foldings.Add(mesh.TopologyEdges.EdgeLine(i));
                    int[] faces = mesh.TopologyEdges.GetConnectedFaces(i, out bool[] direction);
                    if (faces.Length != 2) continue;

                    if (!direction[0]) faces.Reverse();
                    var faceVertices = faces.Select(f => mesh.TopologyVertices.IndicesFromFace(f));

                    var opposites = faceVertices.Select(f => f.First(v => !edgeVertices.Contains(v))).ToArray();

                    var hinge = new Hinge(edgeVertices.I, edgeVertices.J, opposites.First(), opposites.Last(), -degree, 1.0);
                    goals.Add(hinge);
                }
            }

            void Anchors()
            {
                for (int i = 0; i < mesh.TopologyVertices.Count; i++)
                {
                    int edgeCount = mesh.TopologyVertices.ConnectedEdgesCount(i);
                    if (edgeCount <= 3)
                    {
                        Point3d p = mesh.TopologyVertices[i];
                        var anchor = new Anchor(i, p, 10000);
                        goals.Add(anchor);
                    }
                }
            }

            void Simulation()
            {
                //var watch = new Stopwatch();
                //watch.Start();

                int counter = 0;
                double threshold = 1e-9;
                do
                {
                    simulation.Step(goals, true, threshold);
                    counter++;
                } while (simulation.GetvSum() > threshold && counter < 200);

                //watch.Stop();
                //Console.WriteLine(watch.ElapsedMilliseconds);
                //watch.Restart();
                
                var outMesh = new Mesh();
                outMesh.Faces.AddFaces(mesh.Faces);

                var outParticles = simulation.GetPositionArray();
                var outVertices = Enumerable.Range(0, mesh.Vertices.Count)
                    .Select(i => outParticles[mesh.TopologyVertices.TopologyVertexIndex(i)]);

                outMesh.Vertices.AddVertices(outVertices);
                outMesh.Unweld(0.01, true);
                OutMesh = outMesh;
            }
        }
    }
}
