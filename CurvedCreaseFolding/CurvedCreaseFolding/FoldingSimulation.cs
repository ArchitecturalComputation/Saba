using System;
using System.Linq;
using Rhino.Geometry;
using Rhino.Collections;
using KangarooSolver;
using KangarooSolver.Goals;
using System.Collections.Generic;
//using System.Diagnostics;

namespace CurvedCreaseFolding
{
    class FoldingSimulation
    {
        public Mesh OutMesh { get; private set; }
        public List<Line> Foldings { get; private set; }
        public List<Double> AngleDifferences { get; private set; }

        public FoldingSimulation(Mesh mesh, double degree, List<Curve> folds)
        {
            Simulate(mesh, degree, folds);
        }


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
            AngleAnalysis();
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
                var foldings = new List<Line>();
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
                Foldings = foldings;
            }

            void AngleAnalysis()
            {
                var angleDifferences = new List<Double>();
                var normals = mesh.Normals;
                for (int i = 0; i < points.Length; i++)
                {
                    var adjacencies = mesh.TopologyVertices.ConnectedEdges(i);
                    var vectors = new RhinoList<Vector3f>(); //RhinoList so it's sortable by degs
                    var plane = new Plane(points[i], normals[i]); //is it gonna be the same vertice (due to topology vs normal vertices)
                    var degs = new Double[adjacencies.Length]; //degrees on vertice plane, need to calculate planes seperately for mesh degrees
                    //int[] edgeVertices = new int[adjacencies.Length];
                    var edgeVertices = new RhinoList<int>();
                    for (int j = 0; j < adjacencies.Length; j++)
                    {
                        var tempVertices = mesh.TopologyEdges.GetTopologyVertices(adjacencies[j]);
                        if (tempVertices.I != i) edgeVertices[j] = tempVertices.I;
                        else edgeVertices[j] = tempVertices.J;
                        //int edgeVertice;
                        //if (tempVertices.I != i) edgeVertice = tempVertices.I;
                        //else edgeVertice = tempVertices.J;
                        //vectors[j] = points[edgeVertice] - points[i];
                        vectors[j] = points[edgeVertices[j]] - points[i];
                        degs[j] = Vector3d.VectorAngle(plane.XAxis, vectors[j], plane);
                    }
                    vectors.Sort(degs);
                    edgeVertices.Sort(degs);
                    Vector3f[] sortedVectors = vectors.ToArray();
                    double[] degrees = new double[sortedVectors.Length];
                    for (int j = 0; j < sortedVectors.Length; j++)
                        degrees[j] = Vector3d.VectorAngle(sortedVectors[j], sortedVectors[(j + 1) % sortedVectors.Length]);
                    //double sumDegrees;
                    double diffAngle = (Math.PI * 2 - degrees.Sum()) / adjacencies.Length;
                    for (int j = 0; j < adjacencies.Length; j++)
                    {
                        var angle = new Angle(1000.0, degrees[j] + diffAngle, i, edgeVertices[j], i, edgeVertices[(j + 1) % sortedVectors.Length]);
                        goals.Add(angle);
                    }
                    angleDifferences.Add(diffAngle);
                }
                AngleDifferences = angleDifferences;
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
