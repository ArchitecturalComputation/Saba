using System;
using System.Linq;
using Rhino.Geometry;
using Rhino.Collections;
using KangarooSolver;
using KangarooSolver.Goals;
using System.Collections.Generic;

namespace CurvedCreaseFolding
{
    class FoldingSimulation
    {
        public Mesh OutMesh { get; private set; }
        public List<Line> Foldings { get; private set; }

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
            //AngleAnalysis();
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
                    // check tangency to curve as well!

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
                for (int i = 0; i < points.Length; i++)
                {
                    var adjacencies = mesh.TopologyVertices.ConnectedTopologyVertices(i, true);
                    var xx = mesh.TopologyVertices.ConnectedTopologyVertices(i, false);
                    int index = mesh.TopologyVertices.MeshVertexIndices(i).First();
                    Vector3d normal = mesh.Normals[index];
                    var plane = new Plane(points[i], normal);

                    var vectors = new Vector3d[adjacencies.Length];
                    var angles = new Double[adjacencies.Length];
                    for (int j = 0; j < adjacencies.Length; j++)
                    {
                        vectors[j] = new Vector3d(points[adjacencies[j]] - points[i]);
                        if (j < adjacencies.Length - 1) angles[j] = Vector3d.VectorAngle(vectors[j], vectors[j + 1]);
                        else angles[j] = Vector3d.VectorAngle(vectors[j], vectors[0]);
                    }

                    for (int j = 0; j < adjacencies.Length; j++)
                    {
                        var finalAngle = angles[j] * Math.PI * 2 / angles.Sum();
                        Angle angle;
                        if (j < adjacencies.Length - 1) angle = new Angle(1.0, finalAngle, i, adjacencies[j], i, adjacencies[j + 1]);
                        else angle = new Angle(1.0, finalAngle, i, adjacencies[j], i, adjacencies[0]);
                        goals.Add(angle);
                    }
                    // add mesh colors after mesh shows up! :))
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
                int counter = 0;
                double threshold = 1e-9;
                do
                {
                    simulation.Step(goals, true, threshold);
                    counter++;
                } while (simulation.GetvSum() > threshold && counter < 200);

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
