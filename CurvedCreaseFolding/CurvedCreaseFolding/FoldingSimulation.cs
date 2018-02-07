using System;
using System.Linq;
using Rhino.Geometry;
using KangarooSolver;
using KangarooSolver.Goals;
using System.Collections.Generic;

namespace CurvedCreaseFolding
{
    class FoldingSimulation
    {
        public Mesh OutMesh { get; private set; }

        public FoldingSimulation(Mesh mesh, double degree)
        {
            Simulate(mesh, degree);
        }

        void Simulate(Mesh mesh, double degree)
        {
            mesh.Weld(0.000001);
            mesh.Faces.ConvertQuadsToTriangles();
            var simulation = new PhysicalSystem();
            var particles = mesh.TopologyVertices
                .Select(p => new Point3d(p.X, p.Y, p.Z))
                .ToList();

            simulation.SetParticleList(particles);
            var goals = new List<IGoal>();

            Edges();
            Anchors();
            Hinges();
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

            void Hinges()
            {
                for (int i = 0; i < mesh.TopologyEdges.Count; i++)
                {
                    int[] faces = mesh.TopologyEdges.GetConnectedFaces(i, out bool[] direction);
                    if (faces.Length != 2) continue;

                    if (!direction[0]) faces.Reverse();
                    var faceVertices = faces.Select(f => mesh.TopologyVertices.IndicesFromFace(f));
                    var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(i);
                    var opposites = faceVertices.Select(f => f.First(v => !edgeVertices.Contains(v))).ToArray();

                    var hinge = new Hinge(edgeVertices.I, edgeVertices.J, opposites.First(), opposites.Last(), degree, 1.0);
                    goals.Add(hinge);
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
                OutMesh = outMesh;
            }
        }
    }
}
