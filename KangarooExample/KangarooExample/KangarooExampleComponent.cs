using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;


namespace KangarooExample
{
    public class KangarooExampleComponent : GH_Component
    {
        public KangarooExampleComponent() : base("KangarooExample", "ExampleSim", "This component is a Kangaroo simulation.", "AC", "Studio 2") { }
        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("fd8536c5-cd18-49db-8ecf-ffb5de01318d");

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "This is my input mesh.", GH_ParamAccess.item);
            pManager.AddNumberParameter("Force", "F", "This is a force.", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "This is my output mesh.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            double force = 0;

            DA.GetData(0, ref mesh);
            DA.GetData(1, ref force);

            var simulation = new SimpleSimulation(mesh, force);
            Mesh outMesh = simulation.OutMesh;
            DA.SetData(0, outMesh);
        }
    }
}
