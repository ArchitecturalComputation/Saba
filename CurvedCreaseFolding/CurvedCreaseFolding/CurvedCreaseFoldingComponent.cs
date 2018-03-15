using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;
using SpatialSlur.SlurCore;
using SpatialSlur.SlurMesh;

namespace CurvedCreaseFolding
{
    public class CurvedCreaseFoldingComponent : GH_Component
    {
        public CurvedCreaseFoldingComponent() : base("CurvedCreaseFolding", "CrvCrsFld", "Find the curved creases to build a desired geometry", "Folding", "CurvedCreases") { }
        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("0307911d-e027-4751-b6ee-b1e5cfa9b3c7");
        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Initial mesh to create", GH_ParamAccess.item);
            pManager.AddNumberParameter("Angle", "A", "Angle of folding in Radians", GH_ParamAccess.item);
            pManager.AddCurveParameter("Fold Curves", "FC", "Lines to be folded", GH_ParamAccess.list);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh3D", "M3D", "Final 3D mesh that is flattenable", GH_ParamAccess.item);
            pManager.AddMeshParameter("Mesh2D", "M2D", "Final mesh flattened", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            double angle = 0;
            var folds = new List<Curve>();

            DA.GetData(0, ref mesh);
            DA.GetData(1, ref angle);
            DA.GetDataList(2, folds);

            var simulation = new FoldingSimulation(mesh, angle, folds);

            Mesh outMesh = simulation.OutMesh;
            DA.SetData(0, outMesh);

            Mesh outMeshFlat = simulation.OutMeshFlat;
            DA.SetData(1, outMeshFlat);
        }
    }
}
