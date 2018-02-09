using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

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
            pManager.AddNumberParameter("Degree", "D", "Degree of folding", GH_ParamAccess.item);
            pManager.AddCurveParameter("Fold Curves", "FC", "Lines to be folded", GH_ParamAccess.list);
            //pManager[0].Optional = true; to change parameter properties
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Final mesh that is flattenable", GH_ParamAccess.item);
            pManager.AddLineParameter("Edge Lines", "E", "Edges being folded", GH_ParamAccess.item);
            //pManager.HideParameter(0);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            double degree = 0;
            var folds = new List<Curve>();

            DA.GetData(0, ref mesh);
            DA.GetData(1, ref degree);
            /*DA.GetDataList(2, folds)*/;
            //if (!DA.GetData(0, ref mesh)) return;
            //if (!DA.GetData(1, ref force)) return;
            if (!DA.GetDataList(2, folds)) return;

            var simulation = new FoldingSimulation(mesh, degree, folds);
            Mesh outMesh = simulation.OutMesh;
            DA.SetData(0, outMesh);

            List<Line> foldings = simulation.foldings;
            DA.SetDataList(1, foldings);
        }
    }
}
