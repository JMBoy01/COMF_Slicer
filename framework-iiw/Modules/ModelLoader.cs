using System;
using System.Windows.Media.Media3D;
using framework_iiw.Exceptions;
using HelixToolkit.Wpf;


namespace framework_iiw.Modules
{
    class ModelLoader
    {
        private readonly StLReader stlReader;

        private string fileName;

        public ModelLoader(string path) {
            
            stlReader = new StLReader();
            fileName = path;
        }

        public GeometryModel3D Load()
        {
     
            Model3DGroup modelGroup3D = new ModelImporter().Load(fileName) ?? throw new ModelLoadException("No model found in given file: " + fileName + "!");
            
            return FindLargestModel(modelGroup3D);
        }

        public static MeshGeometry3D LoadMesh(GeometryModel3D geometryModel)
        {
            return geometryModel.Geometry as MeshGeometry3D;
        }

        private GeometryModel3D FindLargestModel(Model3DGroup group)
        {
            /*
             * HelixToolkit automatically places all object from a file into one group, 
             * in case there are more models you have to determine which one you need, 
             * or simply run the slicing algorithm for each model. 
             * Another solution is to always take the first model in the list, 
             * but you can also try to find the largest model to avoid picking debris left by errors in the model. 
             */

            if (group.Children.Count == 1)
                return group.Children[0] as GeometryModel3D;

            int maxCount = int.MinValue;
            GeometryModel3D maxModel = null;

            foreach (GeometryModel3D model in group.Children)
            {
                int count = ((MeshGeometry3D)model.Geometry).Positions.Count;
                if (maxCount < count)
                {
                    maxCount = count;
                    maxModel = model;
                }
            }

            return maxModel;
        }
    }
}
