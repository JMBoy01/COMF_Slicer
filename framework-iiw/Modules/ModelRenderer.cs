using HelixToolkit.Wpf;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media.Media3D;

namespace framework_iiw.Modules
{
    class ModelRenderer
    {
        HelixViewport3D viewport3D;

        Model3DGroup model3DGroup;
        ModelVisual3D modelVisual3D;

        public ModelRenderer(HelixViewport3D viewport, Model3DGroup modelGroup) {
            InitializeClassVariables(viewport, modelGroup);
        }

        // --- Initialize Class Variables

        private void InitializeClassVariables(HelixViewport3D viewport, Model3DGroup modelGroup)
        {
            viewport3D = viewport;
            model3DGroup = modelGroup;

            modelVisual3D = new ModelVisual3D{ Content = model3DGroup };

            viewport3D.Children.Add(modelVisual3D);
            viewport3D.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewport3D.PanGesture = new MouseGesture(MouseAction.LeftClick);
        }

        // ------


        // --- Render Model in viewport

        public void RenderModel(GeometryModel3D geometryModel, bool centerModelInView = true)
        {
            model3DGroup.Children.Add(geometryModel);

            // Optional: Makes large models more visible -- can be set to false
            if (centerModelInView) CenterModelInView(geometryModel);
        }

        private void CenterModelInView(GeometryModel3D geometryModel)
        {
            Rect3D geometryBounds = geometryModel.Bounds;

            // Translate the rendered object to the center of the clipping plane
            double displacementX = geometryBounds.X + geometryBounds.SizeX / 2;
            double displacementY = geometryBounds.Y + geometryBounds.SizeY / 2;

            geometryModel.Transform = new TranslateTransform3D(new Vector3D(-displacementX, -displacementY, -geometryBounds.Z));
        }

        // ------

        // --- Clear Viewport Before Rendering

        public void ClearViewport()
        {
            model3DGroup.Children.Clear();
        }

        // ------
    }
}
