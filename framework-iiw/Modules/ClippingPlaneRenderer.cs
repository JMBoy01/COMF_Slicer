using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace framework_iiw.Modules
{
    class ClippingPlaneRenderer
    {
        Viewport3D viewport3D;
        Model3DGroup model3DGroup;

        private GeometryModel3D clippingPlaneModel;

        public ClippingPlaneRenderer(Model3DGroup modelGroup) {
            InitializeClassVariables(modelGroup);
        }
    
        // --- Initialize Class Variables

        public void InitializeClassVariables(Model3DGroup modelGroup)
        {
            model3DGroup = modelGroup;

            clippingPlaneModel = new GeometryModel3D();
        }

        // ------

        
        // --- Render Clipping Plane

        public void RenderClippingPlane(GeometryModel3D geometryModel)
        {
            Rect3D geometryBounds = geometryModel.Bounds;

            double sizeX = geometryBounds.SizeX * 1.3;
            double sizeY = geometryBounds.SizeY * 1.3;

            RectangleVisual3D clippingPlaneRectangle = new RectangleVisual3D
            {
                Origin = new Point3D(0, 0, 0),
                Width = sizeX < sizeY ? sizeY : sizeX,
                Length = sizeX < sizeY ? sizeY : sizeX,
                Fill = new SolidColorBrush(Color.FromRgb(210, 210, 210))
            };

            clippingPlaneModel = clippingPlaneRectangle.Model;


            if (model3DGroup.Children.Count > 1) { model3DGroup.Children.RemoveAt(model3DGroup.Children.Count - 1); }

            model3DGroup.Children.Add(clippingPlaneModel);
        }
    
        // ------

        // --- Set Clipping Plane Offset Z

        public void SetClippingPlaneOffsetZ(double offsetZ)
        {
            Transform3DGroup transform3DGroup = new Transform3DGroup();

            TranslateTransform3D translateTransform3D = new TranslateTransform3D { OffsetZ = offsetZ };

            transform3DGroup.Children.Add(translateTransform3D);

            clippingPlaneModel.Transform = transform3DGroup;
        }
 
        // ------
    }
}
