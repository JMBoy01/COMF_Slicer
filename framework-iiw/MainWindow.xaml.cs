using Clipper2Lib;
using framework_iiw.Modules;
using framework_iiw.Settings;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media.Media3D;

using SlicerSettings = framework_iiw.Settings.Settings;

namespace framework_iiw
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Model3DGroup model3DGroup;

        private ModelSlicer modelSlicer;
        private PathsRenderer pathsRenderer;
        private ModelRenderer stlModelRenderer;
        private ClippingPlaneRenderer clippingPlaneRenderer;

        private List<PathsD> layers;

        public MainWindow()
        {
            InitializeComponent();
            InitializeClassVariables();
        }

        // --- Initialize Class Variables

        public void InitializeClassVariables()
        {
            model3DGroup = new Model3DGroup();

            modelSlicer = new ModelSlicer();

            stlModelRenderer = new ModelRenderer(objectViewPort, model3DGroup);
            clippingPlaneRenderer = new ClippingPlaneRenderer(model3DGroup);
            pathsRenderer = new PathsRenderer(canvas, canvasBorder);
        }

        // --- Load Model Button Functionality

        private void Load_Model_Button_Click(object sender, RoutedEventArgs e)
        {
            var fileDialog = new OpenFileDialog();

            var answer = ShowFileDialog(fileDialog, ".stl");

            if (NoFileUploaded(answer, fileDialog.FileName)) return;

            // !Important! Clear the viewport first
            stlModelRenderer.ClearViewport();

            var geometryModel3D = LoadGeometryModelFromFile(fileDialog.FileName);

            InitializeModel(geometryModel3D);
            InitializeClippingPlane(geometryModel3D);
            InitializeSlicer(geometryModel3D);
        }

        private bool? ShowFileDialog(OpenFileDialog fileDialog, string defaultExtension)
        {
            fileDialog.DefaultExt = defaultExtension;
            return fileDialog.ShowDialog();
        }

        private bool NoFileUploaded(bool? answer, string fileName)
        {
            return answer == null || fileName == "";
        }

        private GeometryModel3D LoadGeometryModelFromFile(string filename)
        {
            var stlModelLoader = new ModelLoader(filename);
            
            return stlModelLoader.Load();
        }

        private void InitializeModel(GeometryModel3D geometryModel)
        {
            stlModelRenderer.RenderModel(geometryModel);

            camera.Position = new Point3D(geometryModel.Bounds.Location.X + (geometryModel.Bounds.SizeX / 2), -50, 50);
        }

        private void InitializeClippingPlane(GeometryModel3D geometryModel)
        {
            var numberOfSlices = (geometryModel.Bounds.SizeZ / SlicerSettings.LayerHeight) - 1;

            clippingPlaneRenderer.RenderClippingPlane(geometryModel);

            clippingPlaneSlider.Maximum = numberOfSlices;
        }
        
        private void InitializeSlicer(GeometryModel3D geometryModel)
        {
            modelSlicer.SetGeometryModel3D(geometryModel);

        }
        
        // -------


        // --- Slicing Plane UI Functionality

        private void ClippingPlaneSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            //modelSlicer.SetSliderValue(clippingPlaneSlider.Value);

            var clippingPlaneSliderValue = clippingPlaneSlider.Value * SlicerSettings.LayerHeight;

            clippingPlaneRenderer.SetClippingPlaneOffsetZ(clippingPlaneSliderValue);

            if (layers == null) return;

            //var slicedLayer = layers[(int)clippingPlaneSlider.Value];*/

            pathsRenderer.RenderPaths(layers[Convert.ToInt32(clippingPlaneSlider.Value)]);
            Console.WriteLine(clippingPlaneSlider.Value);
        }

        // ------


        // --- Slice Model Button Functionality

        private void Slice_Model_Button_Click(object sender, RoutedEventArgs e)
        {
            layers = modelSlicer.Slice();

            ResetInterface(layers);
            RenderFirstSlice(layers);
        }

        private void RenderFirstSlice(List<PathsD> layers)
        {
            pathsRenderer.RenderPaths(layers[0]);
        }

        private void ResetInterface(List<PathsD> layers)
        {
            clippingPlaneSlider.Value = 0;
            pathsRenderer.InitRenderVariables(layers);
        }

        // ------
    }
}
