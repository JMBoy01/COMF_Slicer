using Clipper2Lib;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

using SlicerSettings = framework_iiw.Settings.Settings;


namespace framework_iiw.Modules
{
    internal class PathsRenderer
    {
        private Canvas canvas2D;
        private Border borderParent;

        private double offsetX = 0, offsetY = 0, scaleFactor = 1;

        public PathsRenderer(Canvas canvas, Border parent)
        {
            canvas2D = canvas;
            borderParent = parent;

            canvas2D.HorizontalAlignment = HorizontalAlignment.Center;
            canvas2D.VerticalAlignment = VerticalAlignment.Center;

            ScaleTransform flipTransform = new ScaleTransform(1, -1, 0.5, 0.5); // Flip around center
            canvas2D.RenderTransform = flipTransform;
        }

        // --- Initialise Offset

        public void InitRenderVariables(List<PathsD> layers)
        {
            offsetX = GetOffsetX(layers[0]);
            offsetY = GetOffsetY(layers[0]);
            scaleFactor = GetScaleFactor(layers);
        }

        private double GetOffsetX(PathsD paths)
        {
            double minX = double.MaxValue, maxX = double.MinValue;

            foreach (var path in paths)
            {
                foreach (var point in path)
                {
                    var x = point.x;

                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                }
            }

            return (-(maxX - minX) / 2) - minX;
        }

        private double GetOffsetY(PathsD paths)
        {

            double minY = double.MaxValue, maxY = double.MinValue;

            foreach (var path in paths)
            {
                foreach (var point in path)
                {
                    var y = point.y;

                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }

            return (-(maxY - minY) / 2) - minY;
        }

        private double GetScaleFactor(List<PathsD> layers)
        {
            double targetScreenPercentage = 0.5;

            var (width, height) = GetWidthAndHeight(layers);

            double maxWidth = borderParent.ActualWidth * targetScreenPercentage;
            double maxHeight = borderParent.ActualHeight * targetScreenPercentage;

            double widthScaleFactor = maxWidth / width;
            double heightScaleFactor = maxHeight / height;

            return Math.Min(widthScaleFactor, heightScaleFactor);
        }

        private (double, double) GetWidthAndHeight(List<PathsD> layers)
        {
            double minX = double.MaxValue, maxX = double.MinValue, minY = double.MaxValue, maxY = double.MinValue;

            foreach (var paths in layers)
            {
                foreach (var path in paths)
                {
                    foreach (var point in path)
                    {
                        if (point.x < minX) minX = point.x;
                        if (point.x > maxX) maxX = point.x;

                        if (point.y < minY) minY = point.y;
                        if (point.y > maxY) maxY = point.y;
                    }
                }
            }

            return (maxX - minX, maxY - minY);
        }

        // ------

        // --- Render A Layer

        public void RenderPaths(PathsD paths)
        {
            // !Important! Clear the canvas first
            canvas2D.Children.Clear();

            foreach (var path in paths)
            {
                RenderPath(path);
            }
        }

        private void RenderPath(PathD path)
        {
            var polygon = GetPolygon(path, Brushes.Green, SlicerSettings.NozzleThickness);

            canvas2D.Children.Add(polygon);
        }

        private Polygon GetPolygon(PathD points, Brush brush, double nozzleThickness)
        {
            Polygon polygon = new Polygon
            {
                Stroke = brush,
                StrokeThickness = nozzleThickness
            };

            foreach (PointD point in points)
            {
                // Calculate scaled coordinates
                var scaledX = (point.x + offsetX) * scaleFactor;
                var scaledY = (point.y + offsetY) * scaleFactor;

                var p = new Point(scaledX, scaledY);

                if (!polygon.Points.Contains(p))
                {
                    polygon.Points.Add(p);
                }
            }

            return polygon;
        }

        // ------

    }
}
