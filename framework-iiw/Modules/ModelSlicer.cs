﻿using Clipper2Lib;
using framework_iiw.Data_Structures;
using framework_iiw.Exceptions;
using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Printing.IndexedProperties;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using LineSegment = framework_iiw.Data_Structures.LineSegment;
using SlicerSettings = framework_iiw.Settings.Settings;


namespace framework_iiw.Modules
{
    class ModelSlicer
    {
        GeometryModel3D? geometryModel3D;

        private double sliderValue = 0;

        public ModelSlicer() {}
    
        // --- Slice

        public List<PathsD> Slice()
        {
            if (geometryModel3D == null) { throw new NullException("Geometry Model Must Be Loaded First"); }

            MeshGeometry3D meshGeometry3D = ModelLoader.LoadMesh(geometryModel3D);

            List<int> triangleIndices = meshGeometry3D.TriangleIndices.ToList();
            List<Point3D> positions = meshGeometry3D.Positions.ToList();

            var layers = new List<PathsD>();
            var totalAmountOfLayers  = geometryModel3D.Bounds.SizeZ / SlicerSettings.LayerHeight;

            for (var idx = 0; idx < totalAmountOfLayers; idx++)
            {
                PathsD layer = SliceModelAtSpecificLayer(idx * SlicerSettings.LayerHeight, meshGeometry3D, triangleIndices, positions);
                List<PathsD> layerWithShell = ProcessContours(layer); // Return layerWithShell = {outerShell, innerShell}
                PathsD outerShell = layerWithShell[0];
                PathsD innerShell = layerWithShell[1];

                PathsD infillPaths = ProcessInfill(innerShell, meshGeometry3D);

                PathsD processedLayer = new PathsD();
                processedLayer.AddRange(outerShell);
                processedLayer.AddRange(innerShell);
                processedLayer.AddRange(infillPaths);

                layers.Add(processedLayer);
            }
            generateGCodes(layers);
            
            return layers;
        }

        


        // --- Slice Object At Specific Layer

        private PathsD SliceModelAtSpecificLayer(double layer, MeshGeometry3D meshGeometry, List<int> triangleIndices, List<Point3D> positions)
        {
            var slicingPlaneHeight = GetSlicingPlaneHeight(meshGeometry.Bounds.Z, layer);
            //Console.WriteLine("slicingPlaneHeight:");
            //Console.WriteLine(slicingPlaneHeight);
            
            // Get paths according to slicing
            var paths = SlicingAlgorithm(slicingPlaneHeight, triangleIndices, positions);
            // Console.WriteLine("paths: ");
            // Console.WriteLine(paths);

            // Combine paths
            var combinedPaths = ConnectLineSegments(paths);
            // Console.WriteLine("combinedPaths: ");
            // Console.WriteLine(combinedPaths);
            // Adjust to line segments

            return combinedPaths;
        }

        // ------

        // --- Slicing Algorithm

        private List<LineSegment> SlicingAlgorithm(double slicingPlaneHeight, List<int> triangleIndices, List<Point3D> positions)
        {
            var paths = new List<LineSegment>();
            //Debug.WriteLine(triangleIndices);
            for (int i = 0; i < triangleIndices.Count; i += 3)
            {
                Point3D p1 = positions[triangleIndices[i]];
                Point3D p2 = positions[triangleIndices[i + 1]];
                Point3D p3 = positions[triangleIndices[i + 2]];

                Vector3D planeNormal = new Vector3D(0, 0, 1);
                var intersections = FindTrianglePlaneIntersections(p1, p2, p3, planeNormal, -slicingPlaneHeight);
                if (intersections.Count == 3)
                {

                }
                else if(intersections.Count == 2)
                {
                    var segment = new Data_Structures.LineSegment();
                    segment.AddPoint(intersections[0]);
                    segment.AddPoint(intersections[1]);
                    paths.Add(segment);
                }
            }
            return paths;
        }

        public static Point3D? IntersectLineSegmentWithPlane(Point3D p0, Point3D p1, Vector3D planeNormal, double planeD)
        {
            Vector3D direction = p1 - p0;
            double dotProduct = Vector3D.DotProduct(planeNormal, direction);

            if (Math.Abs(dotProduct) < 1e-6) // Parallel check
            {
                return null;
            }

            double t = -(Vector3D.DotProduct(planeNormal, p0.ToVector3D()) + planeD) / dotProduct;

            if (t >= 0 && t <= 1) // Intersection within the segment
            {
                return p0 + t * direction;
            }

            return null;
        }

        public static List<Point3D> FindTrianglePlaneIntersections(Point3D v0, Point3D v1, Point3D v2, Vector3D planeNormal, double planeD)
        {
            var intersections = new List<Point3D>();

            // Check each edge of the triangle
            var i1 = IntersectLineSegmentWithPlane(v0, v1, planeNormal, planeD);
            if (i1.HasValue) intersections.Add(i1.Value);

            var i2 = IntersectLineSegmentWithPlane(v1, v2, planeNormal, planeD);
            if (i2.HasValue) intersections.Add(i2.Value);

            var i3 = IntersectLineSegmentWithPlane(v2, v0, planeNormal, planeD);
            if (i3.HasValue) intersections.Add(i3.Value);

            return intersections;
        }



        private double GetSlicingPlaneHeight(double zOffset, double layer)
        {
            //return zOffset + (layer * SlicerSettings.LayerHeight);
            return zOffset + layer + 0.000000001;
        }

        // --- Path Combining Algorithm
        
        private PathsD ConnectLineSegments(List<LineSegment> lineSegments) 
        {
            if (lineSegments.Count == 0) return new PathsD(); 
            double distanceThreshold = 5;
            PathsD path = new PathsD(); 
            
            List<LineSegment> lineSegmentsCopy = new List<LineSegment>(lineSegments); 
            while (lineSegmentsCopy.Count > 0) {
                PathD currentPath = new PathD(); 
                LineSegment currentSegment = lineSegmentsCopy[0];

                PointD Start = currentSegment.GetPoints()[0];
                PointD End = currentSegment.GetPoints()[^1]; 

                currentPath.Add(Start); 
                currentPath.Add(End);

                lineSegmentsCopy.RemoveAt(0); 

                bool pathClosed = false;

                while (!pathClosed && lineSegmentsCopy.Count > 0) {
                    // Get the segment where the Start is the same as the current End or the End is the same as the current End.
                    int nextSegmentIndex = lineSegmentsCopy.FindIndex(seg => seg.GetPoints()[0].Equals(End) || seg.GetPoints()[^1].Equals(End));
                    if (nextSegmentIndex != -1) {
                        currentSegment = lineSegmentsCopy[nextSegmentIndex];
                        PointD nextPoint;
                        
                        // Determine the correct point to continue the path
                        if (currentSegment.GetPoints()[0].Equals(End)) {
                            nextPoint = currentSegment.GetPoints()[^1];
                        } else {
                            nextPoint = currentSegment.GetPoints()[0];
                            // Reverse the segment to maintain continuity
                            currentSegment.GetPoints().Reverse();
                        }

                        End = nextPoint;
                        currentPath.Add(End);

                        lineSegmentsCopy.RemoveAt(nextSegmentIndex);

                        // If the first and last point are the same -> path is closed
                        if (currentPath[0].Equals(currentPath[currentPath.Count - 1])) {
                            pathClosed = true;
                        }
                    }
                    else {
                        break;
                    }
                }
                if(currentPath.Count > 2) 
                {
                    path.Add(currentPath);
                }
            }

            return path;
        }


        // ------

        // --- Convert List<LineSegment> To PathD 

        public PathD ConvertListToPathD(List<LineSegment> list)
        {
            var path = new PathD();

            foreach (var segment in list)
            {
                path.AddRange(segment.GetPoints());
            }

            return path;
        }

        // ------

        // --- Default Getter And Setters

        public void SetGeometryModel3D(GeometryModel3D geometryModel)
        {
            geometryModel3D = geometryModel;
        }  
    
        public GeometryModel3D GetGeometryModel3D()
        {
            return geometryModel3D;
        }

        public void SetSliderValue(double newValue) 
        {
            sliderValue = newValue;
        }
    
        private List<PathsD> ProcessContours(PathsD contours)
        {
            PathsD outerShell = new PathsD();
            PathsD innerShell = new PathsD();

            ClipperD clipperOuter = new ClipperD();
            clipperOuter.AddSubject(contours);
            clipperOuter.Execute(ClipType.Xor, FillRule.EvenOdd, outerShell);

            outerShell = Clipper.InflatePaths(outerShell, -(SlicerSettings.NozzleThickness * 0.5), JoinType.Miter, EndType.Polygon);
            outerShell = Clipper.SimplifyPaths(outerShell, 0.1);

            ClipperD clipperInner = new ClipperD();
            clipperInner.AddSubject(contours);
            clipperInner.Execute(ClipType.Xor, FillRule.EvenOdd, innerShell);

            innerShell = Clipper.InflatePaths(innerShell, -(SlicerSettings.NozzleThickness * 1.5), JoinType.Miter, EndType.Polygon);
            innerShell = Clipper.SimplifyPaths(innerShell, 0.1);

            return new List<PathsD>{outerShell, innerShell};
        }

        private PathsD ProcessInfill(PathsD innerShell, MeshGeometry3D meshGeometry3D) 
        {
            var meshBounds = meshGeometry3D.Bounds;
            
            // Bepaal de bounding box van het model in X en Y
            double minX = meshBounds.X;
            double minY = meshBounds.Y;
            double maxX = meshBounds.X + meshBounds.SizeX;
            double maxY = meshBounds.Y + meshBounds.SizeY;

            // Console.WriteLine($"meshBounds -> minX: {minX} | minY: {minY} | maxX: {maxX} | maxY: {maxY}");
            
            // Bereken de infill-spatiëring (bijvoorbeeld 10% van de nozzle-diameter of een andere waarde)
            double lineSpacingX = (maxX - minX)/10; // Aanpassen aan de gewenste infill-dichtheid
            double lineSpacingY = (maxY - minY)/10; // Aanpassen aan de gewenste infill-dichtheid
            
            // Genereer het rasterpatroon
            var gridLines = new PathsD();
            
            // Horizontale lijnen
            for (double y = minY; y <= maxY; y += lineSpacingY)
            {
                PathD horizontalLine = new PathD 
                {
                    new PointD(minX, y),
                    new PointD(maxX, y)
                };
                gridLines.Add(horizontalLine);
            }
            
            // Verticale lijnen
            for (double x = minX; x <= maxX; x += lineSpacingX)
            {
                PathD verticalLine = new PathD 
                {
                    new PointD(x, minY),
                    new PointD(x, maxY)
                };
                gridLines.Add(verticalLine);
            }

            // gridLines = Clipper.InflatePaths(gridLines, -(SlicerSettings.NozzleThickness * 0.5), JoinType.Miter, EndType.Square, 1);
        
            // Intersecteer het grid met de polygonen van de laag met Clipper2Lib
            var _ = new PathsD();
            var infillPathsOpen = new PathsD();
            
            ClipperD clipper = new ClipperD();
            clipper.AddOpenSubject(gridLines);
            clipper.AddPaths(innerShell, PathType.Clip, false);
            clipper.Execute(ClipType.Intersection, FillRule.NonZero, _, infillPathsOpen);

            // Inflate het uitkomende infill patroon de helft van de nozzle dikte naar binnen zodat infill niet meer buiten de inner shell gaat.
            infillPathsOpen = Clipper.InflatePaths(infillPathsOpen, -(SlicerSettings.NozzleThickness * 0.5), JoinType.Miter, EndType.Square);
            
            return infillPathsOpen;
        }

        private void generateGCodes(List<PathsD> layers)
        {            
            List<string> gcodes = new List<string>
            {
                "M140 S60",
                "M190 S60",
                "M104 S200",
                "M109 S200",
                "M82",
                "G28",
                "G92 E0",
                "G1 Z2.0 F3000",
                "G1 X0.1 Y20 Z0.3 F5000",
                "G1 X0.1 Y200 Z0.3 F1500 E15",
                "G1 X0.4 Y200 Z0.3 F5000",
                "G1 X0.4 Y20 Z0.3 F1500 E30",
                "M107"
            };
            double height = 0;
            foreach (PathsD paths in layers)
            {
                height += SlicerSettings.LayerHeight;
                foreach (PathD path in paths)
                {
                    var zHeight = SlicerSettings.LayerHeight*paths.IndexOf(path);
                    if (path.Count > 0)
                    {
                        var start = path[0];
                        // Move to the starting point of the path
                        gcodes.Add($"G0 X{start.x} Y{start.y} Z{height}");

                        // Loop through all points in the path
                        foreach (var point in path)
                        {
                            gcodes.Add($"G1 X{point.x} Y{point.y} Z{height}");
                        }
                    }
                }
            }

            gcodes.Add("M140 S0");
            gcodes.Add("M107");
            gcodes.Add("M220 S100");
            gcodes.Add("M221 S100");
            gcodes.Add("G91");
            gcodes.Add("G1 F1800 E-3");
            gcodes.Add("G1 F3000 Z20");
            gcodes.Add("G90");
            gcodes.Add("G1 X0 Y235 F1000");
            gcodes.Add("M107");
            gcodes.Add("M84");
            gcodes.Add("M82");
            gcodes.Add("M104 S0");

            SaveToFile("testfile.gcode",gcodes);
        }
        
        private void SaveToFile(string filename,List<string> gcodes)
        {
            using System.IO.StreamWriter file = new System.IO.StreamWriter(filename, false);
            foreach (string line in gcodes)
            {
                file.WriteLine(line);
            }
        }
    }
}

