using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;

namespace BeamLayoutAnalyzer.Models;

/// <summary>
/// スラブを表す。AutoCAD上の閉じたポリラインから生成する。
/// </summary>
public class SlabModel
{
    public ObjectId EntityId { get; }
    public List<Point2d> Vertices { get; } = new();
    public double Area { get; }

    public double MinX => Vertices.Min(v => v.X);
    public double MaxX => Vertices.Max(v => v.X);
    public double MinY => Vertices.Min(v => v.Y);
    public double MaxY => Vertices.Max(v => v.Y);

    private const double MmToM = 1.0 / 1000.0;

    public SlabModel(Polyline poly)
    {
        EntityId = poly.ObjectId;
        for (int i = 0; i < poly.NumberOfVertices; i++)
        {
            var p = poly.GetPoint2dAt(i);
            Vertices.Add(new Point2d(p.X * MmToM, p.Y * MmToM));
        }
        Area = ComputeArea();
    }

    /// <summary>ガウスの面積公式（靴ひも公式）</summary>
    private double ComputeArea()
    {
        double area = 0;
        int n = Vertices.Count;
        for (int i = 0; i < n; i++)
        {
            var p0 = Vertices[i];
            var p1 = Vertices[(i + 1) % n];
            area += p0.X * p1.Y - p1.X * p0.Y;
        }
        return Math.Abs(area) / 2.0;
    }

    /// <summary>点がスラブ内部にあるか（レイキャスティング法）</summary>
    public bool Contains(Point2d point)
    {
        bool inside = false;
        int n = Vertices.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            var a = Vertices[i];
            var b = Vertices[j];
            if ((a.Y > point.Y) != (b.Y > point.Y) &&
                point.X < (b.X - a.X) * (point.Y - a.Y) / (b.Y - a.Y) + a.X)
                inside = !inside;
        }
        return inside;
    }
}
