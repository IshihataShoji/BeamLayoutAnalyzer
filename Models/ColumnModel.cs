using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;

namespace BeamLayoutAnalyzer.Models;

/// <summary>
/// 柱を表す。AutoCAD上の円オブジェクトから生成する。
/// </summary>
public class ColumnModel
{
    public ObjectId EntityId { get; }
    public Point2d Center { get; }
    public double Radius { get; }
    public double Diameter => Radius * 2;

    /// <summary>負担スラブ面積 [m²]（TributaryAreaCalculatorが設定する）</summary>
    public double TributaryArea { get; set; }

    /// <summary>ボロノイセルの頂点列（表示用・TributaryAreaCalculatorが設定する）</summary>
    public List<Point2d> VoronoiPolygon { get; set; } = new();

    private const double MmToM = 1.0 / 1000.0;

    public ColumnModel(Circle circle)
    {
        EntityId = circle.ObjectId;
        Center   = new Point2d(circle.Center.X * MmToM, circle.Center.Y * MmToM);
        Radius   = circle.Radius * MmToM;
    }
}
