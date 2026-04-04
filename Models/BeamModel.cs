using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;

namespace BeamLayoutAnalyzer.Models;

public enum BeamType { 大梁, 小梁 }

/// <summary>
/// 梁を表す。AutoCAD上の線分オブジェクトから生成する。
/// </summary>
public class BeamModel
{
    public ObjectId EntityId { get; }
    public Point2d StartPoint { get; }
    public Point2d EndPoint { get; }
    public double Length { get; }

    /// <summary>大梁／小梁の種別（ReadFramingPlanCommandが設定する）</summary>
    public BeamType Type { get; set; } = BeamType.小梁;

    /// <summary>負担スラブ面積 [m²]（TributaryAreaCalculatorが設定する）</summary>
    public double TributaryArea { get; set; }

    /// <summary>負担面積を構成する亀の甲ポリゴン一覧（TributaryAreaCalculatorが設定する）</summary>
    public List<List<Point2d>> TributaryPolygons { get; } = new();

    private const double MmToM = 1.0 / 1000.0;

    public BeamModel(Line line)
    {
        EntityId   = line.ObjectId;
        StartPoint = new Point2d(line.StartPoint.X * MmToM, line.StartPoint.Y * MmToM);
        EndPoint   = new Point2d(line.EndPoint.X   * MmToM, line.EndPoint.Y   * MmToM);
        Length     = line.Length * MmToM;
    }

    public Point2d MidPoint => new(
        (StartPoint.X + EndPoint.X) / 2.0,
        (StartPoint.Y + EndPoint.Y) / 2.0
    );

    /// <summary>梁の角度 [rad]（X軸正方向基準）</summary>
    public double Angle => Math.Atan2(
        EndPoint.Y - StartPoint.Y,
        EndPoint.X - StartPoint.X
    );

    /// <summary>単位方向ベクトル</summary>
    public Vector2d Direction
    {
        get
        {
            double dx  = EndPoint.X - StartPoint.X;
            double dy  = EndPoint.Y - StartPoint.Y;
            double len = Math.Sqrt(dx * dx + dy * dy);
            return len < 1e-10 ? new Vector2d(1, 0) : new Vector2d(dx / len, dy / len);
        }
    }

    /// <summary>許容積載荷重 [kN] = 荷重定数 × 負担面積</summary>
    public double AllowableLoad(double loadConstantKNm2) => TributaryArea * loadConstantKNm2;
}
