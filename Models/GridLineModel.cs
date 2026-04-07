using Autodesk.AutoCAD.Geometry;

namespace BeamLayoutAnalyzer.Models;

/// <summary>
/// 通り芯の方向
/// </summary>
public enum GridLineDirection
{
    /// <summary>水平方向（Y通り）— 同じY座標を持つ柱を結ぶ</summary>
    Horizontal,
    /// <summary>垂直方向（X通り）— 同じX座標を持つ柱を結ぶ</summary>
    Vertical
}

/// <summary>
/// 通り芯（Grid Line）を表すモデル。
/// 柱が同一直線上に並ぶラインを自動検出して生成される。
/// </summary>
public class GridLineModel
{
    /// <summary>通り芯の方向</summary>
    public GridLineDirection Direction { get; }

    /// <summary>
    /// 通り芯の座標位置。
    /// Horizontal の場合は Y座標、Vertical の場合は X座標。
    /// </summary>
    public double Position { get; }

    /// <summary>
    /// 通り芯の描画開始座標（延長含む）。
    /// Horizontal の場合は最小X、Vertical の場合は最小Y。
    /// </summary>
    public double Start { get; set; }

    /// <summary>
    /// 通り芯の描画終了座標（延長含む）。
    /// Horizontal の場合は最大X、Vertical の場合は最大Y。
    /// </summary>
    public double End { get; set; }

    /// <summary>この通り芯上に位置する柱の一覧</summary>
    public List<ColumnModel> Columns { get; } = new();

    /// <summary>ユーザーが編集可能な通り芯名（例: X1, Y2 など）</summary>
    public string Name { get; set; }

    /// <summary>通り芯の始点（ワールド座標）</summary>
    public Point2d StartPoint => Direction == GridLineDirection.Horizontal
        ? new Point2d(Start, Position)
        : new Point2d(Position, Start);

    /// <summary>通り芯の終点（ワールド座標）</summary>
    public Point2d EndPoint => Direction == GridLineDirection.Horizontal
        ? new Point2d(End, Position)
        : new Point2d(Position, End);

    public GridLineModel(GridLineDirection direction, double position, string name)
    {
        Direction = direction;
        Position  = position;
        Name      = name;
    }
}
