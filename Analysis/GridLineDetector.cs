using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 柱の座標から通り芯（Grid Line）を自動検出する。
/// 
/// アルゴリズム:
///   1. 全柱のX座標をソートし、許容誤差内で同じX座標を持つ柱を
///      グルーピング → 2本以上のグループを垂直通り芯（X通り）として検出
///   2. 全柱のY座標をソートし、同様に水平通り芯（Y通り）を検出
///   3. X通りは左→右順にX1,X2,...、Y通りは下→上順にY1,Y2,...と命名
///   4. 各通り芯のStart/Endはスラブの外形端まで延長（+余白）
/// </summary>
public class GridLineDetector
{
    private readonly List<SlabModel>   _slabs;
    private readonly List<ColumnModel> _columns;

    /// <summary>スラブ端からの延長量 [m]</summary>
    private const double Extension = 1.0;

    public GridLineDetector(List<SlabModel> slabs, List<ColumnModel> columns)
    {
        _slabs   = slabs;
        _columns = columns;
    }

    /// <summary>
    /// 柱座標から通り芯を自動検出して返す。
    /// </summary>
    public List<GridLineModel> Detect()
    {
        if (_columns.Count < 2)
            return new List<GridLineModel>();

        // 許容誤差: 柱半径の最大値 × 1.5（柱中心の微小なずれを吸収）
        double tolerance = _columns.Max(c => c.Radius) * 1.5;
        // 最低限の許容誤差 (5cm)
        tolerance = Math.Max(tolerance, 0.05);

        // スラブ全体のバウンディングボックス
        double slabMinX = double.MaxValue, slabMaxX = double.MinValue;
        double slabMinY = double.MaxValue, slabMaxY = double.MinValue;
        foreach (var s in _slabs)
        {
            slabMinX = Math.Min(slabMinX, s.MinX);
            slabMaxX = Math.Max(slabMaxX, s.MaxX);
            slabMinY = Math.Min(slabMinY, s.MinY);
            slabMaxY = Math.Max(slabMaxY, s.MaxY);
        }

        var gridLines = new List<GridLineModel>();

        // ── 垂直通り芯（X通り）の検出 ─────────────────────
        // X座標でグルーピング
        var verticalGroups = GroupByCoordinate(
            _columns, c => c.Center.X, tolerance);

        int xIndex = 1;
        foreach (var group in verticalGroups.OrderBy(g => g.Key))
        {
            var gl = new GridLineModel(
                GridLineDirection.Vertical,
                group.Key,
                $"X{xIndex++}");

            gl.Columns.AddRange(group.Value);
            gl.Start = slabMinY - Extension;
            gl.End   = slabMaxY + Extension;
            gridLines.Add(gl);
        }

        // ── 水平通り芯（Y通り）の検出 ─────────────────────
        // Y座標でグルーピング
        var horizontalGroups = GroupByCoordinate(
            _columns, c => c.Center.Y, tolerance);

        int yIndex = 1;
        foreach (var group in horizontalGroups.OrderBy(g => g.Key))
        {
            var gl = new GridLineModel(
                GridLineDirection.Horizontal,
                group.Key,
                $"Y{yIndex++}");

            gl.Columns.AddRange(group.Value);
            gl.Start = slabMinX - Extension;
            gl.End   = slabMaxX + Extension;
            gridLines.Add(gl);
        }

        return gridLines;
    }

    /// <summary>
    /// 柱を指定座標軸の値でグルーピングする。
    /// 許容誤差内の柱は同じグループにまとめ、グループの代表座標は平均値とする。
    /// 2本以上の柱が含まれるグループのみ返す。
    /// </summary>
    private static Dictionary<double, List<ColumnModel>> GroupByCoordinate(
        List<ColumnModel> columns,
        Func<ColumnModel, double> coordSelector,
        double tolerance)
    {
        // 座標値でソート
        var sorted = columns.OrderBy(coordSelector).ToList();
        var groups = new Dictionary<double, List<ColumnModel>>();

        var currentGroup = new List<ColumnModel> { sorted[0] };
        double currentSum = coordSelector(sorted[0]);

        for (int i = 1; i < sorted.Count; i++)
        {
            double val = coordSelector(sorted[i]);
            double groupCenter = currentSum / currentGroup.Count;

            if (Math.Abs(val - groupCenter) <= tolerance)
            {
                // 同じグループに追加
                currentGroup.Add(sorted[i]);
                currentSum += val;
            }
            else
            {
                // 前のグループを確定（2本以上の場合のみ）
                if (currentGroup.Count >= 2)
                {
                    double avgCoord = currentSum / currentGroup.Count;
                    groups[avgCoord] = new List<ColumnModel>(currentGroup);
                }

                // 新しいグループを開始
                currentGroup.Clear();
                currentGroup.Add(sorted[i]);
                currentSum = val;
            }
        }

        // 最後のグループ
        if (currentGroup.Count >= 2)
        {
            double avgCoord = currentSum / currentGroup.Count;
            groups[avgCoord] = new List<ColumnModel>(currentGroup);
        }

        return groups;
    }
}
