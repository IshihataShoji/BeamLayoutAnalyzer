using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 各部材の負担面積を算出する。
///
/// 梁（大梁・小梁）：亀の甲分割
///   梁を格子線間のセグメントに分割し、セグメント両側のパネルに
///   亀の甲公式 TortoiseshellArea(L, H) を適用して合計する。
///   同時に、各セグメントの支配領域ポリゴンを BeamModel.TributaryPolygons に格納する。
///
/// 柱：ボロノイ分割
///   隣接柱との垂直二等分線でスラブ領域をクリッピングし、
///   各柱のボロノイセルの面積を求める。
/// </summary>
public class TributaryAreaCalculator
{
    private readonly List<SlabModel>   _slabs;
    private readonly List<ColumnModel> _columns;
    private readonly List<BeamModel>   _beams;

    private const double AngleTol = 5.0 * Math.PI / 180.0; // 水平・垂直判定の許容角度

    public TributaryAreaCalculator(
        List<SlabModel>   slabs,
        List<ColumnModel> columns,
        List<BeamModel>   beams)
    {
        _slabs   = slabs;
        _columns = columns;
        _beams   = beams;
    }

    public void Calculate()
    {
        foreach (var b in _beams)
        {
            b.TributaryArea = 0;
            b.TributaryPolygons.Clear();
        }
        foreach (var c in _columns) c.TributaryArea = 0;

        CalculateBeamAreas();
        CalculateColumnAreas();
    }

    // ════════════════════════════════════════════════════════
    //  梁の負担面積（亀の甲分割）
    // ════════════════════════════════════════════════════════

    private void CalculateBeamAreas()
    {
        var hBeams = _beams.Where(IsHorizontal).ToList();
        var vBeams = _beams.Where(IsVertical).ToList();

        foreach (var b in hBeams)
            b.TributaryArea = CalcHBeamArea(b, hBeams, vBeams);

        foreach (var b in vBeams)
            b.TributaryArea = CalcVBeamArea(b, hBeams, vBeams);

        // 斜め梁はレイキャスティング（フォールバック）
        foreach (var b in _beams.Where(x => !IsHorizontal(x) && !IsVertical(x)))
            b.TributaryArea = CalcDiagonalBeamArea(b);
    }

    private bool IsHorizontal(BeamModel b)
        => Math.Abs(Math.Sin(b.Angle)) < Math.Sin(AngleTol);

    private bool IsVertical(BeamModel b)
        => Math.Abs(Math.Cos(b.Angle)) < Math.Sin(AngleTol);

    // ─── 水平梁 ─────────────────────────────────────────────

    private double CalcHBeamArea(
        BeamModel beam, List<BeamModel> hBeams, List<BeamModel> vBeams)
    {
        double y    = beam.MidPoint.Y;
        double xMin = Math.Min(beam.StartPoint.X, beam.EndPoint.X);
        double xMax = Math.Max(beam.StartPoint.X, beam.EndPoint.X);

        // 垂直梁がこの梁を横切る X 座標でセグメント分割
        var splitXs = new SortedSet<double> { xMin, xMax };
        foreach (var vb in vBeams)
        {
            double vx    = vb.MidPoint.X;
            double vyMin = Math.Min(vb.StartPoint.Y, vb.EndPoint.Y);
            double vyMax = Math.Max(vb.StartPoint.Y, vb.EndPoint.Y);
            if (vx > xMin + 1e-6 && vx < xMax - 1e-6
                && y >= vyMin - 1e-6 && y <= vyMax + 1e-6)
                splitXs.Add(vx);
        }

        double total = 0;
        var xs = splitXs.ToList();
        for (int i = 0; i < xs.Count - 1; i++)
        {
            double x0 = xs[i], x1 = xs[i + 1];
            double segLen = x1 - x0;
            var mid = new Point2d((x0 + x1) / 2.0, y);

            double hDown = FullPanelH(mid, new Vector2d(0, -1), hBeams);
            double hUp   = FullPanelH(mid, new Vector2d(0,  1), hBeams);

            total += TortoiseshellArea(segLen, hDown)
                   + TortoiseshellArea(segLen, hUp);

            // 亀の甲ポリゴンを生成
            var polyDown = MakeHSegmentPolygon(x0, x1, y, hDown, -1.0);
            var polyUp   = MakeHSegmentPolygon(x0, x1, y, hUp,    1.0);
            if (polyDown != null) beam.TributaryPolygons.Add(polyDown);
            if (polyUp   != null) beam.TributaryPolygons.Add(polyUp);
        }
        return total;
    }

    // ─── 垂直梁 ─────────────────────────────────────────────

    private double CalcVBeamArea(
        BeamModel beam, List<BeamModel> hBeams, List<BeamModel> vBeams)
    {
        double x    = beam.MidPoint.X;
        double yMin = Math.Min(beam.StartPoint.Y, beam.EndPoint.Y);
        double yMax = Math.Max(beam.StartPoint.Y, beam.EndPoint.Y);

        // 水平梁がこの梁を横切る Y 座標でセグメント分割
        var splitYs = new SortedSet<double> { yMin, yMax };
        foreach (var hb in hBeams)
        {
            double hy    = hb.MidPoint.Y;
            double hxMin = Math.Min(hb.StartPoint.X, hb.EndPoint.X);
            double hxMax = Math.Max(hb.StartPoint.X, hb.EndPoint.X);
            if (hy > yMin + 1e-6 && hy < yMax - 1e-6
                && x >= hxMin - 1e-6 && x <= hxMax + 1e-6)
                splitYs.Add(hy);
        }

        double total = 0;
        var ys = splitYs.ToList();
        for (int i = 0; i < ys.Count - 1; i++)
        {
            double y0 = ys[i], y1 = ys[i + 1];
            double segLen = y1 - y0;
            var mid = new Point2d(x, (y0 + y1) / 2.0);

            double wLeft  = FullPanelH(mid, new Vector2d(-1, 0), vBeams);
            double wRight = FullPanelH(mid, new Vector2d( 1, 0), vBeams);

            total += TortoiseshellArea(segLen, wLeft)
                   + TortoiseshellArea(segLen, wRight);

            // 亀の甲ポリゴンを生成
            var polyLeft  = MakeVSegmentPolygon(y0, y1, x, wLeft,  -1.0);
            var polyRight = MakeVSegmentPolygon(y0, y1, x, wRight,  1.0);
            if (polyLeft  != null) beam.TributaryPolygons.Add(polyLeft);
            if (polyRight != null) beam.TributaryPolygons.Add(polyRight);
        }
        return total;
    }

    // ─── パネル幅を求める ────────────────────────────────────

    /// <summary>
    /// 指定方向の「パネル全幅」を返す。
    /// 平行梁があればその距離（全幅）、なければスラブ境界までの距離。
    /// </summary>
    private double FullPanelH(
        Point2d origin, Vector2d dir, List<BeamModel> parallelBeams)
    {
        double nearest = double.MaxValue;

        // 平行梁との距離（全幅）
        foreach (var pb in parallelBeams)
        {
            double pMin, pMax, pCoord, oCoord;
            double dist;

            if (Math.Abs(dir.Y) > 0.5) // 上下方向（水平梁を探す）
            {
                pCoord = pb.MidPoint.Y;
                oCoord = origin.Y;
                pMin   = Math.Min(pb.StartPoint.X, pb.EndPoint.X);
                pMax   = Math.Max(pb.StartPoint.X, pb.EndPoint.X);
                if (origin.X < pMin - 1e-6 || origin.X > pMax + 1e-6) continue;
                dist = (pCoord - oCoord) * Math.Sign(dir.Y);
            }
            else // 左右方向（垂直梁を探す）
            {
                pCoord = pb.MidPoint.X;
                oCoord = origin.X;
                pMin   = Math.Min(pb.StartPoint.Y, pb.EndPoint.Y);
                pMax   = Math.Max(pb.StartPoint.Y, pb.EndPoint.Y);
                if (origin.Y < pMin - 1e-6 || origin.Y > pMax + 1e-6) continue;
                dist = (pCoord - oCoord) * Math.Sign(dir.X);
            }

            if (dist > 1e-6 && dist < nearest)
                nearest = dist;
        }

        // スラブ境界までの距離
        var slab = _slabs.FirstOrDefault(s => s.Contains(origin));
        if (slab != null)
        {
            var t = GeometryUtils.RayPolygonIntersect(origin, dir, slab.Vertices);
            if (t.HasValue && t.Value > 1e-6 && t.Value < nearest)
                nearest = t.Value;
        }

        return nearest == double.MaxValue ? 0 : nearest;
    }

    // ─── 亀の甲公式 ──────────────────────────────────────────

    /// <summary>
    /// 梁（長さ L）が幅 H のパネルに接する場合の負担面積。
    ///   L >= H（梁が長辺）: (2L - H) × H / 4  （台形）
    ///   L &lt;  H（梁が短辺）: L² / 4            （三角形）
    /// </summary>
    private static double TortoiseshellArea(double L, double H)
    {
        if (L <= 1e-9 || H <= 1e-9) return 0;
        return L >= H
            ? (2 * L - H) * H / 4.0
            : L * L / 4.0;
    }

    // ─── 亀の甲ポリゴン生成（水平梁セグメント）────────────────

    /// <summary>
    /// 水平梁セグメント (x0→x1 at y) の片側支配領域ポリゴン。
    /// dir > 0 → 上側、dir &lt; 0 → 下側
    /// </summary>
    private static List<Point2d>? MakeHSegmentPolygon(
        double x0, double x1, double y, double h, double dir)
    {
        if (h <= 1e-9) return null;
        double L     = x1 - x0;
        double yInner = y + dir * Math.Min(h, L) / 2.0;

        if (L >= h)
        {
            // 台形
            double d = h / 2.0;
            return new List<Point2d>
            {
                new(x0,     y),
                new(x1,     y),
                new(x1 - d, yInner),
                new(x0 + d, yInner),
            };
        }
        else
        {
            // 三角形
            return new List<Point2d>
            {
                new(x0,             y),
                new(x1,             y),
                new((x0 + x1) / 2.0, yInner),
            };
        }
    }

    // ─── 亀の甲ポリゴン生成（垂直梁セグメント）────────────────

    /// <summary>
    /// 垂直梁セグメント (y0→y1 at x) の片側支配領域ポリゴン。
    /// dir > 0 → 右側、dir &lt; 0 → 左側
    /// </summary>
    private static List<Point2d>? MakeVSegmentPolygon(
        double y0, double y1, double x, double w, double dir)
    {
        if (w <= 1e-9) return null;
        double L      = y1 - y0;
        double xInner = x + dir * Math.Min(w, L) / 2.0;

        if (L >= w)
        {
            // 台形
            double d = w / 2.0;
            return new List<Point2d>
            {
                new(x, y0),
                new(x, y1),
                new(xInner, y1 - d),
                new(xInner, y0 + d),
            };
        }
        else
        {
            // 三角形
            return new List<Point2d>
            {
                new(x, y0),
                new(x, y1),
                new(xInner, (y0 + y1) / 2.0),
            };
        }
    }

    // ─── 斜め梁フォールバック（レイキャスティング） ─────────

    private double CalcDiagonalBeamArea(BeamModel target)
    {
        var dir    = target.Direction;
        var normal = new Vector2d(-dir.Y, dir.X);
        var slab   = _slabs.FirstOrDefault(s => s.Contains(target.MidPoint));

        double total = 0;
        const int N = 11;
        for (int i = 0; i < N; i++)
        {
            double t  = (double)i / (N - 1);
            double px = target.StartPoint.X + (target.EndPoint.X - target.StartPoint.X) * t;
            double py = target.StartPoint.Y + (target.EndPoint.Y - target.StartPoint.Y) * t;
            var pt = new Point2d(px, py);

            total += HalfWidth(pt,  normal, target, slab)
                   + HalfWidth(pt, new Vector2d(-normal.X, -normal.Y), target, slab);
        }
        return target.Length * total / N;
    }

    private double HalfWidth(
        Point2d origin, Vector2d dir, BeamModel exclude, SlabModel? slab)
    {
        double nearBeam = double.MaxValue;
        double nearSlab = double.MaxValue;

        foreach (var b in _beams)
        {
            if (b == exclude) continue;
            var t = GeometryUtils.RaySegmentIntersect(origin, dir, b.StartPoint, b.EndPoint);
            if (t.HasValue && t.Value < nearBeam) nearBeam = t.Value;
        }

        if (slab != null)
        {
            var t = GeometryUtils.RayPolygonIntersect(origin, dir, slab.Vertices);
            if (t.HasValue && t.Value < nearSlab) nearSlab = t.Value;
        }

        double result = double.MaxValue;
        if (nearBeam < double.MaxValue) result = nearBeam / 2.0;
        if (nearSlab < double.MaxValue) result = Math.Min(result, nearSlab);
        return result == double.MaxValue ? 0 : result;
    }

    // ════════════════════════════════════════════════════════
    //  柱の負担面積（ボロノイ分割）
    // ════════════════════════════════════════════════════════

    private void CalculateColumnAreas()
    {
        foreach (var col in _columns)
            col.TributaryArea = CalcColumnVoronoi(col);
    }

    private double CalcColumnVoronoi(ColumnModel target)
    {
        var slab = _slabs.FirstOrDefault(s => s.Contains(target.Center));
        if (slab == null) return 0;

        var cell = new List<Point2d>(slab.Vertices);

        // 他の柱との垂直二等分線でクリッピング
        foreach (var other in _columns)
        {
            if (other == target || cell.Count == 0) continue;

            var mid  = new Point2d(
                (target.Center.X + other.Center.X) / 2.0,
                (target.Center.Y + other.Center.Y) / 2.0);
            var diff = new Vector2d(
                other.Center.X - target.Center.X,
                other.Center.Y - target.Center.Y);
            var perp = new Vector2d(-diff.Y, diff.X);

            var lineA = mid;
            var lineB = new Point2d(mid.X + perp.X, mid.Y + perp.Y);

            cell = PolygonUtils.ClipByHalfPlane(cell, lineA, lineB, target.Center);
        }

        // ボロノイポリゴンを保存（表示用）
        target.VoronoiPolygon = cell;

        return PolygonUtils.Area(cell);
    }
}
