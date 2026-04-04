using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 各部材の負担面積を算出する。
///
/// 梁：角度2等分線（アングルバイセクター）分割
///   1. スラブ内部の梁ラインでスラブをサブパネルに分割
///   2. 各サブパネルの頂点で内角の2等分線を引く
///   3. 半平面クリッピングで各辺の支配領域を求める
///   4. 各辺に対応する梁に面積を割り当てる
///
/// 柱：ボロノイ分割（従来通り）
/// </summary>
public class TributaryAreaCalculator
{
    private readonly List<SlabModel>   _slabs;
    private readonly List<ColumnModel> _columns;
    private readonly List<BeamModel>   _beams;

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
    //  梁の負担面積（角度2等分線分割）
    // ════════════════════════════════════════════════════════

    private void CalculateBeamAreas()
    {
        foreach (var slab in _slabs)
        {
            if (slab.Vertices.Count < 3) continue;

            // スラブ内部の梁を取得（分割用）
            var interiorBeams = _beams.Where(b => slab.Contains(b.MidPoint)).ToList();

            // 梁ラインでスラブをサブパネルに分割
            var panels = SubdivideSlab(slab.Vertices, interiorBeams);

            // 各サブパネルに角度2等分線を適用
            foreach (var panel in panels)
            {
                if (panel.Count < 3 || PolygonUtils.Area(panel) < 1e-4) continue;
                ApplyAngleBisectorToPanel(panel);
            }
        }
    }

    /// <summary>
    /// スラブポリゴンを梁ラインで繰り返し分割してサブパネル群を返す。
    /// </summary>
    private static List<List<Point2d>> SubdivideSlab(
        List<Point2d> slabVertices, List<BeamModel> beams)
    {
        var panels = new List<List<Point2d>> { new(slabVertices) };

        foreach (var beam in beams)
        {
            var lineA  = beam.StartPoint;
            var lineB  = beam.EndPoint;
            var normal = new Vector2d(-beam.Direction.Y, beam.Direction.X);
            var ptL = new Point2d(beam.MidPoint.X + normal.X, beam.MidPoint.Y + normal.Y);
            var ptR = new Point2d(beam.MidPoint.X - normal.X, beam.MidPoint.Y - normal.Y);

            var next = new List<List<Point2d>>();

            foreach (var panel in panels)
            {
                // 梁ラインがパネルを二分するか判定
                bool hasL = false, hasR = false;
                foreach (var v in panel)
                {
                    double d = Cross(lineA, lineB, v);
                    if (d >  1e-6) hasL = true;
                    if (d < -1e-6) hasR = true;
                }

                if (hasL && hasR && BBoxOverlap(beam, panel))
                {
                    var h1 = PolygonUtils.ClipByHalfPlane(
                        new List<Point2d>(panel), lineA, lineB, ptL);
                    var h2 = PolygonUtils.ClipByHalfPlane(
                        new List<Point2d>(panel), lineA, lineB, ptR);
                    if (h1.Count >= 3) next.Add(h1);
                    if (h2.Count >= 3) next.Add(h2);
                }
                else
                {
                    next.Add(panel);
                }
            }
            panels = next;
        }
        return panels;
    }

    /// <summary>梁のバウンディングボックスがパネルと重なるか</summary>
    private static bool BBoxOverlap(BeamModel beam, List<Point2d> panel)
    {
        const double tol = 0.5;
        double pMinX = double.MaxValue, pMaxX = double.MinValue;
        double pMinY = double.MaxValue, pMaxY = double.MinValue;
        foreach (var v in panel)
        {
            if (v.X < pMinX) pMinX = v.X;
            if (v.X > pMaxX) pMaxX = v.X;
            if (v.Y < pMinY) pMinY = v.Y;
            if (v.Y > pMaxY) pMaxY = v.Y;
        }
        double bMinX = Math.Min(beam.StartPoint.X, beam.EndPoint.X);
        double bMaxX = Math.Max(beam.StartPoint.X, beam.EndPoint.X);
        double bMinY = Math.Min(beam.StartPoint.Y, beam.EndPoint.Y);
        double bMaxY = Math.Max(beam.StartPoint.Y, beam.EndPoint.Y);
        return bMaxX >= pMinX - tol && bMinX <= pMaxX + tol &&
               bMaxY >= pMinY - tol && bMinY <= pMaxY + tol;
    }

    /// <summary>
    /// サブパネルの各辺に対して角度2等分線で支配領域を求め、
    /// 対応する梁に面積を割り当てる。
    /// </summary>
    private void ApplyAngleBisectorToPanel(List<Point2d> panel)
    {
        int n = panel.Count;

        // 各頂点の内角2等分線方向
        var bis = new Vector2d[n];
        for (int i = 0; i < n; i++)
        {
            var prev = panel[(i - 1 + n) % n];
            var curr = panel[i];
            var next = panel[(i + 1) % n];
            var d1 = Normalize(prev.X - curr.X, prev.Y - curr.Y);
            var d2 = Normalize(next.X - curr.X, next.Y - curr.Y);
            double bx = d1.X + d2.X, by = d1.Y + d2.Y;
            double bLen = Math.Sqrt(bx * bx + by * by);
            bis[i] = bLen > 1e-10
                ? new Vector2d(bx / bLen, by / bLen)
                : new Vector2d(-d1.Y, d1.X);
        }

        // 各辺の支配領域
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            var edgeMid = new Point2d(
                (panel[i].X + panel[j].X) / 2.0,
                (panel[i].Y + panel[j].Y) / 2.0);

            var region = new List<Point2d>(panel);

            // 頂点 i の2等分線でクリップ
            region = PolygonUtils.ClipByHalfPlane(region,
                panel[i],
                new Point2d(panel[i].X + bis[i].X, panel[i].Y + bis[i].Y),
                edgeMid);

            // 頂点 j の2等分線でクリップ
            region = PolygonUtils.ClipByHalfPlane(region,
                panel[j],
                new Point2d(panel[j].X + bis[j].X, panel[j].Y + bis[j].Y),
                edgeMid);

            if (region.Count < 3) continue;
            double area = PolygonUtils.Area(region);
            if (area < 1e-6) continue;

            var beam = FindMatchingBeam(panel[i], panel[j]);
            if (beam != null)
            {
                beam.TributaryArea += area;
                beam.TributaryPolygons.Add(region);
            }
        }
    }

    // ─── 辺→梁マッチング ─────────────────────────────────────

    private BeamModel? FindMatchingBeam(Point2d edgeStart, Point2d edgeEnd)
    {
        double edgeLen = Dist(edgeStart, edgeEnd);
        if (edgeLen < 1e-9) return null;

        var edgeDir = Normalize(edgeEnd.X - edgeStart.X, edgeEnd.Y - edgeStart.Y);
        var edgeMid = new Point2d(
            (edgeStart.X + edgeEnd.X) / 2.0,
            (edgeStart.Y + edgeEnd.Y) / 2.0);

        double tolerance = Math.Max(edgeLen * 0.2, 0.15);

        BeamModel? best = null;
        double bestScore = double.MaxValue;

        foreach (var beam in _beams)
        {
            double dot = Math.Abs(
                edgeDir.X * beam.Direction.X + edgeDir.Y * beam.Direction.Y);
            if (dot < 0.8) continue;

            double lineDist = PointToLineDistance(edgeMid, beam.StartPoint, beam.EndPoint);
            if (lineDist > tolerance) continue;

            double segDist = PointToSegmentDistance(edgeMid, beam.StartPoint, beam.EndPoint);
            if (segDist < bestScore)
            {
                bestScore = segDist;
                best = beam;
            }
        }
        return best;
    }

    // ─── 幾何ヘルパー ────────────────────────────────────────

    private static Vector2d Normalize(double x, double y)
    {
        double len = Math.Sqrt(x * x + y * y);
        return len > 1e-10 ? new Vector2d(x / len, y / len) : new Vector2d(0, 0);
    }

    private static double Dist(Point2d a, Point2d b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    private static double Cross(Point2d a, Point2d b, Point2d p)
        => (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);

    private static double PointToLineDistance(Point2d p, Point2d a, Point2d b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double len = Math.Sqrt(dx * dx + dy * dy);
        if (len < 1e-10) return Dist(p, a);
        return Math.Abs(dx * (a.Y - p.Y) - dy * (a.X - p.X)) / len;
    }

    private static double PointToSegmentDistance(Point2d p, Point2d a, Point2d b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double len2 = dx * dx + dy * dy;
        if (len2 < 1e-20) return Dist(p, a);
        double t = Math.Max(0, Math.Min(1,
            ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / len2));
        return Dist(p, new Point2d(a.X + t * dx, a.Y + t * dy));
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

            cell = PolygonUtils.ClipByHalfPlane(cell, mid,
                new Point2d(mid.X + perp.X, mid.Y + perp.Y), target.Center);
        }

        target.VoronoiPolygon = cell;
        return PolygonUtils.Area(cell);
    }
}
