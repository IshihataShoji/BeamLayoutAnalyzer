using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 各部材の負担面積を算出する（亀甲分割 + ボロノイ分割）。
///
/// 梁：亀甲分割
///   1. スラブ内部の梁ラインでスラブをサブパネルに分割
///   2. 各サブパネルの頂点で内角2等分線を計算
///   3. ClipByHalfPlane で各辺の支配領域を切り出す
///   4. 対応する梁に面積を割り当てる
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
    //  梁の負担面積（亀甲分割）
    // ════════════════════════════════════════════════════════

    private void CalculateBeamAreas()
    {
        foreach (var slab in _slabs)
        {
            if (slab.Vertices.Count < 3) continue;

            var interiorBeams = _beams.Where(b => slab.Contains(b.MidPoint)).ToList();
            var panels = SubdivideSlab(slab.Vertices, interiorBeams);

            foreach (var panel in panels)
            {
                if (panel.Count < 3 || PolygonUtils.Area(panel) < 1e-4) continue;
                ApplyBisectorPartition(panel);
            }
        }
    }

    // ─── サブパネル分割 ───────────────────────────────────────

    private static List<List<Point2d>> SubdivideSlab(
        List<Point2d> slabVerts, List<BeamModel> beams)
    {
        var panels = new List<List<Point2d>> { new(slabVerts) };

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

    private static bool BBoxOverlap(BeamModel beam, List<Point2d> panel)
    {
        const double tol = 0.5;
        double pMinX = double.MaxValue, pMaxX = double.MinValue;
        double pMinY = double.MaxValue, pMaxY = double.MinValue;
        foreach (var v in panel)
        {
            pMinX = Math.Min(pMinX, v.X); pMaxX = Math.Max(pMaxX, v.X);
            pMinY = Math.Min(pMinY, v.Y); pMaxY = Math.Max(pMaxY, v.Y);
        }
        return Math.Max(beam.StartPoint.X, beam.EndPoint.X) >= pMinX - tol &&
               Math.Min(beam.StartPoint.X, beam.EndPoint.X) <= pMaxX + tol &&
               Math.Max(beam.StartPoint.Y, beam.EndPoint.Y) >= pMinY - tol &&
               Math.Min(beam.StartPoint.Y, beam.EndPoint.Y) <= pMaxY + tol;
    }

    // ─── 亀甲分割（ClipByHalfPlane方式）────────────────────

    /// <summary>
    /// サブパネルの各辺に対して、両端頂点の内角2等分線で半平面クリッピングし、
    /// 各辺の支配領域（負担面積）を求める。
    /// </summary>
    private void ApplyBisectorPartition(List<Point2d> panel)
    {
        int n = panel.Count;

        // パネルの重心（内部点の代表）
        double cx = 0, cy = 0;
        foreach (var v in panel) { cx += v.X; cy += v.Y; }
        cx /= n; cy /= n;
        var centroid = new Point2d(cx, cy);

        // 各頂点の内角2等分線の方向ベクトルを計算
        var bisDir = new Vector2d[n];
        for (int i = 0; i < n; i++)
        {
            var prev = panel[(i - 1 + n) % n];
            var curr = panel[i];
            var next = panel[(i + 1) % n];

            // curr から prev, next への単位ベクトル
            var d1 = Normalize(prev.X - curr.X, prev.Y - curr.Y);
            var d2 = Normalize(next.X - curr.X, next.Y - curr.Y);

            // 2等分線 = 2つの単位ベクトルの和
            double bx = d1.X + d2.X;
            double by = d1.Y + d2.Y;
            double bLen = Math.Sqrt(bx * bx + by * by);

            if (bLen > 1e-10)
            {
                bisDir[i] = new Vector2d(bx / bLen, by / bLen);
            }
            else
            {
                // 180°（直線）の場合：辺 curr→next の法線を内向きに
                bisDir[i] = new Vector2d(-d1.Y, d1.X);
                // 内向き確認：centroid 側に向いているか
                double dot = (centroid.X - curr.X) * bisDir[i].X
                           + (centroid.Y - curr.Y) * bisDir[i].Y;
                if (dot < 0)
                    bisDir[i] = new Vector2d(d1.Y, -d1.X);
            }

            // 2等分線が内向きか確認（centroid方向と同じ半平面にあるか）
            double check = (centroid.X - curr.X) * bisDir[i].X
                         + (centroid.Y - curr.Y) * bisDir[i].Y;
            if (check < 0)
            {
                // 外向き → 反転する
                bisDir[i] = new Vector2d(-bisDir[i].X, -bisDir[i].Y);
            }
        }

        // 各辺の支配領域をクリッピングで求める
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;

            // 辺の中点（クリッピングの「内側」判定用）
            var edgeMid = new Point2d(
                (panel[i].X + panel[j].X) / 2.0,
                (panel[i].Y + panel[j].Y) / 2.0);

            // パネル全体からスタート
            var region = new List<Point2d>(panel);

            // 頂点 i の2等分線でクリップ
            // 2等分線は頂点iを通り、bisDir[i]方向へ伸びる直線
            var bisLineEnd_i = new Point2d(
                panel[i].X + bisDir[i].X,
                panel[i].Y + bisDir[i].Y);
            region = PolygonUtils.ClipByHalfPlane(
                region, panel[i], bisLineEnd_i, edgeMid);

            // 頂点 j の2等分線でクリップ
            var bisLineEnd_j = new Point2d(
                panel[j].X + bisDir[j].X,
                panel[j].Y + bisDir[j].Y);
            region = PolygonUtils.ClipByHalfPlane(
                region, panel[j], bisLineEnd_j, edgeMid);

            if (region.Count < 3) continue;
            double area = PolygonUtils.Area(region);
            if (area < 1e-6) continue;

            // 対応する梁を特定して割り当て
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

        var edgeDir = Normalize(
            edgeEnd.X - edgeStart.X, edgeEnd.Y - edgeStart.Y);
        var edgeMid = new Point2d(
            (edgeStart.X + edgeEnd.X) / 2.0,
            (edgeStart.Y + edgeEnd.Y) / 2.0);

        double tolerance = Math.Max(edgeLen * 0.2, 0.15);

        BeamModel? best = null;
        double bestScore = double.MaxValue;

        foreach (var beam in _beams)
        {
            // 方向一致チェック
            double dot = Math.Abs(
                edgeDir.X * beam.Direction.X + edgeDir.Y * beam.Direction.Y);
            if (dot < 0.8) continue;

            // 辺中点→梁直線の距離
            double lineDist = PointToLineDistance(
                edgeMid, beam.StartPoint, beam.EndPoint);
            if (lineDist > tolerance) continue;

            // 辺中点→梁線分の距離（スコア用）
            double segDist = PointToSegmentDistance(
                edgeMid, beam.StartPoint, beam.EndPoint);
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
            var mid = new Point2d(
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
