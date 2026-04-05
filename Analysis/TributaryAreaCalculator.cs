using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

public class TributaryAreaCalculator
{
    private readonly List<SlabModel>   _slabs;
    private readonly List<ColumnModel> _columns;
    private readonly List<BeamModel>   _beams;

    public TributaryAreaCalculator(
        List<SlabModel> slabs, List<ColumnModel> columns, List<BeamModel> beams)
    { _slabs = slabs; _columns = columns; _beams = beams; }

    public void Calculate()
    {
        foreach (var b in _beams) { b.TributaryArea = 0; b.TributaryPolygons.Clear(); }
        foreach (var c in _columns) c.TributaryArea = 0;
        CalculateBeamAreas();
        CalculateColumnAreas();
    }

    // ════════════════════════════════════════════════════════
    //  梁の負担面積
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
                var clean = CleanPolygon(panel);
                if (clean.Count < 3 || PolygonUtils.Area(clean) < 1e-4) continue;
                if (clean.Count == 4)
                    PartitionQuad(clean);
                else if (clean.Count == 3)
                    PartitionTriangle(clean);
                else
                    PartitionGeneral(clean);
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
            var lineA = beam.StartPoint; var lineB = beam.EndPoint;
            var n = new Vector2d(-beam.Direction.Y, beam.Direction.X);
            var ptL = new Point2d(beam.MidPoint.X + n.X, beam.MidPoint.Y + n.Y);
            var ptR = new Point2d(beam.MidPoint.X - n.X, beam.MidPoint.Y - n.Y);
            var next = new List<List<Point2d>>();
            foreach (var p in panels)
            {
                bool hasL = false, hasR = false;
                foreach (var v in p)
                {
                    double d = Cross(lineA, lineB, v);
                    if (d > 1e-6) hasL = true; if (d < -1e-6) hasR = true;
                }
                if (hasL && hasR && BBoxOverlap(beam, p))
                {
                    var h1 = PolygonUtils.ClipByHalfPlane(new List<Point2d>(p), lineA, lineB, ptL);
                    var h2 = PolygonUtils.ClipByHalfPlane(new List<Point2d>(p), lineA, lineB, ptR);
                    if (h1.Count >= 3) next.Add(h1); if (h2.Count >= 3) next.Add(h2);
                }
                else next.Add(p);
            }
            panels = next;
        }
        return panels;
    }

    // ─── 四角形の亀甲分割（LISPアルゴリズム準拠）──────────

    /// <summary>
    /// LISPコード「KIKKOWARI」と同等のアルゴリズム：
    /// 1. 頂点をTL,TR,BL,BRにソート
    /// 2. 各頂点の角度2等分線を計算
    /// 3. 短辺側の2等分線交点2つを求める
    /// 4. 4領域（台形2+三角形2）に分割
    /// </summary>
    private void PartitionQuad(List<Point2d> panel)
    {
        // Y降順→X昇順でTL,TR,BL,BRに分類
        var sorted = panel.OrderByDescending(p => p.Y).ThenBy(p => p.X).ToList();
        var tl = sorted[0]; var tr = sorted[1];
        var bl = sorted[2]; var br = sorted[3];

        // 各頂点の角度2等分線の角度を計算（AutoCADのangle関数と同等）
        double bisTL = BisectorAngle(tl, tr, bl);
        double bisTR = BisectorAngle(tr, tl, br);
        double bisBL = BisectorAngle(bl, tl, br);
        double bisBR = BisectorAngle(br, bl, tr);

        // 短辺判定
        double width  = Math.Abs(tr.X - tl.X);
        double height = Math.Abs(tl.Y - bl.Y);

        // 2等分線の延長距離（十分長く）
        double ext = Math.Max(width, height) * 2;

        var ptTL2 = Polar(tl, bisTL, ext);
        var ptTR2 = Polar(tr, bisTR, ext);
        var ptBL2 = Polar(bl, bisBL, ext);
        var ptBR2 = Polar(br, bisBR, ext);

        Point2d? skA, skB;
        List<Point2d> regTop, regBottom, regLeft, regRight;

        if (width < height)
        {
            // 縦長：上辺と下辺が短辺 → 上2頂点と下2頂点の交点
            skA = LineIntersect(tl, ptTL2, tr, ptTR2);
            skB = LineIntersect(bl, ptBL2, br, ptBR2);
            if (!skA.HasValue || !skB.HasValue) return;
            regTop    = new() { tl, tr, skA.Value };
            regBottom = new() { bl, br, skB.Value };
            regLeft   = new() { tl, bl, skB.Value, skA.Value };
            regRight  = new() { tr, br, skB.Value, skA.Value };
        }
        else
        {
            // 横長：左辺と右辺が短辺 → 左2頂点と右2頂点の交点
            skA = LineIntersect(tl, ptTL2, bl, ptBL2);
            skB = LineIntersect(tr, ptTR2, br, ptBR2);
            if (!skA.HasValue || !skB.HasValue) return;
            regTop    = new() { tl, tr, skB.Value, skA.Value };
            regBottom = new() { bl, br, skB.Value, skA.Value };
            regLeft   = new() { tl, bl, skA.Value };
            regRight  = new() { tr, br, skB.Value };
        }

        // 各領域を対応する梁に割り当て
        AssignRegion(regTop,    tl, tr);
        AssignRegion(regBottom, bl, br);
        AssignRegion(regLeft,   tl, bl);
        AssignRegion(regRight,  tr, br);
    }

    // ─── 三角形の亀甲分割 ────────────────────────────────────

    private void PartitionTriangle(List<Point2d> panel)
    {
        // 3頂点の2等分線は1点で交わる（内心）
        var p0 = panel[0]; var p1 = panel[1]; var p2 = panel[2];
        double b0 = BisectorAngle(p0, p1, p2);
        double b1 = BisectorAngle(p1, p0, p2);
        double ext = Dist(p0, p1) * 2;
        var center = LineIntersect(p0, Polar(p0, b0, ext), p1, Polar(p1, b1, ext));
        if (!center.HasValue) return;
        var c = center.Value;
        AssignRegion(new() { p0, p1, c }, p0, p1);
        AssignRegion(new() { p1, p2, c }, p1, p2);
        AssignRegion(new() { p2, p0, c }, p2, p0);
    }

    // ─── 一般多角形のフォールバック ──────────────────────────

    private void PartitionGeneral(List<Point2d> panel)
    {
        int n = panel.Count;
        double cx = panel.Average(v => v.X), cy = panel.Average(v => v.Y);
        // 重心から各辺へ三角形分割（簡易）
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            var region = new List<Point2d> { panel[i], panel[j], new Point2d(cx, cy) };
            AssignRegion(region, panel[i], panel[j]);
        }
    }

    // ─── 領域→梁割り当て ──────────────────────────────────

    private void AssignRegion(List<Point2d> region, Point2d edgeA, Point2d edgeB)
    {
        if (region.Count < 3) return;
        double area = PolygonUtils.Area(region);
        if (area < 1e-6) return;
        var beam = FindMatchingBeam(edgeA, edgeB);
        if (beam != null)
        {
            beam.TributaryArea += area;
            beam.TributaryPolygons.Add(region);
        }
    }

    // ─── 角度2等分線（LISPのangle関数準拠）──────────────────

    /// <summary>
    /// 頂点 vertex における、隣接2頂点 adj1, adj2 への角度の2等分角を返す。
    /// AutoCADの angle(vertex, adj1) と angle(vertex, adj2) の平均。
    /// 角度の折り返しを考慮して正しい内角の2等分を求める。
    /// </summary>
    private static double BisectorAngle(Point2d vertex, Point2d adj1, Point2d adj2)
    {
        double a1 = Math.Atan2(adj1.Y - vertex.Y, adj1.X - vertex.X);
        double a2 = Math.Atan2(adj2.Y - vertex.Y, adj2.X - vertex.X);

        // 角度差を -π ～ +π に正規化
        double diff = a2 - a1;
        while (diff > Math.PI)  diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        return a1 + diff / 2.0;
    }

    // ─── 幾何ヘルパー ──────────────────────────────────────

    private static Point2d Polar(Point2d pt, double angle, double dist)
        => new(pt.X + Math.Cos(angle) * dist, pt.Y + Math.Sin(angle) * dist);

    private static Point2d? LineIntersect(Point2d a, Point2d b, Point2d c, Point2d d)
    {
        double a1 = b.Y - a.Y, b1 = a.X - b.X, c1 = a1 * a.X + b1 * a.Y;
        double a2 = d.Y - c.Y, b2 = c.X - d.X, c2 = a2 * c.X + b2 * c.Y;
        double det = a1 * b2 - a2 * b1;
        if (Math.Abs(det) < 1e-12) return null;
        return new Point2d((c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det);
    }

    private BeamModel? FindMatchingBeam(Point2d eA, Point2d eB)
    {
        double eLen = Dist(eA, eB);
        if (eLen < 1e-9) return null;
        var eDir = new Vector2d((eB.X - eA.X) / eLen, (eB.Y - eA.Y) / eLen);
        var eMid = new Point2d((eA.X + eB.X) / 2, (eA.Y + eB.Y) / 2);
        double tol = Math.Max(eLen * 0.2, 0.15);
        BeamModel? best = null; double bestS = double.MaxValue;
        foreach (var b in _beams)
        {
            if (Math.Abs(eDir.X * b.Direction.X + eDir.Y * b.Direction.Y) < 0.8) continue;
            double ld = PointToLineDistance(eMid, b.StartPoint, b.EndPoint);
            if (ld > tol) continue;
            double sd = PointToSegDist(eMid, b.StartPoint, b.EndPoint);
            if (sd < bestS) { bestS = sd; best = b; }
        }
        return best;
    }

    private static List<Point2d> CleanPolygon(List<Point2d> pts)
    {
        if (pts.Count < 3) return pts;
        // 重複頂点除去
        var r = new List<Point2d> { pts[0] };
        for (int i = 1; i < pts.Count; i++)
            if (Dist(pts[i], r[^1]) > 1e-6) r.Add(pts[i]);
        if (r.Count > 1 && Dist(r[^1], r[0]) < 1e-6) r.RemoveAt(r.Count - 1);
        // ほぼ直線の頂点除去（角度≒180°）
        var c = new List<Point2d>();
        int n = r.Count;
        for (int i = 0; i < n; i++)
        {
            var prev = r[(i - 1 + n) % n]; var curr = r[i]; var next = r[(i + 1) % n];
            double cross = (next.X - curr.X) * (prev.Y - curr.Y) - (next.Y - curr.Y) * (prev.X - curr.X);
            if (Math.Abs(cross) > 1e-6) c.Add(curr);
        }
        return c.Count >= 3 ? c : r;
    }

    private static double Dist(Point2d a, Point2d b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    private static double Cross(Point2d a, Point2d b, Point2d p)
        => (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);
    private static double PointToLineDistance(Point2d p, Point2d a, Point2d b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, len = Math.Sqrt(dx * dx + dy * dy);
        return len < 1e-10 ? Dist(p, a) : Math.Abs(dx * (a.Y - p.Y) - dy * (a.X - p.X)) / len;
    }
    private static double PointToSegDist(Point2d p, Point2d a, Point2d b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
        if (l2 < 1e-20) return Dist(p, a);
        double t = Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1);
        return Dist(p, new Point2d(a.X + t * dx, a.Y + t * dy));
    }
    private static bool BBoxOverlap(BeamModel beam, List<Point2d> panel)
    {
        const double t = 0.5;
        double pxn = double.MaxValue, pxx = double.MinValue, pyn = double.MaxValue, pyx = double.MinValue;
        foreach (var v in panel) { pxn = Math.Min(pxn, v.X); pxx = Math.Max(pxx, v.X); pyn = Math.Min(pyn, v.Y); pyx = Math.Max(pyx, v.Y); }
        return Math.Max(beam.StartPoint.X, beam.EndPoint.X) >= pxn - t && Math.Min(beam.StartPoint.X, beam.EndPoint.X) <= pxx + t &&
               Math.Max(beam.StartPoint.Y, beam.EndPoint.Y) >= pyn - t && Math.Min(beam.StartPoint.Y, beam.EndPoint.Y) <= pyx + t;
    }

    // ════════════════════════════════════════════════════════
    //  柱の負担面積（ボロノイ分割）
    // ════════════════════════════════════════════════════════

    private void CalculateColumnAreas()
    { foreach (var col in _columns) col.TributaryArea = CalcColumnVoronoi(col); }

    private double CalcColumnVoronoi(ColumnModel target)
    {
        var slab = _slabs.FirstOrDefault(s => s.Contains(target.Center));
        if (slab == null) return 0;
        var cell = new List<Point2d>(slab.Vertices);
        foreach (var o in _columns)
        {
            if (o == target || cell.Count == 0) continue;
            var mid = new Point2d((target.Center.X + o.Center.X) / 2, (target.Center.Y + o.Center.Y) / 2);
            var d = new Vector2d(o.Center.X - target.Center.X, o.Center.Y - target.Center.Y);
            cell = PolygonUtils.ClipByHalfPlane(cell, mid, new Point2d(mid.X - d.Y, mid.Y + d.X), target.Center);
        }
        target.VoronoiPolygon = cell;
        return PolygonUtils.Area(cell);
    }
}
