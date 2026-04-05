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
            // 梁+スラブ境界の線分から平面グラフを構築し、閉じた面(パネル)を列挙
            var panels = FindPanels(slab);
            foreach (var panel in panels)
            {
                var clean = CleanPolygon(panel);
                if (clean.Count < 3 || PolygonUtils.Area(clean) < 1e-4) continue;
                if (clean.Count == 4) PartitionQuad(clean);
                else if (clean.Count == 3) PartitionTriangle(clean);
                else PartitionGeneral(clean);
            }
        }
    }

    // ─── 平面グラフの面列挙 ─────────────────────────────────

    /// <summary>
    /// スラブ境界+梁の線分で平面グラフを構築し、
    /// 梁で囲まれた閉じた領域（サブパネル）を全て列挙する。
    /// </summary>
    private List<List<Point2d>> FindPanels(SlabModel slab)
    {
        // 1. 線分を収集（スラブ境界 + スラブ内の梁）
        var segs = new List<(Point2d a, Point2d b)>();
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
            segs.Add((sv[i], sv[(i + 1) % sv.Count]));
        foreach (var b in _beams)
            if (slab.Contains(b.MidPoint) || slab.Contains(b.StartPoint) || slab.Contains(b.EndPoint))
                segs.Add((b.StartPoint, b.EndPoint));

        // 2. 全線分の交点を求め、交点で線分を分割
        var pts = new List<Point2d>();
        var ptMap = new Dictionary<long, int>();
        int addPt(Point2d p)
        {
            // 0.0001m (0.1mm) 精度で丸め
            long key = ((long)Math.Round(p.X * 10000)) * 100000000L + (long)Math.Round(p.Y * 10000);
            if (ptMap.TryGetValue(key, out int idx)) return idx;
            idx = pts.Count; pts.Add(p); ptMap[key] = idx; return idx;
        }

        int nSeg = segs.Count;
        var segPts = new List<List<(double t, int idx)>>();
        for (int i = 0; i < nSeg; i++)
        {
            segPts.Add(new() { (0.0, addPt(segs[i].a)), (1.0, addPt(segs[i].b)) });
        }

        for (int i = 0; i < nSeg; i++)
            for (int j = i + 1; j < nSeg; j++)
            {
                var inter = SegSegIntersect(segs[i].a, segs[i].b, segs[j].a, segs[j].b);
                if (!inter.HasValue) continue;
                int pidx = addPt(inter.Value);
                double t1 = Param(segs[i].a, segs[i].b, inter.Value);
                double t2 = Param(segs[j].a, segs[j].b, inter.Value);
                if (t1 > 1e-6 && t1 < 1 - 1e-6) segPts[i].Add((t1, pidx));
                if (t2 > 1e-6 && t2 < 1 - 1e-6) segPts[j].Add((t2, pidx));
            }

        // 3. 辺リストを構築
        var edgeSet = new HashSet<(int, int)>();
        for (int i = 0; i < nSeg; i++)
        {
            var sp = segPts[i].OrderBy(x => x.t).ToList();
            for (int k = 0; k < sp.Count - 1; k++)
            {
                int a = sp[k].idx, b = sp[k + 1].idx;
                if (a != b) { edgeSet.Add((a, b)); edgeSet.Add((b, a)); }
            }
        }

        // 4. 隣接リストを角度順にソート
        int nPts = pts.Count;
        var adj = new List<int>[nPts];
        for (int i = 0; i < nPts; i++) adj[i] = new List<int>();
        foreach (var (a, b) in edgeSet) { if (!adj[a].Contains(b)) adj[a].Add(b); }
        for (int i = 0; i < nPts; i++)
        {
            var pi = pts[i];
            adj[i].Sort((a, b) =>
            {
                double aa = Math.Atan2(pts[a].Y - pi.Y, pts[a].X - pi.X);
                double ab = Math.Atan2(pts[b].Y - pi.Y, pts[b].X - pi.X);
                return aa.CompareTo(ab);
            });
        }

        // 5. 面を列挙（各有向辺の左側の面をトレース）
        var visited = new HashSet<long>();
        long eKey(int a, int b) => (long)a * nPts + b;
        var faces = new List<List<Point2d>>();

        foreach (var (a, b) in edgeSet)
        {
            if (visited.Contains(eKey(a, b))) continue;
            var face = new List<int>();
            int cu = a, cv = b;
            bool ok = true;
            for (int step = 0; step < nPts + 5; step++)
            {
                if (visited.Contains(eKey(cu, cv))) break;
                visited.Add(eKey(cu, cv));
                face.Add(cu);
                // cv での次の辺：adj[cv] で cu の位置を探し、1つ前（CW方向）
                int idx = adj[cv].IndexOf(cu);
                if (idx < 0) { ok = false; break; }
                int w = adj[cv][(idx - 1 + adj[cv].Count) % adj[cv].Count];
                cu = cv; cv = w;
            }
            if (!ok || face.Count < 3) continue;
            var poly = face.Select(i => pts[i]).ToList();
            double sa = SignedArea(poly);
            if (sa > 1e-4) // CCW = 有限面（内部）
            {
                var ctr = new Point2d(poly.Average(p => p.X), poly.Average(p => p.Y));
                if (slab.Contains(ctr))
                    faces.Add(poly);
            }
        }
        return faces;
    }

    // ─── 四角形の亀甲分割（LISP準拠）──────────────────────

    private void PartitionQuad(List<Point2d> panel)
    {
        var sorted = panel.OrderByDescending(p => p.Y).ThenBy(p => p.X).ToList();
        var tl = sorted[0]; var tr = sorted[1]; var bl = sorted[2]; var br = sorted[3];
        double bisTL = BisAngle(tl, tr, bl), bisTR = BisAngle(tr, tl, br);
        double bisBL = BisAngle(bl, tl, br), bisBR = BisAngle(br, bl, tr);
        double w = Math.Abs(tr.X - tl.X), h = Math.Abs(tl.Y - bl.Y);
        double ext = Math.Max(w, h) * 2;
        var eTL = Polar(tl, bisTL, ext); var eTR = Polar(tr, bisTR, ext);
        var eBL = Polar(bl, bisBL, ext); var eBR = Polar(br, bisBR, ext);
        Point2d? skA, skB;
        List<Point2d> rT, rB, rL, rR;
        if (w < h) {
            skA = LineInter(tl, eTL, tr, eTR); skB = LineInter(bl, eBL, br, eBR);
            if (!skA.HasValue || !skB.HasValue) return;
            rT = new() { tl, tr, skA.Value }; rB = new() { bl, br, skB.Value };
            rL = new() { tl, bl, skB.Value, skA.Value }; rR = new() { tr, br, skB.Value, skA.Value };
        } else {
            skA = LineInter(tl, eTL, bl, eBL); skB = LineInter(tr, eTR, br, eBR);
            if (!skA.HasValue || !skB.HasValue) return;
            rT = new() { tl, tr, skB.Value, skA.Value }; rB = new() { bl, br, skB.Value, skA.Value };
            rL = new() { tl, bl, skA.Value }; rR = new() { tr, br, skB.Value };
        }
        Assign(rT, tl, tr); Assign(rB, bl, br); Assign(rL, tl, bl); Assign(rR, tr, br);
    }

    private void PartitionTriangle(List<Point2d> panel)
    {
        var p0 = panel[0]; var p1 = panel[1]; var p2 = panel[2];
        double ext = Dist(p0, p1) * 2;
        var c = LineInter(p0, Polar(p0, BisAngle(p0, p1, p2), ext),
                          p1, Polar(p1, BisAngle(p1, p0, p2), ext));
        if (!c.HasValue) return;
        Assign(new() { p0, p1, c.Value }, p0, p1);
        Assign(new() { p1, p2, c.Value }, p1, p2);
        Assign(new() { p2, p0, c.Value }, p2, p0);
    }

    private void PartitionGeneral(List<Point2d> panel)
    {
        int n = panel.Count;
        double cx = panel.Average(v => v.X), cy = panel.Average(v => v.Y);
        for (int i = 0; i < n; i++)
            Assign(new() { panel[i], panel[(i + 1) % n], new(cx, cy) }, panel[i], panel[(i + 1) % n]);
    }

    private void Assign(List<Point2d> region, Point2d eA, Point2d eB)
    {
        if (region.Count < 3) return;
        double area = PolygonUtils.Area(region);
        if (area < 1e-6) return;
        var beam = FindBeam(eA, eB);
        if (beam != null) { beam.TributaryArea += area; beam.TributaryPolygons.Add(region); }
    }

    // ─── ヘルパー ────────────────────────────────────────────

    private static double BisAngle(Point2d v, Point2d a1, Point2d a2)
    {
        double ang1 = Math.Atan2(a1.Y - v.Y, a1.X - v.X);
        double ang2 = Math.Atan2(a2.Y - v.Y, a2.X - v.X);
        double d = ang2 - ang1;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return ang1 + d / 2.0;
    }

    private static Point2d Polar(Point2d p, double a, double d)
        => new(p.X + Math.Cos(a) * d, p.Y + Math.Sin(a) * d);

    private static Point2d? LineInter(Point2d a, Point2d b, Point2d c, Point2d d)
    {
        double a1 = b.Y - a.Y, b1 = a.X - b.X, c1 = a1 * a.X + b1 * a.Y;
        double a2 = d.Y - c.Y, b2 = c.X - d.X, c2 = a2 * c.X + b2 * c.Y;
        double det = a1 * b2 - a2 * b1;
        return Math.Abs(det) < 1e-12 ? null : new((c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det);
    }

    private static Point2d? SegSegIntersect(Point2d a1, Point2d b1, Point2d a2, Point2d b2)
    {
        double d1x = b1.X - a1.X, d1y = b1.Y - a1.Y, d2x = b2.X - a2.X, d2y = b2.Y - a2.Y;
        double den = d1x * d2y - d1y * d2x;
        if (Math.Abs(den) < 1e-10) return null;
        double dx = a2.X - a1.X, dy = a2.Y - a1.Y;
        double t = (dx * d2y - dy * d2x) / den, s = (dx * d1y - dy * d1x) / den;
        if (t < -1e-8 || t > 1 + 1e-8 || s < -1e-8 || s > 1 + 1e-8) return null;
        return new(a1.X + t * d1x, a1.Y + t * d1y);
    }

    private static double Param(Point2d a, Point2d b, Point2d p)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
        return l2 < 1e-20 ? 0 : ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2;
    }

    private static double SignedArea(List<Point2d> p)
    {
        double a = 0; int n = p.Count;
        for (int i = 0; i < n; i++) { var c = p[i]; var d = p[(i + 1) % n]; a += c.X * d.Y - d.X * c.Y; }
        return a / 2;
    }

    private BeamModel? FindBeam(Point2d eA, Point2d eB)
    {
        double eL = Dist(eA, eB); if (eL < 1e-9) return null;
        var eD = new Vector2d((eB.X - eA.X) / eL, (eB.Y - eA.Y) / eL);
        var eM = new Point2d((eA.X + eB.X) / 2, (eA.Y + eB.Y) / 2);
        double tol = Math.Max(eL * 0.2, 0.15);
        BeamModel? best = null; double bs = double.MaxValue;
        foreach (var b in _beams)
        {
            if (Math.Abs(eD.X * b.Direction.X + eD.Y * b.Direction.Y) < 0.8) continue;
            double ld = PtLineDist(eM, b.StartPoint, b.EndPoint); if (ld > tol) continue;
            double sd = PtSegDist(eM, b.StartPoint, b.EndPoint);
            if (sd < bs) { bs = sd; best = b; }
        }
        return best;
    }

    private static List<Point2d> CleanPolygon(List<Point2d> pts)
    {
        if (pts.Count < 3) return pts;
        var r = new List<Point2d> { pts[0] };
        for (int i = 1; i < pts.Count; i++) if (Dist(pts[i], r[^1]) > 1e-5) r.Add(pts[i]);
        if (r.Count > 1 && Dist(r[^1], r[0]) < 1e-5) r.RemoveAt(r.Count - 1);
        var c = new List<Point2d>(); int n = r.Count;
        for (int i = 0; i < n; i++)
        {
            var prev = r[(i - 1 + n) % n]; var cur = r[i]; var next = r[(i + 1) % n];
            if (Math.Abs((next.X - cur.X) * (prev.Y - cur.Y) - (next.Y - cur.Y) * (prev.X - cur.X)) > 1e-5)
                c.Add(cur);
        }
        return c.Count >= 3 ? c : r;
    }

    private static double Dist(Point2d a, Point2d b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    private static double PtLineDist(Point2d p, Point2d a, Point2d b)
    { double dx = b.X - a.X, dy = b.Y - a.Y, l = Math.Sqrt(dx * dx + dy * dy);
      return l < 1e-10 ? Dist(p, a) : Math.Abs(dx * (a.Y - p.Y) - dy * (a.X - p.X)) / l; }
    private static double PtSegDist(Point2d p, Point2d a, Point2d b)
    { double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
      if (l2 < 1e-20) return Dist(p, a);
      double t = Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1);
      return Dist(p, new Point2d(a.X + t * dx, a.Y + t * dy)); }

    // ════════════════════════════════════════════════════════
    //  柱（ボロノイ）
    // ════════════════════════════════════════════════════════
    private void CalculateColumnAreas()
    { foreach (var c in _columns) c.TributaryArea = CalcVoronoi(c); }
    private double CalcVoronoi(ColumnModel tgt)
    {
        var slab = _slabs.FirstOrDefault(s => s.Contains(tgt.Center));
        if (slab == null) return 0;
        var cell = new List<Point2d>(slab.Vertices);
        foreach (var o in _columns)
        {
            if (o == tgt || cell.Count == 0) continue;
            var m = new Point2d((tgt.Center.X + o.Center.X) / 2, (tgt.Center.Y + o.Center.Y) / 2);
            var d = new Vector2d(o.Center.X - tgt.Center.X, o.Center.Y - tgt.Center.Y);
            cell = PolygonUtils.ClipByHalfPlane(cell, m, new Point2d(m.X - d.Y, m.Y + d.X), tgt.Center);
        }
        tgt.VoronoiPolygon = cell; return PolygonUtils.Area(cell);
    }
}
