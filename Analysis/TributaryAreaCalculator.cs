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

            var beamsInSlab = _beams.Where(b =>
                slab.Contains(b.MidPoint) || slab.Contains(b.StartPoint) || slab.Contains(b.EndPoint)).ToList();

            // ① 梁に囲まれたパネル → 亀甲分割
            var panels = FindPanels(slab);
            foreach (var panel in panels)
            {
                var clean = CleanPolygon(panel);
                if (clean.Count < 3 || PolygonUtils.Area(clean) < 1e-4) continue;
                if (clean.Count == 4) PartitionQuad(clean, slab, beamsInSlab);
                else if (clean.Count == 3) PartitionTriangle(clean);
                else PartitionGeneral(clean);
            }

            // ② マージン領域（梁で囲まれていない部分）→ グリッド分割
            double assignedArea = beamsInSlab.Sum(b => b.TributaryArea);
            if (slab.Area - assignedArea > 0.5)
                AssignMarginByGrid(slab, beamsInSlab, panels);
        }
    }

    // ─── マージン領域の分割（グリッドサンプリング方式）────

    /// <summary>
    /// 梁に囲まれていないスラブ領域（マージン）を、
    /// 最寄りの梁線分に割り当てる。
    /// スラブ上にグリッドを敷き、各点が内部パネル外であれば
    /// 最短距離の梁に面積を割り当てる。
    /// 内部パネルの亀甲分割には一切影響しない。
    /// </summary>
    private void AssignMarginByGrid(
        SlabModel slab, List<BeamModel> beamsInSlab, List<List<Point2d>> innerPanels)
    {
        if (beamsInSlab.Count == 0) return;

        double minX = slab.Vertices.Min(v => v.X);
        double maxX = slab.Vertices.Max(v => v.X);
        double minY = slab.Vertices.Min(v => v.Y);
        double maxY = slab.Vertices.Max(v => v.Y);

        double gridSize = 0.1; // 0.1m（10cm）精度
        double cellArea = gridSize * gridSize;

        var marginAreas = new Dictionary<BeamModel, double>();
        var marginPoints = new Dictionary<BeamModel, List<Point2d>>();

        for (double x = minX + gridSize / 2; x < maxX; x += gridSize)
        {
            for (double y = minY + gridSize / 2; y < maxY; y += gridSize)
            {
                var pt = new Point2d(x, y);
                if (!slab.Contains(pt)) continue;
                if (innerPanels.Any(p => PointInPolygon(pt, p))) continue;

                BeamModel? nearest = null;
                double minDist = double.MaxValue;
                foreach (var beam in beamsInSlab)
                {
                    double d = PtSegDist(pt, beam.StartPoint, beam.EndPoint);
                    if (d < minDist) { minDist = d; nearest = beam; }
                }
                if (nearest == null) continue;

                if (!marginAreas.ContainsKey(nearest))
                {
                    marginAreas[nearest] = 0;
                    marginPoints[nearest] = new List<Point2d>();
                }
                marginAreas[nearest] += cellArea;
                marginPoints[nearest].Add(pt);
            }
        }

        const double SHARE_TOL = 0.15;

        foreach (var kvp in marginAreas)
        {
            if (kvp.Value < 0.01) continue;
            kvp.Key.TributaryArea += kvp.Value;

            var beam = kvp.Key;
            var pts = marginPoints[beam];
            if (pts.Count < 3) continue;

            // スラブポリゴンから出発（角がスラブ境界と正確に一致）
            var poly = new List<Point2d>(slab.Vertices);

            // 梁ラインでクリップ → マージン側のみ保持
            var marginCtr = new Point2d(pts.Average(p => p.X), pts.Average(p => p.Y));
            poly = PolygonUtils.ClipByHalfPlane(poly, beam.StartPoint, beam.EndPoint, marginCtr);
            if (poly.Count < 3) continue;

            // 他のマージン梁との分割線でクリップ
            foreach (var other in beamsInSlab)
            {
                if (other == beam || poly.Count < 3) continue;
                if (!marginAreas.ContainsKey(other)) continue;

                // 端点共有判定
                Point2d? sharedPt = null;
                Point2d beamOther = beam.EndPoint, otherAdj = other.EndPoint;
                if (Dist(beam.StartPoint, other.StartPoint) < SHARE_TOL)
                    { sharedPt = beam.StartPoint; beamOther = beam.EndPoint; otherAdj = other.EndPoint; }
                else if (Dist(beam.StartPoint, other.EndPoint) < SHARE_TOL)
                    { sharedPt = beam.StartPoint; beamOther = beam.EndPoint; otherAdj = other.StartPoint; }
                else if (Dist(beam.EndPoint, other.StartPoint) < SHARE_TOL)
                    { sharedPt = beam.EndPoint; beamOther = beam.StartPoint; otherAdj = other.EndPoint; }
                else if (Dist(beam.EndPoint, other.EndPoint) < SHARE_TOL)
                    { sharedPt = beam.EndPoint; beamOther = beam.StartPoint; otherAdj = other.StartPoint; }

                if (sharedPt.HasValue)
                {
                    // 端点共有 → 角度二等分線
                    var sp = sharedPt.Value;
                    double bisAng = BisAngle(sp, beamOther, otherAdj);
                    var bisEnd = Polar(sp, bisAng, 100);
                    poly = PolygonUtils.ClipByHalfPlane(poly, sp, bisEnd, beam.MidPoint);
                }
                else
                {
                    // 非共有 → 中点間の垂直二等分線
                    var mid = new Point2d(
                        (beam.MidPoint.X + other.MidPoint.X) / 2,
                        (beam.MidPoint.Y + other.MidPoint.Y) / 2);
                    var diff = new Vector2d(
                        other.MidPoint.X - beam.MidPoint.X,
                        other.MidPoint.Y - beam.MidPoint.Y);
                    var perp = new Vector2d(-diff.Y, diff.X);
                    poly = PolygonUtils.ClipByHalfPlane(poly, mid,
                        new Point2d(mid.X + perp.X, mid.Y + perp.Y), beam.MidPoint);
                }
            }

            if (poly.Count >= 3)
                beam.TributaryPolygons.Add(poly);
        }
    }

    private static List<Point2d> ConvexHull(List<Point2d> points)
    {
        var pts = points.OrderBy(p => p.X).ThenBy(p => p.Y).ToList();
        int n = pts.Count;
        if (n < 3) return new List<Point2d>(pts);
        var hull = new List<Point2d>();
        foreach (var p in pts)
        {
            while (hull.Count >= 2 && CrossProduct(hull[hull.Count - 2], hull[hull.Count - 1], p) <= 0)
                hull.RemoveAt(hull.Count - 1);
            hull.Add(p);
        }
        int lower = hull.Count + 1;
        for (int i = n - 2; i >= 0; i--)
        {
            while (hull.Count >= lower && CrossProduct(hull[hull.Count - 2], hull[hull.Count - 1], pts[i]) <= 0)
                hull.RemoveAt(hull.Count - 1);
            hull.Add(pts[i]);
        }
        hull.RemoveAt(hull.Count - 1);
        return hull;
    }

    private static double CrossProduct(Point2d o, Point2d a, Point2d b)
        => (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);

    private static bool PointInPolygon(Point2d pt, List<Point2d> polygon)
    {
        bool inside = false;
        int n = polygon.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            var a = polygon[i]; var b = polygon[j];
            if ((a.Y > pt.Y) != (b.Y > pt.Y) &&
                pt.X < (b.X - a.X) * (pt.Y - a.Y) / (b.Y - a.Y) + a.X)
                inside = !inside;
        }
        return inside;
    }

    /// <summary>梁の端点が他の要素に接続しているか判定</summary>
    private bool IsEndpointConnected(
        Point2d ep, BeamModel self, List<BeamModel> beams, SlabModel slab, double tol)
    {
        // (1) 他の梁の端点に近い
        foreach (var other in beams)
        {
            if (other == self) continue;
            if (Dist(ep, other.StartPoint) < tol || Dist(ep, other.EndPoint) < tol)
                return true;
        }
        // (2) 他の梁の線分上にある（T字接合）
        foreach (var other in beams)
        {
            if (other == self) continue;
            if (PtSegDist(ep, other.StartPoint, other.EndPoint) < tol)
                return true;
        }
        // (3) スラブ境界上にある
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
        {
            if (PtSegDist(ep, sv[i], sv[(i + 1) % sv.Count]) < tol)
                return true;
        }
        // (4) 柱に近い
        foreach (var col in _columns)
        {
            if (Dist(ep, new Point2d(col.Center.X, col.Center.Y)) < tol + col.Radius)
                return true;
        }
        return false;
    }
    // ─── 平面グラフの面列挙 ─────────────────────────────────

    /// <summary>自由端を梁方向にスラブ境界まで延長した点を返す</summary>
    private static Point2d ExtendToSlabBoundary(Point2d freeEnd, Point2d connectedEnd, SlabModel slab)
    {
        // 梁の方向: connectedEnd → freeEnd
        var dir = new Vector2d(freeEnd.X - connectedEnd.X, freeEnd.Y - connectedEnd.Y);
        double len = Math.Sqrt(dir.X * dir.X + dir.Y * dir.Y);
        if (len < 1e-10) return freeEnd;
        var d = new Vector2d(dir.X / len, dir.Y / len);

        // freeEnd から方向 d に沿ってスラブ境界との交点を求める
        var sv = slab.Vertices;
        double bestT = double.MaxValue;
        Point2d bestPt = freeEnd;
        for (int i = 0; i < sv.Count; i++)
        {
            var p = sv[i];
            var q = sv[(i + 1) % sv.Count];
            // Ray: freeEnd + t * d  と  線分 p-q の交点を求める
            double ex = q.X - p.X, ey = q.Y - p.Y;
            double denom = d.X * ey - d.Y * ex;
            if (Math.Abs(denom) < 1e-10) continue;
            double t = ((p.X - freeEnd.X) * ey - (p.Y - freeEnd.Y) * ex) / denom;
            double u = ((p.X - freeEnd.X) * d.Y - (p.Y - freeEnd.Y) * d.X) / denom;
            if (t > 1e-6 && u >= -1e-6 && u <= 1 + 1e-6 && t < bestT)
            {
                bestT = t;
                bestPt = new Point2d(freeEnd.X + t * d.X, freeEnd.Y + t * d.Y);
            }
        }
        return bestPt;
    }

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
        var beamsInSlab = _beams.Where(b =>
            slab.Contains(b.MidPoint) || slab.Contains(b.StartPoint) || slab.Contains(b.EndPoint)).ToList();

        // 全梁を面列挙に含める（浮いた梁は自由端をスラブ境界まで延長）
        const double CONN_TOL = 0.15;
        foreach (var b in beamsInSlab)
        {
            bool startOk = IsEndpointConnected(b.StartPoint, b, beamsInSlab, slab, CONN_TOL);
            bool endOk   = IsEndpointConnected(b.EndPoint, b, beamsInSlab, slab, CONN_TOL);

            var segStart = b.StartPoint;
            var segEnd   = b.EndPoint;

            // 自由端をスラブ境界まで延長
            if (!startOk)
                segStart = ExtendToSlabBoundary(b.StartPoint, b.EndPoint, slab);
            if (!endOk)
                segEnd = ExtendToSlabBoundary(b.EndPoint, b.StartPoint, slab);

            segs.Add((segStart, segEnd));
        }


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
                // スラブ境界そのもの（全体）は除外
                if (sa > slab.Area * 0.9) continue;
                var ctr = new Point2d(poly.Average(p => p.X), poly.Average(p => p.Y));
                if (slab.Contains(ctr))
                    faces.Add(poly);
            }
        }
        return faces;
    }

    // ─── 四角形の亀甲分割（LISP準拠）──────────────────────

    /// <summary>パネル角が梁/柱のジャンクションか判定（接続されていれば二等分線を引く）</summary>
    private bool IsCornerAtBeamJunction(Point2d pt, List<BeamModel> beamsInSlab, SlabModel slab, double tol = 0.15)
    {
        // スラブ境界上にある角は基本的に二等分線を引かない
        bool onSlabBoundary = false;
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
        {
            if (PtSegDist(pt, sv[i], sv[(i + 1) % sv.Count]) < tol)
            { onSlabBoundary = true; break; }
        }

        // 柱に近いか
        bool nearColumn = _columns.Any(c =>
            Dist(pt, new Point2d(c.Center.X, c.Center.Y)) < tol + c.Radius);

        // スラブ境界上 → 柱がある場合のみ二等分線
        if (onSlabBoundary)
            return nearColumn;

        // スラブ内部 → 2本以上の梁が接触、または梁+柱
        int beamCount = 0;
        foreach (var beam in beamsInSlab)
        {
            if (Dist(pt, beam.StartPoint) < tol || Dist(pt, beam.EndPoint) < tol ||
                PtSegDist(pt, beam.StartPoint, beam.EndPoint) < tol)
                beamCount++;
        }
        return beamCount >= 2 || (beamCount >= 1 && nearColumn);
    }

    private void PartitionQuad(List<Point2d> panel, SlabModel slab, List<BeamModel> beamsInSlab)
    {
        var sorted = panel.OrderByDescending(p => p.Y).ThenBy(p => p.X).ToList();
        var tl = sorted[0]; var tr = sorted[1]; var bl = sorted[2]; var br = sorted[3];

        // 梁/柱に接続されている角を判定
        bool tlJunction = IsCornerAtBeamJunction(tl, beamsInSlab, slab);
        bool trJunction = IsCornerAtBeamJunction(tr, beamsInSlab, slab);
        bool blJunction = IsCornerAtBeamJunction(bl, beamsInSlab, slab);
        bool brJunction = IsCornerAtBeamJunction(br, beamsInSlab, slab);

        // 接続角の二等分線を収集
        var junctionCorners = new List<(Point2d pt, double bisAng)>();
        if (tlJunction) junctionCorners.Add((tl, BisAngle(tl, tr, bl)));
        if (trJunction) junctionCorners.Add((tr, BisAngle(tr, tl, br)));
        if (blJunction) junctionCorners.Add((bl, BisAngle(bl, tl, br)));
        if (brJunction) junctionCorners.Add((br, BisAngle(br, bl, tr)));

        // 全角が接続 → 通常の亀甲分割
        if (junctionCorners.Count == 4)
        {
            PartitionQuadFull(tl, tr, bl, br, slab);
            return;
        }

        if (junctionCorners.Count >= 2)
        {
            double ext = Math.Max(Math.Abs(tr.X - tl.X), Math.Abs(tl.Y - bl.Y)) * 2;
            var c0 = junctionCorners[0];
            var c1 = junctionCorners[1];
            var e0 = Polar(c0.pt, c0.bisAng, ext);
            var e1 = Polar(c1.pt, c1.bisAng, ext);
            var sk = LineInter(c0.pt, e0, c1.pt, e1);
            if (!sk.HasValue)
            {
                PartitionQuadFull(tl, tr, bl, br, slab);
                return;
            }
            var s = sk.Value;
            AssignClipped(new() { tl, tr, s }, tl, tr, slab);
            AssignClipped(new() { bl, br, s }, bl, br, slab);
            AssignClipped(new() { tl, bl, s }, tl, bl, slab);
            AssignClipped(new() { tr, br, s }, tr, br, slab);
        }
        else if (junctionCorners.Count == 1)
        {
            var c = junctionCorners[0].pt;
            AssignClipped(new() { tl, tr, c }, tl, tr, slab);
            AssignClipped(new() { bl, br, c }, bl, br, slab);
            AssignClipped(new() { tl, bl, c }, tl, bl, slab);
            AssignClipped(new() { tr, br, c }, tr, br, slab);
        }
        else
        {
            PartitionQuadFull(tl, tr, bl, br, slab);
        }
    }

    /// <summary>通常の亀甲分割（全角が梁/柱接続）</summary>
    private void PartitionQuadFull(Point2d tl, Point2d tr, Point2d bl, Point2d br, SlabModel slab)
    {
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
        AssignClipped(rT, tl, tr, slab); AssignClipped(rB, bl, br, slab);
        AssignClipped(rL, tl, bl, slab); AssignClipped(rR, tr, br, slab);
    }

    /// <summary>分割ポリゴンをスラブ範囲内にクリップしてからAssign</summary>
    private void AssignClipped(List<Point2d> region, Point2d eA, Point2d eB, SlabModel slab)
    {
        // スラブ境界の各辺でクリップ（スラブ内に収める）
        var clipped = new List<Point2d>(region);
        var sv = slab.Vertices;
        var center = new Point2d(sv.Average(p => p.X), sv.Average(p => p.Y));
        for (int i = 0; i < sv.Count && clipped.Count >= 3; i++)
            clipped = PolygonUtils.ClipByHalfPlane(clipped, sv[i], sv[(i + 1) % sv.Count], center);
        Assign(clipped, eA, eB);
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

        // 辺上の全梁を取得
        var beams = FindAllBeamsOnEdge(eA, eB);
        if (beams.Count <= 1)
        {
            var beam = beams.Count == 1 ? beams[0] : FindBeam(eA, eB);
            if (beam != null) { beam.TributaryArea += area; beam.TributaryPolygons.Add(region); }
            return;
        }

        // 複数梁：辺方向でソート
        double edx = eB.X - eA.X, edy = eB.Y - eA.Y;
        double eLen = Math.Sqrt(edx * edx + edy * edy);
        if (eLen < 1e-9) return;
        double enx = edx / eLen, eny = edy / eLen;
        beams.Sort((a, b) =>
        {
            double ta = (a.MidPoint.X - eA.X) * enx + (a.MidPoint.Y - eA.Y) * eny;
            double tb = (b.MidPoint.X - eA.X) * enx + (b.MidPoint.Y - eA.Y) * eny;
            return ta.CompareTo(tb);
        });

        // 梁の接合点で分割
        var splitPts = new List<Point2d>();
        for (int i = 0; i < beams.Count - 1; i++)
        {
            // 接合点＝beam[i]の端点とbeam[i+1]の端点の中間
            Point2d bestPt = beams[i].EndPoint;
            double bestD = double.MaxValue;
            foreach (var ep in new[] { beams[i].StartPoint, beams[i].EndPoint })
                foreach (var sp in new[] { beams[i + 1].StartPoint, beams[i + 1].EndPoint })
                {
                    double d = Dist(ep, sp);
                    if (d < bestD) { bestD = d; bestPt = new Point2d((ep.X + sp.X) / 2, (ep.Y + sp.Y) / 2); }
                }
            double t = ((bestPt.X - eA.X) * enx + (bestPt.Y - eA.Y) * eny);
            if (t > eLen * 0.02 && t < eLen * 0.98)
                splitPts.Add(new Point2d(eA.X + t * enx, eA.Y + t * eny));
        }

        if (splitPts.Count == 0)
        {
            beams[0].TributaryArea += area;
            beams[0].TributaryPolygons.Add(region);
            return;
        }

        // 辺に垂直な線で分割
        double px = -eny, py = enx;
        var remaining = new List<Point2d>(region);
        for (int i = 0; i < splitPts.Count && remaining.Count >= 3; i++)
        {
            var sp = splitPts[i];
            var cutB = new Point2d(sp.X + px, sp.Y + py);
            var piece = PolygonUtils.ClipByHalfPlane(new List<Point2d>(remaining), sp, cutB, eA);
            if (piece.Count >= 3)
            {
                double pa = PolygonUtils.Area(piece);
                if (pa > 1e-6) { beams[i].TributaryArea += pa; beams[i].TributaryPolygons.Add(piece); }
            }
            remaining = PolygonUtils.ClipByHalfPlane(remaining, sp, cutB, eB);
            eA = sp;
        }
        if (remaining.Count >= 3)
        {
            double ra = PolygonUtils.Area(remaining);
            if (ra > 1e-6) { beams[^1].TributaryArea += ra; beams[^1].TributaryPolygons.Add(remaining); }
        }
    }

    /// <summary>辺eA→eB上にある全ての梁を返す</summary>
    private List<BeamModel> FindAllBeamsOnEdge(Point2d eA, Point2d eB)
    {
        double eLen = Dist(eA, eB); if (eLen < 1e-9) return new();
        var eDir = new Vector2d((eB.X - eA.X) / eLen, (eB.Y - eA.Y) / eLen);
        double tol = Math.Max(eLen * 0.2, 0.15);
        var result = new List<BeamModel>();
        foreach (var b in _beams)
        {
            if (Math.Abs(eDir.X * b.Direction.X + eDir.Y * b.Direction.Y) < 0.8) continue;
            double ld = PtLineDist(b.MidPoint, eA, eB); if (ld > tol) continue;
            double t = ((b.MidPoint.X - eA.X) * eDir.X + (b.MidPoint.Y - eA.Y) * eDir.Y) / eLen;
            if (t > -0.1 && t < 1.1) result.Add(b);
        }
        return result;
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

    /// <summary>点がスラブ境界上にあるか判定（50mm許容）</summary>
    private static bool IsOnSlabBoundary(SlabModel slab, Point2d pt)
    {
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
            if (PtSegDist(pt, sv[i], sv[(i + 1) % sv.Count]) < 0.05) return true;
        return false;
    }

    /// <summary>点からdir方向へレイを飛ばし、スラブ境界との最初の交点を返す</summary>
    private static Point2d? RaySlabIntersect(SlabModel slab, Point2d origin, Vector2d dir)
    {
        var sv = slab.Vertices;
        Point2d? closest = null; double closestD = double.MaxValue;
        for (int i = 0; i < sv.Count; i++)
        {
            int j = (i + 1) % sv.Count;
            double d2x = sv[j].X - sv[i].X, d2y = sv[j].Y - sv[i].Y;
            double den = dir.X * d2y - dir.Y * d2x;
            if (Math.Abs(den) < 1e-10) continue;
            double dx = sv[i].X - origin.X, dy = sv[i].Y - origin.Y;
            double t = (dx * d2y - dy * d2x) / den;
            double s = (dx * dir.Y - dy * dir.X) / den;
            if (t < 1e-6 || s < -1e-8 || s > 1 + 1e-8) continue;
            double d = t;
            if (d < closestD) { closestD = d; closest = new Point2d(origin.X + t * dir.X, origin.Y + t * dir.Y); }
        }
        return closest;
    }

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
