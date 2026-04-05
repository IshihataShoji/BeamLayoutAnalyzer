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

            // ── Step 1: ベース線分を収集（スラブ境界 + 梁 + 浮き梁延長）──
            var baseSegs = CollectBaseSegments(slab, beamsInSlab);

            // ── Step 2: 全節点を収集し属性を判定 ──
            // ── Step 3: 属性に基づき二等分線を生成 ──
            var bisectors = GenerateBisectors(baseSegs, slab, beamsInSlab);

            // ── Step 3.5: スキューネスポイント接続 ──
            var basePanels = EnumerateFaces(baseSegs, slab);
            ConnectSkewnessPoints(bisectors, baseSegs, basePanels);

            // ── Step 4: 全線分で面列挙 → サブパネル検出 ──
            var allSegs = new List<(Point2d a, Point2d b)>(baseSegs);
            allSegs.AddRange(bisectors);
            var subPanels = EnumerateFaces(allSegs, slab);

            // ── Step 5: パネルの辺を共有する梁に面積を割当 ──
            foreach (var panel in subPanels)
            {
                double area = PolygonUtils.Area(panel);
                if (area < 1e-4) continue;

                var beamsOnEdge = new HashSet<BeamModel>();
                for (int i = 0; i < panel.Count; i++)
                {
                    var eA = panel[i]; var eB = panel[(i + 1) % panel.Count];
                    var mid = new Point2d((eA.X + eB.X) / 2, (eA.Y + eB.Y) / 2);
                    foreach (var beam in beamsInSlab)
                    {
                        if (PtSegDist(mid, beam.StartPoint, beam.EndPoint) < 0.15)
                            beamsOnEdge.Add(beam);
                    }
                }

                if (beamsOnEdge.Count == 0) continue;
                double share = area / beamsOnEdge.Count;
                foreach (var beam in beamsOnEdge)
                {
                    beam.TributaryArea += share;
                    beam.TributaryPolygons.Add(panel);
                }
            }
        }
    }

    // ─── ベース線分の収集 ─────────────────────────────────────

    private List<(Point2d a, Point2d b)> CollectBaseSegments(SlabModel slab, List<BeamModel> beamsInSlab)
    {
        var segs = new List<(Point2d a, Point2d b)>();
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
            segs.Add((sv[i], sv[(i + 1) % sv.Count]));

        const double CONN_TOL = 0.15;
        foreach (var b in beamsInSlab)
        {
            bool startOk = IsEndpointConnected(b.StartPoint, b, beamsInSlab, slab, CONN_TOL);
            bool endOk   = IsEndpointConnected(b.EndPoint, b, beamsInSlab, slab, CONN_TOL);
            var segStart = b.StartPoint;
            var segEnd   = b.EndPoint;
            if (!startOk) segStart = ExtendToSlabBoundary(b.StartPoint, b.EndPoint, slab);
            if (!endOk)   segEnd   = ExtendToSlabBoundary(b.EndPoint, b.StartPoint, slab);
            segs.Add((segStart, segEnd));
        }
        return segs;
    }

    // ─── 二等分線の生成 ──────────────────────────────────────

    private List<(Point2d a, Point2d b)> GenerateBisectors(
        List<(Point2d a, Point2d b)> segs,
        SlabModel slab,
        List<BeamModel> beamsInSlab)
    {
        const double TOL = 0.15;
        var bisectors = new List<(Point2d a, Point2d b)>();

        // ── 全節点を収集（線分端点＝スラブ角・柱・梁端点）──
        var ptMap = new Dictionary<long, Point2d>();
        long ptKey(Point2d p) => ((long)Math.Round(p.X * 10000)) * 100000000L + (long)Math.Round(p.Y * 10000);
        void addNode(Point2d p) { var k = ptKey(p); if (!ptMap.ContainsKey(k)) ptMap[k] = p; }

        foreach (var seg in segs)
        {
            addNode(seg.a);
            addNode(seg.b);
        }

        var nodes = ptMap.Values.ToList();

        // ── Phase 1: 各節点の各線分に対してペアを作り二等分線レイを収集 ──
        var rays = new List<(Point2d origin, double angle)>();

        // デバッグログ（コマンドラインのみ）
        var ed = Autodesk.AutoCAD.ApplicationServices.Application.DocumentManager.MdiActiveDocument?.Editor;
        void logMsg(string s) { ed?.WriteMessage($"\n{s}"); }
        logMsg($"[Bisector] Nodes:{nodes.Count} Segs:{segs.Count} Beams:{beamsInSlab.Count}");

        foreach (var node in nodes)
        {
            // 条件1: 柱の節点か
            bool isColumn = _columns.Any(c =>
                Dist(node, new Point2d(c.Center.X, c.Center.Y)) < TOL + c.Radius);

            // 条件2: 梁の端点が2つ以上重なっているか
            int beamEndpointCount = 0;
            foreach (var beam in beamsInSlab)
            {
                if (Dist(node, beam.StartPoint) < TOL) beamEndpointCount++;
                if (Dist(node, beam.EndPoint)   < TOL) beamEndpointCount++;
            }

            // 条件3: 梁の端点を1つ以上共有し、かつ、いずれかの梁の線分座標上にある
            bool endpointOnBeamLine = false;
            if (beamEndpointCount >= 1)
            {
                foreach (var beam in beamsInSlab)
                {
                    bool atEndpoint = Dist(node, beam.StartPoint) < TOL || Dist(node, beam.EndPoint) < TOL;
                    if (!atEndpoint && PtSegDist(node, beam.StartPoint, beam.EndPoint) < TOL)
                    { endpointOnBeamLine = true; break; }
                }
            }

            // ルール: 柱→引く、梁端点2以上→引く、梁端点共有+梁線分上→引く、それ以外→引かない
            if (!isColumn && beamEndpointCount < 2 && !endpointOnBeamLine)
            {
                // logMsg($"[SKIP] ({node.X:F3},{node.Y:F3}) col={isColumn} ep={beamEndpointCount} onLine={endpointOnBeamLine}");
                continue;
            }
            // logMsg($"[GEN]  ({node.X:F3},{node.Y:F3}) col={isColumn} ep={beamEndpointCount} onLine={endpointOnBeamLine}");

            // この節点を端点としている線分を全て走査し角度を収集（0～2π）
            var edgeAngles = new List<double>();
            int segIdx = 0;
            foreach (var seg in segs)
            {
                double distA = Dist(node, seg.a);
                double distB = Dist(node, seg.b);
                double distSeg = PtSegDist(node, seg.a, seg.b);

                double dx, dy;
                if (distA < TOL)
                {
                    dx = seg.b.X - node.X; dy = seg.b.Y - node.Y;
                    if (Math.Sqrt(dx * dx + dy * dy) > 1e-6)
                    {
                        double ang = NormalizeAngle(Math.Atan2(dy, dx));
                        edgeAngles.Add(ang);
                        // logMsg($"  seg{segIdx}: EP-A dist={distA:F3} -> {ang * 180 / Math.PI:F1} deg");
                    }
                }
                else if (distB < TOL)
                {
                    dx = seg.a.X - node.X; dy = seg.a.Y - node.Y;
                    if (Math.Sqrt(dx * dx + dy * dy) > 1e-6)
                    {
                        double ang = NormalizeAngle(Math.Atan2(dy, dx));
                        edgeAngles.Add(ang);
                        // logMsg($"  seg{segIdx}: EP-B dist={distB:F3} -> {ang * 180 / Math.PI:F1} deg");
                    }
                }
                else if (distSeg < TOL)
                {
                    // T字接合: ノードが線分の途中にある → 両方向を追加
                    double dx1 = seg.a.X - node.X, dy1 = seg.a.Y - node.Y;
                    double dx2 = seg.b.X - node.X, dy2 = seg.b.Y - node.Y;
                    if (Math.Sqrt(dx1 * dx1 + dy1 * dy1) > 1e-6)
                        edgeAngles.Add(NormalizeAngle(Math.Atan2(dy1, dx1)));
                    if (Math.Sqrt(dx2 * dx2 + dy2 * dy2) > 1e-6)
                        edgeAngles.Add(NormalizeAngle(Math.Atan2(dy2, dx2)));
                    // logMsg($"  seg{segIdx}: T-JCT distSeg={distSeg:F3}");
                }
                else
                {
                    // logMsg($"  seg{segIdx}: MISS dA={distA:F3} dB={distB:F3} dSeg={distSeg:F3}");
                }
                segIdx++;
            }

            if (edgeAngles.Count < 2)
            {
                // logMsg($"  => edges={edgeAngles.Count} SKIP(too few)");
                continue;
            }

            // 重複角度を除去してソート（0～2πの範囲で昇順）
            edgeAngles.Sort();
            var unique = new List<double> { edgeAngles[0] };
            for (int i = 1; i < edgeAngles.Count; i++)
                if (edgeAngles[i] - unique[^1] > 0.02) unique.Add(edgeAngles[i]);
            // ラップアラウンド: 末尾(≈360°)と先頭(≈0°)が同方向なら末尾を除去
            if (unique.Count >= 2 && (2 * Math.PI - unique[^1] + unique[0]) < 0.02)
                unique.RemoveAt(unique.Count - 1);
            edgeAngles = unique;
            if (edgeAngles.Count < 2)
            {
                // logMsg($"  => unique={edgeAngles.Count} SKIP(after dedup)");
                continue;
            }
            // logMsg($"  => edges={edgeAngles.Count} angles=[{string.Join(", ", edgeAngles.Select(a => $"{a * 180 / Math.PI:F1}"))}]");

            int N = edgeAngles.Count;

            // 重複統合用: この節点で生成済みの二等分線角度
            var generatedAngles = new HashSet<int>();

            // ── 各線分に対して処理（右回り360°評価）──
            for (int si = 0; si < N; si++)
            {
                double sAngle = edgeAngles[si];

                // ① 他の全線分への右回り(CW)角度を計算
                double minCW = double.MaxValue;
                int minIdx = -1;
                for (int oi = 0; oi < N; oi++)
                {
                    if (oi == si) continue;
                    double cw = NormalizeAngle(sAngle - edgeAngles[oi]);
                    if (cw > 0.02 && cw < minCW)
                    {
                        minCW = cw;
                        minIdx = oi;
                    }
                }

                if (minIdx < 0) continue;

                // ② 最小角度のペアに対してのみ二等分線を生成
                double bisAngle = NormalizeAngle(sAngle - minCW / 2);
                int angleKey = (int)Math.Round(bisAngle * 1000);
                if (generatedAngles.Add(angleKey))
                {
                    rays.Add((node, bisAngle));
                    // logMsg($"  bisector: S={sAngle * 180 / Math.PI:F1} CW={minCW * 180 / Math.PI:F1} -> bis={bisAngle * 180 / Math.PI:F1}");
                }
            }
        }

        // ── Phase 2: 分割線の終点を決定 ──
        //  Step A: 全レイをスラブ境界まで延伸 → 長い線分を生成
        //  Step B: 全線分同士の交点を計算
        //  Step C: 各線分の終点 = 始点（節点）から最も近い交点

        // スラブ境界辺
        var slabEdges = new List<(Point2d a, Point2d b)>();
        var sv2 = slab.Vertices;
        for (int i = 0; i < sv2.Count; i++)
            slabEdges.Add((sv2[i], sv2[(i + 1) % sv2.Count]));

        // Step A: 全レイをスラブ境界まで延伸
        var fullLines = new List<(Point2d origin, Point2d slabEnd)>();
        foreach (var (origin, angle) in rays)
        {
            var dir = new Vector2d(Math.Cos(angle), Math.Sin(angle));
            double bestT = double.MaxValue;
            foreach (var seg in slabEdges)
            {
                double t = RaySegIntersect(origin, dir, seg.a, seg.b);
                if (t > 1e-3 && t < bestT) bestT = t;
            }
            if (bestT < double.MaxValue)
            {
                var slabEnd = new Point2d(origin.X + bestT * dir.X, origin.Y + bestT * dir.Y);
                fullLines.Add((origin, slabEnd));
            }
        }

        logMsg($"[Bisector] FullLines:{fullLines.Count}");

        // Step B & C: 反復トリミング
        //  1パス目: fullLine同士の交差で各線分をトリミング
        //  2パス目以降: トリミング後の線分同士で再交差チェックし、
        //             相手が実際に到達しない交差を除去（収束するまで繰り返し）

        // 現在の終点リスト（初期値 = スラブ境界）
        var currentEnds = fullLines.Select(f => f.slabEnd).ToList();
        var beamSegs = segs.Skip(sv2.Count).ToList();

        for (int pass = 0; pass < 5; pass++) // 最大5パス
        {
            bool changed = false;
            var newEnds = new List<Point2d>();

            for (int li = 0; li < fullLines.Count; li++)
            {
                var origin = fullLines[li].origin;
                var slabEnd = fullLines[li].slabEnd;
                double bestDist = Dist(origin, slabEnd); // スラブ境界までの距離
                Point2d bestEnd = slabEnd;

                // 梁線分との交差もチェック（起点に接続している梁はスキップ）
                foreach (var bseg in beamSegs)
                {
                    if (Dist(origin, bseg.a) < TOL || Dist(origin, bseg.b) < TOL) continue;
                    if (PtSegDist(origin, bseg.a, bseg.b) < TOL) continue;
                    var cross = SegSegIntersect(origin, slabEnd, bseg.a, bseg.b);
                    if (cross.HasValue)
                    {
                        double d = Dist(origin, cross.Value);
                        if (d > TOL && d < bestDist)
                        {
                            bestDist = d;
                            bestEnd = cross.Value;
                        }
                    }
                }

                // 他の分割線との交差
                for (int oi = 0; oi < fullLines.Count; oi++)
                {
                    if (oi == li) continue;
                    var otherOrigin = fullLines[oi].origin;

                    // 同一ノードからの別方向線分はスキップ
                    if (Dist(origin, otherOrigin) < TOL) continue;

                    // 相手の現在の有効区間（origin → currentEnd）との交差をチェック
                    var otherEnd = currentEnds[oi];
                    var cross = SegSegIntersect(origin, slabEnd, otherOrigin, otherEnd);
                    if (cross.HasValue)
                    {
                        double d = Dist(origin, cross.Value);
                        if (d > TOL && d < bestDist)
                        {
                            bestDist = d;
                            bestEnd = cross.Value;
                        }
                    }
                }

                newEnds.Add(bestEnd);
                if (Dist(bestEnd, currentEnds[li]) > 0.001) changed = true;
            }

            currentEnds = newEnds;
            logMsg($"[Bisector] Pass {pass}: changed={changed}");
            if (!changed) break;
        }

        // 結果をbisectorsに格納
        for (int li = 0; li < fullLines.Count; li++)
        {
            var origin = fullLines[li].origin;
            var end = currentEnds[li];
            // logMsg($"  BIS ({origin.X:F3},{origin.Y:F3})->({end.X:F3},{end.Y:F3}) D={Dist(origin, end):F3}");
            bisectors.Add((origin, end));
        }

        // ── 座標が完全に重複する分割線を統合 ──
        var seen = new HashSet<long>();
        var uniqueBisectors = new List<(Point2d a, Point2d b)>();
        foreach (var bis in bisectors)
        {
            long k1 = ((long)Math.Round(bis.a.X * 1000)) * 10000000L + (long)Math.Round(bis.a.Y * 1000);
            long k2 = ((long)Math.Round(bis.b.X * 1000)) * 10000000L + (long)Math.Round(bis.b.Y * 1000);
            long keyAB = k1 * 100000000000L + k2;
            long keyBA = k2 * 100000000000L + k1;
            if (seen.Add(keyAB))
            {
                seen.Add(keyBA);
                uniqueBisectors.Add(bis);
            }
        }

        logMsg($"[Bisector] Rays:{rays.Count} Bisectors:{uniqueBisectors.Count}");
        return uniqueBisectors;
    }

    /// <summary>レイ(origin + t*dir)と線分(a,b)の交点パラメータtを返す</summary>
    private static double RaySegIntersect(Point2d origin, Vector2d dir, Point2d a, Point2d b)
    {
        double ex = b.X - a.X, ey = b.Y - a.Y;
        double den = dir.X * ey - dir.Y * ex;
        if (Math.Abs(den) < 1e-10) return double.MaxValue;
        double dx = a.X - origin.X, dy = a.Y - origin.Y;
        double t = (dx * ey - dy * ex) / den;
        double s = (dx * dir.Y - dy * dir.X) / den;
        return (t > 0 && s >= -1e-8 && s <= 1 + 1e-8) ? t : double.MaxValue;
    }

    /// <summary>2本のレイの交点パラメータt（1本目のレイ上のパラメータ）を返す</summary>
    private static double RayRayIntersect(Point2d o1, Vector2d d1, Point2d o2, Vector2d d2)
    {
        double den = d1.X * d2.Y - d1.Y * d2.X;
        if (Math.Abs(den) < 1e-10) return double.MaxValue;
        double dx = o2.X - o1.X, dy = o2.Y - o1.Y;
        double t = (dx * d2.Y - dy * d2.X) / den;
        double s = (dx * d1.Y - dy * d1.X) / den;
        return (t > 0 && s > 0) ? t : double.MaxValue;
    }

    // ─── スキューネスポイントの接続 ──────────────────────────

    private void ConnectSkewnessPoints(
        List<(Point2d a, Point2d b)> bisectors,
        List<(Point2d a, Point2d b)> baseSegs,
        List<List<Point2d>> basePanels)
    {
        const double TOL = 0.15;
        var freeEnds = new List<(Point2d pt, int panelIdx)>();

        for (int bi = 0; bi < bisectors.Count; bi++)
        {
            foreach (var ep in new[] { bisectors[bi].a, bisectors[bi].b })
            {
                bool onBase = baseSegs.Any(s => PtSegDist(ep, s.a, s.b) < TOL);
                if (!onBase)
                {
                    for (int pi = 0; pi < basePanels.Count; pi++)
                    {
                        if (PointInPolygon(ep, basePanels[pi]))
                        { freeEnds.Add((ep, pi)); break; }
                    }
                }
            }
        }

        var byPanel = freeEnds.GroupBy(f => f.panelIdx);
        foreach (var g in byPanel)
        {
            // 座標の重複除去
            var rawPts = g.Select(f => f.pt).ToList();
            var pts = new List<Point2d> { rawPts[0] };
            for (int i = 1; i < rawPts.Count; i++)
            {
                if (!pts.Any(p => Dist(p, rawPts[i]) < TOL))
                    pts.Add(rawPts[i]);
            }

            // 2点以上の場合、最近傍チェーンで接続
            if (pts.Count >= 2)
            {
                var remaining = new List<Point2d>(pts);
                var current = remaining[0];
                remaining.RemoveAt(0);
                while (remaining.Count > 0)
                {
                    int nearIdx = 0;
                    double nearD = double.MaxValue;
                    for (int i = 0; i < remaining.Count; i++)
                    {
                        double d = Dist(current, remaining[i]);
                        if (d < nearD) { nearD = d; nearIdx = i; }
                    }
                    bisectors.Add((current, remaining[nearIdx]));
                    current = remaining[nearIdx];
                    remaining.RemoveAt(nearIdx);
                }
            }
        }
    }

    // ─── 平面グラフの面列挙 ─────────────────────────────────

    private List<List<Point2d>> EnumerateFaces(
        List<(Point2d a, Point2d b)> segs, SlabModel slab)
    {
        // 1. 全線分の交点を求め、交点で線分を分割
        var pts = new List<Point2d>();
        var ptMapF = new Dictionary<long, int>();
        int addPt(Point2d p)
        {
            long key = ((long)Math.Round(p.X * 10000)) * 100000000L + (long)Math.Round(p.Y * 10000);
            if (ptMapF.TryGetValue(key, out int idx)) return idx;
            idx = pts.Count; pts.Add(p); ptMapF[key] = idx; return idx;
        }

        int nSeg = segs.Count;
        var segPts = new List<List<(double t, int idx)>>();
        for (int i = 0; i < nSeg; i++)
            segPts.Add(new() { (0.0, addPt(segs[i].a)), (1.0, addPt(segs[i].b)) });

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

        // 2. 辺リストを構築
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

        // 3. 隣接リストを角度順にソート
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

        // 4. 面を列挙（各有向辺の左側の面をトレース）
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
                int idx = adj[cv].IndexOf(cu);
                if (idx < 0) { ok = false; break; }
                int w = adj[cv][(idx - 1 + adj[cv].Count) % adj[cv].Count];
                cu = cv; cv = w;
            }
            if (!ok || face.Count < 3) continue;
            var poly = face.Select(i => pts[i]).ToList();
            double sa = SignedArea(poly);
            if (sa > 1e-4 && sa < slab.Area * 0.9)
            {
                var ctr = new Point2d(poly.Average(p => p.X), poly.Average(p => p.Y));
                if (slab.Contains(ctr))
                    faces.Add(poly);
            }
        }
        return faces;
    }

    // ─── ヘルパー ────────────────────────────────────────────

    /// <summary>梁の端点が他の要素に接続しているか判定</summary>
    private bool IsEndpointConnected(
        Point2d ep, BeamModel self, List<BeamModel> beams, SlabModel slab, double tol)
    {
        foreach (var other in beams)
        {
            if (other == self) continue;
            if (Dist(ep, other.StartPoint) < tol || Dist(ep, other.EndPoint) < tol)
                return true;
        }
        foreach (var other in beams)
        {
            if (other == self) continue;
            if (PtSegDist(ep, other.StartPoint, other.EndPoint) < tol)
                return true;
        }
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
        {
            if (PtSegDist(ep, sv[i], sv[(i + 1) % sv.Count]) < tol)
                return true;
        }
        foreach (var col in _columns)
        {
            if (Dist(ep, new Point2d(col.Center.X, col.Center.Y)) < tol + col.Radius)
                return true;
        }
        return false;
    }

    /// <summary>自由端を梁方向にスラブ境界まで延長した点を返す</summary>
    private static Point2d ExtendToSlabBoundary(Point2d freeEnd, Point2d connectedEnd, SlabModel slab)
    {
        var dir = new Vector2d(freeEnd.X - connectedEnd.X, freeEnd.Y - connectedEnd.Y);
        double len = Math.Sqrt(dir.X * dir.X + dir.Y * dir.Y);
        if (len < 1e-10) return freeEnd;
        var d = new Vector2d(dir.X / len, dir.Y / len);
        var sv = slab.Vertices;
        double bestT = double.MaxValue;
        Point2d bestPt = freeEnd;
        for (int i = 0; i < sv.Count; i++)
        {
            var p = sv[i]; var q = sv[(i + 1) % sv.Count];
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

    /// <summary>角度を [0, 2π) に正規化</summary>
    private static double NormalizeAngle(double a)
    {
        while (a < 0) a += 2 * Math.PI;
        while (a >= 2 * Math.PI) a -= 2 * Math.PI;
        return a;
    }

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

    private static bool IsOnSlabBoundary(SlabModel slab, Point2d pt)
    {
        var sv = slab.Vertices;
        for (int i = 0; i < sv.Count; i++)
            if (PtSegDist(pt, sv[i], sv[(i + 1) % sv.Count]) < 0.20) return true;
        return false;
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
