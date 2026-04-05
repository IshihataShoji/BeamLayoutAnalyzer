using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 各部材の負担面積を算出する。
///
/// 梁：亀甲分割（Straight Skeleton）
///   1. スラブ内部の梁ラインでスラブをサブパネルに分割
///   2. 各サブパネルに対してStraight Skeleton（直線骨格）を計算
///   3. 各辺の負担領域を求め、対応する梁に面積を割り当てる
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

            // スラブ内部の梁でサブパネルに分割
            var interiorBeams = _beams.Where(b => slab.Contains(b.MidPoint)).ToList();
            var panels = SubdivideSlab(slab.Vertices, interiorBeams);

            // 各サブパネルで亀甲分割を実行
            foreach (var panel in panels)
            {
                if (panel.Count < 3 || PolygonUtils.Area(panel) < 1e-4) continue;
                ComputeStraightSkeleton(panel);
            }
        }
    }

    // ─── サブパネル分割 ───────────────────────────────────────

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
                bool hasL = false, hasR = false;
                foreach (var v in panel)
                {
                    double d = Cross(lineA, lineB, v);
                    if (d >  1e-6) hasL = true;
                    if (d < -1e-6) hasR = true;
                }
                if (hasL && hasR && BBoxOverlap(beam, panel))
                {
                    var h1 = PolygonUtils.ClipByHalfPlane(new List<Point2d>(panel), lineA, lineB, ptL);
                    var h2 = PolygonUtils.ClipByHalfPlane(new List<Point2d>(panel), lineA, lineB, ptR);
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
        double bMinX = Math.Min(beam.StartPoint.X, beam.EndPoint.X);
        double bMaxX = Math.Max(beam.StartPoint.X, beam.EndPoint.X);
        double bMinY = Math.Min(beam.StartPoint.Y, beam.EndPoint.Y);
        double bMaxY = Math.Max(beam.StartPoint.Y, beam.EndPoint.Y);
        return bMaxX >= pMinX - tol && bMinX <= pMaxX + tol &&
               bMaxY >= pMinY - tol && bMinY <= pMaxY + tol;
    }

    // ─── Straight Skeleton（直線骨格）亀甲分割 ───────────────

    /// <summary>
    /// サブパネル（凸多角形）に対してStraight Skeletonを計算し、
    /// 各辺の負担領域ポリゴンを構築して対応する梁に面積を割り当てる。
    ///
    /// アルゴリズム：
    /// 1. 各頂点で内角の2等分線を求める
    /// 2. 隣接する2等分線同士の交点を全て求める
    /// 3. 各頂点から最も近い交点を特定し、最近接交点が共通のペアを抽出
    /// 4. ペアの交点を新頂点として多角形を縮小
    /// 5. 辺が2本になるまで繰り返す
    /// </summary>
    private void ComputeStraightSkeleton(List<Point2d> panel)
    {
        int n = panel.Count;

        // 各辺に対する負担領域の頂点リスト
        // tributaryVertices[i] = 辺i (panel[i]→panel[(i+1)%n]) の支配ポリゴンの頂点
        var tributaryVertices = new List<List<Point2d>>();
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            tributaryVertices.Add(new List<Point2d> { panel[i], panel[j] });
        }

        // 作業用：現在の多角形頂点と対応する元の辺インデックス
        var currentVerts = new List<Point2d>(panel);
        var edgeIndices  = new List<int>();  // currentVerts[i]→currentVerts[i+1] が元の辺何番か
        for (int i = 0; i < n; i++) edgeIndices.Add(i);

        // 反復的にStraight Skeletonを構築
        int maxIter = n * 2;
        for (int iter = 0; iter < maxIter; iter++)
        {
            int cn = currentVerts.Count;
            if (cn < 3) break;

            // 各頂点の2等分線方向を計算
            var bisectors = new Vector2d[cn];
            for (int i = 0; i < cn; i++)
            {
                var prev = currentVerts[(i - 1 + cn) % cn];
                var curr = currentVerts[i];
                var next = currentVerts[(i + 1) % cn];
                bisectors[i] = CalcBisector(prev, curr, next);
            }

            // 隣接する2等分線の交点を求める
            var intersections = new Point2d?[cn];
            var distances     = new double[cn];
            for (int i = 0; i < cn; i++)
            {
                int j = (i + 1) % cn;
                var pt = RayRayIntersect(
                    currentVerts[i], bisectors[i],
                    currentVerts[j], bisectors[j]);

                if (pt.HasValue && IsInsidePolygon(pt.Value, panel))
                {
                    intersections[i] = pt;
                    distances[i] = Dist(currentVerts[i], pt.Value);
                }
                else
                {
                    intersections[i] = null;
                    distances[i] = double.MaxValue;
                }
            }

            // 各頂点から最も近い交点を探す
            // 頂点iに関連する交点は intersections[i-1] と intersections[i]
            int bestIdx = -1;
            double bestDist = double.MaxValue;
            for (int i = 0; i < cn; i++)
            {
                // 頂点iに隣接する2つの交点のうち近い方
                int prev = (i - 1 + cn) % cn;
                double dPrev = intersections[prev].HasValue ? distances[prev] : double.MaxValue;
                double dNext = intersections[i].HasValue ? distances[i] : double.MaxValue;
                double minD = Math.Min(dPrev, dNext);

                if (minD < bestDist)
                {
                    bestDist = minD;
                    bestIdx = dPrev <= dNext ? prev : i;
                }
            }

            if (bestIdx < 0 || !intersections[bestIdx].HasValue) break;

            var crossPt = intersections[bestIdx]!.Value;
            int idxI = bestIdx;
            int idxJ = (bestIdx + 1) % cn;

            // 負担領域ポリゴンに交点を追加
            int origEdgeI = edgeIndices[idxI];
            int origEdgeJ = edgeIndices[idxJ % edgeIndices.Count];
            tributaryVertices[origEdgeI].Add(crossPt);
            tributaryVertices[origEdgeJ].Insert(0, crossPt);

            // 頂点を統合：idxI と idxJ を crossPt に置換
            var newVerts = new List<Point2d>();
            var newEdges = new List<int>();
            for (int i = 0; i < cn; i++)
            {
                if (i == idxI)
                {
                    newVerts.Add(crossPt);
                    // この新頂点の辺 = 元の辺idxJの辺
                    newEdges.Add(edgeIndices[idxJ % edgeIndices.Count]);
                }
                else if (i == idxJ)
                {
                    // スキップ（idxIに統合済み）
                }
                else
                {
                    newVerts.Add(currentVerts[i]);
                    newEdges.Add(edgeIndices[i]);
                }
            }

            currentVerts = newVerts;
            edgeIndices  = newEdges;

            if (currentVerts.Count <= 2) break;
        }

        // 最後に残った2頂点がある場合、その交点も追加
        if (currentVerts.Count == 2 && edgeIndices.Count == 2)
        {
            // 残り2辺の接点 = 最終的な骨格の端点
            var midPt = new Point2d(
                (currentVerts[0].X + currentVerts[1].X) / 2.0,
                (currentVerts[0].Y + currentVerts[1].Y) / 2.0);
            tributaryVertices[edgeIndices[0]].Add(midPt);
            tributaryVertices[edgeIndices[1]].Insert(0, midPt);
        }

        // 負担領域ポリゴンを閉じて面積を計算、梁に割り当て
        for (int i = 0; i < n; i++)
        {
            var region = OrderPolygon(tributaryVertices[i]);
            if (region.Count < 3) continue;

            double area = PolygonUtils.Area(region);
            if (area < 1e-6) continue;

            var beam = FindMatchingBeam(panel[i], panel[(i + 1) % n]);
            if (beam != null)
            {
                beam.TributaryArea += area;
                beam.TributaryPolygons.Add(region);
            }
        }
    }

    // ─── ポリゴン頂点の順序整理 ──────────────────────────────

    /// <summary>
    /// ポリゴン頂点を重心周りの角度でソートして正しい順序にする
    /// </summary>
    private static List<Point2d> OrderPolygon(List<Point2d> pts)
    {
        if (pts.Count < 3) return pts;

        // 重複除去
        var unique = new List<Point2d> { pts[0] };
        for (int i = 1; i < pts.Count; i++)
        {
            if (Dist(pts[i], unique[^1]) > 1e-8)
                unique.Add(pts[i]);
        }
        if (unique.Count < 3) return unique;

        double cx = unique.Average(p => p.X);
        double cy = unique.Average(p => p.Y);
        unique.Sort((a, b) =>
        {
            double angA = Math.Atan2(a.Y - cy, a.X - cx);
            double angB = Math.Atan2(b.Y - cy, b.X - cx);
            return angA.CompareTo(angB);
        });
        return unique;
    }

    // ─── 2等分線計算 ─────────────────────────────────────────

    private static Vector2d CalcBisector(Point2d prev, Point2d curr, Point2d next)
    {
        var d1 = Normalize(prev.X - curr.X, prev.Y - curr.Y);
        var d2 = Normalize(next.X - curr.X, next.Y - curr.Y);
        double bx = d1.X + d2.X, by = d1.Y + d2.Y;
        double bLen = Math.Sqrt(bx * bx + by * by);
        if (bLen > 1e-10)
            return new Vector2d(bx / bLen, by / bLen);
        // 180°の場合は法線方向
        return new Vector2d(-d1.Y, d1.X);
    }

    // ─── レイ同士の交点 ──────────────────────────────────────

    private static Point2d? RayRayIntersect(
        Point2d o1, Vector2d d1, Point2d o2, Vector2d d2)
    {
        double denom = d1.X * d2.Y - d1.Y * d2.X;
        if (Math.Abs(denom) < 1e-10) return null; // 平行

        double dx = o2.X - o1.X, dy = o2.Y - o1.Y;
        double t = (dx * d2.Y - dy * d2.X) / denom;
        double s = (dx * d1.Y - dy * d1.X) / denom;

        // 両方のレイで正方向のみ
        if (t < -1e-6 || s < -1e-6) return null;

        return new Point2d(o1.X + t * d1.X, o1.Y + t * d1.Y);
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
            double dot = Math.Abs(edgeDir.X * beam.Direction.X + edgeDir.Y * beam.Direction.Y);
            if (dot < 0.8) continue;
            double lineDist = PointToLineDistance(edgeMid, beam.StartPoint, beam.EndPoint);
            if (lineDist > tolerance) continue;
            double segDist = PointToSegmentDistance(edgeMid, beam.StartPoint, beam.EndPoint);
            if (segDist < bestScore) { bestScore = segDist; best = beam; }
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
        double t = Math.Max(0, Math.Min(1, ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / len2));
        return Dist(p, new Point2d(a.X + t * dx, a.Y + t * dy));
    }

    private static bool IsInsidePolygon(Point2d pt, List<Point2d> polygon)
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
            var diff = new Vector2d(other.Center.X - target.Center.X, other.Center.Y - target.Center.Y);
            var perp = new Vector2d(-diff.Y, diff.X);
            cell = PolygonUtils.ClipByHalfPlane(cell, mid,
                new Point2d(mid.X + perp.X, mid.Y + perp.Y), target.Center);
        }
        target.VoronoiPolygon = cell;
        return PolygonUtils.Area(cell);
    }
}
