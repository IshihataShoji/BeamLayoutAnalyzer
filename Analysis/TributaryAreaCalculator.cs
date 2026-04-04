using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 各部材の負担面積を算出する。
///
/// 梁：角度2等分線（アングルバイセクター）分割
///   スラブ各頂点の内角を2等分する線を引き、隣接頂点の2等分線同士の
///   交点でスラブを辺ごとの領域に分割する。各領域の面積を対応する梁に割り当てる。
///   矩形スラブでは従来の亀の甲分割と同一結果となり、
///   非矩形スラブでも正確な負担面積配分が可能。
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
            var vertices = slab.Vertices;
            int n = vertices.Count;
            if (n < 3) continue;

            // ── 各頂点の内角2等分線の方向ベクトルを求める ──
            var bisectors = new Vector2d[n];
            for (int i = 0; i < n; i++)
            {
                var prev = vertices[(i - 1 + n) % n];
                var curr = vertices[i];
                var next = vertices[(i + 1) % n];

                var d1 = Normalize(prev.X - curr.X, prev.Y - curr.Y);
                var d2 = Normalize(next.X - curr.X, next.Y - curr.Y);

                double bx = d1.X + d2.X;
                double by = d1.Y + d2.Y;
                double bLen = Math.Sqrt(bx * bx + by * by);

                bisectors[i] = bLen > 1e-10
                    ? new Vector2d(bx / bLen, by / bLen)
                    : new Vector2d(-d1.Y, d1.X); // 180°の場合は法線方向
            }

            // ── 各辺ごとに半平面クリッピングで支配領域を求める ──
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;

                // 辺の中点（クリッピングの「内側」判定用）
                var edgeMid = new Point2d(
                    (vertices[i].X + vertices[j].X) / 2.0,
                    (vertices[i].Y + vertices[j].Y) / 2.0);

                // スラブ全体から開始
                var region = new List<Point2d>(vertices);

                // 頂点 i の2等分線でクリップ
                var bisA = vertices[i];
                var bisB = new Point2d(
                    vertices[i].X + bisectors[i].X,
                    vertices[i].Y + bisectors[i].Y);
                region = PolygonUtils.ClipByHalfPlane(region, bisA, bisB, edgeMid);

                // 頂点 j の2等分線でクリップ
                var bisC = vertices[j];
                var bisD = new Point2d(
                    vertices[j].X + bisectors[j].X,
                    vertices[j].Y + bisectors[j].Y);
                region = PolygonUtils.ClipByHalfPlane(region, bisC, bisD, edgeMid);

                if (region.Count < 3) continue;

                double area = PolygonUtils.Area(region);

                // この辺に対応する梁を特定して面積を割り当て
                var beam = FindMatchingBeam(vertices[i], vertices[j]);
                if (beam != null)
                {
                    beam.TributaryArea += area;
                    beam.TributaryPolygons.Add(region);
                }
            }
        }
    }

    // ─── 辺に最も近い梁を特定する ──────────────────────────

    /// <summary>
    /// スラブの辺 (edgeStart→edgeEnd) に対応する梁を検索する。
    /// 方向が概ね一致し、距離が最も近い梁を返す。
    /// </summary>
    private BeamModel? FindMatchingBeam(Point2d edgeStart, Point2d edgeEnd)
    {
        double edgeLen = Dist(edgeStart, edgeEnd);
        if (edgeLen < 1e-9) return null;

        var edgeDir = Normalize(
            edgeEnd.X - edgeStart.X,
            edgeEnd.Y - edgeStart.Y);

        var edgeMid = new Point2d(
            (edgeStart.X + edgeEnd.X) / 2.0,
            (edgeStart.Y + edgeEnd.Y) / 2.0);

        // 許容距離（辺の長さの15%、最低0.1m）
        double tolerance = Math.Max(edgeLen * 0.15, 0.1);

        BeamModel? bestBeam = null;
        double bestDist = double.MaxValue;

        foreach (var beam in _beams)
        {
            // 方向の一致判定（内積の絶対値 > 0.85 ≒ 約30°以内）
            double dot = Math.Abs(
                edgeDir.X * beam.Direction.X + edgeDir.Y * beam.Direction.Y);
            if (dot < 0.85) continue;

            // 辺の中点から梁の無限直線までの距離
            double dist = PointToLineDistance(edgeMid, beam.StartPoint, beam.EndPoint);

            if (dist < tolerance && dist < bestDist)
            {
                bestDist = dist;
                bestBeam = beam;
            }
        }

        return bestBeam;
    }

    // ─── 幾何ヘルパー ────────────────────────────────────────

    private static Vector2d Normalize(double x, double y)
    {
        double len = Math.Sqrt(x * x + y * y);
        return len > 1e-10 ? new Vector2d(x / len, y / len) : new Vector2d(0, 0);
    }

    private static double Dist(Point2d a, Point2d b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    /// <summary>
    /// 点 P から直線 AB（無限直線として）までの垂直距離
    /// </summary>
    private static double PointToLineDistance(Point2d p, Point2d a, Point2d b)
    {
        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        double len = Math.Sqrt(dx * dx + dy * dy);
        if (len < 1e-10) return Dist(p, a);
        return Math.Abs(dx * (a.Y - p.Y) - dy * (a.X - p.X)) / len;
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
