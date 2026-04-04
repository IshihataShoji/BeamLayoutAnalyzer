using Autodesk.AutoCAD.Geometry;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 2Dポリゴン操作ユーティリティ（ボロノイ計算用）
/// </summary>
public static class PolygonUtils
{
    /// <summary>ガウスの面積公式</summary>
    public static double Area(IList<Point2d> polygon)
    {
        if (polygon.Count < 3) return 0;
        double area = 0;
        int n = polygon.Count;
        for (int i = 0; i < n; i++)
        {
            var p0 = polygon[i];
            var p1 = polygon[(i + 1) % n];
            area += p0.X * p1.Y - p1.X * p0.Y;
        }
        return Math.Abs(area) / 2.0;
    }

    /// <summary>
    /// Sutherland-Hodgman アルゴリズムで半平面クリッピング。
    /// 有向線 A→B の左側（insidePt が含まれる側）にポリゴンをクリップする。
    /// </summary>
    public static List<Point2d> ClipByHalfPlane(
        List<Point2d> polygon, Point2d lineA, Point2d lineB, Point2d insidePt)
    {
        if (polygon.Count == 0) return polygon;

        // insidePt が右側にあれば線の向きを逆にする
        if (Cross(lineA, lineB, insidePt) < 0)
        {
            (lineA, lineB) = (lineB, lineA);
        }

        var output = new List<Point2d>();
        int n = polygon.Count;

        for (int i = 0; i < n; i++)
        {
            var curr = polygon[i];
            var next = polygon[(i + 1) % n];

            bool currIn = Cross(lineA, lineB, curr) >= -1e-10;
            bool nextIn = Cross(lineA, lineB, next) >= -1e-10;

            if (currIn)
            {
                output.Add(curr);
                if (!nextIn)
                {
                    var pt = LineIntersect(lineA, lineB, curr, next);
                    if (pt.HasValue) output.Add(pt.Value);
                }
            }
            else if (nextIn)
            {
                var pt = LineIntersect(lineA, lineB, curr, next);
                if (pt.HasValue) output.Add(pt.Value);
            }
        }

        return output;
    }

    // 符号付き外積（正=左側, 負=右側）
    private static double Cross(Point2d a, Point2d b, Point2d p)
        => (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);

    // 2直線の交点（無限直線として）
    private static Point2d? LineIntersect(Point2d a, Point2d b, Point2d c, Point2d d)
    {
        double a1 = b.Y - a.Y, b1 = a.X - b.X;
        double c1 = a1 * a.X + b1 * a.Y;
        double a2 = d.Y - c.Y, b2 = c.X - d.X;
        double c2 = a2 * c.X + b2 * c.Y;
        double det = a1 * b2 - a2 * b1;
        if (Math.Abs(det) < 1e-12) return null;
        return new Point2d((c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det);
    }
}
