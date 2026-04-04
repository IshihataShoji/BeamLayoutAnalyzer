using Autodesk.AutoCAD.Geometry;

namespace BeamLayoutAnalyzer.Analysis;

/// <summary>
/// 2D幾何計算ユーティリティ
/// </summary>
public static class GeometryUtils
{
    /// <summary>
    /// レイと線分の交点を求める。
    /// </summary>
    /// <param name="origin">レイの起点</param>
    /// <param name="rayDir">レイの方向（単位ベクトル）</param>
    /// <param name="segA">線分の始点</param>
    /// <param name="segB">線分の終点</param>
    /// <returns>交点までの距離 t（交点なし の場合 null）</returns>
    public static double? RaySegmentIntersect(
        Point2d origin, Vector2d rayDir,
        Point2d segA,   Point2d segB)
    {
        double sdx = segB.X - segA.X;
        double sdy = segB.Y - segA.Y;

        // denom = cross(rayDir, segDir)
        double denom = rayDir.X * sdy - rayDir.Y * sdx;
        if (Math.Abs(denom) < 1e-10) return null; // 平行

        double ax = segA.X - origin.X;
        double ay = segA.Y - origin.Y;

        double t = (ax * sdy - ay * sdx) / denom;
        double s = (ax * rayDir.Y - ay * rayDir.X) / denom;

        const double eps = 1e-6;
        if (t > eps && s >= -eps && s <= 1.0 + eps)
            return t;

        return null;
    }

    /// <summary>
    /// レイとポリゴン境界（全辺）の最近交点を求める。
    /// </summary>
    public static double? RayPolygonIntersect(
        Point2d origin, Vector2d rayDir,
        IList<Point2d> vertices)
    {
        double? minT = null;
        int n = vertices.Count;
        for (int i = 0; i < n; i++)
        {
            var t = RaySegmentIntersect(
                origin, rayDir,
                vertices[i], vertices[(i + 1) % n]);

            if (t.HasValue && (minT is null || t.Value < minT.Value))
                minT = t;
        }
        return minT;
    }
}
