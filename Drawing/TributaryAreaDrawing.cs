using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.Drawing;

public static class TributaryAreaDrawing
{
    private const double MToMm = 1000.0;
    private const double DimOffset1 = 800;
    private const double DimOffset2 = 1500;
    private const double GridLabelRadius = 250;
    private const double GridLabelOffset = 2000;
    private const double GridLineExtension = 300;
    private const double EdgeTolerance = 0.15;

    private const string LayerGridLine  = "23_通り芯";
    private const string LayerText      = "22_文字";
    private const string LayerDimension = "21_寸法線";
    private const string LayerEquipment = "01_RMN_新設設備";
    private const string DimStyleName   = "1-100";

    private static double _offsetX, _offsetY;
    private static double _glMinX, _glMaxX, _glMinY, _glMaxY;
    // 名前付き通り芯のみの範囲（スラブ境界を含まない）
    private static double _namedMinX, _namedMaxX, _namedMinY, _namedMaxY;
    // 寸法スパン重複防止
    private static HashSet<long> _dimSpans = new();

    // 通り芯位置（番号付き + スラブ境界の番号なし）
    private record GridLineInfo(GridLineDirection Dir, double Pos, string? Name);

    public static void Draw(
        BeamModel beam, List<SlabModel> slabs,
        List<BeamModel> allBeams, List<GridLineModel> gridLines)
    {
        if (beam.TributaryPolygons.Count == 0) return;
        _dimSpans.Clear();
        var slab = slabs.FirstOrDefault(s =>
            s.Contains(beam.MidPoint) || IsNearSlab(s, beam.MidPoint));
        if (slab == null) return;
        var beamsInSlab = allBeams.Where(b =>
            slab.Contains(b.MidPoint) || slab.Contains(b.StartPoint)
            || slab.Contains(b.EndPoint) || IsNearSlab(slab, b.MidPoint)).ToList();

        var doc = Application.DocumentManager.MdiActiveDocument;
        if (doc == null) return;
        var db = doc.Database; var ed = doc.Editor;

        var ppr = ed.GetPoint(new PromptPointOptions(
            "\n負担面積図の配置点を指定してください（左下基準）: ") { AllowNone = false });
        if (ppr.Status != PromptStatus.OK) return;

        _offsetX = ppr.Value.X - slab.MinX * MToMm;
        _offsetY = ppr.Value.Y - slab.MinY * MToMm;

        // 通り芯（名前付き）+ スラブ境界（名前なし）を統合
        var allLines = BuildAllGridLines(slab, beam, gridLines);
        ComputeBounds(allLines);

        using var docLock = doc.LockDocument();
        using var tr = db.TransactionManager.StartTransaction();
        try
        {
            EnsureLayers(db, tr); EnsureLinetype(db, "CENTER");
            var dsId = GetDimStyleId(db, tr);
            var bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
            var btr = (BlockTableRecord)tr.GetObject(
                bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite);

            DrawTributaryHatch(btr, tr, beam);
            DrawPanelDimensions(btr, tr, dsId, beamsInSlab);
            DrawTributaryDimensions(btr, tr, dsId, beam);
            DrawAllGridLines(btr, tr, db, allLines);
            DrawLegend(btr, tr);

            tr.Commit();
            ed.WriteMessage("\n負担面積図を作成しました。\n");
            ed.Regen();
        }
        catch (System.Exception ex)
        {
            ed.WriteMessage($"\n負担面積図の作成に失敗しました: {ex.Message}\n");
            tr.Abort();
        }
    }

    // ─── 座標変換 ────────────────────────────────────────────

    private static Point2d P2d(double xm, double ym)
        => new(xm * MToMm + _offsetX, ym * MToMm + _offsetY);
    private static Point3d P3dMm(double xmm, double ymm)
        => new(xmm + _offsetX, ymm + _offsetY, 0);

    // ─── インフラ ────────────────────────────────────────────

    private static void EnsureLayers(Database db, Transaction tr)
    {
        var lt = (LayerTable)tr.GetObject(db.LayerTableId, OpenMode.ForRead);
        foreach (var n in new[] { LayerGridLine, LayerText, LayerDimension, LayerEquipment })
            if (!lt.Has(n)) { lt.UpgradeOpen(); var l = new LayerTableRecord { Name = n }; lt.Add(l); tr.AddNewlyCreatedDBObject(l, true); }
    }
    private static void EnsureLinetype(Database db, string lt)
    { var ltt = (LinetypeTable)db.TransactionManager.TopTransaction.GetObject(db.LinetypeTableId, OpenMode.ForRead); if (!ltt.Has(lt)) { try { db.LoadLineTypeFile(lt, "acad.lin"); } catch { } } }
    private static ObjectId GetDimStyleId(Database db, Transaction tr)
    { var dst = (DimStyleTable)tr.GetObject(db.DimStyleTableId, OpenMode.ForRead); return dst.Has(DimStyleName) ? dst[DimStyleName] : db.Dimstyle; }
    private static void SetLinetype(Entity ent, Database db, string lt)
    { var ltt = (LinetypeTable)db.TransactionManager.TopTransaction.GetObject(db.LinetypeTableId, OpenMode.ForRead); if (ltt.Has(lt)) ent.LinetypeId = ltt[lt]; }

    // ─── 通り芯 + スラブ境界の統合リスト構築 ────────────────

    private static List<GridLineInfo> BuildAllGridLines(
        SlabModel slab, BeamModel beam, List<GridLineModel> gridLines)
    {
        var named = GetRelevantGridLines(beam, gridLines);
        var result = named.Select(g => new GridLineInfo(g.Direction, g.Position, g.Name)).ToList();

        // 負担面積ポリゴンが接触するスラブ境界のみ追加
        var allPts = beam.TributaryPolygons.SelectMany(p => p).ToList();
        void addBoundary(GridLineDirection dir, double pos)
        {
            if (result.Any(r => r.Dir == dir && Math.Abs(r.Pos - pos) < EdgeTolerance)) return;
            bool touches = dir == GridLineDirection.Vertical
                ? allPts.Any(p => Math.Abs(p.X - pos) < EdgeTolerance)
                : allPts.Any(p => Math.Abs(p.Y - pos) < EdgeTolerance);
            if (touches) result.Add(new GridLineInfo(dir, pos, null));
        }
        addBoundary(GridLineDirection.Vertical, slab.MinX);
        addBoundary(GridLineDirection.Vertical, slab.MaxX);
        addBoundary(GridLineDirection.Horizontal, slab.MinY);
        addBoundary(GridLineDirection.Horizontal, slab.MaxY);

        return result;
    }

    private static List<GridLineModel> GetRelevantGridLines(
        BeamModel beam, List<GridLineModel> gridLines)
    {
        var pts = beam.TributaryPolygons.SelectMany(p => p).ToList();
        if (pts.Count == 0) return new();
        double tMinX = pts.Min(p => p.X), tMaxX = pts.Max(p => p.X);
        double tMinY = pts.Min(p => p.Y), tMaxY = pts.Max(p => p.Y);

        var vGLs = gridLines.Where(g => g.Direction == GridLineDirection.Vertical).OrderBy(g => g.Position).ToList();
        double vL = vGLs.Where(g => g.Position <= tMinX + EdgeTolerance).Select(g => g.Position).DefaultIfEmpty(double.NaN).Last();
        double vR = vGLs.Where(g => g.Position >= tMaxX - EdgeTolerance).Select(g => g.Position).DefaultIfEmpty(double.NaN).First();
        var rv = vGLs.Where(g => (double.IsNaN(vL) || g.Position >= vL - EdgeTolerance) && (double.IsNaN(vR) || g.Position <= vR + EdgeTolerance)).ToList();

        var hGLs = gridLines.Where(g => g.Direction == GridLineDirection.Horizontal).OrderBy(g => g.Position).ToList();
        double hB = hGLs.Where(g => g.Position <= tMinY + EdgeTolerance).Select(g => g.Position).DefaultIfEmpty(double.NaN).Last();
        double hT = hGLs.Where(g => g.Position >= tMaxY - EdgeTolerance).Select(g => g.Position).DefaultIfEmpty(double.NaN).First();
        var rh = hGLs.Where(g => (double.IsNaN(hB) || g.Position >= hB - EdgeTolerance) && (double.IsNaN(hT) || g.Position <= hT + EdgeTolerance)).ToList();

        var r = new List<GridLineModel>(); r.AddRange(rv); r.AddRange(rh); return r;
    }

    private static void ComputeBounds(List<GridLineInfo> lines)
    {
        var vp = lines.Where(l => l.Dir == GridLineDirection.Vertical).Select(l => l.Pos).ToList();
        var hp = lines.Where(l => l.Dir == GridLineDirection.Horizontal).Select(l => l.Pos).ToList();
        _glMinX = vp.Count > 0 ? vp.Min() : 0; _glMaxX = vp.Count > 0 ? vp.Max() : 0;
        _glMinY = hp.Count > 0 ? hp.Min() : 0; _glMaxY = hp.Count > 0 ? hp.Max() : 0;

        // 名前付き通り芯のみの範囲
        var nvp = lines.Where(l => l.Dir == GridLineDirection.Vertical && l.Name != null).Select(l => l.Pos).ToList();
        var nhp = lines.Where(l => l.Dir == GridLineDirection.Horizontal && l.Name != null).Select(l => l.Pos).ToList();
        _namedMinX = nvp.Count > 0 ? nvp.Min() : _glMinX; _namedMaxX = nvp.Count > 0 ? nvp.Max() : _glMaxX;
        _namedMinY = nhp.Count > 0 ? nhp.Min() : _glMinY; _namedMaxY = nhp.Count > 0 ? nhp.Max() : _glMaxY;
    }

    // ─── 負担面積ハッチング ──────────────────────────────────

    private static void DrawTributaryHatch(BlockTableRecord btr, Transaction tr, BeamModel beam)
    {
        var ids = new List<ObjectId>();
        foreach (var poly in beam.TributaryPolygons)
        {
            var pl = new Polyline();
            for (int i = 0; i < poly.Count; i++) pl.AddVertexAt(i, P2d(poly[i].X, poly[i].Y), 0, 0, 0);
            pl.Closed = true; pl.Layer = LayerEquipment; AddEntity(btr, tr, pl); ids.Add(pl.ObjectId);
        }
        var h = new Hatch(); h.SetDatabaseDefaults(); h.Layer = LayerEquipment;
        h.SetHatchPattern(HatchPatternType.PreDefined, "ANSI31"); h.PatternScale = 100; h.PatternAngle = Math.PI / 2;
        AddEntity(btr, tr, h);
        foreach (var id in ids) h.AppendLoop(HatchLoopTypes.Outermost, new ObjectIdCollection(new[] { id }));
        h.EvaluateHatch(true);
    }

    // ─── パネル寸法線 ────────────────────────────────────────

    private static void DrawPanelDimensions(
        BlockTableRecord btr, Transaction tr, ObjectId dsId, List<BeamModel> beams)
    {
        double tipB = _glMinY * MToMm - GridLineExtension;
        double tipL = _glMinX * MToMm - GridLineExtension;

        // ── X方向（水平寸法）────
        // 名前付き通り芯の位置を収集
        var xNamed = new List<double> { _namedMinX };
        foreach (var b in beams)
        {
            if (!IsVerticalBeam(b)) continue;
            double bx = (b.StartPoint.X + b.EndPoint.X) / 2;
            if (bx > _namedMinX + EdgeTolerance && bx < _namedMaxX - EdgeTolerance)
                xNamed.Add(bx);
        }
        xNamed.Add(_namedMaxX);
        xNamed = DistinctTol(xNamed);

        // 名前付き通り芯間のセグメント寸法
        for (int i = 0; i < xNamed.Count - 1; i++)
            AddHDim(btr, tr, dsId, xNamed[i] * MToMm, tipB, xNamed[i + 1] * MToMm, tipB, tipB - DimOffset1);
        // 名前付き通り芯の全体寸法
        if (xNamed.Count > 2)
            AddHDim(btr, tr, dsId, _namedMinX * MToMm, tipB, _namedMaxX * MToMm, tipB, tipB - DimOffset2);

        // スラブ境界が名前付き通り芯の外側にある場合のみ、1本だけ追加寸法
        if (_glMinX < _namedMinX - EdgeTolerance)
            AddHDim(btr, tr, dsId, _glMinX * MToMm, tipB, _namedMinX * MToMm, tipB, tipB - DimOffset1);
        if (_glMaxX > _namedMaxX + EdgeTolerance)
            AddHDim(btr, tr, dsId, _namedMaxX * MToMm, tipB, _glMaxX * MToMm, tipB, tipB - DimOffset1);

        // ── Y方向（垂直寸法）────
        var yNamed = new List<double> { _namedMinY };
        foreach (var b in beams)
        {
            if (!IsHorizontalBeam(b)) continue;
            double by = (b.StartPoint.Y + b.EndPoint.Y) / 2;
            if (by > _namedMinY + EdgeTolerance && by < _namedMaxY - EdgeTolerance)
                yNamed.Add(by);
        }
        yNamed.Add(_namedMaxY);
        yNamed = DistinctTol(yNamed);

        for (int i = 0; i < yNamed.Count - 1; i++)
            AddVDim(btr, tr, dsId, tipL, yNamed[i] * MToMm, tipL, yNamed[i + 1] * MToMm, tipL - DimOffset1);
        if (yNamed.Count > 2)
            AddVDim(btr, tr, dsId, tipL, _namedMinY * MToMm, tipL, _namedMaxY * MToMm, tipL - DimOffset2);

        if (_glMinY < _namedMinY - EdgeTolerance)
            AddVDim(btr, tr, dsId, tipL, _glMinY * MToMm, tipL, _namedMinY * MToMm, tipL - DimOffset1);
        if (_glMaxY > _namedMaxY + EdgeTolerance)
            AddVDim(btr, tr, dsId, tipL, _namedMaxY * MToMm, tipL, _glMaxY * MToMm, tipL - DimOffset1);
    }

    /// <summary>近接値を統合してソートした重複なしリストを返す</summary>
    private static List<double> DistinctTol(List<double> vals)
    {
        var sorted = vals.OrderBy(v => v).ToList();
        var result = new List<double> { sorted[0] };
        for (int i = 1; i < sorted.Count; i++)
            if (sorted[i] - result[^1] > EdgeTolerance)
                result.Add(sorted[i]);
        return result;
    }

    // ─── 負担面積寸法線 ──────────────────────────────────────
    // 水平辺 → 幅寸法（上底の幅）
    // 垂直辺 → 高さ寸法
    // 台形の高さ → 梁から最遠頂点への垂直距離

    private static void DrawTributaryDimensions(
        BlockTableRecord btr, Transaction tr, ObjectId dsId, BeamModel beam)
    {
        double beamX = (beam.StartPoint.X + beam.EndPoint.X) / 2;
        double beamY = (beam.StartPoint.Y + beam.EndPoint.Y) / 2;
        bool bH = IsHorizontalBeam(beam), bV = IsVerticalBeam(beam);

        foreach (var poly in beam.TributaryPolygons)
        {
            var hEdges = new List<(double x1, double x2, double y)>();
            var vEdges = new List<(double y1, double y2, double x)>();
            int diagonalCount = 0;

            for (int i = 0; i < poly.Count; i++)
            {
                var a = poly[i]; var b = poly[(i + 1) % poly.Count];
                var mid = new Point2d((a.X + b.X) / 2, (a.Y + b.Y) / 2);
                if (PtSegDist(mid, beam.StartPoint, beam.EndPoint) < EdgeTolerance) continue;
                double dx = Math.Abs(b.X - a.X), dy = Math.Abs(b.Y - a.Y);
                if (dy < 0.05 && dx > 0.05) hEdges.Add((Math.Min(a.X, b.X), Math.Max(a.X, b.X), (a.Y + b.Y) / 2));
                else if (dx < 0.05 && dy > 0.05) vEdges.Add((Math.Min(a.Y, b.Y), Math.Max(a.Y, b.Y), (a.X + b.X) / 2));
                else diagonalCount++;
            }

            // 長方形（対角線辺なし）→ パネル寸法で十分なのでスキップ
            if (diagonalCount == 0) continue;

            double polyCX = poly.Average(p => p.X);
            double polyCY = poly.Average(p => p.Y);
            bool polyRight = polyCX > beamX;
            bool polyAbove = polyCY > beamY;

            // ─── 水平辺 → 上底の幅寸法 ───
            foreach (var e in hEdges)
            {
                bool edgeAbove = e.y > beamY;
                AddHDim(btr, tr, dsId, e.x1 * MToMm, e.y * MToMm,
                    e.x2 * MToMm, e.y * MToMm,
                    e.y * MToMm + (edgeAbove ? DimOffset1 : -DimOffset1));
            }

            // ─── 垂直辺 → 高さ寸法 ───
            foreach (var e in vEdges)
            {
                double dimX = polyRight
                    ? e.x * MToMm + DimOffset1
                    : e.x * MToMm - DimOffset1;
                AddVDim(btr, tr, dsId, e.x * MToMm, e.y1 * MToMm,
                    e.x * MToMm, e.y2 * MToMm, dimX);
            }

            // ─── 台形の高さ（梁から最遠の非梁頂点への垂直距離）───
            // 通り芯寸法線（左・下）の反対側（右・上）に配置
            // 寸法線は通り芯端点から800mm外側
            const double TrapezoidDimOffset = 800;

            if (bH)
            {
                var farPt = poly.OrderByDescending(p => Math.Abs(p.Y - beamY)).First();
                double height = Math.Abs(farPt.Y - beamY);
                if (height > 0.05)
                {
                    bool covered = vEdges.Any(e => Math.Abs(Math.Abs(e.y2 - e.y1) - height) < 0.1);
                    if (!covered)
                    {
                        // 下底側: 通り芯寸法が左にあるので、右側の通り芯端点を使用
                        double tipR = _glMaxX * MToMm + GridLineExtension;

                        // 上底側: 上底を構成する水平辺の端点（なければ最遠頂点）
                        double topX = farPt.X * MToMm, topY = farPt.Y * MToMm;
                        if (hEdges.Count > 0)
                        {
                            // 右端（通り芯端点側）に最も近い上底端点
                            var pts = hEdges.SelectMany(e => new[] { (e.x1, e.y), (e.x2, e.y) });
                            var best = pts.OrderByDescending(p => p.Item1).First();
                            topX = best.Item1 * MToMm; topY = best.Item2 * MToMm;
                        }

                        // 寸法線は右側の通り芯端点から800mm外側
                        double dimX = tipR + TrapezoidDimOffset;
                        AddVDim(btr, tr, dsId, tipR, beamY * MToMm,
                            topX, topY, dimX);
                    }
                }
            }
            else if (bV)
            {
                var farPt = poly.OrderByDescending(p => Math.Abs(p.X - beamX)).First();
                double width = Math.Abs(farPt.X - beamX);
                if (width > 0.05)
                {
                    bool covered = hEdges.Any(e => Math.Abs(Math.Abs(e.x2 - e.x1) - width) < 0.1);
                    if (!covered)
                    {
                        // 下底側: 通り芯寸法が下にあるので、上側の通り芯端点を使用
                        double tipT = _glMaxY * MToMm + GridLineExtension;

                        // 上底側: 上底を構成する辺の端点（なければ最遠頂点）
                        double topX = farPt.X * MToMm, topY = farPt.Y * MToMm;
                        if (vEdges.Count > 0)
                        {
                            // 上端（通り芯端点側）に最も近い上底端点
                            var pts = vEdges.SelectMany(e => new[] { (e.x, e.y1), (e.x, e.y2) });
                            var best = pts.OrderByDescending(p => p.Item2).First();
                            topX = best.Item1 * MToMm; topY = best.Item2 * MToMm;
                        }
                        else if (hEdges.Count > 0)
                        {
                            var pts = hEdges.SelectMany(e => new[] { (e.x1, e.y), (e.x2, e.y) });
                            var best = pts.OrderByDescending(p => Math.Abs(p.Item1 - beamX)).First();
                            topX = best.Item1 * MToMm; topY = best.Item2 * MToMm;
                        }

                        // 寸法線は上側の通り芯端点から800mm外側
                        double dimY = tipT + TrapezoidDimOffset;
                        AddHDim(btr, tr, dsId, beamX * MToMm, tipT,
                            topX, topY, dimY);
                    }
                }
            }
        }
    }

    // ─── 通り芯描画（名前付き + スラブ境界）─────────────────

    private static void DrawAllGridLines(
        BlockTableRecord btr, Transaction tr, Database db, List<GridLineInfo> lines)
    {
        foreach (var gl in lines)
        {
            if (gl.Dir == GridLineDirection.Vertical)
            {
                double y1 = _glMinY * MToMm - GridLineExtension;
                double y2 = _glMaxY * MToMm + GridLineExtension;
                var line = new Line(P3dMm(gl.Pos * MToMm, y1), P3dMm(gl.Pos * MToMm, y2));
                line.Layer = LayerGridLine; SetLinetype(line, db, "CENTER");
                AddEntity(btr, tr, line);
                if (gl.Name != null)
                    DrawCircleLabel(btr, tr,
                        gl.Pos * MToMm + _offsetX, y1 + _offsetY - GridLabelOffset, gl.Name);
            }
            else
            {
                double x1 = _glMinX * MToMm - GridLineExtension;
                double x2 = _glMaxX * MToMm + GridLineExtension;
                var line = new Line(P3dMm(x1, gl.Pos * MToMm), P3dMm(x2, gl.Pos * MToMm));
                line.Layer = LayerGridLine; SetLinetype(line, db, "CENTER");
                AddEntity(btr, tr, line);
                if (gl.Name != null)
                    DrawCircleLabel(btr, tr,
                        x1 + _offsetX - GridLabelOffset, gl.Pos * MToMm + _offsetY, gl.Name);
            }
        }
    }

    private static void DrawCircleLabel(BlockTableRecord btr, Transaction tr,
        double cx, double cy, string text)
    {
        AddEntity(btr, tr, new Circle(new Point3d(cx, cy, 0), Vector3d.ZAxis, GridLabelRadius) { Layer = LayerText });
        AddEntity(btr, tr, new DBText
        {
            TextString = text, Height = 250, Layer = LayerText,
            HorizontalMode = TextHorizontalMode.TextCenter,
            VerticalMode = TextVerticalMode.TextVerticalMid,
            AlignmentPoint = new Point3d(cx, cy, 0),
        });
    }

    // ─── 凡例 ────────────────────────────────────────────────

    private static void DrawLegend(BlockTableRecord btr, Transaction tr)
    {
        double cx = (_glMinX + _glMaxX) / 2 * MToMm + _offsetX;
        double cy = (_glMinY * MToMm - GridLineExtension) + _offsetY - DimOffset2 - 1500;
        double hw = 750, hh = 375;

        var box = new Polyline();
        box.AddVertexAt(0, new Point2d(cx - hw, cy - hh), 0, 0, 0);
        box.AddVertexAt(1, new Point2d(cx + hw, cy - hh), 0, 0, 0);
        box.AddVertexAt(2, new Point2d(cx + hw, cy + hh), 0, 0, 0);
        box.AddVertexAt(3, new Point2d(cx - hw, cy + hh), 0, 0, 0);
        box.Closed = true; box.Layer = LayerEquipment; AddEntity(btr, tr, box);

        var lh = new Hatch(); lh.SetDatabaseDefaults(); lh.Layer = LayerEquipment;
        lh.SetHatchPattern(HatchPatternType.PreDefined, "ANSI31"); lh.PatternScale = 100; lh.PatternAngle = Math.PI / 2;
        AddEntity(btr, tr, lh);
        lh.AppendLoop(HatchLoopTypes.Outermost, new ObjectIdCollection(new[] { box.ObjectId }));
        lh.EvaluateHatch(true);

        AddEntity(btr, tr, new DBText
        {
            TextString = "…負担面積", Height = 300, Layer = LayerText,
            HorizontalMode = TextHorizontalMode.TextLeft,
            VerticalMode = TextVerticalMode.TextVerticalMid,
            AlignmentPoint = new Point3d(cx + hw + 200, cy, 0),
        });
    }

    // ─── 寸法ヘルパー ────────────────────────────────────────

    private static void AddHDim(BlockTableRecord btr, Transaction tr, ObjectId dsId,
        double x1, double y1, double x2, double y2, double dimY)
    {
        long span = (long)Math.Round(Math.Abs(x2 - x1));
        if (span < 1) return;
        long key = span * 1000000L + (long)Math.Round(dimY);
        if (!_dimSpans.Add(key)) return;
        var d = new RotatedDimension(0, P3dMm(x1, y1), P3dMm(x2, y2), P3dMm((x1 + x2) / 2, dimY), "", dsId);
        d.Layer = LayerDimension; d.Dimdec = 0; AddEntity(btr, tr, d);
    }

    private static void AddVDim(BlockTableRecord btr, Transaction tr, ObjectId dsId,
        double x1, double y1, double x2, double y2, double dimX)
    {
        long span = (long)Math.Round(Math.Abs(y2 - y1));
        if (span < 1) return;
        long key = span * 1000000L + (long)Math.Round(dimX);
        if (!_dimSpans.Add(key)) return;
        var d = new RotatedDimension(Math.PI / 2, P3dMm(x1, y1), P3dMm(x2, y2), P3dMm(dimX, (y1 + y2) / 2), "", dsId);
        d.Layer = LayerDimension; d.Dimdec = 0; AddEntity(btr, tr, d);
    }

    // ─── ユーティリティ ──────────────────────────────────────

    private static void AddEntity(BlockTableRecord btr, Transaction tr, Entity ent)
    { btr.AppendEntity(ent); tr.AddNewlyCreatedDBObject(ent, true); }
    private static bool IsVerticalBeam(BeamModel b) { double dx = Math.Abs(b.EndPoint.X - b.StartPoint.X), dy = Math.Abs(b.EndPoint.Y - b.StartPoint.Y); return dy > dx * 3; }
    private static bool IsHorizontalBeam(BeamModel b) { double dx = Math.Abs(b.EndPoint.X - b.StartPoint.X), dy = Math.Abs(b.EndPoint.Y - b.StartPoint.Y); return dx > dy * 3; }

    private static double PtSegDist(Point2d p, Point2d a, Point2d b)
    { double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy; if (l2 < 1e-20) return Math.Sqrt((p.X - a.X) * (p.X - a.X) + (p.Y - a.Y) * (p.Y - a.Y)); double t = Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1); double px = a.X + t * dx, py = a.Y + t * dy; return Math.Sqrt((p.X - px) * (p.X - px) + (p.Y - py) * (p.Y - py)); }

    private static bool IsNearSlab(SlabModel slab, Point2d pt)
    { var sv = slab.Vertices; for (int i = 0; i < sv.Count; i++) if (PtSegDist(pt, sv[i], sv[(i + 1) % sv.Count]) < 0.20) return true; return false; }
}
