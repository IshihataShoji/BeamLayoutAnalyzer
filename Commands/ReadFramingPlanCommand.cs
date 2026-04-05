using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using BeamLayoutAnalyzer.Analysis;
using BeamLayoutAnalyzer.Models;
using BeamLayoutAnalyzer.UI;

namespace BeamLayoutAnalyzer.Commands;

/// <summary>
/// AutoCADカスタムコマンド「FRAMINGPLAN」のエントリポイント。
/// 使い方：
///   1. NETLOAD で BeamLayoutAnalyzer.dll を読み込む
///   2. FRAMINGPLAN コマンドを実行
///   3. スラブ・柱・梁を選択して Enter
/// </summary>
public class ReadFramingPlanCommand
{
    [CommandMethod("FRAMINGPLAN", CommandFlags.Modal)]
    public void Execute()
    {
        var doc = Application.DocumentManager.MdiActiveDocument;
        var db  = doc.Database;
        var ed  = doc.Editor;

        ed.WriteMessage("\n梁伏図解析: オブジェクトを選択して Enter（全選択は Ctrl+A）\n");

        var selRes = ed.GetSelection();
        if (selRes.Status != PromptStatus.OK)
        {
            ed.WriteMessage("キャンセルしました。\n");
            return;
        }

        var slabs   = new List<SlabModel>();
        var columns = new List<ColumnModel>();
        var beams   = new List<BeamModel>();

        using (var tr = db.TransactionManager.StartTransaction())
        {
            foreach (SelectedObject selObj in selRes.Value)
            {
                if (tr.GetObject(selObj.ObjectId, OpenMode.ForRead) is not Entity ent)
                    continue;

                switch (ent)
                {
                    case Polyline poly when poly.Closed:
                        slabs.Add(new SlabModel(poly));
                        break;
                    case Circle circle:
                        columns.Add(new ColumnModel(circle));
                        break;
                    case Line line:
                        beams.Add(new BeamModel(line));
                        break;
                }
            }
            tr.Commit();
        }

        ed.WriteMessage($"読み込み: スラブ {slabs.Count} / 柱 {columns.Count} / 梁 {beams.Count}\n");

        if (slabs.Count == 0 || beams.Count == 0)
        {
            ed.WriteMessage("エラー: スラブまたは梁が見つかりませんでした。\n");
            return;
        }

        // 大梁／小梁の自動判定
        ClassifyBeams(beams, columns);

        int mainCount = beams.Count(b => b.Type == BeamType.大梁);
        int subCount  = beams.Count(b => b.Type == BeamType.小梁);
        ed.WriteMessage($"梁種別判定: 大梁 {mainCount}本 / 小梁 {subCount}本\n");

        // 負担面積を計算
        ed.WriteMessage("負担面積を計算中...\n");
        var calc = new TributaryAreaCalculator(slabs, columns, beams);
        calc.Calculate();

        // 診断ログ
        int matched = beams.Count(b => b.TributaryArea > 0);
        ed.WriteMessage($"負担面積計算完了: {matched}/{beams.Count} 本の梁に面積割当済み\n");
        foreach (var b in beams)
        {
            ed.WriteMessage($"  梁 L={b.Length:F1}m [{b.Type}] " +
                            $"面積={b.TributaryArea:F2} m² " +
                            $"ポリゴン数={b.TributaryPolygons.Count}\n");
        }

        // GUI起動
        var window = new FramingPlanWindow(slabs, columns, beams);
        Application.ShowModalWindow(window);
    }

    /// <summary>
    /// 両端が柱の中心付近に接続している梁 → 大梁、それ以外 → 小梁
    /// </summary>
    private static void ClassifyBeams(List<BeamModel> beams, List<ColumnModel> columns)
    {
        if (columns.Count == 0)
        {
            // 柱なし：全て小梁
            foreach (var b in beams) b.Type = BeamType.小梁;
            return;
        }

        // 柱半径の最小値を基準に許容誤差を設定（最低 5cm）
        double minRadius = columns.Min(c => c.Radius);
        double tolerance = Math.Max(minRadius * 2.0, 0.05);

        foreach (var beam in beams)
        {
            bool startAtCol = columns.Any(c =>
                Dist(c.Center, beam.StartPoint) <= c.Radius + tolerance);
            bool endAtCol = columns.Any(c =>
                Dist(c.Center, beam.EndPoint) <= c.Radius + tolerance);

            beam.Type = (startAtCol && endAtCol) ? BeamType.大梁 : BeamType.小梁;
        }
    }

    private static double Dist(Point2d a, Point2d b)
        => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
}
