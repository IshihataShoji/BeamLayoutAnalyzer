using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using Autodesk.AutoCAD.Geometry;
using BeamLayoutAnalyzer.Models;

namespace BeamLayoutAnalyzer.UI;

public partial class FramingPlanWindow : Window
{
    // ─── データ ─────────────────────────────────────────────
    private readonly List<SlabModel>   _slabs;
    private readonly List<ColumnModel> _columns;
    private readonly List<BeamModel>   _beams;

    // ─── 表示モード ──────────────────────────────────────────
    private enum DisplayMode { Seismic, Beams, Columns }
    private DisplayMode _mode = DisplayMode.Beams;

    // ─── 座標変換 ────────────────────────────────────────────
    private double _scale;
    private double _originX, _originY, _canvasH;
    private const double CanvasMargin = 44.0;

    // ─── 選択状態 ────────────────────────────────────────────
    private BeamModel?   _selectedBeam;
    private ColumnModel? _selectedColumn;

    // ─── Canvas要素マップ ────────────────────────────────────
    private readonly Dictionary<BeamModel,   Line>    _beamVis = new();
    private readonly Dictionary<ColumnModel, Ellipse> _colVis  = new();

    // ─── ハイライト表示用 ─────────────────────────────────────
    private readonly List<System.Windows.Shapes.Polygon> _highlightPolys = new();

    // ─── ブラシ定数 ──────────────────────────────────────────
    private static readonly Brush SlabFill      = Freeze(new SolidColorBrush(Color.FromArgb(45,  100, 149, 237)));
    private static readonly Brush SlabStroke    = Freeze(new SolidColorBrush(Color.FromRgb( 100, 149, 237)));
    private static readonly Brush SlabFillSeism = Freeze(new SolidColorBrush(Color.FromArgb(80,  255, 200,  50)));
    private static readonly Brush GrayBeam      = Freeze(new SolidColorBrush(Color.FromArgb(80,  160, 160, 160)));
    private static readonly Brush FaintBeam     = Freeze(new SolidColorBrush(Color.FromArgb(60,  160, 160, 160)));
    private static readonly Brush MainNormal    = Freeze(new SolidColorBrush(Color.FromRgb( 255, 215,   0)));
    private static readonly Brush MainHover     = Freeze(new SolidColorBrush(Color.FromRgb( 255, 255, 150)));
    private static readonly Brush MainSel       = Freeze(new SolidColorBrush(Color.FromRgb( 100, 255, 100)));
    private static readonly Brush SubNormal     = Freeze(new SolidColorBrush(Color.FromRgb( 255, 180,  60)));
    private static readonly Brush SubHover      = Freeze(new SolidColorBrush(Color.FromRgb( 255, 230, 120)));
    private static readonly Brush SubSel        = Freeze(new SolidColorBrush(Color.FromRgb( 100, 255, 100)));
    private static readonly Brush ColNormal     = Freeze(new SolidColorBrush(Color.FromRgb(  60, 180, 255)));
    private static readonly Brush ColHover      = Freeze(new SolidColorBrush(Color.FromRgb( 160, 220, 255)));
    private static readonly Brush ColSel        = Freeze(new SolidColorBrush(Color.FromRgb( 100, 255, 180)));
    private static readonly Brush GrayCol       = Freeze(new SolidColorBrush(Color.FromArgb(80,  120, 120, 120)));

    private static SolidColorBrush Freeze(SolidColorBrush b) { b.Freeze(); return b; }

    // ─── コンストラクタ ──────────────────────────────────────
    public FramingPlanWindow(
        List<SlabModel>   slabs,
        List<ColumnModel> columns,
        List<BeamModel>   beams)
    {
        InitializeComponent();
        _slabs   = slabs;
        _columns = columns;
        _beams   = beams;

        Loaded      += (_, _) => Render();
        SizeChanged += (_, _) => Render();
    }

    // ─── タブ切替 ────────────────────────────────────────────

    private void ViewMode_Changed(object sender, RoutedEventArgs e)
    {
        if (!IsLoaded) return;
        _mode = RbSeismic.IsChecked  == true ? DisplayMode.Seismic
              : RbColumns.IsChecked == true ? DisplayMode.Columns
              : DisplayMode.Beams;

        _selectedBeam   = null;
        _selectedColumn = null;
        ClearHighlight();
        MemberInfoPanel.Children.Clear();
        MemberInfoPanel.Children.Add(new TextBlock
        {
            Text     = "部材をクリックしてください",
            Foreground = new SolidColorBrush(Color.FromRgb(100, 100, 100)),
            FontSize = 11, TextWrapping = TextWrapping.Wrap
        });

        Render();
    }

    // ─── メイン描画 ──────────────────────────────────────────

    private void Render()
    {
        DrawingCanvas.Children.Clear();
        _beamVis.Clear();
        _colVis.Clear();

        // バウンディングボックス
        var allPts = _slabs.SelectMany(s => s.Vertices)
            .Concat(_columns.Select(c => c.Center))
            .Concat(_beams.SelectMany(b => new[] { b.StartPoint, b.EndPoint }))
            .ToList();
        if (allPts.Count == 0) return;

        double minX = allPts.Min(p => p.X), maxX = allPts.Max(p => p.X);
        double minY = allPts.Min(p => p.Y), maxY = allPts.Max(p => p.Y);
        double cadW = maxX - minX, cadH = maxY - minY;
        if (cadW < 1e-6 || cadH < 1e-6) return;

        double vW = Math.Max(DrawingScroller.ActualWidth  - 20, 400);
        double vH = Math.Max(DrawingScroller.ActualHeight - 20, 400);

        _scale   = Math.Min((vW - 2 * CanvasMargin) / cadW, (vH - 2 * CanvasMargin) / cadH);
        _originX = minX;
        _originY = minY;
        _canvasH = cadH * _scale + 2 * CanvasMargin;

        DrawingCanvas.Width  = cadW * _scale + 2 * CanvasMargin;
        DrawingCanvas.Height = _canvasH;

        switch (_mode)
        {
            case DisplayMode.Seismic:  RenderSeismicMode();  break;
            case DisplayMode.Beams:    RenderBeamsMode();    break;
            case DisplayMode.Columns:  RenderColumnsMode();  break;
        }

        UpdateSummary();
    }

    // ════════════ 地震力モード ════════════

    private void RenderSeismicMode()
    {
        double k = GetConst(SeismicConstBox);

        // スラブ（強調表示）+ 面積ラベル
        foreach (var s in _slabs)
        {
            DrawingCanvas.Children.Add(MakeSlabPolygon(s, SlabFillSeism, SlabStroke, 1.5));

            // スラブ重心にラベル
            double cx = s.Vertices.Average(v => v.X);
            double cy = s.Vertices.Average(v => v.Y);
            var cp = ToCanvas(cx, cy);
            string txt = $"面積\n{s.Area:F1} m²\n地震力\n{s.Area * k:F1} kN";
            AddLabel(cp.X, cp.Y, txt, Color.FromRgb(255, 220, 80), centerX: true);
        }

        // 梁・柱はグレーで背景表示（インタラクティブなし）
        foreach (var b in _beams)
        {
            var sp = ToCanvas(b.StartPoint.X, b.StartPoint.Y);
            var ep = ToCanvas(b.EndPoint.X,   b.EndPoint.Y);
            DrawingCanvas.Children.Add(new Line
            {
                X1 = sp.X, Y1 = sp.Y, X2 = ep.X, Y2 = ep.Y,
                Stroke = GrayBeam, StrokeThickness = 1.5, IsHitTestVisible = false
            });
        }
        foreach (var c in _columns)
        {
            var cp = ToCanvas(c.Center.X, c.Center.Y);
            double r = Math.Max(c.Radius * _scale, 4);
            var el = new Ellipse { Width = r*2, Height = r*2, Fill = GrayCol, IsHitTestVisible = false };
            Canvas.SetLeft(el, cp.X - r); Canvas.SetTop(el, cp.Y - r);
            DrawingCanvas.Children.Add(el);
        }
    }

    // ════════════ 大梁・小梁モード ════════════

    private void RenderBeamsMode()
    {
        // スラブ（薄く）
        foreach (var s in _slabs)
            DrawingCanvas.Children.Add(MakeSlabPolygon(s, SlabFill, SlabStroke, 1.2));

        // 亀の甲支配領域ポリゴン（梁の下に描画）
        foreach (var b in _beams)
        {
            bool isMain = b.Type == BeamType.大梁;
            var fillColor   = isMain
                ? Color.FromArgb(50, 255, 215,   0)
                : Color.FromArgb(50, 255, 160,  40);
            var strokeColor = isMain
                ? Color.FromArgb(150, 220, 190,   0)
                : Color.FromArgb(150, 220, 140,  20);

            foreach (var region in b.TributaryPolygons)
            {
                var poly = new Polygon
                {
                    Fill            = new SolidColorBrush(fillColor),
                    Stroke          = new SolidColorBrush(strokeColor),
                    StrokeThickness = 0.8,
                    StrokeDashArray = new System.Windows.Media.DoubleCollection { 4, 2 },
                    IsHitTestVisible= false,
                };
                foreach (var v in region)
                    poly.Points.Add(ToCanvas(v.X, v.Y));
                DrawingCanvas.Children.Add(poly);
            }
        }

        // 梁（インタラクティブ）
        foreach (var b in _beams) DrawBeam(b);

        // 柱（薄く）
        foreach (var c in _columns)
        {
            var cp = ToCanvas(c.Center.X, c.Center.Y);
            double r = Math.Max(c.Radius * _scale, 5);
            var el = new Ellipse
            {
                Width = r*2, Height = r*2,
                Fill = new SolidColorBrush(Color.FromArgb(80, 60, 180, 255)),
                IsHitTestVisible = false
            };
            Canvas.SetLeft(el, cp.X - r); Canvas.SetTop(el, cp.Y - r);
            DrawingCanvas.Children.Add(el);
        }

        // 支配面積ラベル
        double mainK = GetConst(MainBeamConstBox);
        double subK  = GetConst(SubBeamConstBox);
        foreach (var b in _beams)
        {
            double k    = b.Type == BeamType.大梁 ? mainK : subK;
            var mid = ToCanvas(b.MidPoint.X, b.MidPoint.Y);
            double angle = b.Angle;
            double ox = -Math.Sin(angle) * 14;
            double oy =  Math.Cos(angle) * 14;
            string txt  = $"{b.TributaryArea:F1} m²\n{b.TributaryArea * k:F1} kN";
            var color   = b.Type == BeamType.大梁
                ? Color.FromRgb(255, 230, 80)
                : Color.FromRgb(255, 195, 100);
            AddLabel(mid.X + ox, mid.Y - oy, txt, color, centerX: true);
        }
    }

    // ════════════ 柱モード ════════════

    private void RenderColumnsMode()
    {
        // スラブ（薄く）
        foreach (var s in _slabs)
            DrawingCanvas.Children.Add(MakeSlabPolygon(s, SlabFill, SlabStroke, 1.2));

        // ボロノイ領域（破線ポリゴン）
        foreach (var c in _columns)
        {
            if (c.VoronoiPolygon.Count < 3) continue;
            var poly = new Polygon
            {
                Fill            = new SolidColorBrush(Color.FromArgb(35,  60, 150, 255)),
                Stroke          = new SolidColorBrush(Color.FromArgb(180, 100, 200, 255)),
                StrokeThickness = 1.2,
                StrokeDashArray = new System.Windows.Media.DoubleCollection { 5, 3 },
                IsHitTestVisible= false,
            };
            foreach (var v in c.VoronoiPolygon)
                poly.Points.Add(ToCanvas(v.X, v.Y));
            DrawingCanvas.Children.Add(poly);
        }

        // 梁（薄く）
        foreach (var b in _beams)
        {
            var sp = ToCanvas(b.StartPoint.X, b.StartPoint.Y);
            var ep = ToCanvas(b.EndPoint.X,   b.EndPoint.Y);
            DrawingCanvas.Children.Add(new Line
            {
                X1 = sp.X, Y1 = sp.Y, X2 = ep.X, Y2 = ep.Y,
                Stroke = FaintBeam, StrokeThickness = 1.2, IsHitTestVisible = false
            });
        }

        // 柱（インタラクティブ）
        foreach (var c in _columns) DrawColumn(c);

        // 支配面積ラベル
        double colK = GetConst(ColumnConstBox);
        foreach (var c in _columns)
        {
            var cp = ToCanvas(c.Center.X, c.Center.Y);
            double r = Math.Max(c.Radius * _scale, 5);
            string txt = $"{c.TributaryArea:F1} m²\n{c.TributaryArea * colK:F1} kN";
            AddLabel(cp.X, cp.Y - r - 28, txt, Color.FromRgb(100, 210, 255), centerX: true);
        }
    }

    // ─── 個別描画ヘルパー ────────────────────────────────────

    private Polygon MakeSlabPolygon(SlabModel s, Brush fill, Brush stroke, double thickness)
    {
        var poly = new Polygon
        {
            Fill = fill, Stroke = stroke,
            StrokeThickness = thickness, IsHitTestVisible = false
        };
        foreach (var v in s.Vertices)
            poly.Points.Add(ToCanvas(v.X, v.Y));
        return poly;
    }

    private void DrawBeam(BeamModel beam)
    {
        var sp = ToCanvas(beam.StartPoint.X, beam.StartPoint.Y);
        var ep = ToCanvas(beam.EndPoint.X,   beam.EndPoint.Y);

        bool isMain = beam.Type == BeamType.大梁;
        bool isSel  = beam == _selectedBeam;
        var normalBrush = isMain ? MainNormal : SubNormal;
        var hoverBrush  = isMain ? MainHover  : SubHover;

        var vis = new Line
        {
            X1 = sp.X, Y1 = sp.Y, X2 = ep.X, Y2 = ep.Y,
            Stroke          = isSel ? (isMain ? MainSel : SubSel) : normalBrush,
            StrokeThickness = isMain ? 3.0 : 2.0,
            IsHitTestVisible= false,
        };
        _beamVis[beam] = vis;

        var hit = new Line
        {
            X1 = sp.X, Y1 = sp.Y, X2 = ep.X, Y2 = ep.Y,
            Stroke = Brushes.Transparent, StrokeThickness = 12,
            Cursor = Cursors.Hand, Tag = beam,
        };
        hit.MouseLeftButtonDown += OnBeamClick;
        hit.MouseEnter += (_, _) => { if (beam != _selectedBeam) vis.Stroke = hoverBrush; };
        hit.MouseLeave += (_, _) => { if (beam != _selectedBeam) vis.Stroke = normalBrush; };

        DrawingCanvas.Children.Add(vis);
        DrawingCanvas.Children.Add(hit);
    }

    private void DrawColumn(ColumnModel col)
    {
        var cp = ToCanvas(col.Center.X, col.Center.Y);
        double r = Math.Max(col.Radius * _scale, 6.0);
        bool isSel = col == _selectedColumn;

        var el = new Ellipse
        {
            Width = r*2, Height = r*2,
            Fill            = isSel ? ColSel : ColNormal,
            Stroke          = Brushes.White,
            StrokeThickness = 1.0,
            Cursor          = Cursors.Hand,
            Tag             = col,
        };
        Canvas.SetLeft(el, cp.X - r); Canvas.SetTop(el, cp.Y - r);
        _colVis[col] = el;

        el.MouseLeftButtonDown += OnColClick;
        el.MouseEnter += (_, _) => { if (col != _selectedColumn) el.Fill = ColHover; };
        el.MouseLeave += (_, _) => { if (col != _selectedColumn) el.Fill = ColNormal; };

        DrawingCanvas.Children.Add(el);
    }

    // ─── ラベル描画 ──────────────────────────────────────────

    private void AddLabel(double x, double y, string text, Color textColor, bool centerX = false)
    {
        var tb = new TextBlock
        {
            Text          = text,
            FontSize      = 9.5,
            FontFamily    = new FontFamily("Meiryo UI, Yu Gothic UI, Segoe UI"),
            Foreground    = new SolidColorBrush(textColor),
            TextAlignment = TextAlignment.Center,
            IsHitTestVisible = false,
            LineHeight    = 13,
        };
        var border = new Border
        {
            Child           = tb,
            Background      = new SolidColorBrush(Color.FromArgb(190, 12, 12, 22)),
            CornerRadius    = new CornerRadius(3),
            Padding         = new Thickness(4, 2, 4, 2),
            IsHitTestVisible= false,
        };
        // 幅を固定してセンタリング
        double labelW = 64;
        Canvas.SetLeft(border, centerX ? x - labelW / 2 : x);
        Canvas.SetTop( border, y);
        border.Width = labelW;
        DrawingCanvas.Children.Add(border);
    }

    // ─── 座標変換 ────────────────────────────────────────────

    private System.Windows.Point ToCanvas(double x, double y)
        => new(CanvasMargin + (x - _originX) * _scale,
               _canvasH - CanvasMargin - (y - _originY) * _scale);

    // ─── クリックイベント ────────────────────────────────────

    private void OnBeamClick(object sender, MouseButtonEventArgs e)
    {
        if (_mode != DisplayMode.Beams) return;
        if (sender is not Line { Tag: BeamModel beam }) return;

        // 前回の選択を解除
        ClearHighlight();
        if (_selectedBeam != null && _beamVis.TryGetValue(_selectedBeam, out var prev))
            prev.Stroke = _selectedBeam.Type == BeamType.大梁 ? MainNormal : SubNormal;

        _selectedBeam   = beam;
        _selectedColumn = null;
        if (_beamVis.TryGetValue(beam, out var sel))
            sel.Stroke = beam.Type == BeamType.大梁 ? MainSel : SubSel;

        // 負担面積ポリゴンをハイライト表示
        foreach (var region in beam.TributaryPolygons)
        {
            var poly = new System.Windows.Shapes.Polygon
            {
                Fill            = new SolidColorBrush(Color.FromArgb(100, 100, 255, 100)),
                Stroke          = new SolidColorBrush(Color.FromArgb(200, 100, 255, 150)),
                StrokeThickness = 1.5,
                IsHitTestVisible = false,
            };
            foreach (var v in region)
                poly.Points.Add(ToCanvas(v.X, v.Y));
            DrawingCanvas.Children.Add(poly);
            _highlightPolys.Add(poly);
        }

        ShowBeamInfo(beam);
        e.Handled = true;
    }

    /// <summary>負担面積ハイライトをクリア</summary>
    private void ClearHighlight()
    {
        foreach (var p in _highlightPolys)
            DrawingCanvas.Children.Remove(p);
        _highlightPolys.Clear();
    }

    private void OnColClick(object sender, MouseButtonEventArgs e)
    {
        if (_mode != DisplayMode.Columns) return;
        if (sender is not Ellipse { Tag: ColumnModel col }) return;

        ClearHighlight();
        if (_selectedColumn != null && _colVis.TryGetValue(_selectedColumn, out var prev))
            prev.Fill = ColNormal;

        _selectedColumn = col;
        _selectedBeam   = null;
        if (_colVis.TryGetValue(col, out var sel)) sel.Fill = ColSel;

        // ボロノイ領域をハイライト表示
        if (col.VoronoiPolygon != null && col.VoronoiPolygon.Count >= 3)
        {
            var poly = new System.Windows.Shapes.Polygon
            {
                Fill             = new SolidColorBrush(Color.FromArgb(100, 100, 180, 255)),
                Stroke           = new SolidColorBrush(Color.FromArgb(200, 100, 180, 255)),
                StrokeThickness  = 1.5,
                IsHitTestVisible = false,
            };
            foreach (var v in col.VoronoiPolygon)
                poly.Points.Add(ToCanvas(v.X, v.Y));
            DrawingCanvas.Children.Add(poly);
            _highlightPolys.Add(poly);
        }

        ShowColumnInfo(col);
        e.Handled = true;
    }

    // ─── 情報パネル ──────────────────────────────────────────

    private void ShowBeamInfo(BeamModel beam)
    {
        double k    = beam.Type == BeamType.大梁 ? GetConst(MainBeamConstBox) : GetConst(SubBeamConstBox);
        double load = beam.TributaryArea * k;

        MemberInfoPanel.Children.Clear();
        InfoRow("種別",       beam.Type == BeamType.大梁 ? "大梁" : "小梁");
        InfoRow("長さ",      $"{beam.Length:F3} m");
        InfoRow("始点 X",    $"{beam.StartPoint.X:F3} m");
        InfoRow("始点 Y",    $"{beam.StartPoint.Y:F3} m");
        InfoRow("終点 X",    $"{beam.EndPoint.X:F3} m");
        InfoRow("終点 Y",    $"{beam.EndPoint.Y:F3} m");
        InfoRow("支配面積",  $"{beam.TributaryArea:F2} m²");
        InfoRow("荷重定数",  $"{k:F2} kN/m²");
        InfoSep();
        string label = beam.Type == BeamType.大梁 ? "大梁積載荷重" : "小梁積載荷重";
        InfoRow(label, $"{load:F2} kN",
                valueColor: Color.FromRgb(100, 255, 150), bold: true);
    }

    private void ShowColumnInfo(ColumnModel col)
    {
        double k    = GetConst(ColumnConstBox);
        double load = col.TributaryArea * k;

        MemberInfoPanel.Children.Clear();
        InfoRow("種別",      "柱");
        InfoRow("中心 X",   $"{col.Center.X:F3} m");
        InfoRow("中心 Y",   $"{col.Center.Y:F3} m");
        InfoRow("半径",     $"{col.Radius:F3} m");
        InfoRow("支配面積", $"{col.TributaryArea:F2} m²");
        InfoRow("荷重定数", $"{k:F2} kN/m²");
        InfoSep();
        InfoRow("柱積載荷重", $"{load:F2} kN",
                valueColor: Color.FromRgb(100, 255, 150), bold: true);
    }

    private void InfoRow(string label, string value,
        Color? valueColor = null, bool bold = false)
    {
        var g = new Grid { Margin = new Thickness(0, 2, 0, 2) };
        g.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
        g.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

        var lbl = new TextBlock
        {
            Text = label + "：",
            Foreground = new SolidColorBrush(Color.FromRgb(155, 155, 155)),
            FontSize = 11,
        };
        var val = new TextBlock
        {
            Text                = value,
            Foreground          = new SolidColorBrush(valueColor ?? Color.FromRgb(220, 220, 220)),
            FontSize            = 11,
            FontWeight          = bold ? FontWeights.Bold : FontWeights.Normal,
            HorizontalAlignment = HorizontalAlignment.Right,
        };
        Grid.SetColumn(lbl, 0); Grid.SetColumn(val, 1);
        g.Children.Add(lbl); g.Children.Add(val);
        MemberInfoPanel.Children.Add(g);
    }

    private void InfoSep() =>
        MemberInfoPanel.Children.Add(new Separator
        {
            Background = new SolidColorBrush(Color.FromRgb(80, 80, 80)),
            Margin     = new Thickness(0, 4, 0, 4),
        });

    // ─── 集計 ────────────────────────────────────────────────

    private void UpdateSummary()
    {
        SummaryPanel.Children.Clear();

        double seismicK = GetConst(SeismicConstBox);
        double mainK    = GetConst(MainBeamConstBox);
        double subK     = GetConst(SubBeamConstBox);
        double colK     = GetConst(ColumnConstBox);

        double totalSlab   = _slabs.Sum(s => s.Area);
        double seismicLoad = totalSlab * seismicK;
        double mainLoad    = _beams.Where(b => b.Type == BeamType.大梁).Sum(b => b.TributaryArea * mainK);
        double subLoad     = _beams.Where(b => b.Type == BeamType.小梁).Sum(b => b.TributaryArea * subK);
        double colLoad     = _columns.Sum(c => c.TributaryArea * colK);

        SumRow("スラブ総面積",       $"{totalSlab:F1} m²");
        SumRow("地震力積載荷重",     $"{seismicLoad:F1} kN",  Color.FromRgb(255, 220, 80));
        SumRow("大梁積載荷重 合計",  $"{mainLoad:F1} kN",     Color.FromRgb(255, 215,  0));
        SumRow("小梁積載荷重 合計",  $"{subLoad:F1} kN",      Color.FromRgb(255, 180, 60));
        SumRow("柱積載荷重 合計",    $"{colLoad:F1} kN",      Color.FromRgb( 60, 180, 255));
    }

    private void SumRow(string label, string value, Color? valueColor = null)
    {
        var g = new Grid { Margin = new Thickness(0, 2, 0, 2) };
        g.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
        g.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });

        var lbl = new TextBlock
        {
            Text = label + "：",
            Foreground = new SolidColorBrush(Color.FromRgb(155, 155, 155)),
            FontSize = 11,
        };
        var val = new TextBlock
        {
            Text                = value,
            Foreground          = new SolidColorBrush(valueColor ?? Color.FromRgb(220, 220, 220)),
            FontSize            = 11,
            HorizontalAlignment = HorizontalAlignment.Right,
        };
        Grid.SetColumn(lbl, 0); Grid.SetColumn(val, 1);
        g.Children.Add(lbl); g.Children.Add(val);
        SummaryPanel.Children.Add(g);
    }

    // ─── ボタン・ヘルパー ────────────────────────────────────

    private void RecalcButton_Click(object sender, RoutedEventArgs e)
    {
        Render(); // ラベルの荷重値も含めて再描画
    }

    private double GetConst(TextBox box)
        => double.TryParse(box.Text, out double v) && v > 0 ? v : 1.0;
}
