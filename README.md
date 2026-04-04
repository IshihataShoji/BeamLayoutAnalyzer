# BeamLayoutAnalyzer

AutoCAD 2026 用の梁伏図（フレーミングプラン）構造解析プラグインです。  
AutoCAD上のオブジェクトからスラブ・柱・梁を自動認識し、負担面積と積載荷重を算出します。

![.NET 8](https://img.shields.io/badge/.NET-8.0-blue)
![AutoCAD 2026](https://img.shields.io/badge/AutoCAD-2026-red)
![WPF](https://img.shields.io/badge/UI-WPF-purple)
![License](https://img.shields.io/badge/License-MIT-green)

## 機能

### 🏗️ ジオメトリ自動認識
- **閉じたポリライン** → スラブとして認識
- **円** → 柱として認識
- **線分** → 梁として認識

### 📐 大梁・小梁の自動判定
- 両端が柱に接続している梁 → **大梁**
- それ以外 → **小梁**

### 📊 負担面積の計算
- **梁**: 亀の甲分割（Tortoiseshell Method）による支配面積算出
  - 水平梁・垂直梁はセグメント分割 + 亀の甲公式を適用
  - 斜め梁はレイキャスティングによるフォールバック計算
- **柱**: ボロノイ分割（Voronoi Diagram）による支配面積算出

### 🖥️ WPF ビジュアルビューア
3つの表示タブを切り替えて解析結果を確認できます：

| タブ | 内容 |
|------|------|
| **地震力** | スラブ面積 × 地震力定数の荷重表示 |
| **大梁・小梁** | 梁ごとの亀の甲支配領域と積載荷重表示 |
| **柱** | 柱ごとのボロノイ領域と積載荷重表示 |

- 各部材をクリックすると、詳細情報（長さ、座標、支配面積、荷重）を右パネルに表示
- 積載荷重定数（kN/m²）はユーザーが自由に変更・再計算可能

## 動作環境

| 要件 | バージョン |
|------|------------|
| AutoCAD | 2026 |
| .NET | 8.0 (Windows) |
| Visual Studio | 2022 |
| プラットフォーム | x64 |

## プロジェクト構成

```
BeamLayoutAnalyzer/
├── Commands/
│   └── ReadFramingPlanCommand.cs    # FRAMINGPLAN コマンドのエントリポイント
├── Models/
│   ├── BeamModel.cs                 # 梁モデル（線分→梁データ変換）
│   ├── ColumnModel.cs               # 柱モデル（円→柱データ変換）
│   └── SlabModel.cs                 # スラブモデル（ポリライン→スラブデータ変換）
├── Analysis/
│   ├── TributaryAreaCalculator.cs   # 負担面積計算（亀の甲・ボロノイ）
│   ├── GeometryUtils.cs            # レイキャスト交差判定ユーティリティ
│   └── PolygonUtils.cs             # ポリゴン操作（面積計算・半平面クリッピング）
├── UI/
│   ├── FramingPlanWindow.xaml       # WPFウィンドウ定義（ダークテーマUI）
│   └── FramingPlanWindow.xaml.cs    # 描画ロジック・インタラクション処理
├── BeamLayoutAnalyzer.csproj        # プロジェクト設定
└── README.md
```

## 使い方

### 1. ビルド

Visual Studio 2022 でソリューションを開き、ビルドします。

```
ビルド出力: bin\Debug\net8.0-windows\BeamLayoutAnalyzer.dll
```

> **注意**: AutoCAD 2026 の DLL 参照パスがデフォルトで `C:\Program Files\Autodesk\AutoCAD 2026\` に設定されています。環境に合わせて `.csproj` を修正してください。

### 2. AutoCAD で読み込み

AutoCAD 2026 のコマンドラインで：

```
NETLOAD
```

→ `BeamLayoutAnalyzer.dll` を選択して読み込みます。

### 3. コマンド実行

```
FRAMINGPLAN
```

### 4. オブジェクト選択

図面上のスラブ（閉じたポリライン）、柱（円）、梁（線分）を選択して **Enter** を押します。  
全選択する場合は **Ctrl+A** → **Enter** でも可能です。

### 5. ビューアで解析

WPF ウィンドウが開き、以下の操作が可能です：

- タブ切替で表示モードを変更
- 部材をクリックして詳細情報を確認
- 積載荷重定数を変更して「再計算・更新」ボタンで再計算

## 積載荷重定数（デフォルト値）

| 用途 | デフォルト値 |
|------|-------------|
| 地震力用 | 0.30 kN/m² |
| 大梁用 | 0.65 kN/m² |
| 小梁用 | 0.90 kN/m² |
| 柱用 | 0.65 kN/m² |

## 計算手法

### 亀の甲分割（梁の負担面積）

梁をセグメントに分割し、各セグメントの負担面積を以下の公式で計算します：

- **L ≥ H の場合**（台形）: `(2L - H) × H / 4`
- **L < H の場合**（三角形）: `L² / 4`

ここで、L はセグメント長、H は隣接する平行梁またはスラブ境界までの距離です。

### ボロノイ分割（柱の負担面積）

各柱について、隣接する柱との垂直二等分線で Sutherland-Hodgman アルゴリズムを用いて半平面クリッピングを行い、スラブ内のボロノイセルを算出します。

## 座標系

AutoCAD 上の座標は **mm 単位** を想定しており、内部処理では自動的に **m 単位** に変換されます（1/1000 スケール）。

## ライセンス

MIT License
