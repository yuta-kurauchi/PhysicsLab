# PhysicsLab

PhysicsLabは、2D剛体の運動・衝突・接触を学び、数式に基づく物理計算をC++で実装・検証する個人制作プロジェクトです。

現在は、円・矩形・三角形の並進運動と回転運動、衝突判定、接触点生成、力積による衝突応答などを実装しています。SDL3を使った実験用アプリケーション（Sandbox）で挙動を観察できます。

本プロジェクトは学習・実験を目的としており、商用物理エンジンと同等の精度・安定性・性能を保証するものではありません。現段階では2D物理モジュールと実験用アプリケーションを同じ実行ファイルにビルドします。

## 主な実装機能

- **形状と運動**：Circle、凸Polygon（現在のデモではRectangle／Triangle）、重力、並進・回転運動、質量・慣性モーメント
- **時間更新**：accumulatorを使った固定時間刻み（デモでは1/60秒）
- **衝突判定**：Circle–Circle、Circle–Polygon、Polygon–Polygon（SAT）
- **接触点生成**：Polygon–Polygonでreference／incident edgeとclippingを使用。複数接触点を`ContactManifold`で保持
- **衝突応答**：回転を考慮したSequential Impulse Solver、各接触点の累積法線力積・累積接線力積、Coulomb摩擦
- **接触の安定化**：反発目標速度、Restitution Threshold、Penetration Slopと補正率による位置補正
- **静的境界**：画面の上下左右の壁・床を静的Bodyとして扱い、共通の接触Solverで処理
- **デバッグ**：物理ステップ終了時のBody主要状態に対するNaN／Infチェック（コード中のフラグで切り替え）

力積の累積は各物理ステップの反復処理内で行われます。前のステップの力積を再利用するwarm startingは実装していません。

## デモシーン

| シーン | 内容 |
| --- | --- |
| `Random` | 円3個、矩形3個、三角形2個。固定seedで初期条件を生成 |
| `SingleBox` | 矩形1個を静的床へ落下させる |
| `Stacking` | 矩形3個の積み重ねを観察する |

現在の標準シーンは`Random`で、乱数seedは`12345`です。ウィンドウサイズは800×600です。

`Random`では`std::mt19937`と`std::uniform_real_distribution<float>`を使用しています。同じseedでも、macOSとWindowsで初期配置が一致しないことを確認しています。`mt19937`の疑似乱数列とは別に、実数分布への変換結果は標準ライブラリの実装によって異なる場合があります。このため、現在の固定seedは同一環境での条件再現を目的としており、OSをまたいだ完全に同一の初期条件は保証していません。

シーンは実行中に切り替えられません。変更するときは`sandbox/main.cpp`の`ACTIVE_SCENE`を、次のいずれかに編集してから再ビルドしてください。

```cpp
constexpr SceneType ACTIVE_SCENE = SceneType::Random;
// または SceneType::SingleBox / SceneType::Stacking
```

### 操作方法

- `Esc`：終了
- ウィンドウの閉じるボタン：終了

物体をマウスで動かしたり、キー操作でシーンを切り替えたりする機能は現時点ではありません。

## 使用技術・動作環境

- C++17
- CMake 3.15以上（ビルド設定の要求バージョン）
- SDL3 `release-3.4.16`（ウィンドウ・入力・描画）
- macOS：Apple Clangを使用し、構造整理後のコードをビルド・実行確認済み
- Windows：Visual Studio／MSVCを使用し、最新のmainを取得してビルド・実行確認済み

いずれも開発者の環境で確認した結果であり、すべてのOSバージョン・開発環境での動作を保証するものではありません。

初回CMake構成時にSDL3のソースコードをGitHubから取得するため、ネットワーク接続とGitが必要です。

## ビルド・実行方法

リポジトリのルートディレクトリで実行してください。

### macOS（ターミナル）

CMake、Git、C++17対応のコンパイラ（Apple Clangなど）を用意します。

```bash
cmake -S . -B build
cmake --build build --config Release
```

単一構成のMakefiles／Ninjaジェネレータを使用した場合、通常は次の場所に実行ファイルが生成されます。

```bash
./build/PhysicsLab
```

Xcodeなど複数構成のジェネレータを使用した場合は、実行ファイルの配置が異なります。また、単一構成ジェネレータでReleaseビルドを明示したい場合は、初回構成時に`-DCMAKE_BUILD_TYPE=Release`を指定してください。

### Windows（Visual Studio／MSVC）

Visual StudioのC++開発環境、CMake、Gitを用意します。Visual Studioの開発者コマンドプロンプトなど、CMakeからMSVCを利用できる環境で次を実行します。

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Visual StudioジェネレータでRelease構成をビルドした場合の典型的な実行例です。

```powershell
.\build\Release\PhysicsLab.exe
```

ジェネレータによって出力場所は変わります。CMakeではWindows用にSDL3のDLLをビルド後に実行ファイルと同じディレクトリへコピーする設定を設けています。

## ディレクトリ構成

```text
PhysicsLab/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── docs/
│   └── debug-log.md
├── engine/
│   ├── math/
│   │   └── Vec2.hpp
│   └── physics2d/
│       ├── PhysicsTypes.hpp
│       ├── World2D.hpp
│       └── World2D.cpp
└── sandbox/
    └── main.cpp
```

- `Vec2.hpp`：2Dベクトル演算
- `PhysicsTypes.hpp`：Body、形状、接触情報などのデータ型
- `World2D.hpp`／`World2D.cpp`：物理状態の管理と物理更新
- `sandbox/main.cpp`：SDL3の初期化・入力・描画、シーン生成、fixed timestepの時間管理
- `docs/debug-log.md`：開発過程の数値検証、挙動確認、既知の制限

### 物理更新の流れ

Sandboxのメインループが固定時間刻みで`World2D::step(dt)`を呼び出します。`step()`の処理はおおむね次の順です。

1. 加速度・速度・位置・角度を更新
2. 壁および物体間の衝突を判定し、`ContactManifold`を生成
3. 接触開始時の法線方向速度から反発目標速度を設定
4. Sequential Impulseで法線・接線の力積を反復計算
5. めり込みの位置補正
6. 必要に応じてBody状態の数値チェック

その後、Sandbox側でSDL3を使って描画します。デモのSolver反復回数は10回です。

## 検証内容と既知の制限

開発中に数値ログと描画を使って衝突応答・摩擦・回転・Solverの収束挙動などを確認しました。2Dの基本検証として、以下のV-01〜V-05を実施しています。

| 検証 | 確認内容 |
| --- | --- |
| V-01 | 単体矩形と静的床の接触 |
| V-02 | 矩形3個のStacking（シミュレーション時間60秒） |
| V-03 | 円・矩形・三角形の混在とBody状態のNaN／Infチェック |
| V-04 | 摩擦係数0と0.5の比較 |
| V-05 | Solver Iterationsを変えたStackingの比較 |

これらの基本検証では、想定した条件で大きな破綻がないことを確認しました。ただし、完全な静止や数値的な誤差ゼロは達成していません。

**既知の制限**

- 接触状態によって横方向のdrift、微小な回転、残留めり込みが生じることがある
- 接触点数の切り替わりに伴い、一時的な跳ね上がりや姿勢変化が生じることがある
- Polygon–Wallの接触候補を選ぶ許容差は暫定値
- Circle–Circleで中心がほぼ一致する場合、現在の衝突判定では扱わない
- Circle–Polygonの接触点は近似しており、深い内包などの条件は十分に扱えていない
- 転がり抵抗、warm starting、sleep、CCD、Broad Phaseの最適化などは未実装
- NaN／Infチェックはステップ末尾のBody主要状態を対象とし、すべての中間計算を検証するものではない
- 物理処理とSandboxを分離した後、代表3シーンは目視で回帰確認したが、整理前後の数値的一致は検証していない
- `Random`の固定seedは、macOSとWindowsで同一の初期配置を保証しない

詳細な条件・ログ・考察は[docs/debug-log.md](docs/debug-log.md)を参照してください。過去のログに記載された「今後の実装」は、その記録時点の予定を指す場合があります。

## 今後の開発方針

同じPhysicsLabリポジトリ内で、2Dの検証可能な状態を維持しつつ、3Dの描画・剛体運動・衝突処理へ段階的に進む方針です。現時点で3D物理は実装済み機能には含みません。

2Dの既知の制限についても、必要性を確認しながら改善を検討します。

## 依存ライブラリ・ライセンス

SDL3をウィンドウ・入力・描画に使用しています。CMakeの`FetchContent`によってSDL3の`release-3.4.16`を取得します。SDL3の配布条件については[SDL公式リポジトリ](https://github.com/libsdl-org/SDL)を参照してください。

PhysicsLabの自作コードについては、現時点で利用・再配布・改変を許可するライセンスを設定していません。公開する場合でも、ソースコードを閲覧できることと、第三者による自由な利用を許可することは別です。ライセンス方針は別途決定します。