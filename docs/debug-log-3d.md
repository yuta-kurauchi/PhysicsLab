# PhysicsLab3D Debug Log

---

## Phase 3D-A:View行列の転置忘れ

### 概要
View行列実装時にカメラの基底列ベクトルで構成される行列の転置を取り忘れた。ことで、worlc -> view変換になっていなかった。
View変換はカメラの回転行列の逆変換なので正規直交基底を列に並べた行列を用いる場合は、逆変換は転置となる。

### 原因
行列の列優先管理を忘れていた。

### 修正
列優先管理に従い、転置後の行列になるように実装し直した。

### 結果
しっかりとworld -> viewの変換ができるようになった。

### 注意事項
行列へのアクセスは列優先で以下のような行う。
`m[col][row]`

---

## Phase 3D-A：MVP変換の数値検証

### 目的
Model・View・Perspective行列を組み合わせ、
Local座標からNDC座標への変換を確認する。

### 検証条件と結果

条件1：
- Model：Z軸90°回転、平行移動(1, 0, -3)
- Camera：(0, 0, 5)、前方-Z
- FOV：60°、Aspect：4/3
- Near：0.1、Far：100
- Local：(1, 0, 0)

結果：
- World：(1, 1, -3)
- View：(1, 1, -8)
- NDC：(0.16238, 0.216506, 0.988488)

条件2：
- FOV：90°、Aspect：1
- Near：1、Far：9
- View：(3, 4, -5)

結果：
- NDC：(0.6, 0.8, 0.9)

### 結論
- いずれも理論値と一致した。
- 逐次変換とPVM合成変換の結果が一致した。
- CPU側の基本的なMVP変換を確認できた。
- GPU・Shaderとの接続は未検証。

---

## Phase 3D-A：GPU描画におけるMVP変換の検証

### 目的
CPU側で計算したMVP行列をGPUへ転送し、Vertex Shaderによる座標変換と実際の描画結果の整合性を確認する。

### 基本描画検証
- Model行列による平行移動・回転
- Cameraの位置変更による描画位置・大きさの変化
- Cameraの向き変更によるView変換

いずれも予測どおりの描画結果となった。

### CPU・GPU間の整合性検証

検証条件：
- Model：Z軸90°回転後、平行移動(0.5, 0, 0)
- Camera位置：(0, 0, 3)
- Camera方向：(0.5, 0, -1)
- WorldUp：(0, 1, 0)
- FOV：60°
- Aspect：4/3
- Near：0.1、Far：100
- Screen：800 × 600

CPU側でVertex Bufferに使用する頂点データからClip座標を計算し、透視除算およびViewport変換によってScreen座標を求めた。

結果：
- Red：(325.769, 382.992)
- Green：(325.769, 約217.0)
- Blue：(140.192, 300.000)

CPUで計算した頂点位置とGPUによる描画結果が目視で整合することを確認した。

### 結論
- Model・View・Projection変換がGPU描画に正しく反映されることを確認した。
- CPU側の列優先行列とVertex Shader側の行列の扱いについて、今回の検証条件で整合性を確認できた。
- macOSで検証を実施した。Windowsでの最新実装の動作は未確認。

---

## Phase 3D-A：Depth Buffer・深度テストの検証

### 目的
Depth Bufferを導入し、描画順序に依存せず、3D空間における物体の前後関係が正しく描画されることを確認する。

### 実装内容
- Depth Textureの生成とRAIIによるリソース管理
- D32_FLOATフォーマットの採用
- SDL_GPUTextureSupportsFormatによる対応状況の確認
- Graphics Pipelineでの深度テスト・深度書き込みの有効化
- Render PassへのDepth Targetの接続
- Render Pass開始時に深度値を1.0へクリア

### 検証条件
- Red：Z = 0.5
- Blue：Z = -0.5
- Camera位置：(0, 0, 3)
- Camera方向：(0, 0, -1)
- Model：単位行列
- 深度比較方法：LESS
- 同一Render Pass内で2回のDraw Callを発行

### 検証結果

| Depth Test | 描画順序 | 重なった部分 |
|---|---|---|
| OFF | Red → Blue | Blue |
| OFF | Blue → Red | Red |
| ON | Red → Blue | Red |
| ON | Blue → Red | Red |

すべて予測どおりの描画結果となった。

### 結論
Depth Testが無効な場合は描画順序に依存するが、有効な場合は描画順序にかかわらず、手前の三角形が表示されることを確認した。

### 確認環境・制限
- macOSで動作確認済み。
- 現在の環境ではD32_FLOATに対応し、D24_UNORMには非対応。
- Depth Formatのフォールバック処理は未実装。
- Windowsでの動作は未確認。