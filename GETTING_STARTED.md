# Getting Started with MPC-MATLAB-Examples

## はじめに / Introduction

このガイドでは、MPC-MATLAB-Examplesの各例題を実行し、モデル予測制御（MPC）の基本を学ぶ方法を説明します。

This guide explains how to run the examples in MPC-MATLAB-Examples and learn the basics of Model Predictive Control (MPC).

## クイックスタート / Quick Start

### 1. 必要なソフトウェア / Required Software

- MATLAB R2018b以降 (R2018b or later)
- Optimization Toolbox

### 2. リポジトリのクローン / Clone Repository

```bash
git clone https://github.com/youkoutaku/MPC-MATLAB-Examples.git
cd MPC-MATLAB-Examples
```

### 3. MATLABで実行 / Run in MATLAB

MATLABを起動し、リポジトリディレクトリに移動してください。

Start MATLAB and navigate to the repository directory.

## 推奨学習順序 / Recommended Learning Order

### レベル1: 基礎 / Level 1: Basics

#### 例1: `simple_mpc_example.m`
最もシンプルな一次システムのMPC実装です。ユーティリティ関数の使い方も学べます。

The simplest MPC implementation for a first-order system. Also demonstrates utility function usage.

```matlab
>> simple_mpc_example
```

**学習内容 / Learning Points:**
- MPCの基本構造
- 予測ホライズンの概念
- 重み行列の役割
- 制約の扱い方

#### 例2: `linear_mpc_example.m`
二重積分器に対する線形MPCの実装です。

Linear MPC implementation for a double integrator.

```matlab
>> linear_mpc_example
```

**学習内容 / Learning Points:**
- 二次計画法（QP）の定式化
- 状態空間モデル
- 最適化問題の構築

### レベル2: 制約とトラッキング / Level 2: Constraints and Tracking

#### 例3: `constrained_mpc_example.m`
質量-バネ-ダンパー系に対する制約付きMPCです。

Constrained MPC for a mass-spring-damper system.

```matlab
>> constrained_mpc_example
```

**学習内容 / Learning Points:**
- 入力制約の実装
- 状態制約の実装
- 連続系の離散化
- 位相平面での解析

#### 例4: `tracking_mpc_example.m`
倒立振子の軌道追従制御です。

Trajectory tracking control for an inverted pendulum.

```matlab
>> tracking_mpc_example
```

**学習内容 / Learning Points:**
- 時変参照軌道の追従
- 複雑なシステムの制御
- 追従誤差の評価
- 安定化と追従の同時達成

### レベル3: 高度な技術 / Level 3: Advanced Techniques

#### 例5: `nonlinear_mpc_example.m`
非線形システム（ファン・デル・ポール振動子）の制御です。

Control of a nonlinear system (Van der Pol oscillator).

```matlab
>> nonlinear_mpc_example
```

**学習内容 / Learning Points:**
- 非線形MPC (NMPC)
- 逐次二次計画法（SQP）
- RK4積分法
- ウォームスタート

#### 例6: `offset_free_mpc_example.m`
外乱推定によるオフセットフリー追従制御です。

Offset-free tracking with disturbance estimation.

```matlab
>> offset_free_mpc_example
```

**学習内容 / Learning Points:**
- 状態拡大法
- 外乱オブザーバ
- オフセットフリー制御
- 標準MPCとの比較

## パラメータ調整のヒント / Parameter Tuning Tips

### 予測ホライズン N / Prediction Horizon N

```matlab
N = 10;  % 短い → 計算速い、性能低い
N = 20;  % 中間
N = 50;  % 長い → 計算遅い、性能高い
```

**推奨値 / Recommended:**
- システムの整定時間の1.5〜3倍
- 1.5-3 times the system settling time

### 状態重み行列 Q / State Weight Matrix Q

```matlab
Q = diag([100, 1]);  % 第1状態を重視
Q = diag([1, 100]);  % 第2状態を重視
```

**調整方法 / Tuning Method:**
1. 単位行列から始める (Start with identity matrix)
2. 重視したい状態の重みを増やす (Increase weights for important states)
3. シミュレーションで確認 (Verify through simulation)

### 入力重み R / Input Weight R

```matlab
R = 0.1;   % 小さい → 積極的な制御
R = 1.0;   % 中間
R = 10.0;  % 大きい → 穏やかな制御
```

**効果 / Effect:**
- 小さいR: 速い応答、大きな制御入力
- 大きいR: 緩やかな応答、小さな制御入力
- Small R: Fast response, large control effort
- Large R: Slow response, small control effort

## トラブルシューティング / Troubleshooting

### 問題1: 最適化が失敗する / Problem 1: Optimization Fails

```matlab
Warning: MPC solver failed at step XX
```

**対処法 / Solution:**
- 制約を緩める (Relax constraints)
- 予測ホライズンを短くする (Reduce prediction horizon)
- 初期状態が実行可能か確認 (Check if initial state is feasible)

### 問題2: システムが不安定 / Problem 2: System is Unstable

**対処法 / Solution:**
- Qを大きくする (Increase Q)
- Rを小さくする (Decrease R)
- 終端コストPを設定 (Set terminal cost P using DARE)
- システムの可制御性を確認 (Check system controllability)

### 問題3: 計算が遅い / Problem 3: Computation is Slow

**対処法 / Solution:**
- 予測ホライズンNを減らす (Reduce prediction horizon N)
- 状態次元を減らす (Reduce state dimension if possible)
- ウォームスタートを使用 (Use warm start)
- より高速な最適化ソルバーを使用 (Use faster optimization solver)

## ユーティリティ関数の使い方 / Using Utility Functions

`mpc_utils.m`には便利な関数が含まれています。

The file `mpc_utils.m` contains useful utility functions.

### 例: カスタムMPCの実装 / Example: Custom MPC Implementation

```matlab
% システム定義 / System definition
A = [1 0.1; 0 0.9];
B = [0.005; 0.1];
N = 10;

% リフティング行列の構築 / Build lifted matrices
[Sx, Su] = build_lifted_matrices(A, B, N);

% 終端コストの計算 / Compute terminal cost
P = compute_terminal_cost(A, B, Q, R);

% 安定性チェック / Stability check
is_stable = check_mpc_stability(A, B, Q, R);

% 性能評価 / Performance evaluation
metrics = compute_mpc_metrics(x_history, u_history, x_ref);

% 結果のプロット / Plot results
plot_mpc_results(t, x_history, u_history, x_ref, u_min, u_max);
```

## さらに学ぶために / Further Learning

### 推奨文献 / Recommended Books

1. J. B. Rawlings, et al., "Model Predictive Control: Theory, Computation, and Design" (2017)
2. E. F. Camacho and C. Bordons, "Model Predictive Control" (2007)
3. L. Wang, "Model Predictive Control System Design and Implementation Using MATLAB" (2009)

### オンラインリソース / Online Resources

- MathWorks MPC Toolbox Documentation
- Control Tutorials for MATLAB & Simulink
- MPC研究会 / MPC Research Society

## サポート / Support

問題や質問がある場合は、GitHubのIssueでお知らせください。

If you have any problems or questions, please open an issue on GitHub.

## 貢献 / Contributing

改善提案やバグ報告は歓迎します！Pull Requestをお送りください。

Improvements and bug reports are welcome! Please send a Pull Request.
