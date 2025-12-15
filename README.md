# MPC-MATLAB-Examples
モデル予測制御のMATLAB実装例 (MATLAB implementations of Model Predictive Control (MPC))

このリポジトリでは、モデル予測制御（MPC）のMATLAB実装例を提供します。
This repository provides MATLAB implementation examples of Model Predictive Control (MPC).

## 概要 / Overview

モデル予測制御（MPC）は、システムのモデルを使用して将来の挙動を予測し、制約条件を満たしながら最適な制御入力を計算する先進的な制御手法です。

Model Predictive Control (MPC) is an advanced control method that uses a system model to predict future behavior and calculates optimal control inputs while satisfying constraints.

## 必要な環境 / Requirements

- MATLAB R2018b or later
- Optimization Toolbox (quadprog, fmincon を使用)

## 実装例 / Examples

### 1. 線形MPC / Linear MPC (`linear_mpc_example.m`)

**システム**: 二重積分器（Double Integrator）

**特徴**:
- 基本的な線形MPCの実装
- 二次コスト関数
- 入力制約
- 状態空間モデル

**実行方法**:
```matlab
linear_mpc_example
```

**説明**:
線形離散時間システムに対する基本的なMPCを実装しています。最適化問題は二次計画法（QP）として定式化され、`quadprog`を用いて解かれます。

### 2. 制約付きMPC / Constrained MPC (`constrained_mpc_example.m`)

**システム**: 質量-バネ-ダンパー系（Mass-Spring-Damper System）

**特徴**:
- 入力制約と状態制約
- 連続時間システムの離散化
- 位相平面での可視化
- DAREによる終端コスト設定

**実行方法**:
```matlab
constrained_mpc_example
```

**説明**:
制約を明示的に扱うMPCの実装例です。状態と入力の両方に制約を設定し、実行可能な解を求めます。

### 3. 追従MPC / Tracking MPC (`tracking_mpc_example.m`)

**システム**: 台車上の倒立振子（線形化）（Linearized Inverted Pendulum on Cart）

**特徴**:
- 時変参照軌道の追従
- 正弦波軌道の追跡
- 追従誤差の評価
- 複雑なシステムの制御

**実行方法**:
```matlab
tracking_mpc_example
```

**説明**:
時間変化する参照軌道を追従するMPCを実装しています。倒立振子を直立に保ちながら、台車を目標軌道に追従させます。

### 4. 非線形MPC / Nonlinear MPC (`nonlinear_mpc_example.m`)

**システム**: ファン・デル・ポール振動子（Van der Pol Oscillator）

**特徴**:
- 非線形システムの直接制御
- 逐次二次計画法（SQP）による最適化
- RK4積分法によるシミュレーション
- ウォームスタート機能

**実行方法**:
```matlab
nonlinear_mpc_example
```

**説明**:
非線形システムに対するMPCを`fmincon`を用いて実装しています。非線形制約と非線形ダイナミクスを直接扱います。

## MPCの理論 / MPC Theory

### 基本的な定式化 / Basic Formulation

MPCは以下の最適化問題を各時刻で解きます：

```
Minimize: Σ(k=0 to N-1) [||x(k) - x_ref||²_Q + ||u(k)||²_R] + ||x(N) - x_ref||²_P

Subject to:
  x(k+1) = Ax(k) + Bu(k)  (システムダイナミクス)
  u_min ≤ u(k) ≤ u_max    (入力制約)
  x_min ≤ x(k) ≤ x_max    (状態制約)
```

ここで：
- `N`: 予測ホライズン (Prediction Horizon)
- `Q`: 状態重み行列 (State Weight Matrix)
- `R`: 入力重み行列 (Input Weight Matrix)
- `P`: 終端コスト行列 (Terminal Cost Matrix)
- `x_ref`: 参照状態 (Reference State)

### パラメータの選択 / Parameter Selection

**予測ホライズン N**:
- 長いほど性能向上、計算コスト増加
- システムの時定数の2-3倍程度が目安

**重み行列 Q, R**:
- `Q`を大きく: 状態誤差を重視
- `R`を大きく: 制御入力の変動を抑制
- トレードオフの調整が重要

**終端コスト P**:
- 安定性保証のため、DARE（離散代数リカッチ方程式）の解を使用
- `P = dare(A, B, Q, R)`

## 使用上の注意 / Notes

1. **最適化ツールボックス**: これらの例は MATLAB Optimization Toolbox を必要とします
2. **計算時間**: 予測ホライズンが長いと計算時間が増加します
3. **実時間性**: 実際のシステムに適用する際は計算時間を考慮してください
4. **初期化**: 非線形MPCでは良い初期推定値が収束性能に影響します

## 参考文献 / References

1. J. B. Rawlings, D. Q. Mayne, and M. M. Diehl, "Model Predictive Control: Theory, Computation, and Design," 2nd Edition, 2017
2. E. F. Camacho and C. Bordons, "Model Predictive Control," 2nd Edition, Springer, 2007
3. L. Wang, "Model Predictive Control System Design and Implementation Using MATLAB," Springer, 2009

## ライセンス / License

MIT License - 詳細は [LICENSE](LICENSE) ファイルを参照してください。

## 著者 / Author

Guang-Ze Yang

## 貢献 / Contributing

改善提案やバグ報告は Issue または Pull Request でお願いします。

Contributions, improvements, and bug reports are welcome via Issues or Pull Requests.
