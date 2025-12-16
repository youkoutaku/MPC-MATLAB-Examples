# モデル予測制御（MPC）実装集

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)

> **[English README](README_EN.md)** | **[日本語ドキュメント](README.md)**

MATLABによるモデル予測制御（Model Predictive Control, MPC）の実装例集です。二重積分系を対象とした軌道追従制御、制約付きMPC、LQRとの性能比較などの実装を提供します。

## 目次

- [モデル予測制御（MPC）実装集](#モデル予測制御mpc実装集)
  - [目次](#目次)
  - [概要](#概要)
  - [主な特徴](#主な特徴)
  - [動作環境](#動作環境)
    - [必須要件](#必須要件)
    - [オプション](#オプション)
    - [インストール](#インストール)
  - [クイックスタート](#クイックスタート)
    - [最も簡単な例：軌道追従MPC](#最も簡単な例軌道追従mpc)
    - [制約付きMPC（推奨）](#制約付きmpc推奨)
    - [MPC vs LQR 比較](#mpc-vs-lqr-比較)
  - [参考文献](#参考文献)
  - [報告・質問](#報告質問)
  - [ライセンス](#ライセンス)
  - [著者](#著者)

---

## 概要

本リポジトリは、モデル予測制御（MPC）の理論と実装を学ぶための教育用MATLABコードです。二重積分系（位置・速度系）を対象システムとし、軌道追従制御、制約付きMPC、MATLABツールボックスとの比較、LQRとの性能比較など、多角的な検証を行っています。

## 主な特徴

- **手動QP実装** - 二次計画法（QP）による完全な手動MPC実装
- **制約対応** - 入力、状態、変化率制約を考慮した制約付きMPC
- **ツールボックス検証** - MATLAB MPC Toolboxとの比較検証
- **性能比較** - MPC vs LQRの定量的性能評価
- **動的軌道追従** - 正弦波参照軌道による時変追従制御
- **詳細ドキュメント** - 理論背景と実装の完全な日本語解説

---

## 動作環境

### 必須要件

- **MATLAB** R2020b 以降
- **Optimization Toolbox**（`quadprog` 関数使用）

### オプション

- **Control System Toolbox**（システム解析・LQR比較用）
- **MPC Toolbox**（`MPC_Toolbox.m` 実行時のみ）

### インストール

```bash
# リポジトリのクローン
git clone https://github.com/youkoutaku/MPC-MATLAB-Examples.git
cd MPC-MATLAB-Examples
```

MATLABを起動し、プロジェクトフォルダに移動してください：

```matlab
cd('path/to/MPC-MATLAB-Examples')
```

すべての関数をPATHに追加してください：
```matlab
addpath('lib')
addpath('test')
```

---

## クイックスタート

### 最も簡単な例：軌道追従MPC

```matlab
% プロジェクトフォルダで実行
addpath('lib')  % 補助関数をパスに追加
run('MPC_trajectory_main.m')
```

**実行結果：** 正弦波参照軌道を追従する位置・速度プロットが表示されます。

### 制約付きMPC（推奨）

```matlab
% 制約付きMPCの実行
addpath('lib')  % 初回のみ
run('MPC_con_main.m')

% 制約充足性の検証
run('test/Check_Constraints.m')
```

**実行結果：** 入力・速度・位置制約を考慮した制御結果と制約活性化率が表示されます。

### MPC vs LQR 比較

```matlab
run('MPCvsLQR_QP_main.m')
```

**実行結果：** 両手法の軌道追従性能が並列比較されます。

---

## 参考文献

- [ドキュメント](docs/docs_jp.pdf)
- Wang, L. (2009). *Model Predictive Control System Design and Implementation Using MATLAB*. Springer.
- Camacho, E. F., & Bordons, C. (2007). *Model Predictive Control*. Springer.
- 大塚 敏之, 非線形最適制御入門，コロナ社, 2011.
- 王天威. 控制之美(卷2). 清华大学出版社,2023.

---

## 報告・質問

- **Issue**: [GitHub Issues](https://github.com/youkoutaku/MPC-MATLAB-Examples/issues)
- **Email**: 質問や改善提案をお気軽にお送りください

---

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルをご覧ください。

---

## 著者

**Youkoutaku**

- GitHub: [@Youkoutaku](https://github.com/Youkoutaku)
- Website: [https://youkoutaku.github.io/](https://youkoutaku.github.io/)

---

<div align="center">

**⭐ このリポジトリが役に立ったら、スターをお願いします！ ⭐**

最終更新日：2025年12月15日

</div>
