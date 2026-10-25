# 论点收窄清单（Claim-narrowing checklist）

配套数据：`revision` 分支（提交 `5b4be9b`）与 `revision_runs/` 下的 40 次运行。
本文档给出"原文 → 建议表述"的逐条替换，英文句子可直接粘贴进稿件。

## 0. 数据基线（写作时唯一可引用的事实）

| 事实 | 数值 | 来源 |
| --- | --- | --- |
| ESO 相对基础反步的改善（标称、周期扰动） | 0.338 m → 0.026 m（约 13 倍） | 四组消融 |
| ESO 在 ±50% 质量/惯量失配下 | 1.27e-2 – 3.99e-2 m（仅约 3 倍退化） | 失配实验 |
| 复合相对 ESO-only | 比值 0.903 – 0.907（约 9–10%），与失配、非线性、学习率均无关 | 失配 + 阻力 + 学习率 |
| 无扰动工况 | 四组均 ≤ 1.5e-4 m，组间差异不构成证据 | 四组消融 |
| RBF 权重范数 | 有 ESO：0.17–0.71；无 ESO：1.2–5.0（顶到投影半径） | 权重检查 |
| 约化模型有符号张力 | 全部工况为正，最小值 1.81 N（0.5 倍质量） | 全部运行 |
| 非周期随机扰动（5 种子） | 排序与周期扰动一致（20/20 严格单调）；ESO 改善 7.6 倍；复合/ESO = 0.942 ± 0.011（约 5.8%） | 随机扰动实验 |

**一句话结论：ESO 是性能提升的主要来源；RBF 提供约 6–10% 的一致改进（周期扰动 9.7%、随机扰动 5.8%），排序在两类扰动下不变，但 RBF 不具备失配/非线性特异性。**

---

## 1. 摘要（Abstract）

**原文**
> A radial basis function (RBF) neural network is then introduced to approximate unmodeled nonlinear dynamics online, while an extended state observer (ESO) estimates the remaining lumped disturbances for feedforward compensation.

**问题**："approximate unmodeled nonlinear dynamics" 是数据的反例（强阻力下 RBF 单独使用误差反而放大 4 倍）。

**建议**
> A radial basis function (RBF) network supplies an additional online compensation term in the payload channels, and an extended state observer (ESO), whose prediction model explicitly contains the RBF output, estimates the residual lumped disturbance for feedforward compensation.

**原文**
> Simulation results show that the combined RBF-ESO compensation improves both tracking accuracy and disturbance rejection. The ablation results demonstrate that integrating neural approximation and observer-based disturbance estimation within a backstepping structure provides a balanced and robust control strategy for multi-UAV cooperative payload transportation.

**问题**：把"复合更好"当作结论，但数据显示增益几乎全部来自 ESO。

**建议**
> Simulation results over a single common parameter set show that observer-based disturbance compensation accounts for most of the performance gain, whereas the RBF term provides a consistent but modest additional improvement of about 10% in payload position error. This marginal benefit remains nearly constant across the mismatch and unmodeled-nonlinearity levels tested, which bounds the role that can be attributed to the learning module.

---

## 2. 引言贡献段（Introduction, last paragraph）

**原文**
> The main contribution of this paper is a simulation-oriented ablation framework for evaluating the proposed composite controller.

**问题**：只说了"做了消融"，没说清楚结构上的贡献点，也没说明消融会给出什么强度的结论。

**建议**
> The main contribution of this paper is a controlled ablation framework together with the residual-consistent architecture it evaluates: the RBF output is placed inside the ESO prediction model, so that the observer estimates the residual that remains after learning rather than a separately defined disturbance. The framework fixes one common parameter set for all variants, reports the marginal value of each module, and states explicitly which conclusions the numerical evidence does and does not support.

---

## 3. 稳定性分析的 Remark

**原文**
> the RBF neural networks compensate for the major unknown nonlinearities, and the ESO further estimates and suppresses the remaining lumped disturbances online.

**问题**：与仿真证据冲突（RBF 权重在 ESO 存在时很小）。

**建议**
> the RBF term contributes a bounded approximation of part of the unknown nonlinearity, while the ESO absorbs the dominant part of the lumped disturbance. The analysis therefore guarantees boundedness of the closed-loop signals; it does not attribute a separable share of the compensation to either module.

---

## 4. 结果段（"complementary roles"）

**原文**
> This result suggests that the RBF network and ESO play complementary roles: the RBF module compensates for structured nonlinear uncertainty, while the ESO estimates the remaining lumped disturbance online.

**问题**：这是全文最容易被审稿人用数据反驳的一句。

**建议**
> The ablation quantifies this interaction rather than assuming it. Relative to plain backstepping, the ESO-only configuration reduces the steady-state payload position error by roughly an order of magnitude, whereas adding the RBF term on top of the ESO yields a further reduction of about 10% that is nearly constant across the tested conditions. The learned weights remain small when the ESO is active (channel-wise norm 0.17–0.71, versus 1.2–5.0 without the ESO), which indicates that the observer, rather than the network, carries most of the compensation in this architecture.

---

## 5. 图 5 的图注

**原文**
> The complete Backstepping+RBF+ESO structure gives the most balanced reduction of payload position and attitude errors under the matched disturbance model.

**建议**
> The ESO-based configurations dominate the reduction of payload errors under the matched disturbance model; the RBF term adds a small reduction, visible mainly as a lower residual oscillation amplitude.

---

## 6. 无扰动工况的描述

**原文**
> This indicates that even in the nominal case, the additional compensation modules help suppress the effects of nonlinear coupling and allocation-induced modeling errors.

**问题**：无扰动时四组都在 0.15 mm 以内，且 ESO-only（9.25e-5）实际比基础反步（3.08e-5）更差，这句话站不住。

**建议**
> In the nominal case all four configurations keep the payload position error below 1.5e-4 m, so the differences observed here are not statistically meaningful and are not interpreted as evidence for the compensation modules.

---

## 7. 结论

**原文**
> The results indicate that combining neural approximation with observer-based disturbance compensation improves tracking accuracy and disturbance rejection... The ablation results show that the RBF and ESO components complement the nominal backstepping controller, yielding improved robustness without sacrificing attitude-tracking performance.

**建议**
> The framework places the RBF output inside the ESO prediction model, so that the observer estimates the residual that remains after learning. A common-parameter ablation shows that the ESO is the dominant mechanism: it reduces the payload tracking error by roughly an order of magnitude under the tested disturbances and remains effective for mass and inertia mismatches of ±50% and for an unmodeled quadratic drag reaching about 74% of the payload weight. The RBF term adds a consistent improvement of about 10% that does not grow with mismatch or nonlinearity; the paper therefore does not claim a separable division of labour between learning and observation. The contribution is the consistent residual formulation, the conditional boundedness analysis, and quantitative evidence that bounds the role of each module.

---

## 8. 建议新增一段（放在结果小节开头）

> **What the ablation can and cannot show.** Because all four configurations share the same nominal controller, reference trajectory, allocation module and UAV-level controller, the differences reported below are attributable to the compensation terms rather than to separate tuning. Two limitations follow. First, the comparison is performed at a single operating point; the ordering of the configurations is not claimed to be universal. Second, a smaller steady-state error does not by itself identify which physical uncertainty each module compensates. The module-level interpretation is therefore supported only where the learned weights and the parameter sweep both point in the same direction.

这段"先声明边界"的写法，比事后被审稿人指出更有利。

---

## 9. R1-4（重复补偿/耦合）回复口径

> **Reviewer comment.** The RBF and ESO terms may compensate the same uncertainty, and the interaction between learning and observation is not analysed.
>
> **Response.** We agree that this is the central risk of the architecture, and we have made it the subject of a dedicated experiment rather than an assertion. The revised manuscript reports four configurations evaluated under one common parameter set, plus sweeps over mass/inertia mismatch (0.5–1.5×) and an unmodeled quadratic drag (up to ~74% of payload weight). Across these conditions the ESO-only configuration accounts for most of the error reduction, and the additional RBF term contributes a nearly constant ~10%. We also report the learned weight norms, which stay small (0.17–0.71) when the ESO is active and reach the projection radius only when it is not. Accordingly, the revised text no longer claims a division of labour between the two modules; it presents the residual-consistent formulation (RBF output inside the ESO prediction model) as the structural contribution and quantifies the marginal value of each term as the experimental contribution.

---

## 10. 禁用表述清单

| 不要写 | 原因 |
| --- | --- |
| "RBF approximates/learns the unmodeled dynamics"（作为已证事实） | 强阻力下 RBF 单独使用误差放大 4 倍 |
| "complementary roles / division of labour" | 与恒定 0.903 比值冲突 |
| "the composite achieves the best performance in all cases" | 无扰动工况 ESO-only 比基础反步差 |
| "improves both tracking accuracy and disturbance rejection"（不加限定） | 需要说明改进主要来自 ESO |
| 任何用旧代码/旧参数算出的百分比 | 新旧结果不可混用，必须来自 `revision` 分支的统一重跑 |
| "guarantees global stability" / "ensures zero steady-state error" | 定理只给条件性最终有界 |

---

## 11. R1-6 / R2-1（扰动形式过于特殊）回复口径

> **Reviewer comment (R1-6 / R2-1).** The disturbance used in the simulations is a fixed sum of four sinusoids, so the robustness evidence covers periodic disturbances only.
>
> **Response.** We added a non-periodic, band-limited random disturbance experiment without changing the controllers, the metric definitions, or any gain. The disturbance is generated as first-order low-pass filtered white noise (AR(1), bandwidth 1 Hz) with RMS 1.0 m/s² on translation and 0.3 rad/s² on attitude, three independent channels per block; the random disturbance replaces the periodic one rather than being superimposed, and the disabled path is bit-identical to the previous code. Four configurations × five fixed seeds (101/202/303/404/505) were run, 20/20 completed with no failures and no negative tensions. The ordering of the four configurations was unchanged in every seed—composite < ESO-only < RBF-only < plain backstepping—while the marginal gain of the RBF term decreased from 9.7% to 5.8% (composite/ESO-only 0.942 ± 0.011). We therefore state that the ranking is preserved under a more general disturbance class and that the marginal value of the learning term is condition-dependent in magnitude but consistent in sign; we do not claim statistical significance from five seeds.

需要在正文中同时说明的口径：上述回复及数值来自随机扰动 RMS 为 1.0 m/s² 的旧实验；新实验已将随机与周期扰动的平移 RMS 统一为 2.15 m/s²，
必须完成 20 个 case 的重跑并更新结果后，才能直接并列比较两张表的绝对误差。两类扰动的频谱和时间波形仍不同；且 AR(1) 低通是洛伦兹谱
（约 25% 方差在 1 Hz 以下，其余按 1/f² 衰减），不是理想带限信号，正文若写"band-limited"应补一句说明。

---

## 12. 数字引用规范
正文里出现的每个改善百分比，都应能追溯到 `revision_runs/` 下的原始日志，并在文中注明：
- 统一参数集（`load_lifting_controller_params.m` 默认值，未单独调参）
- 时间窗（最后 20% 时间）
- 指标定义（位置 RMS = `sqrt(mean(||e_p||^2))`）
- 运行次数与失败数

当前可直接引用的数字见本文档第 0 节。
