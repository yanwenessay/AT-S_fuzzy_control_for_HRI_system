#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本文方法对比实验：等价末端阻抗 + AT-S 模糊补偿。

本脚本用于论文第一个对比实验：7关节正弦轨迹跟踪。
程序复用原始实验框架中的机械臂连接、力矩模式、数据记录和绘图，
控制核心为等价末端阻抗 + AT-S 自适应模糊补偿。
"""

import argparse
import os
import select
import sys
import time
import math
from typing import Tuple

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ATS_DIR = os.path.dirname(THIS_DIR)
EXAMPLES_DIR = os.path.dirname(ATS_DIR)
DEFAULT_BASELINE_DB = os.path.join(THIS_DIR, "baseline_tools", "baseline_db", "proposed_method_ats_baseline.npz")
for path in (ATS_DIR, EXAMPLES_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

from Kinematic_fcn import Kinematic, Ttrans7
from ats_imcontrol_sin_cos_jo import VariableImpedanceController
from DiscreteIntegrator import DiscreteIntegrator
from ts_fuzzy_output import ts_fuzzy_output


def damped_cartesian_inverse(jacobian: np.ndarray, damping: float) -> np.ndarray:
    jj_t = jacobian @ jacobian.T
    return np.linalg.pinv(jj_t + damping * damping * np.eye(6))


def damped_right_pseudoinverse(jacobian: np.ndarray, damping: float) -> np.ndarray:
    return jacobian.T @ damped_cartesian_inverse(jacobian, damping)


def weighted_external_torque(tau_e: np.ndarray, weights: np.ndarray) -> float:
    total = float(np.sum(weights))
    if total <= 1e-12:
        return float(np.mean(np.abs(tau_e)))
    return float(np.sum(weights * np.abs(tau_e)) / total)



def skew_symmetric(vector):
    vector = np.asarray(vector, dtype=float).reshape(3)
    return np.array(
        [[0.0, -vector[2], vector[1]],
         [vector[2], 0.0, -vector[0]],
         [-vector[1], vector[0], 0.0]],
        dtype=float,
    )


def left_jacobian_inv_so3(phi):
    """Inverse SO(3) left Jacobian used by analytical pose-error coordinates."""
    phi = np.asarray(phi, dtype=float).reshape(3)
    angle = float(np.linalg.norm(phi))
    phi_x = skew_symmetric(phi)
    if angle < 1e-6:
        return np.eye(3) - 0.5 * phi_x + (1.0 / 12.0) * (phi_x @ phi_x)
    gamma = (1.0 / (angle * angle)
             - (1.0 + np.cos(angle)) / (2.0 * angle * np.sin(angle)))
    return np.eye(3) - 0.5 * phi_x + gamma * (phi_x @ phi_x)

# ========================= AT-S调参区 =========================
# AT-S 补偿主要通过 u2 输出力矩。
# u2 = -1 / b2 * (...)，b2 越小输出越猛，实机上容易抖。
ATS_JOINT_GAIN = np.array([1.0, 3.0, 1.5, 2.0, 3.5, 2.5, 2.5])
ATS_K_MAX_BY_JOINT = [2] * 7
ATS_U2_CLIP = 100.0
ATS_U1_CLIP = 10.0
ATS_SMOOTH_SAT_SCALE = 2.0 / np.pi
ATS_SAT_SOFTNESS = 5

# 论文公式(45)-(47)的 AT-S 降权参数，值可以按实机调。
ATS_K_G = 1.0
ATS_TAU_TH = 0.4
ATS_K_ATS = 1.5
ATS_EXTERNAL_WEIGHTS = np.array([1.2, 1.0, 0.8, 0.6, 0.4, 0.3, 0.2])
ENDPOINT_IMPEDANCE_ALPHA_MIN = 0.15

# 增大 b2 可以降低 AT-S 力矩的陡度；原论文 b2=0.01，实机先保守一点。
ATS_B2_OVERRIDE = 0.01
ATS_B1_OVERRIDE = 0.1

# 暂时把外力阈值设高，避免无接触时误触发降低 AT-S 补偿。
EXTERNAL_DETECTION_THRESHOLDS = np.array([20.0, 25.0, 20.0, 22.0, 18.0, 18.0, 15.0])

# ======================= 动力学补偿调参区 =======================
# 对应论文 tau_i = M(q)qddot_r + C(q,qdot)qdot + F_g(q,qdot) - tau_e。
# 保守模型不是精确辨识模型，所以保留论文结构，同时允许实机微调比例。
DYNAMICS_INERTIA_GAIN = 1.0
DYNAMICS_CORIOLIS_GAIN = 1.0
DYNAMICS_GRAVITY_GAIN = 1.0
DYNAMICS_GRAVITY_SIGN = -1.0     # 当前实机下坠，先反转保守重力补偿方向。
DYNAMICS_FRICTION_GAIN = 1.0
DYNAMICS_EXTERNAL_TAU_GAIN = 1.0 # 对应公式里的 -tau_e。
# ===============================================================



class PaperATSController:
    """AT-S 模糊补偿器：保留论文的自适应结构，饱和函数用平滑形式降低实机抖动。"""

    def __init__(self, u1_init=0.0, u2_init=0.0, K1_init=0.1, K2_init=0.1,
                 w1=0.001, w2=0.001, dagger1_deg=1.0, dagger2_deg=1.0, b1=0.1, b2=0.01,
                 AF11_0=0.2, AF12_0=0.0, AF21_0=2.0, AF22_0=0.2,
                 K_max=1.2):
        self.u1_init = u1_init
        self.u2_init = u2_init
        self.K1_init = K1_init
        self.K2_init = K2_init
        self.w1 = w1
        self.w2 = w2
        self.K_max = K_max
        self.u1_prev = u1_init
        self.u2_prev = u2_init
        self.b1 = b1
        self.b2 = b2
        self.dagger1 = dagger1_deg * math.pi / 180.0
        self.dagger2 = dagger2_deg * math.pi / 180.0
        self.AF11_0 = AF11_0
        self.AF12_0 = AF12_0
        self.AF21_0 = AF21_0
        self.AF22_0 = AF22_0
        self.K1_integrator = DiscreteIntegrator(initial_condition=K1_init, gain=1.0)
        self.K2_integrator = DiscreteIntegrator(initial_condition=K2_init, gain=1.0)
        self.AF11 = 0.0
        self.AF12 = 0.0
        self.AF21 = 0.0
        self.AF22 = 0.0

    @staticmethod
    def sat(value, bound):
        # 使用平滑饱和，避免 sign/sat 硬切换造成力矩抖动。
        bound = max(float(bound), 1e-9)
        return ATS_SMOOTH_SAT_SCALE * math.atan(value / (ATS_SAT_SOFTNESS * bound))

    def adaptive_law(self, x1, x2):
        """匹配平滑饱和函数的自适应律：Kdot = gamma * |e * sat_rho(e)|。"""
        sat_x1 = self.sat(x1, self.dagger1)
        sat_x2 = self.sat(x2, self.dagger2)
        return self.w1 * abs(x1 * sat_x1), self.w2 * abs(x2 * sat_x2)

    def reset_adaptation(self):
        self.u1_prev = self.u1_init
        self.u2_prev = self.u2_init
        self.K1_integrator.reset(self.K1_init)
        self.K2_integrator.reset(self.K2_init)
        self.AF11 = self.AF12 = self.AF21 = self.AF22 = 0.0

    def _clip_adaptive_gains(self):
        self.K1_integrator.output[0] = float(np.clip(self.K1_integrator.output[0], self.K1_init, self.K_max))
        self.K2_integrator.output[0] = float(np.clip(self.K2_integrator.output[0], self.K2_init, self.K_max))
        self.K1_integrator.last_output[0] = self.K1_integrator.output[0]
        self.K2_integrator.last_output[0] = self.K2_integrator.output[0]

    def control_law(self, x1, x2):
        x_ts = [x1, x2, self.u1_prev, self.u2_prev]
        try:
            self.AF11, self.AF12, self.AF21, self.AF22 = ts_fuzzy_output(x_ts)
        except Exception as exc:
            print(f"AT-S fuzzy calculation error: {exc}")
            self.AF11 = self.AF12 = self.AF21 = self.AF22 = 0.0

        K1 = self.K1_integrator.output[0]
        K2 = self.K2_integrator.output[0]
        sat_x1 = self.sat(x1, self.dagger1)
        sat_x2 = self.sat(x2, self.dagger2)

        u1 = -1.0 / self.b1 * (
            (self.AF11_0 + self.AF11) * x1
            + (self.AF12_0 + self.AF12) * x2
            + K1 * sat_x1
        )
        u2 = -1.0 / self.b2 * (
            (self.AF21_0 + self.AF21) * x1
            + (self.AF22_0 + self.AF22) * x2
            + K2 * sat_x2
        )

        u1 = float(np.clip(u1, -ATS_U1_CLIP, ATS_U1_CLIP))
        u2 = float(np.clip(u2, -ATS_U2_CLIP, ATS_U2_CLIP))

        K1_dot, K2_dot = self.adaptive_law(x1, x2)
        self.K1_integrator.step(K1_dot)
        self.K2_integrator.step(K2_dot)
        self._clip_adaptive_gains()
        self.u1_prev = u1
        self.u2_prev = u2
        return u1, u2


class ProposedComparisonController(VariableImpedanceController):
    """论文对比实验控制器：7 关节正弦轨迹 + 等价末端阻抗 + AT-S 补偿。"""

    def __init__(self, router, router_real_time):
        super().__init__(router, router_real_time)

        # 论文表 I 的 AT-S 参数：显式覆盖父类旧参数，避免实验脚本和论文表不一致。
        self.axis_control_params = [
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.2, "AF12_0": 0.0, "AF21_0": 0.5, "AF22_0": 0.2},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.11,
             "w1": 0.001, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.2, "AF12_0": 0.0, "AF21_0": 0.2, "AF22_0": 0.1},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.6, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.6},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.3, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.5},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.1},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.1},
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.010, "w2": 0.001, "dagger1_deg": 0.1, "dagger2_deg": 1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 0.1, "AF22_0": 0.05},
        ]

        # 等价末端阻抗参数，对应论文中 M_X、D_X、K_X。
        # 实机上刚度不宜过大，否则人推机械臂时会明显顶人。
        self.M_X = np.diag([2.0, 2.0, 2.0, 0.25, 0.25, 0.25])
        self.D_X = np.diag([32.0, 32.0, 32.0, 3.8, 3.8, 3.8])
        self.K_X = np.diag([85.0, 85.0, 85.0, 5.0, 5.0, 5.0])
        self.jacobian_damping = 0.05

        # 对应论文公式(45)-(47)中 K_g、alpha(tau_e)、tau_th、K_ats 和 w_i。
        self.ats_joint_gain = ATS_JOINT_GAIN.copy()
        self.ats_k_g = ATS_K_G
        self.ats_tau_th = ATS_TAU_TH
        self.ats_k_ats = ATS_K_ATS
        self.external_torque_threshold = ATS_TAU_TH
        self.ats_decay_scale = ATS_K_ATS
        self.external_torque_weights = ATS_EXTERNAL_WEIGHTS.copy()
        self.endpoint_impedance_alpha_min = ENDPOINT_IMPEDANCE_ALPHA_MIN
        self._last_tau_e_w = 0.0
        self.external_torque_detector.detection_thresholds = EXTERNAL_DETECTION_THRESHOLDS.copy()
        self.ats_reset_release_threshold = 0.55
        self.ats_reset_min_interval = 1.0
        self.ats_reset_required_detection_increase = 3.0
        self._ats_contact_seen = False
        self._last_ats_reset_time = -1e9
        self._last_detection_count_sum = 0.0
        # 基线数据库接口：目前仅保留读取，不参与 AT-S 降权。
        self.baseline_external_threshold = np.array([1.0, 1.8, 1.0, 1.4, 0.7, 0.7, 0.6])
        self.baseline_external_scale = 2.0
        self.min_baseline_ats_scale = 0.35
        self._baseline_ats_scale_lpf = 1.0
        self.baseline_time = None
        self.baseline_tau_e = None
        self.baseline_path = None

        # 保守动力学估计：由 Gen3 7DoF URDF 的质量和惯量提炼而来。
        # 这里不追求精确辨识，只用于让论文中 tau_i 不是空的简化模型。
        # 力矩会被 tau_i_limit 限幅，防止保守模型过补偿。
        self.urdf_link_masses = np.array([1.3773, 1.1636, 1.1636, 0.9302, 0.6781, 0.6781, 0.5006])
        self.urdf_link_inertia_trace = np.array([
            0.00457 + 0.004831 + 0.001409,
            0.011088 + 0.001072 + 0.011255,
            0.010932 + 0.011127 + 0.001043,
            0.008147 + 0.000631 + 0.008316,
            0.001596 + 0.001607 + 0.000399,
            0.001641 + 0.000410 + 0.001641,
            0.000587 + 0.000369 + 0.000609,
        ]) / 3.0
        self.effective_link_radii = np.array([0.18, 0.24, 0.22, 0.18, 0.12, 0.09, 0.06])
        self.reflected_motor_inertia = np.array([0.045, 0.040, 0.032, 0.026, 0.018, 0.014, 0.010])
        self.inertia_min_diag = np.array([0.10, 0.09, 0.075, 0.055, 0.035, 0.025, 0.018])
        self.inertia_max_diag = np.array([0.45, 0.38, 0.30, 0.22, 0.14, 0.10, 0.07])
        self.viscous_friction = np.array([0.10, 0.12, 0.08, 0.075, 0.045, 0.035, 0.025])
        self.coulomb_friction = np.array([0.16, 0.20, 0.12, 0.10, 0.065, 0.045, 0.035])
        # 动力学补偿比例：保持论文结构，但把保守模型的权重集中到文件开头调参区。
        self.dyn_inertia_gain = DYNAMICS_INERTIA_GAIN
        self.dyn_coriolis_gain = DYNAMICS_CORIOLIS_GAIN
        self.dyn_gravity_gain = DYNAMICS_GRAVITY_GAIN
        self.dyn_gravity_sign = DYNAMICS_GRAVITY_SIGN
        self.dyn_friction_gain = DYNAMICS_FRICTION_GAIN
        self.dyn_external_tau_gain = DYNAMICS_EXTERNAL_TAU_GAIN
        self.gravity_gains = np.array([0.00, 1.20, 0.18, 0.85, 0.08, 0.25, 0.04])
        self.gravity_reference = self.q_center.copy()
        self.gravity_comp_limit = np.array([0.0, 1.6, 0.35, 1.2, 0.20, 0.45, 0.12])
        self.tau_i_limit = np.array([18.0, 18.0, 16.0, 14.0, 10.0, 8.0, 6.0])

        self._prev_current_jacobian = None
        self._prev_current_jacobian_time = None
        self._prev_desired_velocity = None
        self._prev_desired_velocity_time = None

        self.eef_error_history = []
        self.eef_wrench_history = []
        self.eef_accel_ref_history = []
        self.ats_coefficient_history = []
        self.tau_impedance_history = []
        self.tau_ats_history = []

        # 这里使用 Response1 内部的 AT-S 实现，方便对实机抖动做平滑处理。
        # 不改原工程 control_main.py，避免影响其他实验脚本。
        self.ats_controllers = []
        ats_k_max_by_joint = ATS_K_MAX_BY_JOINT
        for joint_idx, params in enumerate(self.axis_control_params):
            self.ats_controllers.append(PaperATSController(
                u1_init=params["u1_init"],
                u2_init=params["u2_init"],
                K1_init=params["K1_init"],
                K2_init=params["K2_init"],
                w1=params["w1"],
                w2=params["w2"],
                dagger1_deg=params["dagger1_deg"],
                dagger2_deg=params["dagger2_deg"],
                b1=params["b1"],
                b2=params["b2"],
                AF11_0=params["AF11_0"],
                AF12_0=params["AF12_0"],
                AF21_0=params["AF21_0"],
                AF22_0=params["AF22_0"],
                K_max=ats_k_max_by_joint[joint_idx],
            ))

        print("Response1 本文方法对比实验控制器已初始化")

    def _plot_angle_error_data(self, time_array, error_array, title, ylabel, filename, folder):
        """绘制误差曲线。"""
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink']
        fig, axes = plt.subplots(7, 1, figsize=(15, 14), sharex=True)
        fig.suptitle(title + ' (0.1 deg target boundary)', fontsize=16, fontweight='bold')
        error_degrees = np.degrees(error_array)
        for i in range(7):
            axes[i].plot(time_array, error_degrees[:, i], color=colors[i], linewidth=1.5, label=f'Joint {i+1}')
            axes[i].axhline(y=0.0, color='black', linestyle='-', alpha=0.35, linewidth=1)
            axes[i].axhline(y=0.1, color='red', linestyle='--', alpha=0.8, linewidth=1.4, label='+/-0.1 deg')
            axes[i].axhline(y=-0.1, color='red', linestyle='--', alpha=0.8, linewidth=1.4)
            axes[i].fill_between(time_array, -0.1, 0.1, color='green', alpha=0.10)
            axes[i].set_ylabel(f'J{i+1} (deg)')
            axes[i].grid(True, alpha=0.3)
            max_abs = float(np.max(np.abs(error_degrees[:, i]))) if len(error_degrees) else 0.0
            ylim = max(0.15, min(2.0, max_abs * 1.15))
            axes[i].set_ylim(-ylim, ylim)
            if i == 0:
                axes[i].legend(loc='upper right')
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(os.path.join(folder, filename), dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_improved_tracking_errors_with_fine_analysis(self, time_array, error_array, folder):
        """绘制 0.1 度误差带和统计指标。"""
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'brown', 'pink']
        fig, axes = plt.subplots(7, 1, figsize=(16, 14), sharex=True)
        fig.suptitle('Fine Tracking Errors Analysis: +/-0.1 deg Target Zone', fontsize=16, fontweight='bold')
        error_degrees = np.degrees(error_array)
        for i in range(7):
            axes[i].plot(time_array, error_degrees[:, i], color=colors[i], linewidth=1.4, label=f'Joint {i+1}')
            axes[i].axhline(y=0.0, color='black', linestyle='-', alpha=0.4, linewidth=1)
            axes[i].axhline(y=0.1, color='red', linestyle='-', alpha=0.85, linewidth=1.6, label='+/-0.1 deg')
            axes[i].axhline(y=-0.1, color='red', linestyle='-', alpha=0.85, linewidth=1.6)
            axes[i].fill_between(time_array, -0.1, 0.1, alpha=0.15, color='green')
            axes[i].set_ylim(-0.2, 0.2)
            axes[i].set_ylabel(f'J{i+1} (deg)')
            axes[i].grid(True, alpha=0.3)
            rmse = np.sqrt(np.mean(error_degrees[:, i] ** 2))
            max_val = np.max(np.abs(error_degrees[:, i]))
            within_01 = np.sum(np.abs(error_degrees[:, i]) <= 0.1) / len(error_degrees[:, i]) * 100
            axes[i].text(0.01, 0.92, f'RMSE: {rmse:.3f} deg\nMax: {max_val:.3f} deg\nWithin +/-0.1: {within_01:.1f}%',
                         transform=axes[i].transAxes, fontsize=8, verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.75))
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'improved_tracking_errors_with_fine_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()

    def inertia_matrix(self, q):
        """保守估计的关节空间惯量矩阵。"""
        q = np.asarray(q, dtype=float)
        n = len(q)
        distal_mass = np.array([np.sum(self.urdf_link_masses[i:]) for i in range(n)])
        shape = 0.75 + 0.25 * np.cos(q - self.gravity_reference) ** 2
        diag = self.reflected_motor_inertia + self.urdf_link_inertia_trace + 0.018 * distal_mass * self.effective_link_radii ** 2 * shape
        diag = np.clip(diag, self.inertia_min_diag, self.inertia_max_diag)
        M = np.diag(diag)

        # 加入很弱的相邻关节耦合，让惯量矩阵比纯对角更接近实际。
        # 随后用对角占优保证正定，防止数值上不稳定。
        for i in range(n - 1):
            coupling = 0.035 * np.sqrt(diag[i] * diag[i + 1]) * np.cos(q[i] - q[i + 1])
            M[i, i + 1] = coupling
            M[i + 1, i] = coupling
        for i in range(n):
            off_sum = np.sum(np.abs(M[i])) - abs(M[i, i])
            if M[i, i] <= off_sum + 1e-4:
                M[i, i] = off_sum + diag[i] + 1e-4
        return M

    def coriolis_matrix(self, q, q_dot):
        """简化的科氏/阻尼项，用于抑制速度相关误差。"""
        q_dot = np.asarray(q_dot, dtype=float)
        diag = self.viscous_friction + 0.015 * np.abs(q_dot)
        C = np.diag(diag)
        for i in range(len(q_dot) - 1):
            c = 0.006 * np.sin(q_dot[i] - q_dot[i + 1])
            C[i, i + 1] = c
            C[i + 1, i] = -c
        return C

    def gravity_matrix(self, q):
        """相对初始姿态的保守重力补偿。"""
        q = np.asarray(q, dtype=float)
        g = self.gravity_gains * (np.sin(q) - np.sin(self.gravity_reference))
        return np.clip(g, -self.gravity_comp_limit, self.gravity_comp_limit)

    def friction_torque(self, q_dot):
        """平滑库伦摩擦补偿，避免零速附近符号跳变。"""
        q_dot = np.asarray(q_dot, dtype=float)
        return self.coulomb_friction * np.tanh(q_dot / 0.05)

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray:
        """按论文公式(2)计算 6x7 几何雅可比矩阵。"""
        t_0_i, t_0_7 = Ttrans7(q)
        t_7_e = np.array(
            [[1.0, 0.0, 0.0, 0.0],
             [0.0, -1.0, 0.0, 0.0],
             [0.0, 0.0, -1.0, self.z_tool if hasattr(self, "z_tool") else -0.16746],
             [0.0, 0.0, 0.0, 1.0]],
            dtype=float,
        )
        t_0_e = t_0_7 @ t_7_e
        p_e = t_0_e[:3, 3]
        z_0 = np.array([0.0, 0.0, 1.0])
        j_v = np.zeros((3, 7))
        j_w = np.zeros((3, 7))
        for i in range(7):
            z_i = t_0_i[i, :3, :3] @ z_0
            p_i = t_0_i[i, :3, 3]
            j_v[:, i] = np.cross(z_i, p_e - p_i)
            j_w[:, i] = z_i
        return np.vstack((j_v, j_w))

    def analytical_jacobian(self, q: np.ndarray, phi_error: np.ndarray) -> np.ndarray:
        """Compute Ja = diag(I, J_l^{-1}(phi_error)) Jm."""
        lambda_e = np.eye(6)
        lambda_e[3:, 3:] = left_jacobian_inv_so3(phi_error)
        return lambda_e @ self.geometric_jacobian(q)

    def desired_analytical_velocity_acceleration(self, q, q_dot, q_desired, qdot_desired, phi_error, t: float):
        j_desired_m = self.geometric_jacobian(q_desired)
        v_desired_m = j_desired_m @ qdot_desired
        v_ed = v_desired_m[:3]
        omega_ed = v_desired_m[3:]
        r_err = R.from_rotvec(phi_error).as_matrix()
        jl_inv = left_jacobian_inv_so3(phi_error)
        v_desired_a = np.concatenate((v_ed, jl_inv @ (r_err @ omega_ed)))
        if self._prev_desired_velocity is None or self._prev_desired_velocity_time is None:
            xddot_desired_a = np.zeros(6)
        else:
            dt = max(t - self._prev_desired_velocity_time, self.dt)
            xddot_desired_a = (v_desired_a - self._prev_desired_velocity) / dt
        self._prev_desired_velocity = v_desired_a.copy()
        self._prev_desired_velocity_time = t
        return v_desired_a, xddot_desired_a

    def endpoint_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        z_tool = self.z_tool if hasattr(self, "z_tool") else -0.16746
        p, phi, _, _ = Kinematic(q, z_tool)
        return p, phi

    def endpoint_error(self, q: np.ndarray, q_desired: np.ndarray) -> np.ndarray:
        p_current, phi_current = self.endpoint_pose(q)
        p_desired, phi_desired = self.endpoint_pose(q_desired)
        pos_error = p_current - p_desired
        r_current = R.from_euler("ZYX", phi_current, degrees=False)
        r_desired = R.from_euler("ZYX", phi_desired, degrees=False)
        rot_error = (r_current * r_desired.inv()).as_rotvec()
        return np.concatenate((pos_error, rot_error))

    def current_jdot_qdot(self, jacobian: np.ndarray, q_dot: np.ndarray, t: float) -> np.ndarray:
        if self._prev_current_jacobian is None or self._prev_current_jacobian_time is None:
            self._prev_current_jacobian = jacobian.copy()
            self._prev_current_jacobian_time = t
            return np.zeros(6)
        dt = max(t - self._prev_current_jacobian_time, self.dt)
        j_dot = (jacobian - self._prev_current_jacobian) / dt
        self._prev_current_jacobian = jacobian.copy()
        self._prev_current_jacobian_time = t
        return j_dot @ q_dot

    def desired_endpoint_velocity_acceleration(self, q_desired: np.ndarray, qdot_desired: np.ndarray, t: float):
        j_desired = self.geometric_jacobian(q_desired)
        v_desired = j_desired @ qdot_desired
        if self._prev_desired_velocity is None or self._prev_desired_velocity_time is None:
            xddot_desired = np.zeros(6)
        else:
            dt = max(t - self._prev_desired_velocity_time, self.dt)
            xddot_desired = (v_desired - self._prev_desired_velocity) / dt
        self._prev_desired_velocity = v_desired.copy()
        self._prev_desired_velocity_time = t
        return v_desired, xddot_desired

    def equivalent_external_wrench(self, jacobian: np.ndarray, tau_e: np.ndarray) -> np.ndarray:
        """Map joint external torque into analytical-coordinate generalized force."""
        return damped_cartesian_inverse(jacobian, self.jacobian_damping) @ (jacobian @ tau_e)

    def ats_contact_weight(self, tau_e: np.ndarray) -> float:
        """只有超过外力进入阈值的超量部分，才降低 ATS 补偿。"""
        thresholds = np.asarray(self.external_torque_detector.detection_thresholds, dtype=float)
        tau_e = np.asarray(tau_e, dtype=float)
        if thresholds.shape != tau_e.shape:
            thresholds = np.resize(thresholds, tau_e.shape)

        tau_e_excess = np.sign(tau_e) * np.maximum(np.abs(tau_e) - thresholds, 0.0)
        tau_e_w = weighted_external_torque(tau_e_excess, self.external_torque_weights)
        self._last_tau_e_w = tau_e_w
        if tau_e_w <= 1e-12:
            return 1.0
        return float(np.exp(-tau_e_w / max(self.ats_k_ats, 1e-9)))

    def load_baseline_database(self, path: str = DEFAULT_BASELINE_DB):
        if not path or not os.path.exists(path):
            print("Baseline database not found; baseline detection disabled.")
            return False
        data = np.load(path)
        self.baseline_time = np.asarray(data['time'], dtype=float)
        self.baseline_tau_e = np.asarray(data['tau_e'], dtype=float)
        self.baseline_path = path
        print(f"已加载基线数据库: {path}")
        print(f"基线点数: {len(self.baseline_time)}, 时长: {self.baseline_time[-1]:.2f}s")
        return True

    def baseline_tau_at_time(self, t: float):
        if self.baseline_time is None or self.baseline_tau_e is None or len(self.baseline_time) < 2:
            return np.zeros(7)
        # 基线是同一条 30s 轨迹下采的。
        # 按轨迹相位对齐，重复测试时也能对上对应时刻的力矩基线。
        duration = float(self.baseline_time[-1] - self.baseline_time[0])
        if duration <= 1e-6:
            query_t = t
        else:
            query_t = self.baseline_time[0] + ((t - self.baseline_time[0]) % duration)
        baseline = np.zeros(7)
        for j in range(7):
            baseline[j] = np.interp(query_t, self.baseline_time, self.baseline_tau_e[:, j])
        return baseline

    def baseline_external_ats_scale(self, tau_e: np.ndarray, t: float):
        """基线外力检测接口：当前不改变 AT-S 比例。"""
        return 1.0, 0.0, np.zeros_like(tau_e)

    def maybe_reset_ats_after_contact(self, tau_e: np.ndarray, t: float):
        return

    def inverse_dynamics_control(self, q, q_dot, q_ddot_r, tau_e, t):
        """等价末端阻抗 + AT-S 补偿的合成力矩。"""
        try:
            epsilon_e = self.endpoint_error(q, self.q_r)
            j_current = self.analytical_jacobian(q, epsilon_e[3:])
            v_ed, xddot_ed = self.desired_analytical_velocity_acceleration(
                q, q_dot, self.q_r, self.q_dot_r, epsilon_e[3:], t
            )
            epsilon_dot_e = j_current @ q_dot - v_ed
            wrench_e = self.equivalent_external_wrench(j_current, tau_e)

            baseline_ats_scale, baseline_external_level, baseline_residual = self.baseline_external_ats_scale(tau_e, t)
            alpha = self.ats_contact_weight(tau_e)
            impedance_scale = self.endpoint_impedance_alpha_min + (1.0 - self.endpoint_impedance_alpha_min) * alpha
            K_X_eff = impedance_scale * self.K_X
            D_X_eff = max(0.35, math.sqrt(max(impedance_scale, 1e-6))) * self.D_X
            xddot_ref = xddot_ed + np.linalg.pinv(self.M_X) @ (
                -D_X_eff @ epsilon_dot_e - K_X_eff @ epsilon_e + wrench_e
            )
            jdot_qdot = self.current_jdot_qdot(j_current, q_dot, t)
            qddot_equiv = damped_right_pseudoinverse(j_current, self.jacobian_damping) @ (xddot_ref - jdot_qdot)

            m_q = self.inertia_matrix(q)
            c_q = self.coriolis_matrix(q, q_dot)
            g_q = self.gravity_matrix(q)
            friction_q = self.friction_torque(q_dot)
            tau_impedance = (
                self.dyn_inertia_gain * (m_q @ qddot_equiv)
                + self.dyn_coriolis_gain * (c_q @ q_dot)
                + self.dyn_gravity_sign * self.dyn_gravity_gain * g_q
                + self.dyn_friction_gain * friction_q
                - self.dyn_external_tau_gain * tau_e
            )
            tau_impedance = np.clip(tau_impedance, -self.tau_i_limit, self.tau_i_limit)

            self.maybe_reset_ats_after_contact(tau_e, t)
            q_error = self.normalize_angles(q - self.q_r)
            q_dot_error = q_dot - (self.q_dot_r + self.previous_u1_values)
            ats_output = np.zeros(len(q))
            current_u1 = np.zeros(len(q))
            for i in range(len(q)):
                try:
                    u1, u2 = self.ats_controllers[i].control_law(q_error[i], q_dot_error[i])
                    current_u1[i] = u1
                    ats_output[i] = u2
                except Exception:
                    current_u1[i] = 0.0
                    ats_output[i] = 0.0
            self.previous_u1_values = current_u1.copy()
            #alpha,这是对ats作用的外部力矩感知增益
            tau_ats = self.ats_k_g  *alpha* self.ats_joint_gain * ats_output
            self.nu = qddot_equiv.copy()
            self.tau_c = tau_impedance + tau_ats

            self.eef_error_history.append(epsilon_e.copy())
            self.eef_wrench_history.append(wrench_e.copy())
            self.eef_accel_ref_history.append(xddot_ref.copy())
            self.ats_coefficient_history.append(alpha)
            self.tau_impedance_history.append(tau_impedance.copy())
            self.tau_ats_history.append(tau_ats.copy())

            self._debug_counter += 1
            if self._debug_counter % 1000 == 0:
                print(
                    f"Response1 proposed: |eps_e|={np.linalg.norm(epsilon_e):.4f}, "
                    f"|F_e|={np.linalg.norm(wrench_e):.3f}, tau_e_w={self._last_tau_e_w:.3f}, "
                    f"alpha={alpha:.3f}, Kscale={impedance_scale:.3f}"
                )
            return self.tau_c
        except Exception as exc:
            print(f"Response1 proposed inverse dynamics error: {exc}")
            return np.zeros(len(q))


def main():
    from utilities import DeviceConnection, parseConnectionArguments

    # Keep all relative save paths anchored beside this program, not the shell cwd.
    os.chdir(THIS_DIR)

    parser = argparse.ArgumentParser(description="Response1 proposed comparison experiment")
    parser.add_argument("--skip-initial-move", action="store_true", help="Do not run PlayJointTrajectory before waiting for s/q")
    parser.add_argument("--baseline-db", default=DEFAULT_BASELINE_DB, help="Path to saved no-contact baseline npz")
    parser.add_argument("--no-baseline", action="store_true", help="Disable saved baseline external-force scaling")
    args = parseConnectionArguments(parser)

    with DeviceConnection.createTcpConnection(args) as router:
        with DeviceConnection.createUdpConnection(args) as router_real_time:
            controller = ProposedComparisonController(router, router_real_time)
            if not args.no_baseline:
                controller.load_baseline_database(args.baseline_db)
            print("=" * 72)
            print("Response1 本文方法对比实验")
            print("   7 关节正弦轨迹 + 等价末端阻抗 + AT-S 补偿")
            print("=" * 72)

            if args.skip_initial_move:
                print("Skip initial move; waiting for command.")
            else:
                print("Moving to initial pose; command prompt will appear after motion completes...")
                if not controller.move_to_custom_position():
                    print("Failed to move to initial pose; exiting.")
                    return 1

            print("\nCommand: s = start torque control, q = quit")

            stopped = False
            try:
                while True:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.readline().strip().lower()
                        if key == "s" and not controller.control_active:
                            if controller.init_torque_control():
                                controller.start_control()
                                print("            ")
                            else:
                                print("          ")
                        elif key == "q":
                            break
                    if controller.thread and not controller.thread.is_alive():
                        print("                    ...")
                        controller.stop_control()
                        stopped = True
                        break
                return 0
            except KeyboardInterrupt:
                print("     ")
            finally:
                if not stopped:
                    controller.stop_control()
    return 0


if __name__ == "__main__":
    sys.exit(main())
