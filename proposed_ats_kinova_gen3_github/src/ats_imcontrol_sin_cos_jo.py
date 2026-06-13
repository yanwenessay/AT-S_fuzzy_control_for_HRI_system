#!/usr/bin/env python3
"""
7自由度机械臂阻抗控制算法 - 改进版外部力矩检测
集成自适应阈值和在线校准功能，解决运动中的误检测问题
增加精细跟踪误差分析（±0.2度范围）

主要改进:
1. 自适应外部力矩检测阈值
2. 在线零偏值校准
3. 动态滤波和噪声抑制
4. 基于运动状态的智能检测
5. 精细跟踪误差可视化分析

主要方程:
1. 机械臂动态方程: M(q)q̈ + C(q,q̇)q̇ + g(q) = τc + τe
2. 阻抗控制方程: H q̈̃ + D(t) q̇̃ + K(t) q̃ = τe
3. 控制输入: τc = M(q) ν + C(q,q̇)q̇ + g(q) - τe
4. 虚拟控制: ν = q̈r + H⁻¹(-D(q̇ - q̇r) - K(q - qr) + τe)
"""

import sys
import os
import numpy as np
import time
import threading
import math
import select
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from datetime import datetime
from collections import deque

# Kinova API导入
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions

# 导入ATS自适应T-S模糊控制模块
from control_main import control_main


class AdaptiveExternalTorqueDetector:
    """自适应外部力矩检测器 - 解决运动中的误检测问题"""
    
    def __init__(self, actuator_count=7):
        self.actuator_count = actuator_count
        
        # ====== 外部力矩检测参数 ======
        # 检测阈值 - 根据关节特性和负载调整
        self.detection_thresholds = np.array([
            2.5,  # 关节1: 基座旋转轴，较高阈值
            5.5,  # 关节2: 肩部俯仰轴，承载大负载
            2.8,  # 关节3: 肩部滚转轴
            4.0,  # 关节4: 肘部弯曲轴
            1.8,  # 关节5: 前臂旋转轴，较敏感
            1.5,  # 关节6: 腕部俯仰轴，精细控制
            1.0   # 关节7: 腕部旋转轴，最敏感
        ])
        
        # ====== 在线校准参数 ======
        self.calibration_window_size = 200  # 校准窗口大小
        self.adaptation_rate = 0.02         # 自适应学习率
        self.min_stable_samples = 80        # 最少稳定样本数
        
        # ====== 零偏值估计参数 ======
        self.zero_bias_history_size = 150   # 零偏值历史长度
        self.velocity_threshold = 0.08      # 速度阈值(rad/s)，低于此值认为静止
        self.torque_change_threshold = 0.3  # 扭矩变化阈值，低于此值认为稳定
        
        # ====== 状态变量 ======
        # 初始扭矩偏移
        self.init_torques = np.zeros(actuator_count)
        self.torque_initialized = False
        
        # 动态零偏值估计
        self.zero_bias_estimates = np.zeros(actuator_count)
        self.torque_history = deque(maxlen=self.calibration_window_size)
        self.velocity_history = deque(maxlen=self.calibration_window_size)
        self.external_torque_history = deque(maxlen=self.zero_bias_history_size)
        
        # 滤波器参数
        self.lpf_alpha = 0.15  # 低通滤波器系数
        self.filtered_measured_torques = np.zeros(actuator_count)
        self.filtered_external_torques = np.zeros(actuator_count)
        
        # 稳定性检测
        self.stable_counter = 0
        self.last_torques = np.zeros(actuator_count)
        
        # 动态阈值调整参数
        self.velocity_scale_factor = 2.5    # 速度对阈值的影响系数
        self.max_threshold_scale = 4.0      # 最大阈值放大倍数
        
        # 统计信息
        self.detection_count = np.zeros(actuator_count)
        self.false_positive_reduction_count = 0
        self.calibration_update_count = 0
        
        print(f"🔧 自适应外部力矩检测器已初始化")
        print(f"   基础检测阈值: {self.detection_thresholds} Nm")
        print(f"   校准窗口: {self.calibration_window_size} 样本")
        print(f"   自适应学习率: {self.adaptation_rate}")
        print(f"   速度阈值: {self.velocity_threshold} rad/s")

    def set_initial_torques(self, initial_torques):
        """设置初始扭矩偏移"""
        self.init_torques = initial_torques.copy()
        self.zero_bias_estimates = initial_torques.copy()
        self.filtered_measured_torques = initial_torques.copy()
        self.torque_initialized = True
        print(f"✅ 初始扭矩偏移已设置: {np.round(self.init_torques, 3)}")

    def is_system_stable(self, recent_velocities, recent_torques):
        """判断系统是否处于稳定状态"""
        if len(recent_velocities) < self.min_stable_samples:
            return False, np.zeros(self.actuator_count)
        
        # 速度稳定性检查
        velocity_norms = np.linalg.norm(recent_velocities, axis=1)
        avg_velocity = np.mean(velocity_norms)
        
        # 扭矩稳定性检查
        torque_std = np.std(recent_torques, axis=0)
        
        # 判断哪些关节是稳定的
        stable_joints = (torque_std < self.torque_change_threshold)
        is_stable = avg_velocity < self.velocity_threshold and np.any(stable_joints)
        
        return is_stable, stable_joints

    def update_calibration(self, measured_torques, joint_velocities):
        """改进的在线校准零偏值估计"""
        if not self.torque_initialized:
            return
        
        # 存储历史数据
        self.torque_history.append(measured_torques.copy())
        self.velocity_history.append(joint_velocities.copy())
        
        # 低通滤波测量扭矩
        self.filtered_measured_torques = (self.lpf_alpha * measured_torques + 
                                         (1 - self.lpf_alpha) * self.filtered_measured_torques)
        
        # 检测机械臂是否处于相对静止状态
        if len(self.velocity_history) >= self.min_stable_samples:
            recent_velocities = np.array(list(self.velocity_history)[-self.min_stable_samples:])
            recent_torques = np.array(list(self.torque_history)[-self.min_stable_samples:])
            
            is_stable, stable_joints = self.is_system_stable(recent_velocities, recent_torques)
            
            if is_stable and np.any(stable_joints):
                mean_torques = np.mean(recent_torques, axis=0)
                
                # 使用自适应学习率更新零偏值估计
                # 对于稳定的关节，使用较高的学习率
                adaptive_rates = np.where(stable_joints, 
                                        self.adaptation_rate * 1.5, 
                                        self.adaptation_rate * 0.5)
                
                self.zero_bias_estimates = ((1 - adaptive_rates) * self.zero_bias_estimates + 
                                          adaptive_rates * mean_torques)
                
                self.stable_counter += 1
                self.calibration_update_count += 1
                
                # 定期打印校准信息
                if self.calibration_update_count % 50 == 0:
                    print(f"🔄 零偏值校准更新: 稳定关节 {np.sum(stable_joints)}/7, "
                          f"新估计值: {np.round(self.zero_bias_estimates, 3)}")

    def calculate_adaptive_thresholds(self, joint_velocities):
        """基于关节速度计算自适应检测阈值"""
        adaptive_thresholds = np.zeros(self.actuator_count)
        
        for i in range(self.actuator_count):
            # 基于速度的阈值放大系数
            velocity_factor = 1.0 + abs(joint_velocities[i]) * self.velocity_scale_factor
            velocity_factor = min(velocity_factor, self.max_threshold_scale)
            
            # 考虑关节位置对重力补偿的影响（简化模型）
            position_factor = 1.0  # 可以根据具体关节位置进行调整
            
            adaptive_thresholds[i] = (self.detection_thresholds[i] * 
                                    velocity_factor * position_factor)
        
        return adaptive_thresholds

    def apply_threshold_detection(self, raw_external_torques, adaptive_thresholds):
        """应用阈值检测和渐进归零"""
        detected_external_torques = np.zeros(self.actuator_count)
        
        for i in range(self.actuator_count):
            abs_torque = abs(raw_external_torques[i])
            threshold = adaptive_thresholds[i]
            
            if abs_torque > threshold:
                # 超过阈值，直接使用检测值
                detected_external_torques[i] = raw_external_torques[i]
                self.detection_count[i] += 1
            elif abs_torque > threshold * 0.5:
                # 在半阈值到阈值之间，使用渐进缩放
                scale_factor = (abs_torque - threshold * 0.5) / (threshold * 0.5)
                detected_external_torques[i] = raw_external_torques[i] * scale_factor
            else:
                # 在半阈值以下，使用衰减
                decay_factor = 0.85
                detected_external_torques[i] = raw_external_torques[i] * decay_factor
                
        return detected_external_torques

    def estimate_external_torque(self, measured_torques, joint_velocities):
        """
        改进的外部力矩估计方法
        结合自适应阈值、在线校准和智能滤波
        """
        if not self.torque_initialized:
            return np.zeros(self.actuator_count)
        
        try:
            # 1. 更新在线校准
            self.update_calibration(measured_torques, joint_velocities)
            
            # 2. 使用动态零偏值估计计算原始外部扭矩
            raw_external_torques = measured_torques - self.zero_bias_estimates
            
            # 3. 低通滤波外部扭矩
            self.filtered_external_torques = (self.lpf_alpha * raw_external_torques + 
                                             (1 - self.lpf_alpha) * self.filtered_external_torques)
            
            # 4. 计算自适应阈值
            adaptive_thresholds = self.calculate_adaptive_thresholds(joint_velocities)
            
            # 5. 应用阈值检测
            detected_external_torques = self.apply_threshold_detection(
                self.filtered_external_torques, adaptive_thresholds)
            
            # 6. 存储外部扭矩历史
            self.external_torque_history.append(detected_external_torques.copy())
            
            return detected_external_torques
            
        except Exception as e:
            print(f"❌ 外部扭矩估计错误: {e}")
            return np.zeros(self.actuator_count)

    def get_detection_statistics(self):
        """获取详细的检测统计信息"""
        total_samples = len(self.external_torque_history)
        if total_samples == 0:
            return {}
        
        # 计算各种统计指标
        recent_external_torques = np.array(list(self.external_torque_history)[-100:])
        
        stats = {
            'total_samples': total_samples,
            'detection_count_per_joint': self.detection_count.copy(),
            'detection_rate_per_joint': self.detection_count / max(total_samples, 1),
            'current_base_thresholds': self.detection_thresholds.copy(),
            'zero_bias_estimates': self.zero_bias_estimates.copy(),
            'stable_calibration_count': self.stable_counter,
            'calibration_updates': self.calibration_update_count,
            'recent_max_external_torques': np.max(np.abs(recent_external_torques), axis=0) if len(recent_external_torques) > 0 else np.zeros(self.actuator_count),
            'recent_avg_external_torques': np.mean(np.abs(recent_external_torques), axis=0) if len(recent_external_torques) > 0 else np.zeros(self.actuator_count)
        }
        
        return stats

    def adjust_detection_sensitivity(self, joint_idx, sensitivity_factor):
        """调整特定关节的检测灵敏度"""
        if 0 <= joint_idx < self.actuator_count:
            old_threshold = self.detection_thresholds[joint_idx]
            self.detection_thresholds[joint_idx] *= sensitivity_factor
            print(f"🔧 关节 {joint_idx+1} 检测阈值: {old_threshold:.2f} → {self.detection_thresholds[joint_idx]:.2f} Nm")

    def reset_calibration(self):
        """重置校准状态"""
        self.zero_bias_estimates = self.init_torques.copy()
        self.torque_history.clear()
        self.velocity_history.clear()
        self.stable_counter = 0
        self.calibration_update_count = 0
        print("🔄 检测器校准状态已重置")


class VariableImpedanceController:
    """改进的7自由度机械臂阻抗控制器 - 集成自适应外部力矩检测"""
    
    def __init__(self, router, router_real_time):
        """初始化改进的阻抗控制器"""
        
        # 网络连接设置
        self.router = router
        self.router_real_time = router_real_time
        
        # Kinova API客户端
        device_manager = DeviceManagerClient(router)
        self.actuator_config = ActuatorConfigClient(router)
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_real_time)
        
        # 初始化命令和反馈结构
        self.base_command = BaseCyclic_pb2.Command()
        self.base_feedback = BaseCyclic_pb2.Feedback()
        
        # 获取机器人配置
        self.actuator_count = self.base.GetActuatorCount().count  # 7自由度
        device_handles = device_manager.ReadAllDevices()
        
        for handle in device_handles.device_handle:
            if handle.device_type == Common_pb2.BIG_ACTUATOR or handle.device_type == Common_pb2.SMALL_ACTUATOR:
                self.base_command.actuators.add()
                self.base_feedback.actuators.add()
        
        # 通信选项
        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False
        self.sendOption.delay_ms = 0
        self.sendOption.timeout_ms = 3
        
        # ====== 核心改进：集成自适应外部力矩检测器 ======
        self.external_torque_detector = AdaptiveExternalTorqueDetector(self.actuator_count)
        
        # ====== 机械臂状态变量 ======
        N = 7  # 7自由度机械臂
        self.q = np.zeros(N)              # 当前关节位置 q
        self.q_dot = np.zeros(N)          # 当前关节速度 q̇
        
        # ====== 正余弦轨迹参数设置 ======
        # 轨迹中心位置（度）
        self.center_angles_degrees  = [180, 0, 180, -120, 0, 45, 90]
        self.q_center = np.array([math.radians(angle) for angle in self.center_angles_degrees])
        self.q_center = self.normalize_angles(self.q_center)
        
        # 运动幅度设置（弧度）
        amplitudes_degrees = [5, 5, 5, 5, 10, 10, 10]  # 前3个关节5度，后4个关节10度
        self.amplitudes = np.array([math.radians(amp) for amp in amplitudes_degrees])
        
        # 频率设置（不同关节使用不同频率以产生更丰富的运动）
        self.frequencies = np.array([0.2, 0.3, 0.2, 0.1, 0.3, 0.2, 0.3])  # Hz
        
        # 相位偏移（使各关节运动不完全同步）
        self.phase_offsets = np.array([0, np.pi/4, np.pi/2, 0, np.pi/3, np.pi/6, np.pi/2])
        
        # 当前轨迹状态（时变）
        self.q_r = self.q_center.copy()      # 期望位置 qr(t)
        self.q_dot_r = np.zeros(N)           # 期望速度 q̇r(t)  
        self.q_ddot_r = np.zeros(N)          # 期望加速度 q̈r(t)
        
        # ====== 上一时刻u1值存储（类似matlab的memory模块）======
        self.previous_u1_values = np.zeros(N)
        
        # ====== 轴特异性参数配置 ======
        self.axis_control_params = [
            # 轴1参数: 基座旋转轴，需要较强的控制能力
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.2, "AF12_0": 0.0, "AF21_0": 0.5, "AF22_0": 0.2},
            
            # 轴2参数: 肩部俯仰轴，承载较大负载
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.11,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.1, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.2, "AF12_0": 0.0, "AF21_0": 0.2, "AF22_0": 0.1},
            
            # 轴3参数: 肩部滚转轴，中等负载
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.3, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.6, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.6},
            
            # 轴4参数: 肘部弯曲轴，需要快速响应
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.07,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.0, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.3, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.5},
            
            # 轴5参数: 前臂旋转轴，轻负载高精度
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.3, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.1},
            
            # 轴6参数: 腕部俯仰轴，精细控制
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.001, "w2": 0.001, "dag_deg": 1.2, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 1.0, "AF22_0": 0.1},
            
            # 轴7参数: 腕部旋转轴，最轻负载最高精度
            {"u1_init": 0.0, "u2_init": 0.0, "K1_init": 0.05, "K2_init": 0.05,
             "w1": 0.01, "w2": 0.001, "dag_deg": 1.8, "b1": 0.1, "b2": 0.01,
             "AF11_0": 0.1, "AF12_0": 0.0, "AF21_0": 0.1, "AF22_0": 0.05}
        ]
        
        # 基于轴特性的阻抗参数初值
        self.axis_impedance_params = [
            {"H": 1, "K": 3.0, "D": 3.0},    # 基座轴: 高刚度，中等阻尼
            {"H": 1, "K": 6.0, "D": 6.0},    # 肩部俯仰轴: 高刚度，高阻尼
            {"H": 1, "K": 3.0, "D": 3.0},    # 肩部滚转轴: 中等刚度和阻尼
            {"H": 1, "K": 3.0, "D": 3.0},    # 肘部轴: 中等刚度，快速响应
            {"H": 1, "K": 2.0, "D": 2.0},    # 前臂旋转轴: 低刚度，精细控制
            {"H": 1, "K": 1.0, "D": 1.0},    # 腕部俯仰轴: 低刚度，精细控制
            {"H": 1, "K": 0.5, "D": 1.0}     # 腕部旋转轴: 最低刚度，最精细控制
        ]
        
        # 阻抗参数矩阵初始化
        self.H = np.eye(N)*1
        self.K_t = np.eye(N)*5
        self.D_t = np.eye(N)*10
        
        # 应用轴特定的阻抗参数
        for i in range(N):
            params = self.axis_impedance_params[i]
            self.H[i, i] = params["H"]
            self.K_t[i, i] = params["K"]
            self.D_t[i, i] = params["D"]
        
        # 外部力矩
        self.tau_e = np.zeros(N)
        
        # 机械臂动力学参数(简化模型)
        self.M_q = np.eye(N) * 1.0
        self.C_q = np.zeros((N, N))
        self.g_q = np.zeros(N)
        
        # 创建ATS控制器
        self.ats_controllers = []
        for i in range(N):
            params = self.axis_control_params[i]
            controller = control_main(
                u1_init=params["u1_init"],
                u2_init=params["u2_init"],
                K1_init=params["K1_init"],
                K2_init=params["K2_init"],
                w1=params["w1"],
                w2=params["w2"],
                dag_deg=params["dag_deg"],
                b1=params["b1"],
                b2=params["b2"],
                AF11_0=params["AF11_0"],
                AF12_0=params["AF12_0"],
                AF21_0=params["AF21_0"],
                AF22_0=params["AF22_0"]
            )
            self.ats_controllers.append(controller)
        
        # 控制输入
        self.tau_c = np.zeros(N)
        self.nu = np.zeros(N)
        
        # 控制参数
        self.dt = 0.001  # 控制周期 1ms
        self.control_active = False
        self.kill_thread = False
        self.thread = None
        self.run_duration = 30  # 控制时长(秒)
        
        # 自定义初始位置 (度) - 用于移动到合适位置
        self.custom_joint_angles_degrees = [180, 16, 180, 229, 0, 54, 90]
        
        # ====== 数据记录变量 ======
        self.time_history = []
        self.q_history = []
        self.q_dot_history = []
        self.q_r_history = []              # 期望轨迹记录
        self.q_dot_r_history = []          # 期望速度记录
        self.tau_cmd_history = []
        self.tau_e_history = []
        self.tau_measured_history = []
        self.nu_history = []
        self.q_error_history = []
        
        # ====== 外部力矩检测统计记录 ======
        self.detection_stats_history = []
        self.adaptive_threshold_history = []
        
        self._debug_counter = 0
        
        print(f"📊 改进的机械臂控制器已初始化")
        print(f"📐 机械臂自由度: {N}")
        print(f"🎯 轨迹中心位置: {self.center_angles_degrees} 度")
        print(f"📏 运动幅度: 前3关节±{np.degrees(self.amplitudes[:3])}°, 后4关节±{np.degrees(self.amplitudes[3:])}°")
        print(f"🔄 运动频率: {self.frequencies} Hz")
        print(f"⚙️ 阻抗参数 - H: {np.diag(self.H)}")
        print(f"🔧 阻抗参数 - K: {np.diag(self.K_t)}")
        print(f"🛠️ 阻抗参数 - D: {np.diag(self.D_t)}")
        print(f"🔍 自适应外部力矩检测已启用")

    def normalize_angle(self, angle):
        """将角度正规化到[-π, π]范围内"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    def normalize_angles(self, angles):
        """将角度数组正规化到[-π, π]范围内"""
        return np.array([self.normalize_angle(angle) for angle in angles])

    def normalize_degree_to_range(self, degree):
        """将度数从0-360°归一化到-180°到+180°"""
        while degree > 180:
            degree -= 360
        while degree <= -180:
            degree += 360
        return degree

    def compute_desired_trajectory(self, t):
        """计算时间t时刻的期望轨迹 qr(t), q̇r(t), q̈r(t)"""
        omega = 2 * np.pi * self.frequencies  # 角频率
        
        # 期望位置 qr(t) = q_center + A * sin(ωt + φ)
        self.q_r = self.q_center + self.amplitudes * np.sin(omega * t + self.phase_offsets)
        self.q_r = self.normalize_angles(self.q_r)
        
        # 期望速度 q̇r(t) = A * ω * cos(ωt + φ)
        self.q_dot_r = self.amplitudes * omega * np.cos(omega * t + self.phase_offsets)
        
        # 期望加速度 q̈r(t) = -A * ω² * sin(ωt + φ)
        self.q_ddot_r = -self.amplitudes * (omega**2) * np.sin(omega * t + self.phase_offsets)
        
        return self.q_r, self.q_dot_r, self.q_ddot_r

    def update_state(self, feedback):
        """更新系统状态"""
        q_current = np.zeros(self.actuator_count)
        q_dot_current = np.zeros(self.actuator_count)
        
        for i in range(self.actuator_count):
            q_current[i] = math.radians(feedback.actuators[i].position)
            q_dot_current[i] = math.radians(feedback.actuators[i].velocity)
        
        q_current = self.normalize_angles(q_current)
        self.q = q_current
        self.q_dot = q_dot_current

    def inertia_matrix(self, q):
        """计算惯性矩阵M(q)"""
        return np.eye(len(q)) * 1.0
    
    def coriolis_matrix(self, q, q_dot):
        """计算科里奥利矩阵C(q,q̇)"""
        return np.zeros((len(q), len(q)))
    
    def gravity_matrix(self, q):
        """计算重力矩阵g(q)"""
        return np.zeros(len(q))
    
    def stiffness_matrix(self, t):
        """计算时变刚度矩阵K(t)"""
        return self.K_t
    
    def damping_matrix(self, t):
        """计算时变阻尼矩阵D(t)"""
        return self.D_t

    def inverse_dynamics_control(self, q, q_dot, q_ddot_r, tau_e, t):
        """
        改进的逆动力学控制 - 集成自适应外部力矩
        控制输入: τc = M(q) ν + C(q,q̇)q̇ + g(q) - τe
        虚拟控制: ν = q̈r + H⁻¹(-D(q̇ - q̇r) - K(q - qr) + τe)
        """
        try:
            # 计算机械臂动力学矩阵
            M = self.inertia_matrix(q)
            C = self.coriolis_matrix(q, q_dot)
            g = self.gravity_matrix(q)
            
            # 获取当前时间的阻抗参数
            H = self.H
            K = self.stiffness_matrix(t)
            D = self.damping_matrix(t)
            
            # 计算虚拟控制输入ν
            try:
                H_inv = np.linalg.inv(H)
            except np.linalg.LinAlgError:
                H_inv = np.linalg.pinv(H)
            
            # 计算误差（确保角度误差在合理范围内）
            q_error_raw = q - self.q_r
            q_error = self.normalize_angles(q_error_raw)
            
            # 使用上一时刻的u1值计算修正后的速度误差
            q_dot_error = q_dot - (self.q_dot_r + self.previous_u1_values)
            
            # 计算虚拟控制输入（按照论文公式）
            self.nu =  q_ddot_r + H_inv @ (-D @ q_dot_error - K @ q_error + tau_e)
            
            # ATS控制替代传统动力学项
            ats_control_output = np.zeros(len(q))
            current_u1_values = np.zeros(len(q))
            
            for i in range(len(q)):
                try:
                    u1, u2 = self.ats_controllers[i].control_law(q_error[i], q_dot_error[i])
                    current_u1_values[i] = u1
                    ats_control_output[i] = np.clip(u2, -40, 40)
                except Exception:
                    current_u1_values[i] = 0.0
                    ats_control_output[i] = 0.0
            
            # 更新previous_u1_values为当前时刻的u1值
            self.previous_u1_values = current_u1_values.copy()
            
            # 动态ATS系数计算 - 基于权值的均值方法
            joint_weights = np.array([1.2, 1.0, 0.8, 0.6, 0.4, 0.3, 0.2])  # 关节权重，远端关节权重更小
            tau_e_weighted = np.abs(tau_e) * joint_weights / np.sum(joint_weights)  # 权重归一化
            tau_e_weighted_mean = np.sum(tau_e_weighted)  # 加权均值
            threshold = 1  # 调整阈值适应新的计算方法
            scale_factor = 5.0  # 调整缩放因子
            if tau_e_weighted_mean <= threshold:
                ats_coefficient = 1.0  # 在阈值以下保持系数为1
            else:
                ats_coefficient = np.exp(-(tau_e_weighted_mean - threshold) / scale_factor)  # 超过阈值后从1开始下降

            # 调试信息
            self._debug_counter += 1
            if self._debug_counter % 1000 == 0:
                print(f"🔧 动态ATS: tau_e_weighted_mean={tau_e_weighted_mean:.4f}, coefficient={ats_coefficient:.4f}")
            
            # 计算控制输入τc
            self.tau_c = M @ self.nu + 1.1*ats_coefficient * ats_control_output - tau_e

            return self.tau_c
            
        except Exception as e:
            print(f"❌ 逆动力学控制计算错误: {e}")
            return np.zeros(len(q))

    def impedance_control_algorithm(self, t):
        """
        主控制算法 - 集成改进的外部力矩检测
        """
        # 计算时变期望轨迹
        self.compute_desired_trajectory(t)
        
        # 使用改进的外部扰动估计
        self.tau_e = self.estimate_external_torque()
        
        # 计算逆动力学控制输入
        tau_control = self.inverse_dynamics_control(
            self.q, self.q_dot, self.q_ddot_r, self.tau_e, t
        )
        
        # 安全限制
        max_torque = 40.0
        tau_control = np.clip(tau_control, -max_torque, max_torque)
        
        return tau_control

    def move_to_custom_position(self):
        """移动到自定义关节位置"""
        print("移动到初始位置...")
        print(f"目标角度 (度): {self.custom_joint_angles_degrees}")
        
        # 切换到单层伺服模式
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # 准备关节角度目标
        constrained_joint_angles = Base_pb2.ConstrainedJointAngles()
        
        for joint_id in range(len(self.custom_joint_angles_degrees)):
            joint_angle = constrained_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = self.custom_joint_angles_degrees[joint_id]
        
        # 事件同步逻辑
        finished_event = threading.Event()
        
        def check_for_end_or_abort(notification, event=finished_event):
            if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
                event.set()
        
        notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort,
            Base_pb2.NotificationOptions()
        )
        
        print("执行关节运动...")
        self.base.PlayJointTrajectory(constrained_joint_angles)
        
        print("等待运动完成...")
        finished = finished_event.wait(30.0)
        self.base.Unsubscribe(notification_handle)
        
        if finished:
            print("✅ 到达初始位置")
            # 设置正余弦轨迹的中心位置
            self.q_center = np.array([math.radians(angle) for angle in self.center_angles_degrees])
            self.q_center = self.normalize_angles(self.q_center)
            
            print(f"✅ 正余弦轨迹中心位置设置完成")
            return True
        else:
            print("❌ 运动超时")
            return False

    def init_torque_offsets(self):
        """改进的初始扭矩偏移初始化"""
        try:
            feedback = self.base_cyclic.RefreshFeedback()
            
            init_torques = np.zeros(self.actuator_count)
            for i in range(self.actuator_count):
                init_torques[i] = feedback.actuators[i].torque
            
            # 使用自适应检测器设置初始值
            self.external_torque_detector.set_initial_torques(init_torques)
            
            print("✅ 改进的扭矩偏移初始化完成")
            return True
            
        except Exception as e:
            print(f"❌ 初始化扭矩偏移失败: {e}")
            return False

    def estimate_external_torque(self):
        """使用改进的外部力矩估计方法"""
        try:
            # 获取当前测量扭矩
            measured_torques = np.zeros(self.actuator_count)
            for i in range(self.actuator_count):
                measured_torques[i] = self.base_feedback.actuators[i].torque
            
            # 使用自适应检测器估计外部扭矩
            external_torques = self.external_torque_detector.estimate_external_torque(
                measured_torques, self.q_dot
            )
            
            return external_torques
            
        except Exception as e:
            print(f"❌ 外部扭矩估计错误: {e}")
            return np.zeros(self.actuator_count)
        
    def init_torque_control(self):
        """初始化力矩控制模式"""
        try:
            self.base_feedback = self.base_cyclic.RefreshFeedback()

            # 使用改进的扭矩偏移初始化
            if not self.init_torque_offsets():
                print("❌ 扭矩偏移初始化失败")
                return False
            
            for i in range(self.actuator_count):
                self.base_command.actuators[i].flags = 1
                self.base_command.actuators[i].position = self.normalize_degree_to_range(self.base_feedback.actuators[i].position)
            
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            
            self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
            
            control_mode_msg = ActuatorConfig_pb2.ControlModeInformation()
            control_mode_msg.control_mode = ActuatorConfig_pb2.ControlMode.TORQUE
            for device_id in range(1, self.actuator_count + 1):
                self.actuator_config.SetControlMode(control_mode_msg, device_id)
            
            print("✅ 改进的力矩控制模式初始化完成")
            return True
            
        except Exception as e:
            print(f"❌ 力矩控制初始化失败: {e}")
            return False
    
    def start_control(self):
        """启动控制"""
        if self.thread and self.thread.is_alive():
            print("控制已在运行")
            return
        
        print("启动改进的阻抗控制...")
        self.control_active = True
        self.kill_thread = False
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()
    
    def control_loop(self):
        """主控制循环 - 集成改进的外部力矩检测"""
        print(f"运行正余弦轨迹控制 {self.run_duration} 秒...")
        start_time = time.time()
        frame_id = 0
        loop_count = 0
        
        while not self.kill_thread and (time.time() - start_time) < self.run_duration:
            loop_start = time.time()
            current_time = time.time() - start_time
            loop_count += 1
            
            try:
                # 获取反馈
                self.base_feedback = self.base_cyclic.RefreshFeedback()
                
                # 更新状态
                self.update_state(self.base_feedback)
                
                # 计算控制力矩（包含改进的外部力矩检测）
                tau_cmd = self.impedance_control_algorithm(current_time)

                # 记录测量扭矩
                tau_measured_current = np.zeros(self.actuator_count)
                for i in range(self.actuator_count):
                    tau_measured_current[i] = self.base_feedback.actuators[i].torque
                
                # 计算关节角度误差
                q_error_raw = self.q - self.q_r
                q_error = self.normalize_angles(q_error_raw)
                
                # 数据记录
                if loop_count % 10 == 0:
                    self.time_history.append(current_time)
                    self.q_history.append(self.q.copy())
                    self.q_dot_history.append(self.q_dot.copy())
                    self.q_r_history.append(self.q_r.copy())
                    self.q_dot_r_history.append(self.q_dot_r.copy())
                    self.tau_cmd_history.append(tau_cmd.copy())
                    self.tau_e_history.append(self.tau_e.copy())
                    self.tau_measured_history.append(tau_measured_current.copy())
                    self.nu_history.append(self.nu.copy())
                    self.q_error_history.append(q_error.copy())
                    
                    # 记录检测统计
                    stats = self.external_torque_detector.get_detection_statistics()
                    if stats:
                        self.detection_stats_history.append(stats)
                     
                # 更新命令
                for i in range(self.actuator_count):
                    self.base_command.actuators[i].position = self.normalize_degree_to_range(self.base_feedback.actuators[i].position)
                    self.base_command.actuators[i].torque_joint = float(tau_cmd[i])
                
                frame_id = (frame_id + 1) % 65536
                self.base_command.frame_id = frame_id
                for i in range(self.actuator_count):
                    self.base_command.actuators[i].command_id = frame_id
                
                # 发送命令
                self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                
                # 改进的状态监控
                if loop_count % 1000 == 0:
                    q_error_degrees = np.degrees(q_error)
                    max_error = np.max(np.abs(q_error_degrees))
                    tau_ext_norm = np.linalg.norm(self.tau_e)
                    u1_norm = np.linalg.norm(self.previous_u1_values)
                    
                    # 获取检测统计
                    stats = self.external_torque_detector.get_detection_statistics()
                    detection_rate = np.mean(stats['detection_rate_per_joint']) if stats else 0
                    
                    print(f"⏰ 时间: {current_time:.1f}s | "
                          f"循环: {loop_count} | "
                          f"数据: {len(self.time_history)} | "
                          f"误差: {max_error:.2f}° | "
                          f"控制: {np.linalg.norm(tau_cmd):.2f}Nm | "
                          f"外部: {tau_ext_norm:.3f}Nm | "
                          f"检测率: {detection_rate:.3f}")
                    
                # 定期打印检测统计
                if loop_count % 5000 == 0:
                    self.print_detection_statistics()
                    
                    # 显示当前轨迹状态
                    q_r_degrees = np.degrees(self.q_r)
                    q_current_degrees = np.degrees(self.q)
                    print(f"📍 期望轨迹: {np.round(q_r_degrees, 1)}")
                    print(f"📍 当前位置: {np.round(q_current_degrees, 1)}")
                
                # 控制频率
                elapsed = time.time() - loop_start
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)
                
            except Exception as err:
                err_text = str(err)
                print(f"Control loop error: {err}")
                if "WRONG_SERVOING_MODE" in err_text or "Wrong servoing mode" in err_text:
                    print("Not in low level servoing mode; stopping control loop. Re-run torque initialization.")
                    self.kill_thread = True
                    break
        
        print(f"✅ 改进的控制循环结束 - 总循环: {loop_count}, 数据点: {len(self.time_history)}")
        self.control_active = False
    
    def print_detection_statistics(self):
        """打印详细的检测统计信息"""
        stats = self.external_torque_detector.get_detection_statistics()
        if stats:
            print(f"\n📊 外部力矩检测统计报告:")
            print(f"   📈 总样本数: {stats['total_samples']}")
            print(f"   🎯 各关节检测次数: {stats['detection_count_per_joint']}")
            print(f"   📊 各关节检测率: {np.round(stats['detection_rate_per_joint'], 4)}")
            print(f"   🔧 当前基础阈值 (Nm): {np.round(stats['current_base_thresholds'], 2)}")
            print(f"   📐 零偏值估计 (Nm): {np.round(stats['zero_bias_estimates'], 3)}")
            print(f"   ✅ 稳定校准次数: {stats['stable_calibration_count']}")
            print(f"   🔄 校准更新次数: {stats['calibration_updates']}")
            print(f"   📈 近期最大外部扭矩: {np.round(stats['recent_max_external_torques'], 3)}")
            print(f"   📊 近期平均外部扭矩: {np.round(stats['recent_avg_external_torques'], 3)}")

    def stop_control(self):
        """停止控制"""
        print("停止改进的控制...")
        self.kill_thread = True
        self.control_active = False
        
        if self.thread:
            self.thread.join()
        
        # 恢复位置控制
        try:
            control_mode_msg = ActuatorConfig_pb2.ControlModeInformation()
            control_mode_msg.control_mode = ActuatorConfig_pb2.ControlMode.Value('POSITION')
            for device_id in range(1, self.actuator_count + 1):
                self.actuator_config.SetControlMode(control_mode_msg, device_id)
            
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
        except Exception as e:
            print(f"❌ 恢复控制模式错误: {e}")
        
        print("✅ 控制停止完成")
        
        # 打印最终统计
        self.print_detection_statistics()
        
        # 保存和绘图
        self.save_data()
        self.plot_control_data()
    
    def save_data(self):
        """保存实验数据"""
        if not self.time_history:
            print("没有数据需要保存")
            return
        
        try:
            import csv
            
            data_folder = "impedance_improved_data"
            if not os.path.exists(data_folder):
                os.makedirs(data_folder)
            
            # 保存主要控制数据
            filename = os.path.join(data_folder, 'improved_trajectory_data.csv')
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # 写入头部
                header = ['time']
                for i in range(self.actuator_count):
                    header.extend([f'q{i}', f'qdot{i}', f'qr{i}', f'qdotr{i}',
                                  f'tau_cmd{i}', f'tau_e{i}', f'tau_measured{i}', 
                                  f'nu{i}', f'q_error{i}'])
                writer.writerow(header)
                
                # 写入数据
                for i in range(len(self.time_history)):
                    row = [self.time_history[i]]
                    for j in range(self.actuator_count):
                        row.extend([
                            self.q_history[i][j] if j < len(self.q_history[i]) else 0,
                            self.q_dot_history[i][j] if j < len(self.q_dot_history[i]) else 0,
                            self.q_r_history[i][j] if i < len(self.q_r_history) and j < len(self.q_r_history[i]) else 0,
                            self.q_dot_r_history[i][j] if i < len(self.q_dot_r_history) and j < len(self.q_dot_r_history[i]) else 0,
                            self.tau_cmd_history[i][j] if j < len(self.tau_cmd_history[i]) else 0,
                            self.tau_e_history[i][j] if j < len(self.tau_e_history[i]) else 0,
                            self.tau_measured_history[i][j] if i < len(self.tau_measured_history) and j < len(self.tau_measured_history[i]) else 0,
                            self.nu_history[i][j] if i < len(self.nu_history) and j < len(self.nu_history[i]) else 0,
                            self.q_error_history[i][j] if i < len(self.q_error_history) and j < len(self.q_error_history[i]) else 0
                        ])
                    writer.writerow(row)
            
            # 保存检测统计数据
            if self.detection_stats_history:
                stats_filename = os.path.join(data_folder, 'detection_statistics.csv')
                with open(stats_filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # 写入统计数据头部
                    header = ['sample_index', 'total_samples', 'stable_calibrations', 'calibration_updates']
                    for i in range(self.actuator_count):
                        header.extend([f'detection_count_j{i}', f'detection_rate_j{i}', 
                                      f'threshold_j{i}', f'zero_bias_j{i}'])
                    writer.writerow(header)
                    
                    # 写入统计数据
                    for idx, stats in enumerate(self.detection_stats_history):
                        row = [idx, stats['total_samples'], stats['stable_calibration_count'], 
                               stats.get('calibration_updates', 0)]
                        for j in range(self.actuator_count):
                            row.extend([
                                stats['detection_count_per_joint'][j],
                                stats['detection_rate_per_joint'][j],
                                stats['current_base_thresholds'][j],
                                stats['zero_bias_estimates'][j]
                            ])
                        writer.writerow(row)
            
            print(f"✅ 改进版数据已保存至 {data_folder}")
            print(f"📊 主要数据: {len(self.time_history)} 点, 时间: 0-{self.time_history[-1]:.1f}s")
            print(f"📈 统计数据: {len(self.detection_stats_history)} 点")
            
        except Exception as e:
            print(f"❌ 数据保存失败: {e}")

    def plot_control_data(self):
        """Generate visualization replicating Method A style (renamed to Proposed Method)."""
        if not self.time_history:
            print("没有数据需要绘图")
            return
        
        try:
            plot_folder = "proposed_method_plots"
            if not os.path.exists(plot_folder):
                os.makedirs(plot_folder)
            
            time_array = np.array(self.time_history)
            q_array = np.array(self.q_history)
            q_r_array = np.array(self.q_r_history)
            tau_cmd_array = np.array(self.tau_cmd_history)
            tau_e_array = np.array(self.tau_e_history)
            q_error_array = np.array(self.q_error_history)
            
            print(f"📊 Proposed Method plotting data:")
            print(f"   时间点: {len(time_array)} | 范围: {time_array[0]:.2f}-{time_array[-1]:.2f}s")
            print(f"   数据形状: q{q_array.shape}, qr{q_r_array.shape}, tau_e{tau_e_array.shape}")
            
            # 设置matplotlib
            plt.rcParams['font.family'] = 'DejaVu Sans'
            plt.rcParams['axes.unicode_minus'] = False
            
            # Match Method A sequence (renamed)
            self._plot_trajectory_tracking(time_array, q_array, q_r_array, plot_folder)
            self._plot_torque_data(time_array, tau_cmd_array, 
                                   "Joint Control Torques (Proposed Method)",
                                   "Torque (Nm)",
                                   "joint_control_torques_proposed.png", plot_folder)
            self._plot_angle_error_data(time_array, q_error_array, 
                                        "Joint Angle Tracking Error (Proposed Method)",
                                        "Angle Error (°)",
                                        "joint_angle_tracking_error_proposed.png", plot_folder)
            self._plot_torque_data(time_array, np.array(self.nu_history), 
                                   "Impedance Virtual Control ν (Proposed Method)",
                                   "Virtual Control (rad/s²)",
                                   "impedance_virtual_control_nu_proposed.png", plot_folder)
            self._plot_comprehensive_comparison(time_array, tau_cmd_array, np.array(self.nu_history),
                                                q_error_array, tau_e_array, plot_folder, method_tag="Proposed Method")
            self._plot_control_vs_external(time_array, tau_cmd_array, tau_e_array, plot_folder)
            # optional 3D skipped unless needed
            print(f"✅ Proposed Method plots saved to: {plot_folder}")
            
        except Exception as e:
            print(f"❌ 绘图失败: {e}")
    
    # === Replicated Method A style plotting functions (renamed to Proposed Method) ===
    def _plot_torque_data(self, time_array, data_array, title, ylabel, filename, folder):
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle(f'{title} - 7DOF Robot Arm (Proposed Method)', fontsize=16, fontweight='bold')
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        for i in range(7):
            axes[i].plot(time_array, data_array[:, i], color=colors[i], linewidth=1.5, label=f'Joint {i+1}')
            axes[i].set_ylabel(f'J{i+1}\n{ylabel}', fontsize=10)
            axes[i].grid(True, alpha=0.3)
            axes[i].legend(loc='upper right', fontsize=8)
            mean_val = np.mean(data_array[:, i])
            std_val = np.std(data_array[:, i])
            max_val = np.max(np.abs(data_array[:, i]))
            axes[i].text(0.02, 0.95, f'Mean: {mean_val:.3f}\nStd: {std_val:.3f}\nMax: {max_val:.3f}',
                         transform=axes[i].transAxes, fontsize=8, va='top',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        plt.tight_layout()
        plt.savefig(os.path.join(folder, filename), dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_angle_error_data(self, time_array, error_array, title, ylabel, filename, folder):
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle(f'{title} - 7DOF Robot Arm (Proposed Method)', fontsize=16, fontweight='bold')
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        error_degrees = np.degrees(error_array)
        for i in range(7):
            axes[i].plot(time_array, error_degrees[:, i], color=colors[i], linewidth=1.5, label=f'Joint {i+1}')
            axes[i].set_ylabel(f'J{i+1}\nError (°)', fontsize=10)
            axes[i].grid(True, alpha=0.3)
            axes[i].legend(loc='upper right', fontsize=8)
            axes[i].axhline(y=0.2, color='red', linestyle='--', alpha=0.6, linewidth=1, label='±0.2° Reference')
            axes[i].axhline(y=-0.2, color='red', linestyle='--', alpha=0.6, linewidth=1)
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        plt.tight_layout()
        plt.savefig(os.path.join(folder, filename), dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_comprehensive_comparison(self, time_array, tau_cmd_array, nu_array, q_error_array, tau_e_array, folder, method_tag="Proposed Method"):
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'Impedance Control Performance Overview ({method_tag})', fontsize=16, fontweight='bold')
        tau_cmd_norm = np.linalg.norm(tau_cmd_array, axis=1)
        nu_norm = np.linalg.norm(nu_array, axis=1)
        q_error_norm = np.linalg.norm(q_error_array, axis=1)
        tau_e_norm = np.linalg.norm(tau_e_array, axis=1)
        axes[0, 0].plot(time_array, tau_cmd_norm, 'b-', linewidth=2, label='||tau_cmd||')
        axes[0, 0].set_title('Control Torque Norm', fontweight='bold')
        axes[0, 0].set_ylabel('Control Torque Norm (Nm)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 1].plot(time_array, nu_norm, 'g-', linewidth=2, label='||nu||')
        axes[0, 1].set_title('Virtual Control Input Norm', fontweight='bold')
        axes[0, 1].set_ylabel('Impedance Input Norm (rad/s²)')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()
        q_error_norm_deg = np.degrees(q_error_norm)
        axes[1, 0].plot(time_array, q_error_norm_deg, 'r-', linewidth=2, label='||q_error||')
        axes[1, 0].axhline(y=0.2, color='red', linestyle='--', alpha=0.6, label='±0.2° Reference')
        axes[1, 0].set_title('Angle Tracking Error Norm', fontweight='bold')
        axes[1, 0].set_ylabel('Error Norm (°)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()
        axes[1, 1].plot(time_array, tau_e_norm, 'm-', linewidth=2, label='||tau_e||')
        axes[1, 1].set_title('External Torque Norm', fontweight='bold')
        axes[1, 1].set_ylabel('External Torque Norm (Nm)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.20)
        plt.savefig(os.path.join(folder, 'performance_overview_proposed.png'), dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_control_vs_external(self, time_array, tau_cmd_array, tau_e_array, folder):
        fig, axes = plt.subplots(7,1, figsize=(12,14))
        fig.suptitle('Control vs External Torque (Proposed Method)', fontsize=16, fontweight='bold')
        colors_cmd = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        colors_ext = ['#87CEEB', '#FFD700', '#90EE90', '#FFB6C1', '#DDA0DD', '#F4A460', '#FFB6C1']
        for i in range(7):
            axes[i].plot(time_array, tau_cmd_array[:, i], color=colors_cmd[i], linewidth=1.5, label=f'J{i+1} Control')
            axes[i].plot(time_array, tau_e_array[:, i], '--', color=colors_ext[i], linewidth=1.2, alpha=0.85, label=f'J{i+1} External')
            axes[i].set_ylabel(f'J{i+1}\nTorque (Nm)', fontsize=10)
            axes[i].grid(True, alpha=0.3)
            axes[i].legend(loc='upper right', fontsize=8)
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'control_vs_external_proposed.png'), dpi=300, bbox_inches='tight')
        plt.close()

    def _plot_trajectory_tracking(self, time_array, q_array, q_r_array, folder):
        fig, axes = plt.subplots(7, 1, figsize=(12, 18))
        fig.suptitle('Sin/Cos Time-Varying Trajectory Tracking (Proposed Method)', fontsize=16, fontweight='bold')
        colors_actual = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        colors_ref = ['#87CEEB', '#FFD700', '#90EE90', '#FFB6C1', '#DDA0DD', '#F4A460', '#FFB6C1']
        for i in range(7):
            q_deg = np.degrees(q_array[:, i])
            qr_deg = np.degrees(self.normalize_angles(q_r_array[:, i]))
            axes[i].plot(time_array, qr_deg, '--', color=colors_ref[i], linewidth=2.0, label=f'Desired J{i+1}')
            axes[i].plot(time_array, q_deg, '-', color=colors_actual[i], linewidth=1.4, label=f'Actual J{i+1}')
            err_deg = self.normalize_angles(q_array[:, i] - q_r_array[:, i])
            err_deg = np.degrees(err_deg)
            rmse = np.sqrt(np.mean(err_deg**2))
            max_err = np.max(np.abs(err_deg))
            axes[i].text(0.02, 0.92, f'RMSE: {rmse:.2f}°\nMax: {max_err:.2f}°', transform=axes[i].transAxes,
                         fontsize=8, va='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            axes[i].grid(True, alpha=0.3)
            axes[i].set_ylabel(f'J{i+1} (°)')
            axes[i].legend(loc='upper right', fontsize=8)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'trajectory_tracking.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_improved_tracking_errors_with_fine_analysis(self, time_array, error_array, folder):
        """绘制改进版跟踪误差分析（专注于±0.2度范围内的精细分析）"""
        fig, axes = plt.subplots(7, 1, figsize=(14, 18))
        fig.suptitle('Fine Tracking Errors Analysis: ±0.2° Range Focus', 
                     fontsize=16, fontweight='bold')
        
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        error_degrees = np.degrees(error_array)
        
        legend_handles = []
        legend_labels = []
        for i in range(7):
            axes[i].plot(time_array, error_degrees[:, i], color=colors[i], linewidth=1.5, 
                        label=f'J{i+1} Error')
            axes[i].axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=1.5)
            
            # ±0.2度边界线
            axes[i].axhline(y=0.2, color='red', linestyle='-', alpha=0.8, linewidth=2, 
                           label='±0.2° Boundary')
            axes[i].axhline(y=-0.2, color='red', linestyle='-', alpha=0.8, linewidth=2)
            
            # 填充±0.2度区域
            fill = axes[i].fill_between(time_array, -0.2, 0.2, alpha=0.15, color='green', 
                                        label='Target Precision Zone')
            
            # 设置y轴范围专注于±0.2度
            axes[i].set_ylim(-0.25, 0.25)
            
            axes[i].set_ylabel(f'Joint {i+1}\nError (°)', fontsize=10)
            axes[i].grid(True, alpha=0.3)
            if i == 0:
                # 仅收集一次句柄
                line_handle = axes[i].lines[0]
                legend_handles.extend([line_handle, fill])
                legend_labels.extend(['Joint Error', '±0.2° Zone'])
            
            # 精细跟踪统计信息
            rmse = np.sqrt(np.mean(error_degrees[:, i]**2))
            std_val = np.std(error_degrees[:, i])
            max_val = np.max(np.abs(error_degrees[:, i]))
            mean_abs = np.mean(np.abs(error_degrees[:, i]))
            
            # 计算在±0.2度范围内的时间百分比
            within_fine_zone = np.sum(np.abs(error_degrees[:, i]) <= 0.2) / len(error_degrees[:, i]) * 100
            
            # 计算超出±0.2度的次数和时间
            exceed_count = np.sum(np.abs(error_degrees[:, i]) > 0.2)
            exceed_percentage = (1 - within_fine_zone / 100) * 100
            
            axes[i].text(0.02, 0.95, f'RMSE: {rmse:.3f}°\nStd: {std_val:.3f}°\nMax: {max_val:.3f}°\nMAE: {mean_abs:.3f}°\nWithin ±0.2°: {within_fine_zone:.1f}%\nExceed Count: {exceed_count}', 
                        transform=axes[i].transAxes, fontsize=8, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9))
        
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        if legend_handles:
            fig.legend(legend_handles, legend_labels, loc='upper right', bbox_to_anchor=(0.985,0.995), fontsize=9)
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'improved_tracking_errors_with_fine_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_fine_tracking_error_analysis(self, time_array, error_array, folder):
        """专门的精细跟踪误差分析图（±0.2度范围）"""
        fig, axes = plt.subplots(2, 4, figsize=(20, 12))
        fig.suptitle('Fine Tracking Error Analysis: ±0.2° Precision Zone Performance', 
                     fontsize=16, fontweight='bold')
        
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        error_degrees = np.degrees(error_array)
        
        # 为每个关节绘制精细误差分析
        for i in range(7):
            row = i // 4
            col = i % 4
            
            if i == 7:  # 最后一个子图用于汇总分析
                break
                
            ax = axes[row, col]
            
            # 绘制误差曲线
            ax.plot(time_array, error_degrees[:, i], color=colors[i], linewidth=1.5, 
                   label=f'Joint {i+1} Error')
            
            # ±0.2度区域高亮
            ax.fill_between(time_array, -0.2, 0.2, alpha=0.2, color='green', 
                           label='±0.2° Precision Zone')
            
            # 精细参考线
            ax.axhline(y=0.2, color='red', linestyle='-', alpha=0.8, linewidth=2)
            ax.axhline(y=-0.2, color='red', linestyle='-', alpha=0.8, linewidth=2)
            ax.axhline(y=0, color='black', linestyle='-', alpha=0.5, linewidth=1)
            
            # 更细致的参考线
            ax.axhline(y=0.1, color='blue', linestyle=':', alpha=0.6, linewidth=1, label='±0.1°')
            ax.axhline(y=-0.1, color='blue', linestyle=':', alpha=0.6, linewidth=1)
            ax.axhline(y=0.05, color='gray', linestyle=':', alpha=0.4, linewidth=1, label='±0.05°')
            ax.axhline(y=-0.05, color='gray', linestyle=':', alpha=0.4, linewidth=1)
            
            # 设置y轴范围专注于精细区域
            ax.set_ylim(-0.5, 0.5)
            ax.set_title(f'Joint {i+1} Fine Error Analysis', fontsize=12, fontweight='bold')
            ax.set_ylabel('Error (°)', fontsize=10)
            ax.grid(True, alpha=0.3)
            # 移除子图单独图例，集中到汇总子图
            
            # 精细统计信息
            within_02 = np.sum(np.abs(error_degrees[:, i]) <= 0.2) / len(error_degrees[:, i]) * 100
            within_01 = np.sum(np.abs(error_degrees[:, i]) <= 0.1) / len(error_degrees[:, i]) * 100
            within_005 = np.sum(np.abs(error_degrees[:, i]) <= 0.05) / len(error_degrees[:, i]) * 100
            max_error_in_range = np.max(np.abs(error_degrees[:, i]))
            rmse_fine = np.sqrt(np.mean(error_degrees[:, i]**2))
            
            info_text = f'Within Zones:\n±0.2°: {within_02:.1f}%\n±0.1°: {within_01:.1f}%\n±0.05°: {within_005:.1f}%\n\nMax: {max_error_in_range:.3f}°\nRMSE: {rmse_fine:.3f}°'
            ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=8, 
                   verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))
        
        # 最后一个子图：汇总分析
        ax_summary = axes[1, 3]
        
        # 计算所有关节的精细跟踪性能
        within_02_all = []
        within_01_all = []
        within_005_all = []
        rmse_all = []
        
        for i in range(7):
            within_02_all.append(np.sum(np.abs(error_degrees[:, i]) <= 0.2) / len(error_degrees[:, i]) * 100)
            within_01_all.append(np.sum(np.abs(error_degrees[:, i]) <= 0.1) / len(error_degrees[:, i]) * 100)
            within_005_all.append(np.sum(np.abs(error_degrees[:, i]) <= 0.05) / len(error_degrees[:, i]) * 100)
            rmse_all.append(np.sqrt(np.mean(error_degrees[:, i]**2)))
        
        # 柱状图显示各关节在不同精度区间的表现
        x_pos = np.arange(7)
        width = 0.25
        
        bars1 = ax_summary.bar(x_pos - width, within_005_all, width, label='±0.05°', 
                              color='green', alpha=0.7)
        bars2 = ax_summary.bar(x_pos, within_01_all, width, label='±0.1°', 
                              color='blue', alpha=0.7)
        bars3 = ax_summary.bar(x_pos + width, within_02_all, width, label='±0.2°', 
                              color='orange', alpha=0.7)
        
        ax_summary.set_title('Fine Tracking Performance Summary', fontsize=12, fontweight='bold')
        ax_summary.set_xlabel('Joint Number', fontsize=10)
        ax_summary.set_ylabel('Time in Zone (%)', fontsize=10)
        ax_summary.set_xticks(x_pos)
        ax_summary.set_xticklabels([f'J{i+1}' for i in range(7)])
        # 汇总子图统一图例
        ax_summary.legend(loc='upper right')
        ax_summary.grid(True, alpha=0.3, axis='y')
        
        # 在柱状图上添加数值标签
        for bars in [bars1, bars2, bars3]:
            for bar in bars:
                height = bar.get_height()
                ax_summary.text(bar.get_x() + bar.get_width()/2., height,
                               f'{height:.1f}', ha='center', va='bottom', fontsize=8)
        
        # 添加总体性能统计
        overall_02 = np.mean(within_02_all)
        overall_01 = np.mean(within_01_all)
        overall_005 = np.mean(within_005_all)
        overall_rmse = np.mean(rmse_all)
        
        summary_text = f'Overall Performance:\n±0.2° Zone: {overall_02:.1f}%\n±0.1° Zone: {overall_01:.1f}%\n±0.05° Zone: {overall_005:.1f}%\nAvg RMSE: {overall_rmse:.3f}°'
        ax_summary.text(0.02, 0.98, summary_text, transform=ax_summary.transAxes, fontsize=9, 
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9))
        
        # 调整布局并保存
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'fine_tracking_error_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_external_torque_detection(self, time_array, tau_e_array, folder):
        """绘制外部力矩检测效果图"""
        fig, axes = plt.subplots(7, 1, figsize=(14, 18))
        fig.suptitle('Adaptive External Torque Detection Performance', 
                     fontsize=16, fontweight='bold')
        
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        
        legend_handles = []
        legend_labels = []
        for i in range(7):
            axes[i].plot(time_array, tau_e_array[:, i], color=colors[i], linewidth=1.5, 
                        label=f'J{i+1} External')
            axes[i].axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=1)
            
            # 显示检测阈值
            threshold = self.external_torque_detector.detection_thresholds[i]
            axes[i].axhline(y=threshold, color='red', linestyle=':', alpha=0.6, linewidth=1.5, 
                           label=f'Base Threshold: {threshold:.1f}Nm')
            axes[i].axhline(y=-threshold, color='red', linestyle=':', alpha=0.6, linewidth=1.5)
            
            axes[i].set_ylabel(f'Joint {i+1}\nExternal τ (Nm)', fontsize=10)
            axes[i].grid(True, alpha=0.3)
            if i == 0:
                line_handle = axes[i].lines[0]
                threshold_line = axes[i].lines[-2]  # the positive threshold line
                legend_handles.extend([line_handle, threshold_line])
                legend_labels.extend(['External Torque', 'Base Threshold'])
            
            # 检测统计
            abs_torques = np.abs(tau_e_array[:, i])
            detection_ratio = np.sum(abs_torques > threshold) / len(abs_torques)
            max_detected = np.max(abs_torques)
            avg_detected = np.mean(abs_torques)
            
            axes[i].text(0.02, 0.95, f'Max: {max_detected:.3f}Nm\nAvg: {avg_detected:.3f}Nm\nDetection: {detection_ratio:.3f}', 
                        transform=axes[i].transAxes, fontsize=8, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='orange', alpha=0.8))
        
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        if legend_handles:
            fig.legend(legend_handles, legend_labels, loc='upper right', bbox_to_anchor=(0.985,0.995), fontsize=9)
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'external_torque_detection.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_improved_control_torques(self, time_array, tau_cmd_array, tau_e_array, folder):
        """绘制改进的控制力矩对比图"""
        fig, axes = plt.subplots(7, 1, figsize=(14, 18))
        fig.suptitle('Proposed Method: Control Torques vs External Torques (Adaptive Detection)', 
                     fontsize=16, fontweight='bold')
        
        colors_cmd = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        colors_ext = ['#FFB6C1', '#FFA07A', '#98FB98', '#F0E68C', '#DDA0DD', '#D2B48C', '#F5DEB3']
        
        legend_handles = []
        legend_labels = []
        for i in range(7):
            # 控制力矩
            h_cmd, = axes[i].plot(time_array, tau_cmd_array[:, i], color=colors_cmd[i], linewidth=1.5, 
                                  label=f'Ctrl J{i+1}')
            
            # 外部力矩（放大显示）
            h_ext, = axes[i].plot(time_array, tau_e_array[:, i] * 5, color=colors_ext[i], linewidth=1.2, 
                                  alpha=0.8, label=f'Ext J{i+1} (×5)')
            
            axes[i].set_ylabel(f'Joint {i+1}\nTorque (Nm)', fontsize=10)
            axes[i].grid(True, alpha=0.3)
            if i == 0:
                legend_handles.extend([h_cmd, h_ext])
                legend_labels.extend(['Control Torque','External Torque (×5)'])
            
            # 改进的统计信息
            cmd_mean = np.mean(np.abs(tau_cmd_array[:, i]))
            cmd_std = np.std(tau_cmd_array[:, i])
            cmd_max = np.max(np.abs(tau_cmd_array[:, i]))
            ext_mean = np.mean(np.abs(tau_e_array[:, i]))
            ext_max = np.max(np.abs(tau_e_array[:, i]))
            
            axes[i].text(0.02, 0.95, f'Ctrl: μ={cmd_mean:.2f}, σ={cmd_std:.2f}, max={cmd_max:.2f}\nExt: μ={ext_mean:.3f}, max={ext_max:.3f}', 
                        transform=axes[i].transAxes, fontsize=8, verticalalignment='top',
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        axes[-1].set_xlabel('Time (s)', fontsize=12)
        if legend_handles:
            fig.legend(legend_handles, legend_labels, loc='upper right', bbox_to_anchor=(0.985,0.995), fontsize=9)
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'improved_control_torques.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_detection_statistics(self, folder):
        """绘制检测统计分析图"""
        if not self.detection_stats_history:
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('External Torque Detection Statistics Analysis', 
                     fontsize=16, fontweight='bold')
        
        # 提取统计数据
        stats_array = []
        for stats in self.detection_stats_history:
            stats_array.append([
                stats['stable_calibration_count'],
                stats.get('calibration_updates', 0),
                np.mean(stats['detection_rate_per_joint']),
                np.mean(stats['recent_avg_external_torques'])
            ])
        
        stats_array = np.array(stats_array)
        time_indices = np.arange(len(stats_array))
        
        # 子图1: 校准次数变化
        axes[0, 0].plot(time_indices, stats_array[:, 0], 'b-', linewidth=2, label='Stable Calibrations')
        axes[0, 0].plot(time_indices, stats_array[:, 1], 'r--', linewidth=2, label='Calibration Updates')
        axes[0, 0].set_title('Calibration Progress')
        axes[0, 0].set_ylabel('Count')
        axes[0, 0].legend(loc='upper right')  # 保留单独图例（统计类图保持可读性）
        axes[0, 0].grid(True, alpha=0.3)
        
        # 子图2: 检测率变化
        axes[0, 1].plot(time_indices, stats_array[:, 2], 'g-', linewidth=2, label='Avg Detection Rate')
        axes[0, 1].set_title('Detection Rate Evolution')
        axes[0, 1].set_ylabel('Detection Rate')
        axes[0, 1].legend(loc='upper right')
        axes[0, 1].grid(True, alpha=0.3)
        
        # 子图3: 各关节检测率对比
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        final_stats = self.detection_stats_history[-1]
        
        axes[1, 0].bar(range(7), final_stats['detection_rate_per_joint'], 
                      color=colors, alpha=0.7)
        axes[1, 0].set_title('Final Detection Rate by Joint')
        axes[1, 0].set_xlabel('Joint Number')
        axes[1, 0].set_ylabel('Detection Rate')
        axes[1, 0].set_xticks(range(7))
        axes[1, 0].set_xticklabels([f'J{i+1}' for i in range(7)])
        axes[1, 0].grid(True, alpha=0.3, axis='y')
        
        # 子图4: 阈值 vs 零偏值估计
        axes[1, 1].bar(np.arange(7) - 0.2, final_stats['current_base_thresholds'], 
                      width=0.4, color='red', alpha=0.7, label='Detection Thresholds')
        axes[1, 1].bar(np.arange(7) + 0.2, np.abs(final_stats['zero_bias_estimates']), 
                      width=0.4, color='blue', alpha=0.7, label='|Zero Bias Estimates|')
        axes[1, 1].set_title('Thresholds vs Zero Bias Estimates')
        axes[1, 1].set_xlabel('Joint Number')
        axes[1, 1].set_ylabel('Torque (Nm)')
        axes[1, 1].set_xticks(range(7))
        axes[1, 1].set_xticklabels([f'J{i+1}' for i in range(7)])
        axes[1, 1].legend(loc='upper right')
        axes[1, 1].grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(os.path.join(folder, 'detection_statistics.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_improved_performance_overview(self, time_array, q_array, q_r_array, 
                                          error_array, tau_cmd_array, tau_e_array, folder):
        """绘制改进版综合性能分析图"""
        fig, axes = plt.subplots(2, 3, figsize=(20, 12))
        fig.suptitle('Proposed Method Impedance Control with Adaptive External Torque Detection - Performance Overview', 
                     fontsize=16, fontweight='bold')
        
        # 计算各指标
        error_norm = np.linalg.norm(np.degrees(error_array), axis=1)
        tau_cmd_norm = np.linalg.norm(tau_cmd_array, axis=1)
        tau_e_norm = np.linalg.norm(tau_e_array, axis=1)
        
        # 子图1: 总体跟踪误差
        axes[0, 0].plot(time_array, error_norm, 'r-', linewidth=2, label='Tracking Error Norm')
        axes[0, 0].set_title('Overall Tracking Performance', fontweight='bold')
        axes[0, 0].set_ylabel('Error Norm (°)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend(loc='upper right')
        
        # 子图2: 控制力矩 vs 外部力矩
        axes[0, 1].plot(time_array, tau_cmd_norm, 'b-', linewidth=2, label='Control Torque Norm')
        axes[0, 1].plot(time_array, tau_e_norm * 10, 'orange', linewidth=2, label='External Torque Norm (×10)')
        axes[0, 1].set_title('Control vs External Torques', fontweight='bold')
        axes[0, 1].set_ylabel('Torque Norm (Nm)')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend(loc='upper right')
        
        # 子图3: 轨迹跟踪示例（选择性关节）
        selected_joints = [1, 3, 5]  # 选择代表性关节
        colors = ['#1f77b4', '#d62728', '#9467bd']
        for idx, joint_i in enumerate(selected_joints):
            q_degrees = np.degrees(q_array[:, joint_i])
            qr_degrees = np.degrees(q_r_array[:, joint_i])
            axes[0, 2].plot(time_array, qr_degrees, '--', color=colors[idx], alpha=0.7, 
                           label=f'Desired J{joint_i+1}')
            axes[0, 2].plot(time_array, q_degrees, '-', color=colors[idx], linewidth=1.5, 
                           label=f'Actual J{joint_i+1}')
        axes[0, 2].set_title('Selected Joint Tracking', fontweight='bold')
        axes[0, 2].set_ylabel('Angle (°)')
        axes[0, 2].grid(True, alpha=0.3)
        axes[0, 2].legend(fontsize=8, loc='upper right')
        
        # 子图4: 误差分布直方图
        axes[1, 0].hist(error_norm, bins=30, alpha=0.7, color='red', edgecolor='black')
        axes[1, 0].set_title('Error Distribution', fontweight='bold')
        axes[1, 0].set_xlabel('Error Norm (°)')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].grid(True, alpha=0.3, axis='y')
        
        # 子图5: 外部力矩检测效果
        detection_events = tau_e_norm > 0.1  # 检测事件阈值
        axes[1, 1].plot(time_array, tau_e_norm, 'orange', linewidth=1.5, label='External Torque Norm')
        axes[1, 1].scatter(time_array[detection_events], tau_e_norm[detection_events], 
                          color='red', s=10, alpha=0.6, label='Detection Events')
        axes[1, 1].set_title('External Torque Detection', fontweight='bold')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('External Torque Norm (Nm)')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend(loc='upper right')
        
        # 子图6: 性能指标对比（增加精细跟踪指标）
        final_stats = self.external_torque_detector.get_detection_statistics()
        
        # 计算各种性能指标（包含精细跟踪）
        overall_rmse = np.sqrt(np.mean(error_norm**2))
        max_error = np.max(error_norm)
        avg_control_effort = np.mean(tau_cmd_norm)
        max_control_effort = np.max(tau_cmd_norm)
        avg_external_detection = np.mean(tau_e_norm)
        detection_rate = np.mean(final_stats['detection_rate_per_joint']) if final_stats else 0
        
        # 计算精细跟踪指标
        error_degrees_all = np.degrees(error_array)
        within_02_overall = np.mean([np.sum(np.abs(error_degrees_all[:, i]) <= 0.2) / len(error_degrees_all[:, i]) * 100 for i in range(7)])
        within_01_overall = np.mean([np.sum(np.abs(error_degrees_all[:, i]) <= 0.1) / len(error_degrees_all[:, i]) * 100 for i in range(7)])
        
        metrics = ['RMSE (°)', 'Max Error (°)', 'Avg Control (Nm)', 'Max Control (Nm)', 
                  'Avg Ext Torque (Nm)', 'Detection Rate', '±0.2° Zone (%)', '±0.1° Zone (%)']
        values = [overall_rmse, max_error, avg_control_effort, max_control_effort, 
                 avg_external_detection, detection_rate * 10, within_02_overall / 10, within_01_overall / 10]  # 归一化显示
        
        bars = axes[1, 2].bar(range(len(metrics)), values, 
                             color=['red', 'orange', 'blue', 'navy', 'green', 'purple', 'cyan', 'magenta'], 
                             alpha=0.7)
        axes[1, 2].set_title('Performance Metrics Summary', fontweight='bold')
        axes[1, 2].set_ylabel('Value')
        axes[1, 2].set_xticks(range(len(metrics)))
        axes[1, 2].set_xticklabels(metrics, rotation=45, ha='right')
        axes[1, 2].grid(True, alpha=0.3, axis='y')
        
        # 在柱状图上添加数值标签
        for bar, value, metric in zip(bars, values, metrics):
            height = bar.get_height()
            if '(%)' in metric:
                # 百分比指标需要还原显示
                display_value = value * 10
                axes[1, 2].text(bar.get_x() + bar.get_width()/2., height,
                               f'{display_value:.1f}%', ha='center', va='bottom', fontsize=8)
            else:
                axes[1, 2].text(bar.get_x() + bar.get_width()/2., height,
                               f'{value:.3f}', ha='center', va='bottom', fontsize=8)
        
        # 添加改进版性能摘要（包含精细跟踪分析）
        improvement_text = f"""Proposed Method Impedance Control Performance Summary with Fine Tracking Analysis:
        
        Trajectory Tracking:
        • Overall RMSE: {overall_rmse:.3f}°
        • Maximum Error: {max_error:.3f}°
        
        Fine Tracking Performance:
        • Time in ±0.2° Zone: {within_02_overall:.1f}%
        • Time in ±0.1° Zone: {within_01_overall:.1f}%
        
        Control Effort:
        • Average Control Torque: {avg_control_effort:.3f} Nm
        • Maximum Control Torque: {max_control_effort:.3f} Nm
        
        External Torque Detection:
        • Average Detection Level: {avg_external_detection:.3f} Nm
        • Overall Detection Rate: {detection_rate:.3f}
        • Stable Calibrations: {final_stats['stable_calibration_count'] if final_stats else 0}
        
        Key Improvements:
        • Adaptive threshold adjustment
        • Online zero-bias calibration  
        • Motion-aware detection
        • Reduced false positives
        • Fine precision tracking analysis"""

        fig.text(0.02, 0.02, improvement_text, fontsize=9, 
                 bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9),
                 verticalalignment='bottom')

        plt.tight_layout()
        plt.subplots_adjust(bottom=0.25)
        plt.savefig(os.path.join(folder, 'improved_performance_overview.png'), 
                    dpi=300, bbox_inches='tight')
        plt.close()


def main():
    """主函数 - 改进版外部力矩检测"""
    import argparse
    
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    
    try:
        from utilities import DeviceConnection, parseConnectionArguments
    except ImportError:
        print("错误：无法导入utilities模块，请检查路径")
        return 1
    
    parser = argparse.ArgumentParser()
    args = parseConnectionArguments(parser)
    
    with DeviceConnection.createTcpConnection(args) as router:
        with DeviceConnection.createUdpConnection(args) as router_real_time:
            controller = VariableImpedanceController(router, router_real_time)
            
            print("=" * 80)
            print("7自由度机械臂改进版阻抗控制系统")
            print("集成自适应外部力矩检测技术 + 精细误差分析")
            print("=" * 80)
            print("核心改进:")
            print("🔧 自适应检测阈值 - 基于运动状态动态调整")
            print("🎯 在线零偏值校准 - 持续优化检测基准")  
            print("📊 智能滤波机制 - 减少噪声和误检测")
            print("📈 运动感知检测 - 区分动力学效应和外部扰动")
            print("🔍 统计分析功能 - 实时监控检测性能")
            print("📏 精细误差分析 - ±0.2度精度区间跟踪分析")
            print()
            print("轨迹参数:")
            print(f"• 轨迹中心: {controller.center_angles_degrees} 度")
            print(f"• 运动幅度: 前3关节±5°, 后4关节±10°")
            print(f"• 运动频率: {controller.frequencies} Hz")
            print(f"• 检测阈值: {controller.external_torque_detector.detection_thresholds} Nm")
            print()
            print("精细跟踪分析功能:")
            print("• ±0.2度精度区间性能评估")
            print("• ±0.1度超精细区间分析")
            print("• ±0.05度极限精度统计")
            print("• 各关节精度分布可视化")
            print("=" * 80)
            
            if not controller.move_to_custom_position():
                print("❌ 移动到初始位置失败")
                return 1
            print("✅ 已到达初始位置，系统准备就绪")
            
            try:
                print("\n🚀 改进版控制系统已准备就绪（含精细误差分析）")
                print("操作说明:")
                print("  's' - 启动改进版阻抗控制（自适应外部力矩检测 + 精细跟踪分析）")
                print("  'q' - 退出程序")
                print("🎯 控制完成后将生成详细的检测性能分析报告")
                print("📊 包含±0.2度、±0.1度、±0.05度精度区间的详细分析")
                
                while True:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.readline().strip().lower()
                        
                        if key == 's' and not controller.control_active:
                            print("\n🔧 正在初始化改进版力矩控制...")
                            if controller.init_torque_control():
                                print("✅ 初始化完成，启动改进版控制...")
                                controller.start_control()
                                print("🚀 改进版阻抗控制已启动（含精细误差分析）")
                            else:
                                print("❌ 力矩控制初始化失败")
                                
                        elif key == 'q':
                            print("👋 退出程序...")
                            break
                    
                    if controller.control_active and not controller.thread.is_alive():
                        print("✅ 改进版控制完成")
                        break
                        
                return 0
                    
            except KeyboardInterrupt:
                print("\n⚠️ 用户中断")
            finally:
                controller.stop_control()

if __name__ == "__main__":
    sys.exit(main())