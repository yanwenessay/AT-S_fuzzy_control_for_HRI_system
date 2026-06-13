import numpy as np
from fuzzy_membership_fcn import fuzzy_membership_fcn
from DiscreteIntegrator import DiscreteIntegrator

# 全局积分器
integrator_w11 = None
integrator_w12 = None
integrator_w21 = None
integrator_w22 = None

def ts_fuzzy_output(x):
    """
    模糊输出函数 - 计算并返回 AF11, AF12, AF21, AF22
    参数:
        x: 输入向量 [x1, x2, u1_prev, u2_prev]
    返回:
        AF11, AF12, AF21, AF22: 四个模糊输出参数的值
    """
    global integrator_w11, integrator_w12, integrator_w21, integrator_w22
    
    # 如果积分器未初始化，则自动初始化
    if integrator_w11 is None:
        # 调用模糊隶属度函数获取规则数量
        _, wdot = fuzzy_membership_fcn(np.zeros(4))
        rule_count = wdot.shape[0]  # 规则数量
        
        # 创建积分器（使用 DiscreteIntegrator 的默认时间步长）
        integrator_w11 = DiscreteIntegrator(dim=rule_count, initial_condition=0.0)
        integrator_w12 = DiscreteIntegrator(dim=rule_count, initial_condition=0.0)
        integrator_w21 = DiscreteIntegrator(dim=rule_count, initial_condition=0.0)
        integrator_w22 = DiscreteIntegrator(dim=rule_count, initial_condition=0.0)
    
    # 调用模糊隶属度函数获取 beta 和 wdot
    beta, wdot = fuzzy_membership_fcn(x)
    
    # 使用积分器更新后件参数 w
    w11 = integrator_w11.step(wdot)
    w12 = integrator_w12.step(wdot)
    w21 = integrator_w21.step(wdot)
    w22 = integrator_w22.step(wdot)
    
    # 计算模糊输出：AF = ∑(w_i * beta_i)
    AF11 = np.sum(w11 * beta)
    AF12 = np.sum(w12 * beta)
    AF21 = np.sum(w21 * beta)
    AF22 = np.sum(w22 * beta)
    
    return AF11, AF12, AF21, AF22

def reset():
    """重置所有状态"""
    global integrator_w11, integrator_w12, integrator_w21, integrator_w22
    if integrator_w11 is not None:
        integrator_w11.reset(0.0)
        integrator_w12.reset(0.0)
        integrator_w21.reset(0.0)
        integrator_w22.reset(0.0)