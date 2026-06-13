import math
from DiscreteIntegrator import DiscreteIntegrator
from ts_fuzzy_output import ts_fuzzy_output

class control_main:
    def __init__(self, u1_init=0, u2_init=0, K1_init=0.1, K2_init=0.1, 
                 w1=0.0001, w2=0.0001, dag_deg=1.0, b1=0.1, b2=0.01,
                 AF11_0=0.2, AF12_0=0.0, AF21_0=2.0, AF22_0=0.2):
        """
        初始化控制器
        :param u1_init: u1初始值
        :param u2_init: u2初始值  
        :param K1_init: K1初始值
        :param K2_init: K2初始值
        :param w1: 自适应律参数1
        :param w2: 自适应律参数2
        :param dag_deg: 角度阈值(度)
        :param b1: 控制参数b1
        :param b2: 控制参数b2
        :param AF11_0: 模糊基础参数AF11_0
        :param AF12_0: 模糊基础参数AF12_0
        :param AF21_0: 模糊基础参数AF21_0
        :param AF22_0: 模糊基础参数AF22_0
        """
        # 初始化状态变量
        self.u1_prev = u1_init
        self.u2_prev = u2_init
       
        # 自适应律参数
        self.w1 = w1
        self.w2 = w2      
        
        # 控制参数
        self.b1 = b1
        self.b2 = b2
        
        # 角度阈值（转换为弧度）
        self.dag = dag_deg * math.pi / 180.0
        
        # 模糊基础参数
        self.AF11_0 = AF11_0
        self.AF12_0 = AF12_0
        self.AF21_0 = AF21_0
        self.AF22_0 = AF22_0
       
        # 创建自适应参数积分器
        self.K1_integrator = DiscreteIntegrator(
            initial_condition=K1_init,
            gain=1.0
        )
        self.K2_integrator = DiscreteIntegrator(
            initial_condition=K2_init,
            gain=1.0
        )
       
        # 添加AF参数记录
        self.AF11 = 0
        self.AF12 = 0
        self.AF21 = 0
        self.AF22 = 0
     
    def adaptive_law(self, x1, x2):
        """计算K1和K2的导数（用于积分器输入）"""
        K1_dot = self.w1 * x1 * (4 / math.pi) * math.atan(x1 / self.dag)
        K2_dot = self.w2 * x2 * (4 / math.pi) * math.atan(x2 / self.dag)
        return K1_dot, K2_dot
   
    def reset(self):
        """重置控制器状态"""
        self.u1_prev = 0
        self.u2_prev = 0
        self.K1_integrator.reset(0)
        self.K2_integrator.reset(0)
        self.AF11 = 0
        self.AF12 = 0
        self.AF21 = 0
        self.AF22 = 0
   
    def control_law(self, x1, x2):
        """
        主控制函数，每次调用执行一步控制计算
        :param x1: 位置误差 (joint_angles - target_angles)
        :param x2: 速度误差 (joint_velocities - target_velocities)
        :return: 控制力矩输出
        """
        # 使用上一时刻的控制量计算模糊输出
        x_ts = [x1, x2, self.u1_prev, self.u2_prev]
       
        try:
            [self.AF11, self.AF12, self.AF21, self.AF22] = ts_fuzzy_output(x_ts)
        except Exception as e:
            print(f"模糊计算错误: {e}")
            # 使用默认值防止系统崩溃
            self.AF11, self.AF12, self.AF21, self.AF22 = 0, 0, 0, 0
       
        # 获取当前自适应参数值
        K1 = self.K1_integrator.output[0]  # 标量值
        K2 = self.K2_integrator.output[0]  # 标量值
       
        # 计算当前控制量 u1
        u1 = -1 / self.b1 * (
            (self.AF11_0 + self.AF11) * x1 +
            (self.AF12_0 + self.AF12) * x2 +
            K1 * (4 / math.pi) * math.atan(x1 / self.dag)
        )
       
        # 计算当前控制量 u2
        u2 = -1 / self.b2 * (
            (self.AF21_0 + self.AF21) * x1 +
            (self.AF22_0 + self.AF22) * x2 +
            K2 * (4 / math.pi) * math.atan(x2 / self.dag)
        )
       
        # 计算导数
        K1_dot, K2_dot = self.adaptive_law(x1, x2)
           
        # 使用积分器更新自适应参数
        self.K1_integrator.step(K1_dot)
        self.K2_integrator.step(K2_dot)
       
        # 保存当前控制量供下一时刻使用
        self.u1_prev = u1
        self.u2_prev = u2
        
        # 返回控制力矩
        return u1, u2
    
    # imprort torque
    # torque= u2