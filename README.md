# AT-S_fuzzy_control_for_HRI_system

https://github.com/user-attachments/assets/180bafd0-7812-4f9b-a3f8-51e0bd93270f

<img width="505" height="449" alt="屏幕截图 2026-06-13 210141" src="https://github.com/user-attachments/assets/e60e4109-c2d1-4201-902c-fe908b597bd1" />

https://github.com/user-attachments/assets/b6b6c6be-d588-4486-b501-26d7a278a1f0

<img width="1865" height="262" alt="experimental_process" src="https://github.com/user-attachments/assets/f20ea02a-b7e4-4443-8dbc-ff42cc24ff5f" />

<img width="1931" height="817" alt="experimental_process4" src="https://github.com/user-attachments/assets/ea5c31ae-9162-4036-bd74-6df1aae729a0" />



# Kinova Gen3 的 AT-S 阻抗控制

本仓库提供一套面向 Kinova Gen3 7 自由度机械臂的自适应 Takagi-Sugeno（AT-S）补偿阻抗控制 Python 实现。

该程序用于真实机器人实验，基于 Kinova Kortex Python API，包含 AT-S 阻抗主控制器、变阻抗力矩控制框架、运动学、模糊逻辑工具，以及需要用户自行配置机器人网络参数的安全启动脚本。

## 主要内容

主控制器位于：

```text
src/comparison_proposed_method_ats.py
```

该控制器集成了：

* 等效末端阻抗控制；
* AT-S 模糊自适应补偿；
* 基于外部关节力矩加权的力感知衰减；
* 保守的模型动力学补偿；
* 通过 Kortex API 实现 Kinova Gen3 底层力矩控制；
* 跟踪误差、外力矩、控制力矩和 AT-S 补偿的数据记录与绘图。

## 文件结构

```text
.
+-- README.md
+-- SAFETY.md
+-- requirements.txt
+-- setup_env.sh
+-- run_kinova_gen3.sh
+-- .env.example
+-- .gitignore
+-- scripts/
|   +-- check_environment.py
|   +-- run_proposed_ats.py
+-- src/
    +-- comparison_proposed_method_ats.py
    +-- ats_imcontrol_sin_cos_jo.py
    +-- Kinematic_fcn.py
    +-- DiscreteIntegrator.py
    +-- ts_fuzzy_output.py
    +-- fuzzy_membership_fcn.py
    +-- fuzzyoutput.py
    +-- control_main.py
    +-- utilities.py
```

## 硬件与软件要求

硬件：

* Kinova Gen3 7 自由度机械臂；
* 一台连接机器人网络的 Linux/Ubuntu 电脑；
* 现场操作人员需能够随时使用急停按钮。

软件：

* Python 3.8 或更高版本；
* 已安装 Kinova 官方 Kortex Python API，并可通过 `kortex_api` 导入；
* `requirements.txt` 中列出的 Python 依赖包。

安装依赖：

```bash
pip install -r requirements.txt
```

验证 Kortex API：

```bash
python3 -c "import kortex_api; print('kortex_api OK')"
```

## 配置机器人连接

复制环境变量模板：

```bash
cp .env.example .env
```

然后在 `.env` 中填写自己的机器人参数：

```bash
KINOVA_IP=<YOUR_KINOVA_ROBOT_IP>
KINOVA_USERNAME=<YOUR_KINOVA_USERNAME>
KINOVA_PASSWORD=<YOUR_KINOVA_PASSWORD>
```

本仓库不提供默认 IP、用户名或密码。请勿将私人机器人网络信息提交到 GitHub。

## 检查环境

运行：

```bash
python3 scripts/check_environment.py
```

该脚本会检查：

* `src/` 中的必要源码文件；
* 必需的 Python 依赖；
* `kortex_api` 是否可用；
* Kinova 连接辅助文件 `src/utilities.py` 是否存在。

## 运行控制器

推荐使用启动脚本：

```bash
./run_kinova_gen3.sh
```

也可以直接运行：

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD>
```

如果机器人已经处于安全初始位姿，可跳过初始运动：

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD> --skip-initial-move
```

程序启动后会等待键盘输入：

```text
s = 开始底层力矩控制
q = 退出
```

只有输入 `s` 后，程序才会发送力矩命令。

## 输出数据

程序运行时以 `src/` 作为工作目录，通常会生成：

```text
src/impedance_improved_data/
src/proposed_method_plots/
```

这些输出文件夹已被 `.gitignore` 忽略，默认不提交到仓库，除非需要发布实验数据。

## 安全说明

该程序会向真实机器人发送底层力矩命令。运行前请务必：

* 阅读 `SAFETY.md`；
* 保持急停按钮在可触及范围内；
* 确保机器人工作空间清空；
* 先进行无接触测试；
* 不要让机器人无人值守运行；
* 只有在机械臂稳定且现场人员准备好的情况下，才输入 `s`。

如果机器人出现异常加速、碰撞环境，或频繁报告伺服模式错误，应立即停止程序，并通过 Kinova Web App 或标准恢复流程恢复机器人。

## 常见问题

### `ModuleNotFoundError: No module named 'kortex_api'`

请安装 Kinova 官方 Kortex Python API，并确认当前 Python 环境可以导入 `kortex_api`。

### `Missing Kinova connection settings`

请根据 `.env.example` 创建 `.env`，并填写机器人 IP、用户名和密码。

### `WRONG_SERVOING_MODE`

可能有其他程序正在控制机器人，或机器人未处于期望的底层伺服模式。请停止其他控制程序，恢复机器人后重新运行。

### Baseline database not found

即使没有无接触基准数据库，控制器也可以运行。此时基准缩放功能会被关闭，程序会继续执行。也可以使用：

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD> --no-baseline
```

## 引用

如果本代码用于学术研究，请在论文发表后引用对应文章。

```bibtex
@article{yanwen_ats_impedance_kinova,
  title   = {Adaptive Takagi-Sugeno Impedance Control for Human-Robot Interaction},
  author  = {Yanwen et al.},
  journal = {To be updated},
  year    = {2026}
}
```

