#! /usr/bin/env python
# coding=utf-8
import math
from robotcontrol import *

class aubo_test():
    def __init__(self):
        pass
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def main(test_count):
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))
    try:

        # 链接服务器
        ip = '192.168.1.11'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            # robot.robot_shutdown()
            #
            # # 上电
            # robot.robot_startup()
            #
            # # 设置碰撞等级
            # robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

            # 获取控制柜用户DI
            io_config = robot.get_board_io_config(RobotIOType.User_DI)

            # 输出DI配置
            logger.info(io_config)

            # 获取控制柜用户DO
            io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # 循环测试
            while test_count > 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                logger.info("joint_status={0}".format(joint_status))

                # 初始化全局配置文件
                robot.init_profile()

                # 设置关节最大加速度
                robot.set_joint_maxacc((2.5, 2.5, 2.5, 2.5, 2.5, 2.5))

                # 设置关节最大加速度
                robot.set_joint_maxvelc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))
                # 设置关节最大加速度
                # robot.set_joint_maxacc((3.5, 3.5, 3.5, 3.5, 3.5, 3.5))
                #
                # # 设置关节最大加速度
                # robot.set_joint_maxvelc((1.5, .5, 2.5, 2.5, 2.5, 2.5))
                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.5)
                logger.info("-------go-----to-----start-------step--01")
                # 获取机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_velc(0.2)
                robot.set_end_max_line_velc(0.5)
                joint_radian = deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                fk_ret = robot.forward_kin(joint_radian)
                logger.info("fk--------")
                logger.info(fk_ret)

                # print fk_ret
                pos_test=(-0.5672477590258516, -0.51507448660946279, 0.57271770314023)
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("ik--------")#-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----left-point-------step--02")
                # print ik_result
                # joint_radian = deg_to_rad((-40.99,24.35,-74.62,-99.7,-53.8,0.164))
                joint_radian=(0.6519264479813265, 0.9655345419607202, -0.21383388879282367, -1.1296294269013867, -2.211889308626811, 0.03535820844149829)#(0.6283641729015998, 0.8566696288158852, -0.4217806828388399, -1.2295570370410864, -2.188355675678444, 0.033921459573828625)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # robot.move_line(joint_radian)

                # 获取关节最大加速度
                logger.info(robot.get_joint_maxacc())

                # print fk_ret
                pos_test=(-0.5672477590258516, 0.51507448660946279, 0.57271770314023)#the right
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("----ik----after--------")
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-point-------step--03--")
                joint_radian =(-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)

                fk_ret = robot.forward_kin(joint_radian)
                logger.info("-------step03--fk--------")
                logger.info(fk_ret)#'pos': [-0.5672477590258515, 0.5150744866094628, 0.57271770314023], 'ori': [0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817576]
                # print fk_ret
                pos_test=(-0.5672477590258515, 0.5150744866094628, 0.37)
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("step03---ik--------")#-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down-point-------step--04--")
                joint_radian =tuple(ik_result['joint'])#(-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)

                # print fk_ret
                pos_test=(-0.5672477590258515, -0.5150744866094628, 0.37)#-0.1,0.17,-0.1
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("step04---ik--------")#-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down---left-point-------step--05--")
                joint_radian =tuple(ik_result['joint'])#(-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)
                """-----------------------------------"""

                fk_ret = robot.forward_kin(joint_radian)
                logger.info("-------step03--fk--------")
                logger.info(fk_ret)#'pos': [-0.5672477590258515, 0.5150744866094628, 0.57271770314023], 'ori': [0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817576]
                # print fk_ret
                pos_test=(-0.5672477590258515, -0.5150744866094628, 0.17)
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("step03---ik--------")#-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down-point-------step--04--")
                joint_radian =tuple(ik_result['joint'])#(-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)

                # print fk_ret
                pos_test=(-0.5672477590258515, 0.5150744866094628, 0.17)#-0.1,0.17,-0.1
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("step04---ik--------")#-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down---left-point-------step--05--")
                joint_radian =tuple(ik_result['joint'])#(-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)
                """-----------------------------------"""

                fk_ret = robot.forward_kin(joint_radian)
                logger.info("-------step03--fk--------")
                logger.info(
                    fk_ret)  # 'pos': [-0.5672477590258515, 0.5150744866094628, 0.57271770314023], 'ori': [0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817576]
                # print fk_ret
                pos_test = (-0.5672477590258515, 0.5150744866094628, -0.1)
                ori_test = (0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info(
                    "step03---ik--------")  # -1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down-point-------step--04--")
                joint_radian = tuple(ik_result[
                                         'joint'])  # (-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)

                # print fk_ret
                pos_test = (-0.5672477590258515, -0.5150744866094628, -0.1)  # -0.1,0.17,-0.1
                ori_test = (0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info(
                    "step04---ik--------")  # -1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down---left-point-------step--05--")
                joint_radian = tuple(ik_result[
                                         'joint'])  # (-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)
                """-----------------------------------"""

                # print fk_ret
                pos_test=(-0.5672477590258515, -0.5150744866094628, 0.57)
                ori_test=(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("step05---ik--------")#-1.0029347552109935, 0.9543022727077345, -0.23537602810540328, -1.1144605036252324, -0.5588935386712407, -0.058211690935044025
                ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
                logger.info(ik_result)
                logger.info("-------go-----to-----right-down---left-up-point-------step--05--")
                joint_radian =tuple(ik_result['joint'])#(-1.0029347552109935, 1.7118641074963585, -0.5346747672652009, 0.9702715760161391, 0.5588935386712407, 3.0833809626547493)# (-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)
                #-1.0029347552109935, 1.4321478441170552, -0.8779999270231473, -2.2349299739522968, -0.5588935386712407, -0.058211690935044025
                # print robot.get_joint_maxacc()
                # 正解测试
                # fk_ret = robot.forward_kin((-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008))
                # logger.info(fk_ret)
                # print fk_ret
                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                # ik_result = robot.inverse_kin(joint_radian, fk_ret['pos'], fk_ret['ori'])
                # logger.info(ik_result)
                # print ik_result
                #
                # # 轴动1
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)
                #
                # # 轴动2
                # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)
                #
                # # 轴动3
                # joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                # logger.info("move joint to {0}".format(joint_radian))
                # robot.move_joint(joint_radian)



                # 清除所有已经设置的全局路点
                robot.remove_all_waypoint()
                #(-3.3364,12.406,-81.09,-91.207,-86.08,0.164)
                #(-40.99,24.35,-74.62,-99.7,-53.8,0.164)

                # 添加全局路点1,用于轨迹运动
                # joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                # robot.add_waypoint(joint_radian)
                #
                # # 添加全局路点2,用于轨迹运动
                # joint_radian = (-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680)
                # robot.add_waypoint(joint_radian)
                #
                # # 添加全局路点3,用于轨迹运动
                # joint_radian = (-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191)
                # robot.add_waypoint(joint_radian)
                #
                # # 设置圆运动圈数
                # robot.set_circular_loop_times(3)
                #
                # # 圆弧运动
                # logger.info("move_track ARC_CIR")
                # robot.move_track(RobotMoveTrackType.ARC_CIR)
                #
                # # 清除所有已经设置的全局路点
                # robot.remove_all_waypoint()

                # 机械臂轴动 回到0位
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

            # 断开服务器链接
            robot.disconnect()

    except RobotError, e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

if __name__=="__main__":
    main(1)
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))