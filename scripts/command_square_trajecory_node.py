#!/usr/bin/env python3

import rospy
import actionlib

from tobas_msgs.msg import (
    Odometry,
    PosVelAccYaw,
    TakeoffAction,
    TakeoffGoal,
    TakeoffResult,
    LandAction,
    LandGoal,
    LandResult,
)

ELEVATION = 3.0  # [m]
SIDE_LENGTH = 5.0  # [m]
INTERVAL = 5.0  # [s]
NUM_CYCLES = 3  # 周回数
WAIT_FOR_ACTION_SERVER = 10.0  # [s]


if __name__ == "__main__":
    # ROSノードの初期化
    rospy.init_node("command_square_trajectory")

    # アクションクライアントの作成
    takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)
    land_client = actionlib.SimpleActionClient("landing_action", LandAction)

    # アクションサーバーが起動するのを待つ
    rospy.loginfo("Waiting for action servers.")
    if not takeoff_client.wait_for_server(rospy.Duration(WAIT_FOR_ACTION_SERVER)):
        raise rospy.ROSException("Failed to connect to takeoff action server.")
    if not land_client.wait_for_server(rospy.Duration(WAIT_FOR_ACTION_SERVER)):
        raise rospy.ROSException("Failed to connect to landing action server.")

    # 現在の位置を取得
    rospy.loginfo("Getting the initial state.")
    init_odom: Odometry = rospy.wait_for_message("odom", Odometry)
    x0 = init_odom.frame.trans.x
    y0 = init_odom.frame.trans.y
    z0 = init_odom.frame.trans.z
    rospy.loginfo(f"Start position [m]: ({x0}, {y0}, {z0})")

    # 軌跡を描く高度
    altitude = z0 + ELEVATION

    # 離陸アクションゴールを作成
    takeoff_goal = TakeoffGoal()
    takeoff_goal.target_altitude = altitude
    takeoff_goal.target_duration = INTERVAL

    # 離陸アクションゴールを送信
    rospy.loginfo("Requesting takeoff.")
    takeoff_client.send_goal_and_wait(takeoff_goal)

    # 離陸アクションの結果を取得
    takeoff_result: TakeoffResult = takeoff_client.get_result()
    if takeoff_result.error_code < 0:
        rospy.logerr("Takeoff action failed.")
        exit()

    # コマンドのパブリッシャーを作成
    command_pub = rospy.Publisher("command/pos_vel_acc_yaw", PosVelAccYaw, queue_size=1)

    # 正方形の頂点を指令
    rospy.loginfo("Start to navigate along the edges of a square.")
    for _ in range(NUM_CYCLES):
        # 頂点1
        command = PosVelAccYaw()
        command.pos.x = x0 + SIDE_LENGTH / 2
        command.pos.y = y0 + SIDE_LENGTH / 2
        command.pos.z = altitude
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点2
        command = PosVelAccYaw()
        command.pos.x = x0 - SIDE_LENGTH / 2
        command.pos.y = y0 + SIDE_LENGTH / 2
        command.pos.z = altitude
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点3
        command = PosVelAccYaw()
        command.pos.x = x0 - SIDE_LENGTH / 2
        command.pos.y = y0 - SIDE_LENGTH / 2
        command.pos.z = altitude
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

        # 頂点4
        command = PosVelAccYaw()
        command.pos.x = x0 + SIDE_LENGTH / 2
        command.pos.y = y0 - SIDE_LENGTH / 2
        command.pos.z = altitude
        command_pub.publish(command)
        rospy.sleep(INTERVAL)

    # ホームポジションを指令
    rospy.loginfo("Returning to the start position.")
    command = PosVelAccYaw()
    command.pos.x = x0
    command.pos.y = y0
    command.pos.z = altitude
    command_pub.publish(command)
    rospy.sleep(INTERVAL)

    # 離陸アクションゴールを作成
    land_goal = LandGoal()

    # 離陸アクションゴールを送信
    rospy.loginfo("Requesting landing.")
    land_client.send_goal_and_wait(land_goal)

    # 離陸アクションの結果を取得
    land_result: LandResult = land_client.get_result()
    if land_result.error_code < 0:
        rospy.logerr("Landing action failed.")
        exit()

    rospy.loginfo("Finished successfully.")
