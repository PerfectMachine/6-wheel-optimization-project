import mujoco
import math
import mujoco_viewer
import matplotlib.pyplot as plt
import numpy as np
import os
import lxml
from lxml import etree
import mujoco.viewer
import time
import csv

visualisationActive = False  # Set to True if want to see visualisation

# Writing results into csv file
csv_filename = "results.csv"
with open(csv_filename, mode='w', newline='', encoding='utf-8') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["Итерация","Тип подвески","Xmech1","Xmech2","Xmech3","","Плавность хода","Энергопотребление","Угол крена","Угол дифферента","Проходимость","Итоговая Парето"])

# Parcing, belongs to Egor Rakshin
def swap_par(tree, element_type, element_name, attribute_name, new_value):
    element = tree.find(f'.//{element_type}[@name="{element_name}"]')
    element.set(attribute_name, new_value)

# Torque function
def set_torque(mj_data, KP, KI, KV, targetPos, dt, inte_limit):
    global integral_x_err

    # Интегрируем ошибку по X
    integral_x_err += (targetPos - mj_data.qpos[0]) * dt
    if integral_x_err > inte_limit:
        integral_x_err = inte_limit
    elif integral_x_err < -inte_limit:
        integral_x_err = -inte_limit

    if (mj_data.qpos[1] - 0) > 1:
        data.ctrl[1] = 0
        data.ctrl[3] = 0
        data.ctrl[5] = 0
        data.ctrl[0] = 10 * KI * integral_x_err + KV * (0 - mj_data.qvel[0]) + 2 * mj_data.qvel[5]
        data.ctrl[2] = 10 * KI * integral_x_err + KV * (0 - mj_data.qvel[0]) + 2 * mj_data.qvel[5]
        data.ctrl[4] = 10 * KI * integral_x_err + KV * (0 - mj_data.qvel[0]) + 2 * mj_data.qvel[5]

    elif (mj_data.qpos[1] - 0) < -1:
        data.ctrl[0] = 0
        data.ctrl[2] = 0
        data.ctrl[4] = 0
        data.ctrl[1] = 10 * KI * integral_x_err + KV * (0 - mj_data.qvel[0]) + 2 * mj_data.qvel[5]
        data.ctrl[3] = 10 * KI * integral_x_err + KV * (0 - mj_data.qvel[0]) + 2 * mj_data.qvel[5]
        data.ctrl[5] = 10 * KI * integral_x_err + KV * (0 - mj_data.qvel[0]) + 2 * mj_data.qvel[5]

    else:
        data.ctrl[0] = 10 * KP * (targetPos - mj_data.qpos[0]) + KI * integral_x_err + KV * (0 - mj_data.qvel[0])
        data.ctrl[2] = 10 * KP * (targetPos - mj_data.qpos[0]) + KI * integral_x_err + KV * (0 - mj_data.qvel[0])
        data.ctrl[4] = 10 * KP * (targetPos - mj_data.qpos[0]) + KI * integral_x_err + KV * (0 - mj_data.qvel[0])
        data.ctrl[1] = 10 * KP * (targetPos - mj_data.qpos[0]) + KI * integral_x_err + KV * (0 - mj_data.qvel[0])
        data.ctrl[3] = 10 * KP * (targetPos - mj_data.qpos[0]) + KI * integral_x_err + KV * (0 - mj_data.qvel[0])
        data.ctrl[5] = 10 * KP * (targetPos - mj_data.qpos[0]) + KI * integral_x_err + KV * (0 - mj_data.qvel[0])

targetX = 120
delay = 1
duration = 30 + delay
suspensionType = 0
integral_x_err = 0

itr = 1

KP = 35
KI = 10
KV = 0.01

parsStep = 0.05

#Pareto Wieghts
w1 = 0.3
w2 = 0.0001
w3 = 0.7
w4 = 0.7
w5 = 100000000000000

# Double Wishbone params
armLen = 0.15
wheelSites = 0.05
wishDamp = 1

#Rocker Bogie params
rockerStiff = 10
rockerL1 = 0.5
rockerL2 = 0.5

#Torsion params
springRef = 0
torsionDiam = 0.02
torsionDamp = 1


while suspensionType < 3:
    if suspensionType == 0:
        while armLen < 0.501:
            while wheelSites < 0.21:
                while wishDamp < 20.1:
                    f1 = "doubleWishbone.xml"
                    f2 = "currentModel.xml"

                    tree = etree.parse(f1)
                    # Parsing of arms
                    swap_par(tree, 'geom', 'upLArm1', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'upLArm1', 'pos', f"0 {armLen} 0")
                    swap_par(tree, 'body', 'upLArm1', 'pos', f"0 {2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'downLArm1', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'downLArm1', 'pos', f"0 {armLen} 0")
                    swap_par(tree, 'body', 'downLArm1', 'pos', f"0 {2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'upRArm1', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'upRArm1', 'pos', f"0 -{armLen} 0")
                    swap_par(tree, 'body', 'upRArm1', 'pos', f"0 -{2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'downRArm1', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'downRArm1', 'pos', f"0 -{armLen} 0")
                    swap_par(tree, 'body', 'downRArm1', 'pos', f"0 -{2*armLen + 0.03} 0")

                    swap_par(tree, 'body', 'wheelL1', 'pos', f"0.1 {0.35 + 2 * armLen} 0")
                    swap_par(tree, 'body', 'wheelR1', 'pos', f"0.1 -{0.35 + 2 * armLen} 0")

                    swap_par(tree, 'geom', 'upLArm2', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'upLArm2', 'pos', f"0 {armLen} 0")
                    swap_par(tree, 'body', 'upLArm2', 'pos', f"0 {2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'downLArm2', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'downLArm2', 'pos', f"0 {armLen} 0")
                    swap_par(tree, 'body', 'downLArm2', 'pos', f"0 {2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'upRArm2', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'upRArm2', 'pos', f"0 -{armLen} 0")
                    swap_par(tree, 'body', 'upRArm2', 'pos', f"0 -{2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'downRArm2', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'downRArm2', 'pos', f"0 -{armLen} 0")
                    swap_par(tree, 'body', 'downRArm2', 'pos', f"0 -{2*armLen + 0.03} 0")

                    swap_par(tree, 'body', 'wheelL2', 'pos', f"0.1 {0.35 + 2 * armLen} 0")
                    swap_par(tree, 'body', 'wheelR2', 'pos', f"0.1 -{0.35 + 2 * armLen} 0")

                    swap_par(tree, 'geom', 'upLArm3', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'upLArm3', 'pos', f"0 {armLen} 0")
                    swap_par(tree, 'body', 'upLArm3', 'pos', f"0 {2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'downLArm3', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'downLArm3', 'pos', f"0 {armLen} 0")
                    swap_par(tree, 'body', 'downLArm3', 'pos', f"0 {2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'upRArm3', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'upRArm3', 'pos', f"0 -{armLen} 0")
                    swap_par(tree, 'body', 'upRArm3', 'pos', f"0 -{2*armLen + 0.03} 0")
                    swap_par(tree, 'geom', 'downRArm3', 'size', f"0.03 {armLen} 0.03")
                    swap_par(tree, 'geom', 'downRArm3', 'pos', f"0 -{armLen} 0")
                    swap_par(tree, 'body', 'downRArm3', 'pos', f"0 -{2*armLen + 0.03} 0")

                    swap_par(tree, 'body', 'wheelL3', 'pos', f"0.1 {0.35 + 2 * armLen} 0")
                    swap_par(tree, 'body', 'wheelR3', 'pos', f"0.1 -{0.35 + 2 * armLen} 0")

                    # Parsing of wheels
                    swap_par(tree, 'site', 'connectLUp1', 'pos', f"0 -0.05 {wheelSites}")
                    swap_par(tree, 'site', 'connectLDown1', 'pos', f"0 -0.05 -{wheelSites}")
                    swap_par(tree, 'site', 'connectRUp1', 'pos', f"0 0.05 {wheelSites}")
                    swap_par(tree, 'site', 'connectRDown1', 'pos', f"0 0.05 -{wheelSites}")

                    swap_par(tree, 'site', 'connectLUp2', 'pos', f"0 -0.05 {wheelSites}")
                    swap_par(tree, 'site', 'connectLDown2', 'pos', f"0 -0.05 -{wheelSites}")
                    swap_par(tree, 'site', 'connectRUp2', 'pos', f"0 0.05 {wheelSites}")
                    swap_par(tree, 'site', 'connectRDown2', 'pos', f"0 0.05 -{wheelSites}")

                    swap_par(tree, 'site', 'connectLUp3', 'pos', f"0 -0.05 {wheelSites}")
                    swap_par(tree, 'site', 'connectLDown3', 'pos', f"0 -0.05 -{wheelSites}")
                    swap_par(tree, 'site', 'connectRUp3', 'pos', f"0 0.05 {wheelSites}")
                    swap_par(tree, 'site', 'connectRDown3', 'pos', f"0 0.05 -{wheelSites}")

                    # Parsing of damping
                    swap_par(tree, 'joint', 'downLArm1', 'damping', f"{wishDamp}")
                    swap_par(tree, 'joint', 'downRArm1', 'damping', f"{wishDamp}")
                    swap_par(tree, 'joint', 'downLArm2', 'damping', f"{wishDamp}")
                    swap_par(tree, 'joint', 'downRArm2', 'damping', f"{wishDamp}")
                    swap_par(tree, 'joint', 'downLArm3', 'damping', f"{wishDamp}")
                    swap_par(tree, 'joint', 'downRArm3', 'damping', f"{wishDamp}")

                    tree.write(f2, pretty_print=True, xml_declaration=True, encoding='UTF-8')

                    print("============")
                    print(f"Iteration №{itr}")
                    print(f"Suspension: Double Wishbone")
                    print(f"armLen = {armLen}")
                    print(f"wheelSites = {wheelSites}")
                    print(f"Damping = {wishDamp}")
                    print("============")

                    model = mujoco.MjModel.from_xml_path(f2)
                    data = mujoco.MjData(model)
                    integral_x_err = 0.0

                    dt = model.opt.timestep
                    steps = int(duration / dt)

                    J1 = 0
                    J2 = 0
                    J3 = 0
                    J4 = 0
                    J5 = 1
                    sum_z_acc_sq = 0
                    sum_power = 0

                    wheel_joint_names = ['wheelL1', 'wheelR1', 'wheelL2', 'wheelR2', 'wheelL3', 'wheelR3']
                    wheel_dof_indices = [model.joint(name).dofadr[0] for name in wheel_joint_names]

                    if visualisationActive:
                        viewer = mujoco_viewer.MujocoViewer(model, data, title="doubleWishbone", width=1920, height=1080)
                    else:
                        viewer = None

                    for i in range(steps):
                        if visualisationActive and not viewer.is_alive:
                            break

                        qpos_x = data.qpos[0]
                        qpos_y = data.qpos[1]
                        qvel_x = data.qvel[0]
                        print(data.ctrl[:])
                            
                        if data.time < delay:
                            data.ctrl[:] = 0
                        else:
                            set_torque(data, KP, KI, KV, targetX, model.opt.timestep, 50)

                        z_acc = data.qacc[2]
                        sum_z_acc_sq += z_acc * z_acc

                        power = 0.0
                        for j in range(6):
                            tau = data.actuator_force[j]
                            omega = data.qvel[wheel_dof_indices[j]]
                            power += abs(tau * omega)
                        sum_power += power

                        quat = data.qpos[3:7]
                        rotmat = np.zeros(9)
                        mujoco.mju_quat2Mat(rotmat, quat)

                        pitch = -math.asin(rotmat[2])
                        roll  = math.atan2(rotmat[5], rotmat[8])

                        if roll > J3 and data.time > delay:
                            J3 = roll
                        if pitch > J4 and data.time > delay:
                            J4 = pitch

                        if abs(data.qpos[0] - targetX) < 0.05:
                            J5 = 0

                        mujoco.mj_step(model, data)
                        if visualisationActive:
                            viewer.render()

                    if visualisationActive:
                        viewer.close()

                    if steps > 0:
                        J1 = np.sqrt(sum_z_acc_sq * dt / duration)
                        J2 = sum_power * dt

                    print(f"J1={round(J1,4)}, J2={round(J2,4)}, J3={round(J3,4)}, J4={round(J4,4)}, J5={round(J5,4)}, Paretto={round(w1*J1+w2*J2+w3*J3+w4*J4+w5*J5,4)}")

                    with open(csv_filename, mode='a', newline='', encoding='utf-8') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([itr, "Double Wishbone",wishDamp,wheelSites,armLen,"",round(J1,4), round(J2,4), round(J3,4), round(J4,4), round(J5,4),round(w1*J1+w2*J2+w3*J3+w4*J4+w5*J5,4)])
                    itr += 1
                    wishDamp = round(wishDamp + parsStep*20, 2)

                wishDamp = 1
                wheelSites = round(wheelSites + parsStep, 2)

            wheelSites = 0.05
            armLen = round(armLen + parsStep, 2)

        print("Double Wishbone suspension is over!")
        suspensionType += 1
    if suspensionType == 1:
        while rockerL1 < 1.51:
            while rockerL2 < 1.51:
                while rockerStiff < 100.1:
                    f1 = "rockerBogie.xml"
                    f2 = "currentModel.xml"

                    tree = etree.parse(f1)
                    # Parsing of stiffness
                    swap_par(tree, 'joint', 'firstPair', 'stiffness', f"{rockerStiff}")
                    swap_par(tree, 'joint', 'secondPair', 'stiffness', f"{rockerStiff}")

                    # Parsing of L1
                    swap_par(tree, 'geom', 'rockerLL1', 'size', f"{rockerL1} 0.05 0.05")
                    swap_par(tree, 'body', 'connectionLL2Lead', 'pos', f"{rockerL1+0.3} 0. -0.33")
                    swap_par(tree, 'body', 'connectionLL2Mid', 'pos', f"-{rockerL1+0.17} 0 -0.19")

                    swap_par(tree, 'geom', 'rockerRL1', 'size', f"{rockerL1} 0.05 0.05")
                    swap_par(tree, 'body', 'connectionRL2Lead', 'pos', f"{rockerL1+0.3} 0 -0.33")
                    swap_par(tree, 'body', 'connectionRL2Mid', 'pos', f"-{rockerL1+0.17} 0 -0.19")

                    # Parsing of L2
                    swap_par(tree, 'geom', 'rockerLL2', 'size', f"{rockerL2} 0.05 0.05")
                    swap_par(tree, 'body', 'midConnectionL', 'pos', f"{rockerL1+0.15} 0 -0.17")
                    swap_par(tree, 'body', 'endConnectionL', 'pos', f"-{rockerL1+0.15} 0 -0.17")

                    swap_par(tree, 'geom', 'rockerRL2', 'size', f"{rockerL2} 0.05 0.05")
                    swap_par(tree, 'body', 'midConnectionR', 'pos', f"{rockerL1+0.15} 0 -0.17")
                    swap_par(tree, 'body', 'endConnectionR', 'pos', f"-{rockerL1+0.15} 0 -0.17")

                    tree.write(f2, pretty_print=True, xml_declaration=True, encoding='UTF-8')

                    print("============")
                    print(f"Iteration №{itr}")
                    print(f"Suspension: Rocker Bogie")
                    print(f"L1 = {rockerL1}")
                    print(f"L2 = {rockerL2}")
                    print(f"Stiffness = {rockerStiff}")
                    print("============")

                    model = mujoco.MjModel.from_xml_path(f2)
                    data = mujoco.MjData(model)

                    dt = model.opt.timestep
                    steps = int(duration / dt)

                    J1 = 0
                    J2 = 0
                    J3 = 0
                    J4 = 0
                    J5 = 1
                    sum_z_acc_sq = 0
                    sum_power = 0

                    wheel_joint_names = ['wheelLLead', 'wheelLMid', 'wheelLEnd', 'wheelRLead', 'wheelRMid', 'wheelREnd']
                    wheel_dof_indices = [model.joint(name).dofadr[0] for name in wheel_joint_names]

                    if visualisationActive:
                        viewer = mujoco_viewer.MujocoViewer(model, data, title="rockerBogie", width=1920, height=1080)
                    else:
                        viewer = None

                    for i in range(steps):
                        if visualisationActive and not viewer.is_alive:
                            break

                        qpos_x = data.qpos[0]
                        qpos_y = data.qpos[1]
                        qvel_x = data.qvel[0]
                            
                        if data.time < delay:
                            data.ctrl[:] = 0
                        else:
                            set_torque(data, KP, KI, KV, targetX, model.opt.timestep, 50)

                        z_acc = data.qacc[2]
                        sum_z_acc_sq += z_acc * z_acc

                        power = 0.0
                        for j in range(6):
                            tau = data.actuator_force[j]
                            omega = data.qvel[wheel_dof_indices[j]]
                            power += abs(tau * omega)
                        sum_power += power

                        quat = data.qpos[3:7]
                        rotmat = np.zeros(9)
                        mujoco.mju_quat2Mat(rotmat, quat)

                        pitch = -math.asin(rotmat[2])
                        roll  = math.atan2(rotmat[5], rotmat[8])

                        if roll > J3 and data.time > delay:
                            J3 = roll
                        if pitch > J4 and data.time > delay:
                            J4 = pitch
                        if abs(data.qpos[0] - targetX) < 0.05:
                            J5 = 0

                        mujoco.mj_step(model, data)
                        if visualisationActive:
                            viewer.render()

                    if visualisationActive:
                        viewer.close()

                    if steps > 0:
                        J1 = np.sqrt(sum_z_acc_sq * dt / duration)
                        J2 = sum_power * dt

                    print(f"J1={round(J1,4)}, J2={round(J2,4)}, J3={round(J3,4)}, J4={round(J4,4)}, J5={round(J5,4)}, Paretto={round(w1*J1+w2*J2+w3*J3+w4*J4+w5*J5,4)}")

                    with open(csv_filename, mode='a', newline='', encoding='utf-8') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([itr, "Rocker Bogie",rockerStiff,rockerL1,rockerL2,"",round(J1,4), round(J2,4), round(J3,4), round(J4,4), round(J5,4),round(w1*J1+w2*J2+w3*J3+w4*J4+w5*J5,4)])
                    itr += 1
                    rockerStiff = round(rockerStiff + parsStep*200, 2)

                rockerStiff = 10
                rockerL2 = round(rockerL2 + parsStep, 2)

            rockerL2 = 0.5
            rockerL1 = round(rockerL1 + parsStep, 2)

        print("Rocker Bogie suspension is over!")
        suspensionType += 1
    if suspensionType == 2:
        while torsionDamp < 20.1:
            while torsionDiam < 0.151:
                while springRef < 10.1:
                    f1 = "torsion.xml"
                    f2 = "currentModel.xml"

                    tree = etree.parse(f1)
                    # Parsing of springRef
                    swap_par(tree, 'joint', 'torsionL1', 'springref', f"{springRef}")
                    swap_par(tree, 'joint', 'torsionL2', 'springref', f"{springRef}")
                    swap_par(tree, 'joint', 'torsionL3', 'springref', f"{springRef}")

                    swap_par(tree, 'joint', 'torsionR1', 'springref', f"{springRef}")
                    swap_par(tree, 'joint', 'torsionR2', 'springref', f"{springRef}")
                    swap_par(tree, 'joint', 'torsionR3', 'springref', f"{springRef}")

                    # Parsing of torsion diameters
                    swap_par(tree, 'geom', 'torsionL1', 'size', f"{torsionDiam} 0.2")
                    swap_par(tree, 'geom', 'torsionL2', 'size', f"{torsionDiam} 0.2")
                    swap_par(tree, 'geom', 'torsionL3', 'size', f"{torsionDiam} 0.2")

                    swap_par(tree, 'geom', 'torsionR1', 'size', f"{torsionDiam} 0.2")
                    swap_par(tree, 'geom', 'torsionR2', 'size', f"{torsionDiam} 0.2")
                    swap_par(tree, 'geom', 'torsionR3', 'size', f"{torsionDiam} 0.2")

                    # Damping
                    swap_par(tree, 'joint', 'torsionL1', 'damping', f"{torsionDamp}")
                    swap_par(tree, 'joint', 'torsionL2', 'damping', f"{torsionDamp}")
                    swap_par(tree, 'joint', 'torsionL3', 'damping', f"{torsionDamp}")

                    swap_par(tree, 'joint', 'torsionR1', 'damping', f"{torsionDamp}")
                    swap_par(tree, 'joint', 'torsionR2', 'damping', f"{torsionDamp}")
                    swap_par(tree, 'joint', 'torsionR3', 'damping', f"{torsionDamp}")

                    tree.write(f2, pretty_print=True, xml_declaration=True, encoding='UTF-8')

                    print("============")
                    print(f"Iteration №{itr}")
                    print(f"Suspension: Torsion")
                    print(f"SpringRef = {springRef}")
                    print(f"torsionDiam = {torsionDiam}")
                    print(f"torsionDamp = {torsionDamp}")
                    print("============")

                    model = mujoco.MjModel.from_xml_path(f2)
                    data = mujoco.MjData(model)

                    dt = model.opt.timestep
                    steps = int(duration / dt)

                    J1 = 0
                    J2 = 0
                    J3 = 0
                    J4 = 0
                    J5 = 1
                    sum_z_acc_sq = 0
                    sum_power = 0

                    wheel_joint_names = ['wheelL1', 'wheelR1', 'wheelL2', 'wheelR2', 'wheelL3', 'wheelR3']
                    wheel_dof_indices = [model.joint(name).dofadr[0] for name in wheel_joint_names]

                    if visualisationActive:
                        viewer = mujoco_viewer.MujocoViewer(model, data, title="torsion", width=1920, height=1080)
                    else:
                        viewer = None

                    for i in range(steps):
                        if visualisationActive and not viewer.is_alive:
                            break

                        qpos_x = data.qpos[0]
                        qpos_y = data.qpos[1]
                        qvel_x = data.qvel[0]
                            
                        if data.time < delay:
                            data.ctrl[:] = 0
                        else:
                            set_torque(data, KP, KI, KV, targetX, model.opt.timestep, 50)

                        z_acc = data.qacc[2]
                        sum_z_acc_sq += z_acc * z_acc

                        power = 0.0
                        for j in range(6):
                            tau = data.actuator_force[j]
                            omega = data.qvel[wheel_dof_indices[j]]
                            power += abs(tau * omega)
                        sum_power += power

                        quat = data.qpos[3:7]
                        rotmat = np.zeros(9)
                        mujoco.mju_quat2Mat(rotmat, quat)

                        pitch = -math.asin(rotmat[2])
                        roll  = math.atan2(rotmat[5], rotmat[8])

                        if roll > J3 and data.time > delay:
                            J3 = roll
                        if pitch > J4 and data.time > delay:
                            J4 = pitch

                        if abs(data.qpos[0] - targetX) < 0.05:
                            J5 = 0

                        mujoco.mj_step(model, data)
                        if visualisationActive:
                            viewer.render()

                    if visualisationActive:
                        viewer.close()

                    if steps > int(delay / dt):
                        J1 = np.sqrt(sum_z_acc_sq * dt / duration)
                        J2 = sum_power * dt

                    print(f"J1={round(J1,4)}, J2={round(J2,4)}, J3={round(J3,4)}, J4={round(J4,4)}, J5={round(J5,4)}, Paretto={round(w1*J1+w2*J2+w3*J3+w4*J4+w5*J5,4)}")

                    with open(csv_filename, mode='a', newline='', encoding='utf-8') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow([itr, "Torsion",springRef,torsionDiam,torsionDamp,"",round(J1,4), round(J2,4), round(J3,4), round(J4,4), round(J5,4),round(w1*J1+w2*J2+w3*J3+w4*J4+w5*J5,4)])
                    itr += 1
                    springRef = round(springRef + parsStep*20, 2)

                springRef = 0
                torsionDiam = round(torsionDiam + (parsStep/5)*2, 2)

            torsionDiam = 0.02
            torsionDamp = round(torsionDamp + parsStep*20, 2)

        print("Rocker Bogie suspension is over!")
        suspensionType += 1
print("Simulation is over!")