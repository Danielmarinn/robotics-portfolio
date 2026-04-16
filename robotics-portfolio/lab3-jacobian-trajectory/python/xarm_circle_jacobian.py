#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Exercício 2 – controlo em velocidade de juntas (Jacobiano)
Lite6: seguimento de um círculo no espaço de tarefa.
"""

# Script para gerar um círculo no plano XY em task-space
# usando o Jacobiano e controlo em velocidade de juntas.

import time
import math
import numpy as np
from xarm.wrapper import XArmAPI
from xarm import version

# ---------------- Parâmetros gerais ----------------
XARM_IP = "192.168.1.182"  # IP do robô

# Círculo no espaço de tarefa (mm e rad/s)
CX, CY, CZ = 250.0, 0.0, 300.0
R = 30.0
ALPHA_DOT = math.pi / 2.0  # rad/s
N_CYCLES = 1
DT = 0.02  # s (≈ 50 Hz)

T_TOTAL = N_CYCLES * 2.0 * math.pi / ALPHA_DOT
N_STEPS = int(T_TOTAL / DT) + 1

# Ganhos de controlo
Kp_pos = 0.02  # [1/s] posição
Kp_ori = 0.5   # [1/s] orientação

# Limites e regularização
MAX_QDOT_DEG = 60.0
MAX_QDOT_RAD = math.radians(MAX_QDOT_DEG)
LAMBDA_DAMP = 1e-3

# Movimento em posição até ao ponto inicial
CIRC_SPEED = R * ALPHA_DOT
LINEAR_SPEED = CIRC_SPEED
MVACC = 1000

# ---------------- Modelo cinemático Lite6 ----------------
# Parâmetros DH (mm e graus) e offsets de theta
a = np.array([0.0, 200.0, 87.0, 0.0, 0.0, 0.0])
d = np.array([243.3, 0.0, 0.0, 227.6, 0.0, 61.5])
alpha_deg = np.array([-90.0, 180.0, 90.0, 90.0, -90.0, 0.0])
theta_offset_deg = np.array([0.0, -90.0, -90.0, 0.0, 0.0, 0.0])


def dh_tf_rad(theta, alpha_i, a_i, d_i):
    """Matriz de transformação homogénea DH (rad, mm)."""
    ct = np.cos(theta); st = np.sin(theta)
    ca = np.cos(alpha_i); sa = np.sin(alpha_i)
    T = np.array([
        [ ct, -st*ca,  st*sa, a_i*ct],
        [ st,  ct*ca, -ct*sa, a_i*st],
        [  0,     sa,     ca,    d_i],
        [  0,      0,      0,     1 ],
    ], dtype=float)
    return T

def fk_all_rad(q_rad):
    """
    FK completo:
      q_rad: array(6) em radianos (juntas do robô)
      devolve T06 (4x4) e lista de Ts intermédias
    """
    alpha  = np.deg2rad(alpha_deg)
    theta0 = np.deg2rad(theta_offset_deg)
    T = np.eye(4)
    Ts = [T.copy()]
    for i in range(6):
        theta_i = q_rad[i] + theta0[i]     # aplica offset de junta
        Ti = dh_tf_rad(theta_i, alpha[i], a[i], d[i])
        T = T @ Ti
        Ts.append(T.copy())
    return T, Ts

def geometric_jacobian_from_fk(q_rad):
    """Jacobiano geométrico 6x6 (Jv em cima, Jw em baixo)."""
    T06, Ts = fk_all_rad(q_rad)
    o_n = T06[0:3, 3]
    Jv = np.zeros((3, 6))
    Jw = np.zeros((3, 6))
    for i in range(6):
        Zi_1 = Ts[i][0:3, 2]      # eixo z do frame i-1
        Oi_1 = Ts[i][0:3, 3]      # origem do frame i-1
        Jv[:, i] = np.cross(Zi_1, (o_n - Oi_1))
        Jw[:, i] = Zi_1
    return np.vstack((Jv, Jw))     # 6x6

def damped_pseudoinverse(J, lam=LAMBDA_DAMP):
    """Inversa amortecida: J^+ = J^T (J J^T + lam^2 I)^{-1}."""
    JJt = J @ J.T
    reg = (lam ** 2) * np.eye(JJt.shape[0])
    return J.T @ np.linalg.inv(JJt + reg)

def desired_rotation_constant():
    """Rotação desejada: roll=pi, pitch=0, yaw=0 (XYZ)."""
    roll = math.pi
    Rx = np.array([
        [1,          0,           0],
        [0,  math.cos(roll), -math.sin(roll)],
        [0,  math.sin(roll),  math.cos(roll)],
    ])
    return Rx

R_DES = desired_rotation_constant()

def rotation_error_vec(R_des, R_act):
    """Erro de orientação em forma de eixo-ângulo (vetor 3x1)."""
    R_err = R_des @ R_act.T
    cos_ang = (np.trace(R_err) - 1.0) / 2.0
    cos_ang = np.clip(cos_ang, -1.0, 1.0)
    ang = math.acos(cos_ang)
    if abs(ang) < 1e-9:
        return np.zeros(3)
    rx = (R_err[2,1] - R_err[1,2])
    ry = (R_err[0,2] - R_err[2,0])
    rz = (R_err[1,0] - R_err[0,1])
    axis = (1.0/(2.0*math.sin(ang))) * np.array([rx, ry, rz])
    return axis * ang

# ---------------- FUNÇÕES DE APOIO AO xARM ----------------

def print_status(arm, titulo=""):
    if titulo:
        print("====", titulo, "====")
    try:
        print(f"  state      : {arm.state}")
        print(f"  mode       : {arm.mode}")
        print(f"  warn_code  : {arm.warn_code}")
        print(f"  error_code : {arm.error_code}")
        print(f"  angles(deg): {arm.angles}")
    except Exception as e:
        print("[print_status] erro a ler atributos:", e)

def move_to_home_and_start_point(arm):
    """Vai ao home de fábrica e depois ao ponto inicial do círculo."""
    # 1) home de fábrica
    print("[go_home] a mover para home de fábrica ...")
    ret = arm.move_gohome(wait=True)
    print("[go_home] ret =", ret)
    print_status(arm, "Depois de move_gohome")
    if ret != 0:
        print("[AVISO] move_gohome falhou (ret != 0).")
        return False
    time.sleep(0.5)

    # 2) ponto inicial do círculo (alpha=0)
    alpha0 = 0.0
    x0 = CX + R * math.cos(alpha0)
    y0 = CY + R * math.sin(alpha0)
    z0 = CZ
    print(f"[start_circle] a mover para ponto inicial do círculo: "
          f"x={x0:.1f}, y={y0:.1f}, z={z0:.1f}")
    ret = arm.set_position(
        x=x0,
        y=y0,
        z=z0,
        roll=180,     # graus (porque XArmAPI está em modo degrees)
        pitch=0,
        yaw=0,
        speed=LINEAR_SPEED,
        mvacc=MVACC,
        wait=True
    )
    print("[start_circle] set_position ->", ret)
    print_status(arm, "Depois de ir ao ponto inicial")
    if ret != 0:
        print("[AVISO] Falha ao ir ao ponto inicial do círculo.")
        return False
    time.sleep(0.2)
    return True

def run_task_space_circle_jacobian(arm):
    """Segue o círculo em task-space usando Jacobiano e velocidade de juntas."""
    print("\n[Jacobian] A iniciar controlo em velocidade de juntas (modo 4) ...")
    # Modo 4: controlo em velocidade de juntas
    ret = arm.set_mode(4)   # 4 = joint velocity control mode
    print("[set_mode(4)] ->", ret)
    ret = arm.set_state(0)  # 0 = ready
    print("[set_state(0)] ->", ret)
    time.sleep(0.2)
    print_status(arm, "Antes do loop Jacobiano")

    if arm.error_code != 0:
        print("[FATAL] error_code != 0 antes do loop Jacobiano.")
        return

    alpha0 = 0.0
    t_start = time.time()
    print(f"[Jacobian] dt={DT:.3f} s, N={N_STEPS}, T_total≈{T_TOTAL:.2f} s")

    # Ler juntas actuais no ponto inicial
    q_deg = arm.angles
    q_rad = np.deg2rad(np.array(q_deg[:6], dtype=float))

    # FK do modelo
    T06, Ts = fk_all_rad(q_rad)
    p_act_model = T06[0:3, 3]

    # Pose que o xArm acha que tem
    ret, tcp_pose = arm.get_position(is_radian=False)
    p_act_robot = np.array(tcp_pose[:3])

    print("---- VERIFICAÇÃO FK NO PONTO INICIAL ----")
    print("p (modelo DH)   [mm]:", p_act_model)
    print("p (xArm real)   [mm]:", p_act_robot)
    print("diferença (mm)      :", p_act_robot - p_act_model)
    print("-----------------------------------------")

    for k in range(N_STEPS):
        # Se aparecer erro durante o movimento, parar
        if arm.error_code != 0:
            print("[Jacobian] error_code != 0 durante o movimento, a abortar.")
            print_status(arm, "Estado ao abortar")
            break

        t = k * DT
        alpha = alpha0 + ALPHA_DOT * t

        # Trajetória desejada no espaço de tarefa
        px_des = CX + R * math.cos(alpha)
        py_des = CY + R * math.sin(alpha)
        pz_des = CZ
        p_des = np.array([px_des, py_des, pz_des])

        vx_des = -R * math.sin(alpha) * ALPHA_DOT
        vy_des =  R * math.cos(alpha) * ALPHA_DOT
        vz_des = 0.0
        v_des = np.array([vx_des, vy_des, vz_des])
        w_des = np.zeros(3)

        # 2) Ler juntas actuais e converter para rad
        q_deg = arm.angles
        if q_deg is None or len(q_deg) < 6:
            print("[Jacobian] Não consegui ler arm.angles, a parar.")
            break
        q_rad = np.deg2rad(np.array(q_deg[:6], dtype=float))

        # 3) FK e Jacobiano
        T06, Ts = fk_all_rad(q_rad)
        p_act = T06[0:3, 3]
        R_act = T06[0:3, 0:3]
        J = geometric_jacobian_from_fk(q_rad)

        # 4) Erros de posição e orientação
        p_err = p_des - p_act                     # mm
        e_rot = rotation_error_vec(R_DES, R_act)  # rad

        # 5) Twist de comando (feedforward + termos proporcionais)
        twist_cmd = np.hstack((
            v_des + Kp_pos * p_err,
            w_des + Kp_ori * e_rot
        )) 

        # 6) Inversa amortecida do Jacobiano
        J_pinv = damped_pseudoinverse(J, LAMBDA_DAMP)

        # 7) Velocidade de juntas
        qdot_rad = J_pinv @ twist_cmd

        # 8) Saturar velocidades para segurança (em rad/s)
        qdot_rad = np.clip(qdot_rad, -MAX_QDOT_RAD, MAX_QDOT_RAD)

        # Converter para graus/s
        qdot_deg = np.rad2deg(qdot_rad)

        # 9) Enviar comando de velocidade de juntas em graus/s
        ret = arm.vc_set_joint_velocity(qdot_deg.tolist(), is_sync=True)

        if ret != 0:
            print(f"[Jacobian] vc_set_joint_velocity falhou no passo {k} com ret={ret}")
            print_status(arm, "Estado no erro de vc_set_joint_velocity")
            break

        # 10) Esperar até ao próximo passo
        t_next = t_start + (k + 1) * DT
        restante = t_next - time.time()
        if restante > 0:
            time.sleep(restante)

    # Parar o robô (velocidade nula)
    try:
        arm.vc_set_joint_velocity([0, 0, 0, 0, 0, 0], is_radian=True)
    except Exception:
        pass
    print("[Jacobian] Loop terminado; velocidade de juntas mandada para zero.")

# ---------------- MAIN ----------------

def main():
    print("SDK_VERSION:", version.__version__)
    arm = XArmAPI(XARM_IP)   # API em graus

    print_status(arm, "Depois de criar XArmAPI")

    # Se arrancar logo com erro, abortar
    if arm.error_code != 0:
        print("[FATAL] error_code != 0 ao ligar.")
        print("       Limpa o erro no teach pendant / software UFactory e tenta de novo.")
        arm.disconnect()
        return

    # Ativar movimento em modo posição (0)
    ret = arm.motion_enable(enable=True)
    print("[motion_enable] ->", ret)
    ret = arm.set_mode(0)
    print("[set_mode(0)] ->", ret)
    ret = arm.set_state(0)
    print("[set_state(0)] ->", ret)

    time.sleep(0.5)
    print_status(arm, "Depois de motion_enable + set_mode + set_state")

    if arm.error_code != 0:
        print("[FATAL] error_code != 0 depois de init, a abortar.")
        print_status(arm, "Antes de abortar")
        arm.disconnect()
        return

    # 2) Home de fábrica + ponto inicial do círculo (task-space)
    ok = move_to_home_and_start_point(arm)
    if not ok:
        print("[FATAL] Não foi possível ir ao ponto inicial do círculo.")
        arm.disconnect()
        return

    # 3) Loop de controlo no espaço de tarefa usando Jacobiano
    try:
        run_task_space_circle_jacobian(arm)
    except Exception as e:
        print("[EXCEPTION] Erro no loop Jacobiano:", e)
        print_status(arm, "Estado após excepção no loop Jacobiano")

    # 4) Voltar a modo 0 e home no fim (opcional mas recomendado)
    print("\n[go_home] a voltar a home de fábrica ...")
    try:
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.2)
        ret = arm.move_gohome(wait=True)
        print("[go_home] ret =", ret)
        print_status(arm, "Depois de voltar a home")
    except Exception as e:
        print("[AVISO] Erro ao voltar a home:", e)

    # 5) Desligar
    print("[cleanup] A desligar o robô...")
    try:
        arm.set_state(4)  # stop
    except Exception:
        pass
    arm.disconnect()

if __name__ == "__main__":
    main()
