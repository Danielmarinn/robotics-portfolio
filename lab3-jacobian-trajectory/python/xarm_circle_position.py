#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
exercise2_circle.py

Exercicio 2 – primeira versão:
  - Controlo em posição no espaço de tarefa
  - Trajectória circular:
        x = Cx + r*cos(alpha)
        y = Cy + r*sin(alpha)
        z = constante
        roll = 180°, pitch = 0°, yaw = 0°
  - Implementado com set_position (IK interna do xArm)

Passos:
  1) Ligar ao Lite6
  2) motion_enable + set_mode(0) + set_state(0)
  3) move_gohome()
  4) Ir ao ponto inicial do círculo
  5) Executar um círculo completo
"""

import time
import math
from xarm.wrapper import XArmAPI
from xarm import version

# ---------- CONFIGURAÇÕES ----------
XARM_IP = "192.168.1.182"   # IP do robô

# Parâmetros da trajectória (tudo em mm e rad/s)
CX = 200.0     # centro do círculo em X (mm)
CY = 0.0       # centro do círculo em Y (mm)
CZ = 200.0     # altura constante (mm)
R = 100.0       # raio do círculo (mm)

ALPHA_DOT = math.pi / 2.0   # rad/s (como no enunciado)
N_CYCLES = 1                # quantos círculos queremos fazer
DT = 0.02                   # passo de tempo (s) ~ 50 Hz

T_TOTAL = N_CYCLES * 2.0 * math.pi / ALPHA_DOT
N_STEPS = int(T_TOTAL / DT) + 1

# Velocidade de movimento do robô (mm/s) – próxima de r*alpha_dot
CIRC_SPEED = R * ALPHA_DOT   # ≈ 78 mm/s para R=50
LINEAR_SPEED = CIRC_SPEED
MVACC = 1000                 # aceleração “grande mas razoável”


def print_status(arm, titulo=""):
    if titulo:
        print("====", titulo, "====")
    try:
        print(f"  state      : {arm.state}")
        print(f"  mode       : {arm.mode}")
        print(f"  warn_code  : {arm.warn_code}")
        print(f"  error_code : {arm.error_code}")
        print(f"  angles     : {arm.angles}")
    except Exception as e:
        print("[print_status] erro a ler atributos:", e)


def main():
    print("SDK_VERSION:", version.__version__)
    arm = XArmAPI(XARM_IP)

    print_status(arm, "Depois de criar XArmAPI")

    # Se arrancar logo com erro, abortamos
    if arm.error_code != 0:
        print("[FATAL] error_code != 0 ao ligar.")
        print("       Limpa o erro no teach pendant / software UFactory e tenta de novo.")
        arm.disconnect()
        return

    # 1) Habilitar movimento e modo 0 (posição) – padrão do SDK
    ret = arm.motion_enable(enable=True)
    print("[motion_enable] ->", ret)

    ret = arm.set_mode(0)     # 0 = position mode (set_position)
    print("[set_mode(0)] ->", ret)

    ret = arm.set_state(0)    # 0 = ready
    print("[set_state(0)] ->", ret)

    time.sleep(0.5)
    print_status(arm, "Depois de motion_enable + set_mode + set_state")

    if arm.error_code != 0:
        print("[FATAL] error_code != 0 depois de init, a abortar.")
        print_status(arm, "Antes de abortar")
        arm.disconnect()
        return

    # 2) Ir para home de fábrica
    print("[go_home] a mover para home de fábrica ...")
    ret = arm.move_gohome(wait=True)
    print("[go_home] ret =", ret)
    print_status(arm, "Depois de move_gohome")
    if ret != 0:
        print("[AVISO] move_gohome falhou (ret != 0).")
        arm.disconnect()
        return

    time.sleep(0.5)

    # 3) Ponto inicial do círculo (alpha = 0)
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
        roll=180,     # roll = pi (equivalente a -180)
        pitch=0,
        yaw=0,
        speed=LINEAR_SPEED,
        mvacc=MVACC,
        wait=True
    )
    print("[start_circle] set_position ->", ret)
    if ret != 0:
        print("[AVISO] Falha ao ir ao ponto inicial do círculo.")
        arm.disconnect()
        return

    time.sleep(0.2)

    # 4) Executar o círculo – trajectória no espaço de tarefa
    print(f"[circle] a executar {N_CYCLES} círculo(s), "
          f"T_total ≈ {T_TOTAL:.2f} s, dt={DT:.3f} s, N={N_STEPS}")

    t0 = time.time()
    for k in range(N_STEPS):
        t = k * DT
        alpha = alpha0 + ALPHA_DOT * t

        x = CX + R * math.cos(alpha)
        y = CY + R * math.sin(alpha)
        z = CZ

        # Orientação constante: roll=180°, pitch=0°, yaw=0°
        ret = arm.set_position(
            x=x,
            y=y,
            z=z,
            roll=180,
            pitch=0,
            yaw=0,
            speed=LINEAR_SPEED,
            mvacc=MVACC,
            wait=False
        )
        if ret != 0:
            print(f"[circle] set_position falhou no passo {k} com ret={ret}")
            print_status(arm, "Estado no erro durante o círculo")
            break

        # Manter ritmo dt (aproximado)
        t_next = t0 + (k + 1) * DT
        restante = t_next - time.time()
        if restante > 0:
            time.sleep(restante)

    print("[circle] trajectória terminada (ou interrompida).")

    # 5) Opcional: voltar ao home
    print("[go_home] a voltar a home de fábrica ...")
    ret = arm.move_gohome(wait=True)
    print("[go_home] ret =", ret)
    print_status(arm, "Depois de voltar a home")

    # 6) Desligar
    print("[cleanup] A desligar o robô...")
    try:
        arm.set_state(4)  # stop
    except Exception:
        pass
    arm.disconnect()


if __name__ == "__main__":
    main()
