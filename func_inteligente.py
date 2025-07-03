#!/usr/bin/env pybricks-micropython

# Importação das bibliotecas necessárias para o controle dos motores e sensores EV3
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
import math
import time

# Inicializa o bloco EV3 e os motores nas portas A (esquerda) e B (direita)
ev3 = EV3Brick()
motor_esquerda = Motor(Port.A)
motor_direita = Motor(Port.B)

# Inicializa os sensores de cor nas portas S2 (esquerdo) e S3 (direito)
sensor_esq = ColorSensor(Port.S2)
sensor_dir = ColorSensor(Port.S3)

# Parâmetros físicos do robô
D = 0.0555  # Diâmetro da roda (m)
R = D / 2.0 # Raio da roda (m)
L = 0.152   # Distância entre as rodas (m)

# Variáveis de tempo e odometria
dt = 0.1     # Intervalo de tempo (s)
t = 0.0
t_last = time.time()

xp = yp = phip = x = y = phi = 0  # Estados da odometria

# Função que converte velocidades linear (u) e angular (w) para velocidades de roda
def cmd_uniciclo(u, w):
    # Calcula velocidades angulares das rodas (rad/s)
    wd_ref = (2.0 * u + w * L) / (2.0 * R)
    we_ref = (2.0 * u - w * L) / (2.0 * R)

    # Converte rad/s para graus/s e aplica aos motores
    motor_direita.run(wd_ref * 180 / math.pi)
    motor_esquerda.run(we_ref * 180 / math.pi)

# Função para parar o robô
def parar_uniciclo():
    motor_direita.brake()
    motor_esquerda.brake()

# Leitura das velocidades atuais das rodas (em rad/s)
def ler_velocidades_rodas():
    we = math.radians(motor_esquerda.speed())
    wd = math.radians(motor_direita.speed())
    return we, wd

# Atualiza a odometria com base nas velocidades das rodas
def processar_odometria(wd_a, we_a, x_a, y_a, phi_a):
    t_xp = R/2 * (we_a + wd_a) * math.cos(phi_a)
    t_yp = R/2 * (we_a + wd_a) * math.sin(phi_a)
    t_phip = R/L * (wd_a - we_a)
    t_x = x_a + t_xp * dt
    t_y = y_a + t_yp * dt
    t_phi = phi_a + t_phip * dt
    return t_xp, t_yp, t_phip, t_x, t_y, t_phi

# ========= SISTEMA FUZZY ===========

# UNIVERSO DE DISCURSO:
# Refletância dos sensores: de 0 (preto) a 100 (branco)

# FUNÇÕES DE PERTINÊNCIA:
def preto(valor):
    # Totalmente preto até 20, decrescendo até 40
    if valor < 20:
        return 1.0
    elif valor < 40:
        return (40 - valor) / 20
    else:
        return 0.0

def branco(valor):
    # Totalmente branco acima de 80, decrescendo até 60
    if valor > 80:
        return 1.0
    elif valor > 60:
        return (valor - 60) / 20
    else:
        return 0.0

def cinza(valor):
    # Cinza é uma função triangular entre 30 e 70 com pico em 50
    if 30 < valor < 70:
        if valor < 50:
            return (valor - 30) / 20
        else:
            return (70 - valor) / 20
    else:
        return 0.0

# LÓGICA FUZZY: CÁLCULO DA VELOCIDADE ANGULAR 'w'
def fuzzy_w(leitura_esq, leitura_dir):
    # GRAUS DE PERTINÊNCIA (mu):
    mu_preto_esq = preto(leitura_esq)
    mu_branco_esq = branco(leitura_esq)
    mu_cinza_esq = cinza(leitura_esq)

    mu_preto_dir = preto(leitura_dir)
    mu_branco_dir = branco(leitura_dir)
    mu_cinza_dir = cinza(leitura_dir)

    # REGRAS FUZZY:
    # Regra 1: Se Esq=Preto e Dir=Branco, virar para a direita (+1.0 rad/s)
    # Regra 2: Se Esq=Branco e Dir=Preto, virar para a esquerda (-1.0 rad/s)
    # Regra 3: Se Esq=Cinza e Dir=Cinza, seguir em frente (0.0 rad/s)

    # CLIPPING + AGREGAÇÃO: cálculo ponderado das regras
    peso_esq = mu_preto_esq * mu_branco_dir
    peso_dir = mu_branco_esq * mu_preto_dir
    peso_reto = mu_cinza_esq * mu_cinza_dir

    numerador = (peso_esq * 1.0) + (peso_dir * -1.0) + (peso_reto * 0.0)
    denominador = peso_esq + peso_dir + peso_reto

    # DEFUZZIFICAÇÃO: método do centro de massa (média ponderada)
    if denominador == 0:
        return 0
    return numerador / denominador

# ========== LOOP PRINCIPAL ==========

while True:
    t_atual = time.time()
    if (t_atual - t_last) >= dt:
        t = t + (t_atual - t_last)
        t_last = t_atual

        # ENTRADAS: leituras dos sensores de refletância
        leitura_esq = sensor_esq.reflection()
        leitura_dir = sensor_dir.reflection()

        # Velocidade linear constante (m/s)
        u = 0.1

        # Velocidade angular definida pelo sistema fuzzy
        w = fuzzy_w(leitura_esq, leitura_dir)

        # Aplica o comando ao robô
        cmd_uniciclo(u, w)

        # Atualiza a odometria
        we, wd = ler_velocidades_rodas()
        xp, yp, phip, x, y, phi = processar_odometria(wd, we, x, y, phi)

        # Exibe valores para depuração
        print("L: ({:.1f}, {:.1f}) | w = {:.2f} rad/s | x = {:.2f}".format(
            leitura_esq, leitura_dir, w, x
        ))

        # Para após 20 segundos
        #if t >= 20:
        #    parar_uniciclo()
        #    print("Parado. x = {:.2f}, y = {:.2f}, phi = {:.1f}°".format(x, y, math.degrees(phi)))
        #    break
