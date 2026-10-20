"""Modelo calibrado: pixel da câmera <-> coordenada do robô (mm) <-> ângulos J1/J2.

Referencial do robô: origem no eixo do J1, x = direção do braço com J1 = 0, y = 90° anti-horário (visto de cima).
z = posição do eixo Z (mm) com a qual o topo da ponta (centro do rolamento) fica na altura considerada.
"""
import json
import math
import os

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))

# Alturas (mm). A ventosa encosta na mesa com o Z em Z_TOQUE_MESA (medido com régua).
Z_TOQUE_MESA = -135.0
# topo do rolamento (o que a câmera vê) acima da boca da ventosa. Estimado pelo "quase no centro"; AJUSTAR com o
# tabuleiro ou régua
MARCADOR_ACIMA_VENTOSA = 20.0
ALTURA_CAIXA = 30.0


class Modelo:
    def __init__(self, arq=os.path.join(AQUI, "calibracao_braco.json")):
        c = json.load(open(arq))
        self.P = np.array(c["P"])
        self.L1, self.L2, self.o2 = c["L1"], c["L2"], c["o2"]
        self.lim2 = tuple(c.get("lim2", (-112, 135)))     # limites do J2 no zero atual (gravados na calibração)
        self.parque = tuple(c.get("parque", (20.0, -110.0)))

    # ---- câmera ----
    def z_do_plano(self, altura_mm):
        """valor de z (no referencial do eixo Z) do plano horizontal a `altura_mm` acima da mesa"""
        return altura_mm + Z_TOQUE_MESA - MARCADOR_ACIMA_VENTOSA

    def H(self, z):
        P = self.P
        return np.c_[P[:, 0], P[:, 1], P[:, 3] + z * P[:, 2]]

    def robo_para_pixel(self, x, y, z=0.0):
        u = self.H(z) @ [x, y, 1.0]
        return u[0] / u[2], u[1] / u[2]

    def pixel_para_robo(self, u, v, z):
        X = np.linalg.solve(self.H(z), [u, v, 1.0])
        return X[0] / X[2], X[1] / X[2]

    def caixa_para_robo(self, u, v, altura_caixa=ALTURA_CAIXA):
        """centro da caixa visto na imagem (topo da caixa) -> x, y do robô"""
        return self.pixel_para_robo(u, v, self.z_do_plano(altura_caixa))

    # ---- cinemática ----
    def fk(self, j1, j2):
        a = math.radians(j1)
        b = math.radians(j1 + j2 + self.o2)
        return self.L1 * math.cos(a) + self.L2 * math.cos(b), self.L1 * math.sin(a) + self.L2 * math.sin(b)

    def ik_todas(self, x, y, lim1=(-130, 130), lim2=None):
        """todas as soluções (j1, j2) dentro dos limites para a ponta em (x, y)"""
        lim2 = lim2 or self.lim2
        r2 = x * x + y * y
        c2 = (r2 - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if abs(c2) > 1:
            return []
        sols = []
        for s in (1, -1):
            t2 = s * math.acos(c2)
            t1 = math.atan2(y, x) - math.atan2(self.L2 * math.sin(t2), self.L1 + self.L2 * math.cos(t2))
            for k in (-360, 0, 360):
                j1 = math.degrees(t1) + k
                j2 = math.degrees(t2) - self.o2
                j2 = (j2 + 180) % 360 - 180
                if lim1[0] <= j1 <= lim1[1] and lim2[0] <= j2 <= lim2[1]:
                    sols.append((j1, j2))
        return sols

    def ik(self, x, y, ref=None, lim1=(-130, 130), lim2=None, livre=None):
        """ângulos (j1, j2) para a ponta em (x, y): a solução dentro dos limites mais perto de `ref`;
        com `livre(j1, j2) -> bool`, só as soluções em que o braço não bate em nada"""
        sols = self.ik_todas(x, y, lim1, lim2)
        if livre:
            sols = [q for q in sols if livre(*q)]
        if not sols:
            raise ValueError("sem solução livre dentro dos limites para (%.1f, %.1f)" % (x, y))
        ref = ref or (0.0, 0.0)
        return min(sols, key=lambda q: abs(q[0] - ref[0]) + abs(q[1] - ref[1]))

    def cotovelo(self, j1):
        a = math.radians(j1)
        return self.L1 * math.cos(a), self.L1 * math.sin(a)
