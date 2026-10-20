"""Depois de desligar/mover o braço com a mão: descobre J1 e J2 pela câmera, sem mexer as juntas.

Referencia o Z pela chave de cima, fica no alto, acha a ponta (rolamento) e o cotovelo (motor) na foto e escolhe a
solução da cinemática inversa cujo cotovelo bate com o visto. Grava as contagens (definir) e confere com um giro
pequeno do J1.

Uso: python localizar.py
"""
import math
import time

import cv2
import numpy as np

from cam import Cam
from modelo import Modelo
from ponta import acha_ponta, hexagonos, ponta_precisa
from scara import Z_CHAVE, Scara

Z_FOTO = 45.0      # altura da grade de calibração: a câmera acha bem a ponta e o braço passa por cima da coletora


def cotovelo_px(f, ponta, base):
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    cands = []
    for h in hexagonos(f)[0]:
        d = np.zeros_like(g)
        cv2.circle(d, tuple(int(v) for v in h["c"]), 18, 255, -1)
        brilho = cv2.mean(g, d)[0]
        if 45 < h["r"] < 130 and h["circ"] >= 0.45 and 280 < math.dist(h["c"], ponta) < 480 \
                and 450 < math.dist(h["c"], base) < 650 and brilho < 110:
            cands.append((brilho, h["c"]))
    return min(cands)[1] if cands else None


def main():
    r = Scara(fator_vel=1.0)
    c = Cam()
    m = Modelo()
    b = r.placas["braco"]
    try:
        if b.cmd("info")["fimcfg"]["f2"]:
            b.cmd("fim", f2=0)               # fim de curso do J2 desconectado: lê sempre "acionado"
        zc = r.referenciar_z()
        r.definir("z", Z_CHAVE)
        print("Z referenciado (chave em %.2f da contagem anterior)" % zc)
        r.ir("z", Z_FOTO)

        f = c.foto("fotos/localizar.jpg")
        pp = acha_ponta(f)
        if pp is None:
            raise RuntimeError("ponta não encontrada na foto: não movo as juntas")
        ponta = pp["c"]
        base = m.robo_para_pixel(0.0, 0.0, Z_FOTO)
        cot = cotovelo_px(f, ponta, base)
        x, y = m.pixel_para_robo(ponta[0], ponta[1], Z_FOTO)
        sols = m.ik_todas(x, y, lim1=(-180, 180), lim2=(-250, 250))
        if not sols:
            raise RuntimeError("ponta em (%.0f, %.0f) mm fora do alcance do modelo" % (x, y))
        if cot is not None:
            ce = m.pixel_para_robo(cot[0], cot[1], Z_FOTO)
            j1, j2 = min(sols, key=lambda s: math.dist(m.cotovelo(s[0]), ce))
            print("ponta px %s, cotovelo px %s" % (np.round(ponta), np.round(cot)))
        else:
            j1, j2 = min(sols, key=lambda s: abs(s[1]))      # sem cotovelo: o ramo menos dobrado
            print("cotovelo não visto: escolhi o ramo menos dobrado")
        print("ponta (%.1f, %.1f) mm -> J1 %.1f, J2 %.1f" % (x, y, j1, j2))
        lo, hi = m.lim2
        if not (lo - 5 <= j2 <= hi + 5):
            raise RuntimeError("J2 %.1f fora dos limites %s: confira a foto fotos/localizar.jpg, não movo" % (j2, m.lim2))
        r.definir("j1", j1)
        r.definir("j2", j2)

        # conferência: gira o J1 5° em direção ao centro da área e mede a ponta
        d = -5.0 if j1 > 0 else 5.0
        r.ir_juntas(j1 + d, j2, amax={"j1": 20, "j2": 20})
        time.sleep(0.3)
        p = ponta_precisa(c.foto(), esperado=m.robo_para_pixel(*m.fk(j1 + d, j2), Z_FOTO))
        if p is None:
            print("conferência: ponta não vista")
        else:
            xr, yr = m.pixel_para_robo(p[0], p[1], Z_FOTO)
            xm, ym = m.fk(j1 + d, j2)
            print("conferência: erro %.1f mm" % math.hypot(xr - xm, yr - ym))
    finally:
        c.fechar()
        r.fechar()


if __name__ == "__main__":
    main()
