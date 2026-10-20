"""Recalibração completa depois de mudar a câmera/robô de lugar (ou de reiniciar as placas).

1) referencia o Z pela chave de cima e sobe para Z_GRADE[0];
2) mede pela câmera o centro do J1 e o cotovelo/ponta da pose atual;
3) gera uma grade de poses prevendo (por rotação na imagem) onde a ponta vai aparecer, e fica só com as poses com a
   ponta dentro da imagem e o J2 longe de dobrar contra a coluna;
4) leva a ponta a cada pose em duas alturas e registra o pixel da ponta (dados_grade.json);
5) ajusta o modelo (calibrar.ajustar) e grava calibracao_braco.json (o antigo vira .bak).

Uso: python recalibrar.py
"""
import json
import math
import os
import shutil
import time

import cv2
import numpy as np

from cam import Cam
from medir_rot import rot
from ponta import hexagonos, ponta_precisa
from scara import Scara, Z_CHAVE

Z_GRADE = (80.0, 45.0)        # alturas da grade (bem acima da coletora de 12,6 cm)
MARGEM_PX = 90
DESVIO_MAX_PX = 250          # ponta longe da posição prevista = braço bateu / forçou / perdeu passo
REL_MIN, REL_MAX = -130.0, 90.0   # dobra do antebraço em relação ao braço permitida na grade (graus, sinal da imagem)
J1_FAIXA = (-130, 130)
ACEL = {"j1": 20.0, "j2": 5.0}
VEL_GRADE = {"j1": 15.0, "j2": 5.0}      # J2 lento: perde passo em movimentos rápidos/grandes nesta montagem
ACEL_INICIO = {"j1": 15.0, "j2": 5.0}
PASSO_J1, PASSO_J2 = 10, 10
MEDIR_CADA = 3                # passa por todas as poses (caminhos curtos e seguros), mede em 1 de cada 3
# área segura (px da imagem): só a superfície da mesa, longe da parede, da borda e da coletora (area_segura.json)
_A = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "area_segura.json")))
AREA = np.array(_A["poligono_px"], np.float32)
MESA = np.array(_A.get("poligono_mesa_px", _A["poligono_px"]), np.float32)   # superfície da mesa (sem cortes)


def dentro(p, poli=None):
    return cv2.pointPolygonTest(AREA if poli is None else poli, (float(p[0]), float(p[1])), False) >= 0


def dist_area(p):
    return cv2.pointPolygonTest(AREA, (float(p[0]), float(p[1])), True)       # > 0 dentro, < 0 fora


def gira(p, c, ang_img):
    """gira o ponto p em volta de c pelo ângulo da imagem (graus, y para baixo)"""
    a = math.radians(ang_img)
    dx, dy = p[0] - c[0], p[1] - c[1]
    return (c[0] + dx * math.cos(a) - dy * math.sin(a), c[1] + dx * math.sin(a) + dy * math.cos(a))


def ang(v):
    return math.degrees(math.atan2(v[1], v[0]))


def main():
    r = Scara(fator_vel=1.0)
    c = Cam()
    b = r.placas["braco"]
    try:
        if b.cmd("info")["fimcfg"]["f2"]:
            b.cmd("fim", f2=0)               # fim de curso do J2 desconectado: lê sempre "acionado"
        zc = r.referenciar_z()
        r.definir("z", Z_CHAVE)
        print("Z referenciado (chave em %.2f da contagem anterior)" % zc)
        r.ir("z", Z_GRADE[0])

        # contagens zeradas na pose atual (as antigas podem ter perdido passo); a calibração define o resto
        r.definir("j1", 0.0)
        r.definir("j2", 0.0)
        # centro do J1 e sentido: +8 / -8
        pos0 = r.posicoes()
        a = c.foto()
        r.ir_juntas(pos0["j1"] + 8, pos0["j2"], amax={"j1": 20, "j2": 20})
        bb = c.foto()
        g1, centro, _, _ = rot(a, bb)
        r.ir_juntas(pos0["j1"], pos0["j2"], amax={"j1": 20, "j2": 20})
        s1 = g1 / 8.0                          # graus de imagem por grau de J1 (sinal = sentido)
        print("J1: %.3f grau de imagem por grau, centro %s" % (s1, np.round(centro, 1)))

        # cotovelo (centro escuro, motor) e ponta (centro claro, rolamento) na pose atual
        f = c.foto("fotos/recal_pose0.jpg")
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        hs = []
        for h in hexagonos(f)[0]:
            d = np.zeros_like(g)
            cv2.circle(d, tuple(int(v) for v in h["c"]), 18, 255, -1)
            hs.append((cv2.mean(g, d)[0], h["c"], h["r"], h["circ"]))
        from ponta import acha_ponta
        pp = acha_ponta(f)
        if pp is None:
            raise RuntimeError("ponta não encontrada na foto inicial: não movo")
        ponta = pp["c"]
        # cotovelo: outro hexágono, escuro (motor), a uma distância da ponta compatível com o antebraço
        # distâncias do braço nesta câmera: eixo->cotovelo ~550 px, cotovelo->ponta ~370 px
        cands = [h for h in hs if 45 < h[2] < 130 and h[3] >= 0.45 and 280 < math.dist(h[1], ponta) < 480
                 and 450 < math.dist(h[1], centro) < 650 and h[0] < 110]
        if not cands:
            raise RuntimeError("cotovelo não encontrado na foto inicial: não movo")
        cot = min(cands, key=lambda h: h[0])[1]
        print("cotovelo %s, ponta %s" % (np.round(cot), np.round(ponta)))
        rel0 = ((ang(np.subtract(ponta, cot)) - ang(np.subtract(cot, centro))) + 180) % 360 - 180
        s2 = s1                                # J2 gira no mesmo sentido que o J1 (medido antes)
        H, W = f.shape[:2]

        def preve(j1, j2):
            d1, d2 = j1 - pos0["j1"], j2 - pos0["j2"]
            e = gira(cot, centro, s1 * d1)
            t = gira(gira(ponta, centro, s1 * d1), e, s2 * d2)
            rel = rel0 + s2 * d2
            return e, t, rel

        def pose_ok(j1, j2):
            e, t, rel = preve(j1, j2)
            return REL_MIN <= rel <= REL_MAX and dentro(e) and dentro(t)

        def caminho_ok(qa, qb):
            """a trajetória sincronizada anda em linha reta no espaço das juntas: confere pontos ao longo dela"""
            return all(pose_ok(qa[0] + (qb[0] - qa[0]) * k, qa[1] + (qb[1] - qa[1]) * k) for k in np.linspace(0, 1, 25))

        def retirada_ok(qa, qb):
            """1º trecho, saindo de fora da área: sempre sobre a mesa e se aproximando da área segura"""
            ant, rel_ant = None, None
            for k in np.linspace(0, 1, 25):
                e, t, rel = preve(qa[0] + (qb[0] - qa[0]) * k, qa[1] + (qb[1] - qa[1]) * k)
                # pode partir mais dobrado que REL_MAX, mas a dobra só pode diminuir
                fora_dobra = not (REL_MIN <= rel <= REL_MAX)
                if (fora_dobra and rel_ant is not None and abs(rel) > rel_ant + 0.5) or not (dentro(e, MESA) and dentro(t, MESA)):
                    return False
                rel_ant = abs(rel)
                d = min(dist_area(e), dist_area(t))
                if d < 0 and ant is not None and d < ant - 5:     # fora da área: só pode se aproximar dela
                    return False
                ant = d
            return True
        fora = not pose_ok(pos0["j1"], pos0["j2"])
        if fora:
            print("pose atual fora da área segura: o 1º movimento será uma retirada para dentro dela")
        livres = []
        for j1 in range(J1_FAIXA[0], J1_FAIXA[1] + 1, PASSO_J1):
            for j2 in range(-240, 241, PASSO_J2):
                j1a, j2a = pos0["j1"] + j1, pos0["j2"] + j2
                if pose_ok(j1a, j2a):
                    livres.append((j1a, j2a, preve(j1a, j2a)[1]))
        # ordem: vizinho mais próximo, só por caminhos que ficam dentro da área segura
        cand, atual = [], (pos0["j1"], pos0["j2"])
        if not livres:
            raise RuntimeError("nenhuma pose livre prevista")
        while livres:
            ok = [q for q in livres if (retirada_ok(atual, q) if fora and not cand else caminho_ok(atual, q))]
            if not ok:
                break
            q = min(ok, key=lambda q: abs(q[0] - atual[0]) + abs(q[1] - atual[1]))
            cand.append(q); livres.remove(q); atual = q[:2]
        if not cand:
            raise RuntimeError("sem caminho seguro da pose atual para a grade: não movo")
        volta = cand[0][:2] if fora else (pos0["j1"], pos0["j2"])     # termina dentro da área, sem voltar para fora
        print("%d poses na grade (J1 %s, J2 %s)" % (len(cand), (min(q[0] for q in cand), max(q[0] for q in cand)),
                                                 (min(q[1] for q in cand), max(q[1] for q in cand))))
        # os limites do J2 em scara.py eram do zero antigo: na grade vale só o limite de dobra medido na imagem
        import scara
        scara.EIXOS["j2"]["min"], scara.EIXOS["j2"]["max"] = -250, 250
        if len(cand) < 10:
            raise RuntimeError("poucas poses visíveis: confira o enquadramento da câmera")

        dados = []
        for i, (j1, j2, t) in enumerate(cand):
            r.ir_juntas(j1, j2, vmax=VEL_GRADE, amax=ACEL_INICIO if i < 5 else ACEL)
            zs = Z_GRADE if i % 2 == 0 else Z_GRADE[::-1]
            # olha a câmera: a ponta tem que estar perto de onde a rotação na imagem prevê; se não, bateu/forçou
            f = c.foto()
            p = ponta_precisa(f, esperado=t)
            if p is None or math.dist(p, t) > DESVIO_MAX_PX:
                cv2.imwrite("fotos/recal_parada.jpg", f)
                r.placas["braco"].cmd("en", v=0)
                raise RuntimeError("pose %d (J1 %.0f, J2 %.0f): ponta %s, prevista %s -> parei e soltei os motores"
                                   % (i, j1, j2, p and np.round(p), np.round(t)))
            print("pose %d/%d ok: ponta %s (prevista %s)" % (i + 1, len(cand), np.round(p), np.round(t)), flush=True)
            if i % MEDIR_CADA:
                continue                          # câmera conferida; mede as alturas só em 1 de cada 3
            for z in zs:
                r.ir("z", z)
                p = ponta_precisa(c.foto(), esperado=t if z == Z_GRADE[0] else None)
                dados.append({"j1": j1, "j2": j2, "z": z, "px": p and [round(p[0], 2), round(p[1], 2)]})
                print(dados[-1], flush=True)
                json.dump(dados, open("dados_grade_parcial.json", "w"))     # salva a cada medida
        r.ir("z", Z_GRADE[0])
        r.ir_juntas(*volta, vmax=VEL_GRADE, amax=ACEL)
        if os.path.exists("dados_grade.json"):
            shutil.copy("dados_grade.json", "dados_grade.json.bak")
        json.dump(dados, open("dados_grade.json", "w"), indent=1)
        print("grade salva: %d medidas" % sum(1 for d in dados if d["px"]))
        # ajuste do modelo + limites do J2 (pela dobra) + pose de descanso (ponta mais à esquerda: em cima do tabuleiro)
        from calibrar import ajustar
        bons = [d for d in dados if d["px"]]
        cal, err, _ = ajustar(bons)
        # limites do J2 no referencial do modelo: as dobras testadas na grade (com folga de 5°)
        js = [q[1] for q in cand]
        cal["lim2"] = [min(js) - 5, max(js) + 5]
        cal["parque"] = list(min(cand, key=lambda q: q[2][0])[:2])
        cal["erro_mediano_px"] = float(np.median(err))
        if os.path.exists("calibracao_braco.json"):
            shutil.copy("calibracao_braco.json", "calibracao_braco.json.bak")
        json.dump(cal, open("calibracao_braco.json", "w"), indent=1)
        print("modelo: erro mediano %.1f px, p90 %.1f px, o2 %.1f, lim2 %s, parque %s"
              % (np.median(err), np.percentile(err, 90), cal["o2"], np.round(cal["lim2"], 1), np.round(cal["parque"], 1)))
    except Exception as e:
        print("ERRO", e)
        r.parar()
        raise
    finally:
        c.fechar()
        r.fechar()


if __name__ == "__main__":
    main()
