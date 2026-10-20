"""Simulação de pick and place: acha as caixas de remédio e a caixa coletora pela câmera; em cada caixa de remédio
desce, "pega", sobe acima da coletora e "solta" dentro dela. A ventosa ainda não está ligada: as caixas não se
movem, só o braço faz o ciclo.

Alturas (acima da mesa) e o Z correspondente: z = altura + Z_TOQUE_MESA (-135).

Uso: python pick_place.py [--nome micpure]     (com --nome, o Gemini lê os nomes e só pega as caixas desse remédio)
     --mira: confere a ponta pela câmera em cima de cada caixa (mais preciso, mais lento); sem ela só há a foto inicial
"""
import difflib
import math
import sys
import time
import unicodedata
from pathlib import Path

import cv2
import numpy as np

from bomba import Bomba
from caixa import acha_caixas
from cam import Cam
from modelo import ALTURA_CAIXA, Z_TOQUE_MESA, Modelo
from ponta import ponta_precisa
from scara import Scara

ALTURA_COLETORA = 126.0   # mm (medido)
FOLGA_SOLTA = 10.0        # fundo da caixa de remédio fica 1 cm acima da borda ao soltar
FOLGA_PASSA = 34.0        # caixa pendurada passa 3,4 cm acima da borda
Z_ALTO = Z_TOQUE_MESA + ALTURA_COLETORA + ALTURA_CAIXA + FOLGA_PASSA      # +55: deslocamentos
Z_SOLTA = Z_TOQUE_MESA + ALTURA_COLETORA + ALTURA_CAIXA + FOLGA_SOLTA     # +31: solta dentro da coletora
Z_MIRA = -65.0            # altura em que a câmera confere a ponta em cima da caixa
Z_PEGA = -97.0            # o fim de curso de baixo aciona em ~-98,3 (ventosa ~7 mm acima de uma caixa de 30 mm)
PARQUE = (20.0, -110.0)   # braço recolhido, fora da área das caixas, para a foto
LARGURA_BRACO = 50.0      # mm de folga entre o braço (baixo) e a coletora
PAUSA_VENTOSA = 0.5
PAUSA_SOLTA = 0.3
# relé da bomba: desligado em repouso (estado de partida); liga para pegar, desliga para soltar
RELE_REPOUSO, RELE_PEGA = 0, 1


def acha_coletora(f):
    """caixa de MDF (cor de madeira): maior região cor de madeira -> retângulo e polígono (px)"""
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (8, 50, 80), (25, 200, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    if cv2.contourArea(c) < 50000:
        return None
    r = cv2.minAreaRect(c)
    return {"rect": r, "cantos": cv2.boxPoints(r)}


def obstaculo_mm(m, cantos_px, altura):
    """polígono (mm) ocupado por um objeto de `altura` cujos cantos aparecem em `cantos_px`: fecho do topo e da base"""
    topo = [m.pixel_para_robo(u, v, m.z_do_plano(altura)) for u, v in cantos_px]
    base = [m.pixel_para_robo(u, v, m.z_do_plano(0)) for u, v in cantos_px]
    return topo, cv2.convexHull(np.float32(topo + base))


VISAO = Path(__file__).resolve().parent.parent / "only-vision"


def _norm(t):
    return "".join(c for c in unicodedata.normalize("NFD", t.lower()) if unicodedata.category(c) != "Mn")


def mesmo_nome(pedido, lido):
    """tolera um erro de leitura do Gemini ("Miopure" = "Micpure")"""
    a = _norm(pedido)
    return any(a in w or difflib.SequenceMatcher(None, a, w).ratio() >= 0.75 for w in _norm(lido).split())


def _gemini(f):
    """detecta com a chave do .env; se ela falhar (ex.: cota esgotada), tenta a reserva GEMINI_API_KEY_2"""
    sys.path.insert(0, str(VISAO))
    from dotenv import load_dotenv
    load_dotenv(VISAO / ".env")
    import vision_remedios as vr
    modelo = vr.os.getenv("GEMINI_MODEL", "gemini-robotics-er-2-preview")
    try:
        g = vr.AnalisadorGemini(modelo)
        return g, g.detectar(f)[0]
    except Exception as ex:
        reserva = vr.os.getenv("GEMINI_API_KEY_2")
        if not reserva:
            raise
        print("[aviso] Gemini falhou (%s): usando a chave reserva" % type(ex).__name__)
        vr.os.environ["GEMINI_API_KEY"] = reserva
        g = vr.AnalisadorGemini(modelo)
        return g, g.detectar(f)[0]


def caixas_gemini(f):
    """caixas pelo Gemini (only-vision): centro em px e nome; útil para caixas brancas que o OpenCV não acha"""
    t = time.time()
    g, lidas = _gemini(f)
    h, w = f.shape[:2]
    print("Gemini (%s, %.0f s): %d caixas" % (g.modelo, time.time() - t, len(lidas)))
    return [{"centro": (c["centro"][1] / 1000 * w, c["centro"][0] / 1000 * h), "nome": c.get("nome", ""), "cantos": []}
            for c in lidas]


def nomes_gemini(f, caixas):
    """lê o nome de cada caixa com o Gemini (only-vision) e associa à caixa do OpenCV mais próxima"""
    t = time.time()
    g, lidas = _gemini(f)
    h, w = f.shape[:2]
    print("Gemini (%s, %.0f s): %d caixas lidas (as de dentro da coletora são ignoradas)" % (g.modelo, time.time() - t, len(lidas)))
    for k in caixas:
        k["nome"] = ""
    for c in lidas:
        y, x = c["centro"]
        p = (x / 1000 * w, y / 1000 * h)
        if not caixas:
            break
        k = min(caixas, key=lambda k: math.dist(k["centro"], p))
        if math.dist(k["centro"], p) < 80:
            k["nome"] = c.get("nome", "")
    return caixas


_AREAS = __import__("json").load(open(Path(__file__).resolve().parent / "area_segura.json"))
AREA_SEGURA = np.float32(_AREAS["poligono_px"])                       # braço baixo
AREA_ALTO = np.float32(_AREAS.get("poligono_alto_px", _AREAS["poligono_px"]))   # braço no alto (passa sobre a coletora)


class PickPlace:
    def __init__(self):
        self.m = Modelo()
        self.r = Scara(fator_vel=1.0)
        self.c = Cam()
        self.obstaculo = None      # polígono da coletora em mm (referencial do robô)
        self.tolerante = False     # caixa fora da área segura: só o cotovelo fica preso à área (ver _na_area)
        self.bomba = Bomba()

    def fechar(self):
        self.c.fechar()
        self.r.fechar()
        self.bomba.fechar()

    # ---- colisão (só vale com o braço baixo: no alto ele passa por cima da coletora) ----
    def livre(self, j1, j2):
        if self.obstaculo is None:
            return True
        cot = self.m.cotovelo(j1)
        pon = self.m.fk(j1, j2)
        for a, b in (((0.0, 0.0), cot), (cot, pon)):
            for t in np.linspace(0, 1, 40):
                p = (float(a[0] + (b[0] - a[0]) * t), float(a[1] + (b[1] - a[1]) * t))
                if cv2.pointPolygonTest(self.obstaculo, p, True) > -LARGURA_BRACO:
                    return False
        return True

    # ---- deslocamentos que não saem da área segura (paredes e pessoas não aparecem como obstáculo no chão) ----
    def _na_area(self, j1, j2, z, area=None):
        m = self.m
        area = AREA_ALTO if area is None else area
        if self.tolerante:
            # caixa fora da área: o cotovelo continua preso à área no alto (parede); a ponta só precisa estar em
            # cima da mesa (projetada no plano da caixa, sem a paralaxe da altura, que a joga para fora da imagem)
            u, v = m.robo_para_pixel(*m.cotovelo(j1), z)
            if cv2.pointPolygonTest(AREA_ALTO, (float(u), float(v)), False) < 0:
                return False
            u, v = m.robo_para_pixel(*m.fk(j1, j2), m.z_do_plano(ALTURA_CAIXA))
            return cv2.pointPolygonTest(AREA_SEGURA, (float(u), float(v)), False) >= 0
        for pt in (m.cotovelo(j1), m.fk(j1, j2)):
            u, v = m.robo_para_pixel(pt[0], pt[1], z)
            if cv2.pointPolygonTest(area, (float(u), float(v)), False) < 0:
                return False
        return True

    def _trecho_ok(self, a, b, z, k0=0.0):
        return all(self._na_area(a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, z) for k in np.linspace(k0, 1, 30))

    def plano_caminho(self, alvo):
        """lista de poses (J1, J2) até o alvo sem sair da área segura: direto, ou dobrando o cotovelo antes de girar"""
        pos = self.r.posicoes()
        atual, z = (pos["j1"], pos["j2"]), pos["z"]
        if self._trecho_ok(atual, alvo, z):
            return [alvo]
        fora = not self._na_area(*atual, z)        # partindo de fora: o 1º trecho (só dobrar) pode começar fora
        for j2d in self.m.lim2:                     # tenta dobrar para um lado e para o outro
            via = [(atual[0], j2d), (alvo[0], j2d), alvo]
            pts = [atual] + via
            if all(self._trecho_ok(pts[i], pts[i + 1], z, 0.5 if (i == 0 and fora) else 0.0) for i in range(len(pts) - 1)):
                return via
        return None

    def ir_seguro(self, alvo):
        via = self.plano_caminho(alvo)
        if via is None:
            raise ValueError("sem caminho dentro da área segura até J1 %.1f J2 %.1f" % alvo)
        for q in via:
            self.r.ir_juntas(*q)

    def mira(self, x, y, cmd, tol_mm=2.0):
        """com a ponta em cima de (x, y), confere pela câmera e corrige uma vez"""
        z = self.r.posicao("z")
        p = ponta_precisa(self.c.foto(), esperado=self.m.robo_para_pixel(*self.m.fk(*cmd), z))
        if p is None:
            print("   ponta não encontrada: sem correção")
            return cmd
        xr, yr = self.m.pixel_para_robo(p[0], p[1], z)
        e = math.hypot(x - xr, y - yr)
        if e <= tol_mm:
            print("   erro %.1f mm: sem correção" % e)
            return cmd
        xa, ya = self.m.fk(*cmd)
        novo = self.m.ik(xa + x - xr, ya + y - yr, ref=cmd, livre=self.livre)
        self.r.ir_juntas(*novo)
        p = ponta_precisa(self.c.foto(), esperado=self.m.robo_para_pixel(*self.m.fk(*novo), z))
        if p is not None:
            xr, yr = self.m.pixel_para_robo(p[0], p[1], z)
            print("   erro %.1f mm -> %.1f mm depois da correção" % (e, math.hypot(x - xr, y - yr)))
        return novo

    def checar_eventos(self):
        """para tudo se uma placa reiniciou o TMC, perdeu o 24 V ou bateu num fim de curso no meio do ciclo"""
        for n, p in self.r.placas.items():
            ruins = [e for e in p.eventos if any(t in e for t in ("TMC", "FIM", "24 V", "EMERG", "erro"))]
            p.eventos.clear()
            if ruins:
                raise RuntimeError("placa %s: %s -- posição pode estar errada, rode diagnostico.py" % (n, ruins))

    def planejar(self, f, usar_gemini=False):
        m = self.m
        # a coletora não sai do lugar e o braço em descanso fica em cima dela: detecta na foto de referência
        # (fotos/agora.jpg, tirada com o braço longe dela); só sem ela usa a foto atual
        ref = cv2.imread(str(Path(__file__).resolve().parent / "fotos" / "agora.jpg"))
        col = acha_coletora(ref) if ref is not None else None
        if col is None:
            col = acha_coletora(f)
        if col is None:
            raise RuntimeError("caixa coletora não encontrada")
        topo, self.obstaculo = obstaculo_mm(m, col["cantos"], ALTURA_COLETORA)
        # pontos de soltura: ao longo do eixo maior da boca, até 30% do comprimento para cada lado
        cx, cy = np.mean(topo, axis=0)
        lados = [np.subtract(topo[(i + 1) % 4], topo[i]) for i in range(4)]
        eixo = max(lados, key=lambda v: np.hypot(*v))
        comp = float(np.hypot(*eixo))
        u = eixo / comp
        todas = caixas_gemini(f) if usar_gemini else acha_caixas(f)
        caixas = [k for k in todas if cv2.pointPolygonTest(np.float32(col["cantos"]), k["centro"], False) < 0]
        n = max(len(caixas), 1)
        ts = np.linspace(-0.3 * comp, 0.3 * comp, n) if n > 1 else [0.0]
        solta = [(float(cx + u[0] * t), float(cy + u[1] * t)) for t in ts]
        def solta_ok(xy):
            try:
                return self._na_area(*self.m.ik(*xy, ref=self.m.parque), Z_ALTO)
            except ValueError:
                return False
        solta = [s for s in solta if solta_ok(s)] or solta
        return col, caixas, solta, (cx, cy, comp)

    def rodar(self, nome=None, alvo_px=None, ordem=None, usar_gemini=False, mirar=False):
        r, m = self.r, self.m
        self.bomba.rele(RELE_REPOUSO)        # percurso começa com o relé em repouso (desligado)
        if r.posicao("z") < Z_ALTO:
            r.ir("z", Z_ALTO)
        self.ir_seguro(m.parque)
        for p in r.placas.values():
            p.eventos.clear()
        f = self.c.foto("fotos/pp_mesa.jpg")
        col, caixas, solta, (cx, cy, comp) = self.planejar(f, usar_gemini)
        if alvo_px:                           # caixa indicada à mão (ex.: cortada na borda da imagem)
            caixas = [{"centro": alvo_px, "nome": nome or "", "cantos": []}]
        elif ordem:                           # todas as caixas dos remédios da lista, na ordem da lista
            if not usar_gemini:
                nomes_gemini(f, caixas)
            for k in caixas:
                print("   caixa px (%.0f, %.0f): %s" % (*k["centro"], k["nome"] or "?"))
            pos_lista = lambda k: next((i for i, n in enumerate(ordem) if mesmo_nome(n, k["nome"])), None)
            caixas = sorted([k for k in caixas if pos_lista(k) is not None], key=pos_lista)
            if not caixas:
                print("nenhuma caixa da lista encontrada: o braço não se move")
                return
        elif nome:
            if not usar_gemini:
                nomes_gemini(f, caixas)
            for k in caixas:
                print("   caixa px (%.0f, %.0f): %s" % (*k["centro"], k["nome"] or "?"))
            caixas = [k for k in caixas if mesmo_nome(nome, k["nome"])]
            if not caixas:
                print("nenhuma caixa de %s encontrada: o braço não se move" % nome)
                return
        print("coletora: centro (%.0f, %.0f) mm, boca %.0f mm; %d caixas de remédio" % (cx, cy, comp, len(caixas)))
        vis = f.copy()
        cv2.drawContours(vis, [np.int32(col["cantos"])], -1, (0, 165, 255), 3)
        for k in caixas:
            if len(k["cantos"]):
                cv2.drawContours(vis, [np.int32(k["cantos"])], -1, (0, 255, 0), 3)
        for xs, ys in solta:
            a, b = m.robo_para_pixel(xs, ys, m.z_do_plano(ALTURA_COLETORA))
            cv2.drawMarker(vis, (int(a), int(b)), (255, 0, 255), cv2.MARKER_SQUARE, 50, 3)
        cv2.imwrite("fotos/pp_plano.jpg", vis)

        t0 = time.time()
        feitas = 0
        for k in (caixas if ordem else sorted(caixas, key=lambda k: k["centro"][1])):
            u_, v_ = k["centro"]
            x, y = m.pixel_para_robo(u_, v_, m.z_do_plano(ALTURA_CAIXA))
            pos = r.posicoes()
            self.tolerante = False
            try:
                cmd = m.ik(x, y, ref=(pos["j1"], pos["j2"]), livre=self.livre)
                cmd_solta = m.ik(*solta[feitas % len(solta)], ref=cmd)
                if not self._na_area(*cmd, Z_ALTO, AREA_SEGURA):
                    self.tolerante = True     # tenta mesmo assim, se a ponta está sobre a mesa e o cotovelo longe da parede
                    if not self._na_area(*cmd, Z_ALTO):
                        self.tolerante = False
                        raise ValueError("caixa fora da área segura/calibrada")
                    print("caixa px (%.0f, %.0f) fora da área segura: tentando pegar (ponta sobre a mesa)" % (u_, v_))
            except ValueError as ex:
                print("caixa px (%.0f, %.0f): pulada (%s)" % (u_, v_, ex))
                continue
            print("caixa %d %s: px (%.0f, %.0f) -> (%.1f, %.1f) mm, juntas (%.1f, %.1f)" % (feitas + 1, k.get("nome", ""), u_, v_, x, y, *cmd))
            t = time.time()
            try:
                self.ir_seguro(cmd)           # no alto, sem sair da área segura
            except ValueError as ex:
                print("   pulada: %s" % ex)
                continue
            # ajuste rápido na descida: para a meia altura (onde a câmera vê bem a ponta), uma foto, corrige uma vez
            # e desce direto, sem foto de conferência
            if not mirar:
                r.ir("z", Z_MIRA)
            p = ponta_precisa(self.c.foto(), esperado=m.robo_para_pixel(*m.fk(*cmd), r.posicao("z")))
            if p is not None:
                xr, yr = m.pixel_para_robo(p[0], p[1], r.posicao("z"))
                e = math.hypot(x - xr, y - yr)
                if e > 60.0:                   # longe demais: detecção errada, não confia
                    print("   ponta vista a %.0f mm (detecção duvidosa): desce sem ajustar" % e)
                elif e > 3.0:
                    xa, ya = m.fk(*cmd)
                    try:
                        cmd = m.ik(xa + x - xr, ya + y - yr, ref=cmd, livre=self.livre)
                        r.ir_juntas(*cmd)
                        print("   ajuste %.1f mm" % e)
                    except ValueError:
                        print("   ajuste de %.1f mm fora dos limites: desce sem ajustar" % e)
            else:
                print("   ponta não vista: desce sem ajustar")
            if mirar:                          # correção pela câmera (mais lenta): só com --mira
                r.ir("z", Z_MIRA)
                try:
                    for _ in range(3):         # J2 pode escorregar: a câmera corrige até 3 vezes
                        novo = self.mira(x, y, cmd)
                        if novo == cmd:
                            break
                        cmd = novo
                except ValueError as ex:       # a correção pediria sair dos limites: pula a caixa
                    print("   pulada na mira: %s" % ex)
                    r.ir("z", Z_ALTO)
                    continue
            r.ir("z", Z_PEGA)
            self.bomba.rele(RELE_PEGA)        # só encostado na caixa: relé liga, ventosa puxa
            time.sleep(PAUSA_VENTOSA)
            r.ir("z", Z_ALTO)
            try:
                self.ir_seguro(cmd_solta)
            except ValueError as ex:
                print("   sem caminho seguro até a coletora: %s -- parei com a caixa" % ex)
                break
            self.bomba.rele(RELE_REPOUSO)     # já no alto, em cima da coletora: relé desliga, a caixa cai
            time.sleep(PAUSA_SOLTA)
            feitas += 1
            self.tolerante = False
            self.checar_eventos()
            print("   ciclo %.1f s" % (time.time() - t))
        self.tolerante = False
        try:
            self.ir_seguro(m.parque)
        except ValueError:
            pass
        print("%d caixas, total %.1f s" % (feitas, time.time() - t0))


if __name__ == "__main__":
    pp = PickPlace()
    try:
        nome = sys.argv[sys.argv.index("--nome") + 1] if "--nome" in sys.argv else None
        alvo = tuple(float(v) for v in sys.argv[sys.argv.index("--alvo-px") + 1].split(",")) if "--alvo-px" in sys.argv else None
        ordem = sys.argv[sys.argv.index("--ordem") + 1].split(",") if "--ordem" in sys.argv else None
        pp.rodar(nome, alvo, ordem, "--gemini" in sys.argv, "--mira" in sys.argv)
    except Exception as ex:
        print("ERRO", ex)
        pp.r.parar()
        raise
    finally:
        pp.fechar()
