"""Controle do SCARA pelas placas ScaraNode (firmware "painel" no.py, JSON por linha na USB).

Placa 801c (modo "dois"): J1 no DM556, J2 no TMC2209.   Placa 5b1c (modo "tmc"): Z no TMC2209.
Movimentos ponto a ponto, um eixo por vez, com velocidade limitada por FATOR_VEL.
"""
import base64
import json
import math
import struct
import time

import serial
import serial.tools.list_ports

UID_BRACO = "504446688896801c"
UID_Z = "5044466888c15b1c"

# passos por unidade (grau ou mm), canal e limites — vindos do config.json das placas
EIXOS = {
    "j1": {"placa": "braco", "k": "dm556", "passos_un": 3200 * 14 / 360, "min": -130, "max": 130, "v": 30, "a": 60},
    "j2": {"placa": "braco", "k": "tmc", "passos_un": 200 * 32 * 12 / 360, "min": -250, "max": 250, "v": 45, "a": 90},
    "z": {"placa": "z", "k": "tmc", "passos_un": 200 * 16 / 8, "min": -106, "max": 99, "v": 45, "a": 150},   # 45 mm/s ≈ 340 rpm: acima disso o Z perde passo subindo (testado)
}

FATOR_VEL = 0.2   # fração da velocidade/aceleração configurada nos movimentos de um eixo só (ir)
Z_CHAVE = 100.31  # posição do Z (referencial da calibração) em que o fim de curso de cima aciona
Z_TOQUE_MESA = -135.0   # ventosa encosta na mesa (medido: a -40 faltavam 65 mm até a caixa de 30 mm)

# movimento conjunto J1+J2 (trajetória sincronizada): limites em grau/s e grau/s²
VEL_JUNTAS = {"j1": 90.0, "j2": 45.0}   # °/s: testado pela câmera (erro < 0,6°) em 19/10
VEL_PONTA = 3000.0  # mm/s: teto da velocidade da ponta (cartesiana)
L1, L2, O2 = 300.0, 200.0, -22.53   # braço (mm) e zero do J2 (calibracao_braco.json)
ACEL_JUNTAS = {"j1": 120.0, "j2": 60.0}   # °/s²
DT = 0.01          # s por segmento (igual ao painel)
FREQ = 10_000_000  # relógio do gerador de passos da placa
TJ_DMIN = 50


def perfil_sincrono(deltas, vmax, amax):
    """trapézio com a mesma forma no tempo para todas as juntas (começam e terminam juntas).
    Devolve (T, f): duração total e fração de aceleração (ta = f*T)."""
    melhor = None
    for i in range(1, 51):
        f = i / 100.0
        T = 0.0
        for k, d in deltas.items():
            d = abs(d)
            if d == 0:
                continue
            T = max(T, math.sqrt(d / (amax[k] * f * (1 - f))), d / (vmax[k] * (1 - f)))
        if melhor is None or T < melhor[0]:
            melhor = (T, f)
    return melhor


def s_trapezio(t, T, f):
    """posição normalizada (0..1) do trapézio de duração T e aceleração em f*T"""
    ta = f * T
    v = 1.0 / (T - ta)
    a = v / ta
    if t <= 0:
        return 0.0
    if t >= T:
        return 1.0
    if t < ta:
        return 0.5 * a * t * t
    if t <= T - ta:
        return 0.5 * a * ta * ta + v * (t - ta)
    u = T - t
    return 1.0 - 0.5 * a * u * u


def codifica(q, spu, p0, freq=FREQ, dmin=TJ_DMIN):
    """ângulos a cada DT -> palavras do _traj (no.py): d | DIR << 20 | n << 21 (cópia do codifica do robo.html)"""
    Tc = round(DT * freq)
    w = []
    A, prev, dirb = 0, p0, 1
    for m, qm in enumerate(q):
        alvo = round(qm * spu)
        n = alvo - prev
        prev = alvo
        t = (m + 1) * Tc - A
        if n == 0:
            d = min(max(t - 9, 1), 0xFFFFF)
            A += d + 9
            w.append(dirb * 0x100000 + d)
            continue
        an = abs(n)
        dirb = 1 if n > 0 else 0
        if an > 2047:
            raise ValueError("junta rápida demais (%d passos em %g ms)" % (an, DT * 1000))
        d = min(0xFFFFF, round(((t - 5) / an - 7) / 2))
        if d < dmin:
            raise ValueError("junta rápida demais: %d passos/s" % round(an / DT))
        A += 5 + an * (2 * d + 7)
        w.append(an * 0x200000 + dirb * 0x100000 + d)
    return w, prev


class Placa:
    def __init__(self, porta):
        self.s = serial.Serial(porta, 115200, timeout=0.2)
        time.sleep(0.2)
        self.s.reset_input_buffer()
        self.id = 0
        self.eventos = []

    def cmd(self, c, timeout=3.0, **kw):
        self.id += 1
        m = {"c": c, "id": self.id, **kw}
        self.s.write((json.dumps(m) + "\n").encode())
        fim = time.time() + timeout
        while time.time() < fim:
            linha = self.s.readline().decode(errors="replace").strip()
            if not linha.startswith("{"):
                continue
            try:
                r = json.loads(linha)
            except ValueError:
                continue
            if r.get("ev"):
                self.eventos += r["ev"]
                for e in r["ev"]:
                    print("[placa]", e)
            if r.get("id") == self.id:
                if not r.get("ok"):
                    raise RuntimeError("%s: %s" % (c, r.get("erro")))
                return r
        raise TimeoutError("sem resposta para %s" % c)

    def fechar(self):
        self.s.close()


def acha_portas():
    portas = {}
    for p in serial.tools.list_ports.comports():
        sn = (p.serial_number or "").lower()
        if sn == UID_BRACO:
            portas["braco"] = p.device
        elif sn == UID_Z:
            portas["z"] = p.device
    return portas


class Scara:
    def __init__(self, fator_vel=FATOR_VEL):
        portas = acha_portas()
        if set(portas) != {"braco", "z"}:
            raise RuntimeError("placas encontradas: %s" % portas)
        self.placas = {n: Placa(p) for n, p in portas.items()}
        self.fator_vel = fator_vel
        # Z: fim de curso de baixo (FIM1) bloqueia a descida, o de cima (FIM2) a subida (só na RAM da placa)
        self.placas["z"].cmd("fim", f1=-1, f2=1)

    # ---- leitura ----
    def st(self, placa):
        return self.placas[placa].cmd("st")

    def posicao(self, eixo):
        e = EIXOS[eixo]
        pk = self.st(e["placa"])["pk"]
        return pk[e["k"]] / e["passos_un"]

    def posicoes(self):
        return {n: round(self.posicao(n), 3) for n in EIXOS}

    # ---- movimento ----
    def definir(self, eixo, valor):
        """diz à placa que o eixo está em `valor` (grau/mm) agora, sem mover"""
        e = EIXOS[eixo]
        self.placas[e["placa"]].cmd("zero", k=e["k"], p=int(round(valor * e["passos_un"])))

    def energizar(self, placa, v=True):
        self.placas[placa].cmd("en", v=1 if v else 0)

    def esperar(self, placa, timeout=120):
        fim = time.time() + timeout
        while time.time() < fim:
            s = self.st(placa)
            if not s["mov"]:
                return s
            time.sleep(0.1)
        self.parar()
        raise TimeoutError("movimento não terminou")

    def ir(self, eixo, alvo, fator=None, esperar=True, checar_limite=True):
        e = EIXOS[eixo]
        if checar_limite and not (e["min"] <= alvo <= e["max"]):
            raise ValueError("%s=%.2f fora de [%s, %s]" % (eixo, alvo, e["min"], e["max"]))
        f = self.fator_vel if fator is None else fator
        pl = self.placas[e["placa"]]
        pl.cmd("ir", k=e["k"], p=int(round(alvo * e["passos_un"])),
               v=e["v"] * f * e["passos_un"], a=e["a"] * f * e["passos_un"])
        if esperar:
            self.esperar(e["placa"])

    def ir_juntas(self, j1, j2, vmax=None, amax=None):
        """J1 e J2 juntos (trajetória sincronizada na placa do braço), perfil trapezoidal"""
        alvo = {"j1": j1, "j2": j2}
        for k, v in alvo.items():
            e = EIXOS[k]
            if not (e["min"] <= v <= e["max"]):
                raise ValueError("%s=%.2f fora de [%s, %s]" % (k, v, e["min"], e["max"]))
        pl = self.placas["braco"]
        pk = pl.cmd("st")["pk"]
        ini = {k: pk[EIXOS[k]["k"]] / EIXOS[k]["passos_un"] for k in alvo}
        deltas = {k: alvo[k] - ini[k] for k in alvo}
        if all(abs(d) < 1e-3 for d in deltas.values()):
            return 0.0
        T, f = perfil_sincrono(deltas, vmax or VEL_JUNTAS, amax or ACEL_JUNTAS)
        # limita a velocidade da ponta: estica o tempo se passar de VEL_PONTA
        def ponta(t):
            a = math.radians(ini["j1"] + deltas["j1"] * s_trapezio(t, T, f))
            b = a + math.radians(ini["j2"] + deltas["j2"] * s_trapezio(t, T, f) + O2)
            return L1 * math.cos(a) + L2 * math.cos(b), L1 * math.sin(a) + L2 * math.sin(b)
        vp = max(math.dist(ponta(t), ponta(t + DT)) / DT for t in [i * DT for i in range(int(T / DT) + 1)])
        if vp > VEL_PONTA:
            T *= vp / VEL_PONTA
        N = max(1, math.ceil(T / DT))
        ws = []
        for k in ("j1", "j2"):
            q = [ini[k] + deltas[k] * s_trapezio((m + 1) * DT, T, f) for m in range(N)]
            q[-1] = alvo[k]
            w, _ = codifica(q, EIXOS[k]["passos_un"], pk[EIXOS[k]["k"]])
            ws.append(w)
        ks = [EIXOS["j1"]["k"], EIXOS["j2"]["k"]]
        self.tid = getattr(self, "tid", 0) + 1
        r = pl.cmd("tjini", k=ks, tid=self.tid)
        b64 = lambda w: base64.b64encode(struct.pack("<%dI" % len(w), *w)).decode()
        i, iniciou, CH, PRE = 0, False, 120, 150
        while True:
            while i < N and r["livre"] >= min(CH, N - i):
                m = min(CH, N - i, r["livre"])
                fim = i + m >= N
                go = not iniciou and i + m >= min(N, PRE)
                r = pl.cmd("tj", w=[b64(ws[0][i:i + m]), b64(ws[1][i:i + m])], fim=fim, go=go, timeout=4)
                i += m
                iniciou = iniciou or go
            if i >= N and iniciou:
                break
            time.sleep(0.12)
            r = pl.cmd("tj", timeout=3)
        while True:
            s = pl.cmd("st")
            if not s.get("tj"):
                tjf = s.get("tjf") or {}
                if tjf.get("tid") != self.tid or not tjf.get("ok"):
                    raise RuntimeError("trajetória interrompida: %s" % tjf.get("msg"))
                return T
            time.sleep(0.1)

    def referenciar_z(self, vbusca=4.0, recuo=3.0, vfina=1.0, curso=260.0):
        """sobe o Z até o fim de curso de cima (FIM2, a PIO para no ato), recua e aproxima devagar.
        Devolve a posição (mm, no referencial atual) em que a chave aciona."""
        e = EIXOS["z"]
        pl = self.placas["z"]
        spu = e["passos_un"]
        for v, dist in ((vbusca, curso), (vfina, recuo + 2)):
            if v == vfina:
                self.ir("z", self.posicao("z") - recuo, checar_limite=False)
            try:
                pl.cmd("mover", k=e["k"], n=int(dist * spu), v=v * spu, a=20 * spu)
            except RuntimeError as ex:
                if "FIM2" not in str(ex):
                    raise
            s = self.esperar("z", timeout=dist / v + 30)
            if not s["f2"]:
                raise RuntimeError("Z andou %.0f mm sem acionar o fim de curso" % dist)
        return self.posicao("z")

    def mover(self, eixo, delta, **kw):
        self.ir(eixo, self.posicao(eixo) + delta, **kw)

    def parar(self):
        for p in self.placas.values():
            try:
                p.cmd("parar")
            except Exception as ex:
                print("parar:", ex)

    def emergencia(self):
        for p in self.placas.values():
            try:
                p.cmd("emerg")
            except Exception as ex:
                print("emerg:", ex)

    def soltar(self):
        for p in self.placas.values():
            p.cmd("en", v=0)

    def fechar(self):
        for p in self.placas.values():
            p.fechar()
