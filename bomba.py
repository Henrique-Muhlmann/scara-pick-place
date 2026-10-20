"""Relé da bomba de vácuo: GP4 da placa RP2040 b02d1c (sem firmware de motor: comandada pelo REPL do MicroPython).
O pino fica no estado pedido mesmo com a porta fechada.

Uso: python bomba.py liga|desliga|firmware      (firmware = desliga o relé e volta ao firmware do painel)
"""
import sys
import time

import serial
import serial.tools.list_ports

UID_BOMBA = "5044466888b02d1c"
PINO = 4


def porta():
    for p in serial.tools.list_ports.comports():
        if (p.serial_number or "").lower() == UID_BOMBA:
            return p.device
    raise RuntimeError("placa da bomba não encontrada")


class Bomba:
    """conexão aberta com o REPL da placa: cada troca do relé leva ~20 ms"""

    def __init__(self):
        self.s = serial.Serial(porta(), 115200, timeout=0.5, write_timeout=1.0)
        time.sleep(0.1)
        self.s.write(b"\x03\x03")              # para o firmware (ou um comando) e cai no REPL
        time.sleep(0.5)
        self.s.reset_input_buffer()
        self._exec("from machine import Pin; rele = Pin(%d, Pin.OUT, value=Pin(%d).value())" % (PINO, PINO))

    def _exec(self, linha):
        self.s.write(linha.encode() + b"\r\n")
        resp = b""
        fim = time.time() + 1.0
        while time.time() < fim and not resp.endswith(b">>> "):
            resp += self.s.read(self.s.in_waiting or 1)
        if b"Error" in resp:
            raise RuntimeError("bomba: %s" % resp.decode(errors="replace"))
        return resp.decode(errors="replace")

    def rele(self, v, tentativas=4):
        """troca o relé; se a placa travou (ruído da bomba), reabre a conexão e tenta de novo"""
        for k in range(tentativas):
            try:
                self._exec("rele.value(%d)" % (1 if v else 0))
                return
            except Exception as e:
                print("   [bomba] sem resposta (%s): reconectando..." % type(e).__name__, flush=True)
                try:
                    self.s.close()
                except Exception:
                    pass
                time.sleep(1.5)
                try:
                    self.__init__()
                except Exception:
                    pass
        raise RuntimeError("placa da bomba não responde: desconecte/reconecte o USB dela")

    def estado(self):
        r = self._exec("print('R', rele.value())")
        return int(r.split("R ")[1][0])

    def fechar(self):
        self.s.close()


def firmware():
    s = serial.Serial(porta(), 115200, timeout=0.5)
    s.write(b"\x03\x03"); time.sleep(0.5)
    s.write(b"from machine import Pin; Pin(%d, Pin.OUT, value=0)\r\n" % PINO); time.sleep(0.3)
    s.write(b"\x04"); time.sleep(1.5)
    out = s.read(4000).decode(errors="replace")
    s.close()
    return out


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "liga"
    if cmd == "firmware":
        print(firmware())
    else:
        b = Bomba()
        b.rele(cmd == "liga")
        print("GP%d =" % PINO, b.estado())
        b.fechar()
