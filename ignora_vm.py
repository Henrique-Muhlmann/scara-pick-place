"""Desliga o monitor de 24 V (GP1) da placa do braço SÓ NA RAM: para o firmware (Ctrl+C), troca o pino `vs` do
no.py por um que sempre lê 1 e chama no.rodar() de novo. Um reinício da placa (Ctrl+D, USB, energia) volta ao normal.

Use só com o sensor de 24 V com defeito e alguém perto da emergência: sem ele a placa não para se a fonte cair.
Uso: python ignora_vm.py [braco|z]
"""
import sys
import time

import serial

from scara import acha_portas

CODIGO = b"""import no
class _VmSempre:
    def value(self, *a):
        return 1
no.vs = _VmSempre()
no.rodar()
"""


def aplicar(placa="braco"):
    s = serial.Serial(acha_portas()[placa], 115200, timeout=0.5)
    time.sleep(0.1)
    s.write(b"\x03\x03")                     # para o firmware (os motores ficam soltos) -> REPL
    time.sleep(0.6)
    s.read(4000)
    s.write(b"\x05")                         # modo colar
    time.sleep(0.2)
    s.write(CODIGO.replace(b"\n", b"\r\n"))
    s.write(b"\x04")                         # executa
    fim, out = time.time() + 5, b""
    while time.time() < fim and b"pronto" not in out:
        out += s.read(s.in_waiting or 1)
    s.close()
    if b"pronto" not in out:
        raise RuntimeError("firmware não voltou: %r" % out[-300:])
    return out.decode(errors="replace").strip().splitlines()[-1]


if __name__ == "__main__":
    print(aplicar(sys.argv[1] if len(sys.argv) > 1 else "braco"))
