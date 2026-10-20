"""Câmera de cima (Logitech C920). O índice no Windows muda ao trocar de porta/hub: sem índice, procura a
primeira câmera que entrega 1920x1080 (a webcam do notebook só chega a 1280x720)."""
import cv2


def acha_indice(w=1920, h=1080, maximo=5):
    for i in range(maximo):
        c = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        try:
            if not c.isOpened():
                continue
            c.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            for _ in range(5):
                r, f = c.read()
            if r and f is not None and f.shape[1] == w and f.shape[0] == h:
                return i
        finally:
            c.release()
    raise RuntimeError("câmera %dx%d não encontrada (C920 desconectada ou aberta em outro programa?)" % (w, h))


class Cam:
    def __init__(self, idx=None, w=1920, h=1080):
        self.idx = acha_indice(w, h) if idx is None else idx
        self.c = cv2.VideoCapture(self.idx, cv2.CAP_DSHOW)
        self.c.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.c.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        for _ in range(10):
            self.c.read()

    def foto(self, arq=None, descarta=5):
        for _ in range(descarta):
            self.c.read()
        r, f = self.c.read()
        if not r or f is None:
            raise RuntimeError("câmera não devolveu imagem")
        if arq:
            cv2.imwrite(arq, f)
        return f

    def fechar(self):
        self.c.release()
