"""Acha a ponta do braço (centro do hexágono azul escuro do rolamento da ventosa) numa foto de cima."""
import cv2, numpy as np

CENTRO_J1 = (1113.0, 690.0)   # eixo do J1 na imagem (medido girando o J1)

def hexagonos(f):
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, np.array((105, 60, 35)), np.array((130, 255, 150)))   # azul dos hexágonos (câmeras antiga e nova)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cs:
        h = cv2.convexHull(c)
        a = cv2.contourArea(h)
        if a < 3000:
            continue
        (x, y), r = cv2.minEnclosingCircle(h)
        circ = a / (np.pi * r * r)
        M = cv2.moments(h)
        out.append({"c": (M["m10"] / M["m00"], M["m01"] / M["m00"]), "area": a, "r": r, "circ": circ, "hull": h})
    return out, m

def acha_ponta(f, centro=CENTRO_J1):
    hs, _ = hexagonos(f)
    # ponta: hexágono com o rolamento e o engate branco (centro claro); o cotovelo tem o motor (centro escuro)
    g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    cand = []
    for h in hs:
        if not (h["circ"] > 0.5 and 40 < h["r"] < 115):
            continue
        x, y = int(h["c"][0]), int(h["c"][1])
        disco = np.zeros_like(g)
        cv2.circle(disco, (x, y), 18, 255, -1)
        h["brilho"] = cv2.mean(g, disco)[0]
        if h["brilho"] > 110:
            cand.append(h)
    if not cand:
        return None
    return max(cand, key=lambda h: h["brilho"])

if __name__ == "__main__":
    import sys
    f = cv2.imread(sys.argv[1])
    hs, m = hexagonos(f)
    for h in hs: print({k: (np.round(v, 2) if k != "hull" else None) for k, v in h.items()})
    p = acha_ponta(f); print("PONTA", p and np.round(p["c"], 1))
    v = f.copy()
    for h in hs: cv2.drawContours(v, [h["hull"]], -1, (0, 255, 0), 2)
    if p: cv2.drawMarker(v, tuple(int(x) for x in p["c"]), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    cv2.imwrite(sys.argv[2] if len(sys.argv) > 2 else "fotos/ponta_dbg.jpg", cv2.resize(v, (960, 540)))


def _anel(f, cx, cy, raio_busca=70):
    g = cv2.GaussianBlur(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (5, 5), 1)
    x0, y0 = max(cx - raio_busca, 0), max(cy - raio_busca, 0)
    crop = g[y0:cy + raio_busca, x0:cx + raio_busca]
    if crop.shape[0] < 2 * raio_busca - 10 or crop.shape[1] < 2 * raio_busca - 10:
        return None                                     # ponto previsto fora (ou na borda) da imagem
    cs = cv2.HoughCircles(crop, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=80, param2=25, minRadius=22, maxRadius=40)
    if cs is None:
        return None
    x, y, _ = min(cs[0], key=lambda c: np.hypot(c[0] + x0 - cx, c[1] + y0 - cy))
    return float(x + x0), float(y + y0)


def ponta_precisa(f, esperado=None):
    """centro do anel do rolamento (Hough) dentro do hexágono da ponta; cai no centro do hexágono se não achar.
    Com `esperado` (pixel previsto pelo modelo): se o hexágono não aparecer ali (ex.: azul da caixa embaixo),
    procura o anel do rolamento em volta do ponto previsto."""
    p = acha_ponta(f)
    if esperado is not None and (p is None or np.hypot(p["c"][0] - esperado[0], p["c"][1] - esperado[1]) > 60):
        return _anel(f, int(esperado[0]), int(esperado[1]), 60)
    if p is None:
        return None
    cx, cy = map(int, p["c"])
    g = cv2.GaussianBlur(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (5, 5), 1)
    x0, y0 = max(cx - 70, 0), max(cy - 70, 0)
    crop = g[y0:cy + 70, x0:cx + 70]
    cs = cv2.HoughCircles(crop, cv2.HOUGH_GRADIENT, dp=1, minDist=50, param1=80, param2=25, minRadius=22, maxRadius=40)
    if cs is not None:
        x, y, _ = cs[0][0]
        if np.hypot(x + x0 - p["c"][0], y + y0 - p["c"][1]) < 12:
            return (float(x + x0), float(y + y0))
    return p["c"]
