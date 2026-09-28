"""Calibracao automatica da densidade de pixels em funcao da altitude.

Mede, em varias altitudes, o tamanho aparente em pixels de um alvo de
dimensoes conhecidas posicionado na origem do mundo. Escreve os dados
brutos em CSV e ajusta o modelo.
"""
import asyncio
import csv
import math

import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from gazebo_opencv import Video

# ---------------------------------------------------------------- parametros
LADO_ALVO_M = 2.0                  # lado da placa no mundo, em metros
# Referencias verticais no mundo:
#   asphalt_plane e 200x200x0.1 centrado em z=0 -> topo em 0.05
#   placa de calibracao esta em z=0.20 com 0.02 -> topo em 0.21
ALTURA_TOPO_ALVO_M = 0.21
ALTURA_DECOLAGEM_M = 0.05
OFFSET_ALVO_M = ALTURA_TOPO_ALVO_M - ALTURA_DECOLAGEM_M   # 0.16 m
ALTITUDES_M = [3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0, 17.0, 20.0]
ESPERA_ESTABILIZAR_S = 12.0        # tempo para assentar em cada altitude
QUADROS_POR_ALTITUDE = 15          # amostras promediadas por altitude
ARQUIVO_CSV = 'calibracao.csv'
JANELA = 'Calibracao'

# Geometria declarada no SDF do sensor, para comparacao teorica
FOV_RAD = 1.5009831567
LARGURA_PX = 848

# Curva atualmente usada no controlador
CAL_A = 231.71
CAL_B = -0.813

# Faixa HSV do azul do alvo
HSV_MIN = np.array([100, 120, 50])
HSV_MAX = np.array([130, 255, 255])


import os

SALVAR_IMAGENS = True
DIR_IMAGENS = 'calibracao_imgs'


def medir_alvo(quadro):
    """Mede o alvo. Retorna (larg, alt, lado_area, caixa) ou None."""
    hsv = cv2.cvtColor(quadro, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
    nucleo = np.ones((5, 5), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, nucleo)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contornos:
        return None

    c = max(contornos, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < 40:
        return None

    x, y, w, h = cv2.boundingRect(c)
    if x <= 0 or y <= 0 or x + w >= quadro.shape[1] - 1 \
            or y + h >= quadro.shape[0] - 1:
        return None

    return float(w), float(h), math.sqrt(area), (x, y, w, h)


def anotar_medicao(quadro, caixa, altura_m, lado_px, rho):
    """Desenha a regiao detectada e as grandezas medidas sobre o quadro."""
    img = quadro.copy()
    x, y, w, h = caixa

    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # cota horizontal sob a caixa
    yc = y + h + 22
    cv2.arrowedLine(img, (x, yc), (x + w, yc), (0, 0, 255), 1, tipLength=0.03)
    cv2.arrowedLine(img, (x + w, yc), (x, yc), (0, 0, 255), 1, tipLength=0.03)

    fonte = cv2.FONT_HERSHEY_SIMPLEX
    linhas = [
        f'h = {altura_m:.2f} m',
        f'L = {lado_px:.1f} px',
        f'rho = {rho:.2f} px/m',
    ]
    for i, texto in enumerate(linhas):
        pos = (12, 26 + i * 24)
        cv2.putText(img, texto, pos, fonte, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, texto, pos, fonte, 0.65, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return img
async def coletar(video, drone, altitude_alvo):
    """Sobe ate a altitude e devolve as medidas promediadas."""
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -altitude_alvo, 0.0))
    await asyncio.sleep(ESPERA_ESTABILIZAR_S)

    larguras, alturas, lados, altitudes = [], [], [], []
    primeiro = None
    tentativas = 0
    while len(larguras) < QUADROS_POR_ALTITUDE and tentativas < 100:
        tentativas += 1
        await asyncio.sleep(0.15)
        if not video.frame_available():
            continue
        quadro = video.frame().copy()
        cv2.imshow(JANELA, quadro)
        cv2.waitKey(1)
        medida = medir_alvo(quadro)
        if medida is None:
            continue
        w, h, lado, caixa = medida
        larguras.append(w)
        alturas.append(h)
        lados.append(lado)
        altitudes.append(await altitude_relativa(drone) - OFFSET_ALVO_M)
        if primeiro is None:
            primeiro = (quadro, caixa)

    if not larguras:
        return None

    h_med = float(np.mean(altitudes))
    lado_med = (float(np.mean(larguras)) + float(np.mean(alturas))
                + float(np.mean(lados))) / 3.0
    rho = lado_med / LADO_ALVO_M

    if SALVAR_IMAGENS and primeiro is not None:
        os.makedirs(DIR_IMAGENS, exist_ok=True)
        quadro, caixa = primeiro
        img = anotar_medicao(quadro, caixa, h_med, lado_med, rho)
        nome = os.path.join(DIR_IMAGENS,
                            f'calibracao_{altitude_alvo:04.1f}m.png')
        cv2.imwrite(nome, img)
        print(f'   imagem salva em {nome}')

    return {
        'altitude_comandada_m': altitude_alvo,
        'altitude_medida_m': h_med,
        'largura_px': float(np.mean(larguras)),
        'desvio_largura_px': float(np.std(larguras)),
        'altura_px': float(np.mean(alturas)),
        'lado_area_px': float(np.mean(lados)),
        'amostras': len(larguras),
    }

def rho_geometrico(h_m):
    """Densidade de pixels prevista pela geometria da camera."""
    return LARGURA_PX / (2.0 * h_m * math.tan(FOV_RAD / 2.0))


async def altitude_relativa(drone):
    async for p in drone.telemetry.position():
        return p.relative_altitude_m


async def esta_armado(drone):
    async for a in drone.telemetry.armed():
        return a


async def aguardar_pouso(drone):
    async for no_ar in drone.telemetry.in_air():
        if not no_ar:
            return


def ajustar_e_relatar(registros):
    if len(registros) < 3:
        print('\nAmostras insuficientes para ajuste.')
        return

    h = np.array([r['altitude_medida_m'] for r in registros])
    # tres estimadores independentes da densidade de pixels
    rho_larg = np.array([r['largura_px'] for r in registros]) / LADO_ALVO_M
    rho_alt = np.array([r['altura_px'] for r in registros]) / LADO_ALVO_M
    rho_area = np.array([r['lado_area_px'] for r in registros]) / LADO_ALVO_M
    rho = (rho_larg + rho_alt + rho_area) / 3.0

    print('\n' + '=' * 78)
    print('TABELA DE CALIBRACAO')
    print('=' * 78)
    print(f"{'h_cmd':>7} {'h_med':>7} {'larg_px':>9} {'dp_px':>7} "
          f"{'rho_med':>9} {'rho_geo':>9} {'rho_cal':>9} {'dif_cal':>8}")
    for r, rr in zip(registros, rho):
        hm = r['altitude_medida_m']
        rg = rho_geometrico(hm)
        rc = CAL_A * (hm ** CAL_B)
        print(f"{r['altitude_comandada_m']:7.1f} {hm:7.2f} "
              f"{r['largura_px']:9.1f} {r['desvio_largura_px']:7.2f} "
              f"{rr:9.2f} {rg:9.2f} {rc:9.2f} "
              f"{100.0 * (rc - rr) / rr:7.1f}%")

    # Ajuste de potencia: ln(rho) = ln(a) + b*ln(h)
    b, ln_a = np.polyfit(np.log(h), np.log(rho), 1)
    a = math.exp(ln_a)
    prev = a * h ** b
    ss_res = float(np.sum((rho - prev) ** 2))
    ss_tot = float(np.sum((rho - rho.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    # Ajuste do modelo geometrico rho = k/h (minimos quadrados em k)
    k = float(np.sum(rho / h) / np.sum(1.0 / h ** 2))
    prev_k = k / h
    r2_k = 1.0 - float(np.sum((rho - prev_k) ** 2)) / ss_tot if ss_tot > 0 \
        else float('nan')

    print('\n' + '=' * 78)
    print('AJUSTES')
    print('=' * 78)
    print(f'Lei de potencia livre : rho(h) = {a:.3f} * h^({b:.4f})     '
          f'R2 = {r2:.5f}')
    print(f'Modelo geometrico     : rho(h) = {k:.3f} / h             '
          f'R2 = {r2_k:.5f}')
    print(f'Previsao pela optica  : rho(h) = '
          f'{LARGURA_PX / (2.0 * math.tan(FOV_RAD / 2.0)):.3f} / h')
    print(f'Curva em uso hoje     : rho(h) = {CAL_A} * h^({CAL_B})')
    print('\nA geometria de uma camera orientada para baixo exige expoente')
    print('exatamente -1. Um expoente ajustado distante de -1 indica erro')
    print('sistematico na medida ou na altitude de referencia.')


async def main():
    video = Video()
    cv2.namedWindow(JANELA, cv2.WINDOW_AUTOSIZE)

    drone = System()
    await drone.connect(system_address='udpin://0.0.0.0:14540')

    print('Aguardando conexao...')
    async for estado in drone.core.connection_state():
        if estado.is_connected:
            print('-- Conectado')
            break

    print('Aguardando estimativa de posicao...')
    async for saude in drone.telemetry.health():
        if saude.is_global_position_ok and saude.is_home_position_ok:
            print('-- Estimativa de posicao OK')
            break

    if not await esta_armado(drone):
        print('-- Armando')
        await drone.action.arm()

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as erro:
        print(f'Falha ao iniciar offboard: {erro._result.result}')
        await drone.action.disarm()
        return

    registros = []
    try:
        for altitude in ALTITUDES_M:
            print(f'-- Subindo para {altitude:.1f} m...')
            r = await coletar(video, drone, altitude)
            if r is None:
                print(f'   alvo nao detectado a {altitude:.1f} m; ignorado')
                continue
            print(f"   h={r['altitude_medida_m']:.2f} m  "
                  f"largura={r['largura_px']:.1f} px  "
                  f"(dp {r['desvio_largura_px']:.2f}, "
                  f"n={r['amostras']})")
            registros.append(r)

        if registros:
            with open(ARQUIVO_CSV, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(registros[0].keys()))
                w.writeheader()
                w.writerows(registros)
            print(f'\n-- Dados brutos gravados em {ARQUIVO_CSV}')

        ajustar_e_relatar(registros)
    finally:
        try:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await drone.offboard.stop()
        except Exception:
            pass
        try:
            print('\n-- Pousando')
            await drone.action.land()
            await asyncio.wait_for(aguardar_pouso(drone), timeout=90)
            print('-- Pousado')
        except Exception as erro:
            print(f'-- Falha ao pousar: {erro}')
        cv2.destroyAllWindows()


if __name__ == '__main__':
    asyncio.run(main())
