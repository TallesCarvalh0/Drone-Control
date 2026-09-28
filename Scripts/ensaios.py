"""Ensaios automatizados do sistema de posicionamento assistido.

Dois modos de operação:

  MODO = 'solo'
      Campanha estatística. A altitude de operação e a posição de partida são
      sorteadas de distribuições uniformes declaradas, com semente fixa. O
      alvo é a placa de solo de coordenadas conhecidas no mundo simulado. O
      ensaio termina com o pouso sobre o alvo, o que torna mensurável o erro
      entre o ponto de pouso desejado e o efetivamente alcançado.

  MODO = 'condutor'
      Ensaios representativos de aproximação à linha. O VANT parte de
      afastamentos controlados em relação a um condutor de coordenadas
      conhecidas e executa a aproximação horizontal seguida da descida até
      uma folga acima do cabo.

Em ambos os modos a seleção do operador é obtida por segmentação da imagem,
e não calculada a partir das coordenadas do mundo. Essa distinção é
essencial: calcular o pixel a partir da geometria conhecida usaria a
calibração para gerar a seleção e novamente para convertê-la, cancelando-a.
Detectando na imagem, o erro medido contra a coordenada verdadeira do alvo
inclui projeção, calibração e controle.

Saídas em ensaios/:
    ensaio_<id>.csv         série temporal
    quadro_<id>_inicio.png  quadro no instante da seleção, anotado
    quadro_<id>_fim.png     quadro ao final, anotado
    resumo.csv              uma linha por ensaio
    metricas.txt            métricas consolidadas
"""
import asyncio
import csv
import math
import os
import random
import signal
import time

import cv2
import numpy as np
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw
from gazebo_opencv import Video

# ======================================================= modo de operação
MODO = 'solo'                 # 'solo' ou 'condutor'

# ======================================================= controlador (Cap. 2)
KP = 0.3                      # ganho proporcional horizontal
V_MAX_MS = 1.5                # saturação do módulo da velocidade horizontal
TOLERANCIA_M = 0.05           # piso absoluto da tolerância
TOLERANCIA_PX = 2.0           # tolerância em pixels do plano da imagem
PERIODO_COMANDO_S = 0.25      # período do laço de controle
RHO_K = 461.7                 # calibração: rho(h) = RHO_K / h
SINAL_LESTE = +1.0
SINAL_NORTE = -1.0

# Braco de alavanca da camera. No modelo iris_downward_depth_camera a camera
# esta deslocada 0,108 m a frente da origem do corpo (base_link), que e o
# ponto cuja posicao a telemetria reporta. Centrar o alvo na imagem coloca a
# CAMERA sobre ele, nao o veiculo. Para o pouso interessa o veiculo, de modo
# que o deslocamento deduzido da imagem e somado a este braco.
# Com o yaw mantido em zero, o eixo x do corpo coincide com o norte.
CAMERA_FRENTE_M = 0.108
CAMERA_LATERAL_M = 0.0

# ======================================================= geometria do mundo
# Coordenadas de linha_transmissao.world (Gazebo, ENU).
# Conversão ENU -> NED do PX4:  norte = y_enu,  leste = x_enu,  baixo = -z_enu.
# A origem do referencial local NED coincide com o ponto de nascimento do
# VANT, que NAO e a origem do mundo. Verificado experimentalmente: dois
# ensaios independentes situaram o alvo de solo em NED (-0.5, +3.0), sendo
# sua coordenada no mundo (x=4.0, y=0.5). O deslocamento constante de (1, 1)
# corresponde a posicao de nascimento do veiculo.
ORIGEM_NED_NO_MUNDO = {'x': 1.0, 'y': 1.0}


def mundo_para_ned(x_enu, y_enu):
    """Converte coordenadas do mundo (Gazebo, ENU) para o local NED."""
    return (y_enu - ORIGEM_NED_NO_MUNDO['y'],
            x_enu - ORIGEM_NED_NO_MUNDO['x'])


_n_alvo, _e_alvo = mundo_para_ned(4.0, 0.5)
ALVO_SOLO = {'nome': 'alvo_solo', 'norte_m': _n_alvo, 'leste_m': _e_alvo,
             'altura_m': 0.0, 'lado_m': 0.7}

ALTURA_CONDUTOR_M = 7.3
ALVOS_CONDUTOR = [
    {'nome': 'cabo1', 'leste_m': mundo_para_ned(2.3, 0.0)[1], 'lado': -1},
    {'nome': 'cabo2', 'leste_m': mundo_para_ned(3.7, 0.0)[1], 'lado': +1},
]

# ======================================================= campanha aleatória
SEMENTE = 20260926            # semente fixa: a amostra é reproduzível
N_ENSAIOS = 30

ALTITUDE_MIN_M = 5.0
ALTITUDE_MAX_M = 18.0
RAIO_MIN_M = 1.0              # afastamento inicial mínimo em relação ao alvo
RAIO_MAX_M = 6.0              # afastamento inicial máximo
FRACAO_MAX_QUADRO = 0.65      # fração da meia-largura do quadro utilizável

ALTURA_POUSO_M = 0.12         # altura em que a descida controlada termina

# Deteccao de toque. O trem de pouso impede o veiculo de alcancar
# ALTURA_POUSO_M: ele repousa alguns centimetros acima. Sem um criterio de
# parada fisico, o laco continua comandando velocidade contra o solo, o que
# infla o tempo do ensaio e desloca a posicao horizontal ja alcancada.
JANELA_TOQUE_S = 1.5          # janela em que a altitude e observada
FRACAO_DESCIDA_MIN = 0.30     # razao entre a descida obtida e a comandada
DESCIDA_RESIDUAL_MS = 0.10    # taxa absoluta abaixo da qual se considera parado
FAIXA_TOQUE_M = 0.15          # so aceita toque junto da altura pedida

# ======================================================= ensaios no condutor
ALTITUDE_CONDUTOR_M = 10.0
AFASTAMENTOS_M = [0.8, 1.5, 2.0]
REPETICOES_CONDUTOR = 5
FOLGA_PARADA_M = 0.25

# ======================================================= comuns
V_DESCIDA_MS = 0.8

# Assentamento antes de cada ensaio. A espera termina quando o veiculo
# permanece parado e proximo da posicao pedida, e nao ao fim de um prazo
# fixo: com prazo fixo o veiculo ainda se move ao capturar o quadro, e a
# altitude usada para calcular a densidade de pixels nao corresponde a
# altitude em que a imagem foi formada.
ESPERA_MIN_REPOSICIONAMENTO_S = 6.0
TEMPO_MAX_REPOSICIONAMENTO_S = 60.0
RAPIDEZ_ASSENTADO_MS = 0.08   # velocidade abaixo da qual se considera parado
RAIO_ASSENTADO_M = 0.60       # distancia maxima ate a posicao pedida
PERMANENCIA_ASSENTADO_S = 2.0  # tempo que a condicao precisa se manter
TEMPO_MAX_HORIZONTAL_S = 120.0
TEMPO_MAX_DESCIDA_S = 120.0

LARGURA_QUADRO_PX = 848
DIR_SAIDA = 'ensaios'
JANELA = 'Ensaios'

# ======================================================= segmentação
# Alvo de solo: placa clara sobre grama. Segmenta-se por saturação baixa e
# valor alto, o que isola o branco do padrão.
HSV_SOLO_MIN = np.array([0, 0, 165])
HSV_SOLO_MAX = np.array([179, 70, 255])
AREA_MIN_SOLO_PX = 60

# Condutor: preto sobre grama. Segmenta-se por valor baixo.
HSV_CABO_MIN = np.array([0, 0, 0])
HSV_CABO_MAX = np.array([179, 255, 70])
AREA_MIN_CABO_PX = 150
COMPRIMENTO_MIN_CABO_PX = 150

# ======================================================= encerramento
parar = False


def _sigint(sig, frame):
    global parar
    parar = True
    print('\n-- Interrupção solicitada; encerrando após o ensaio corrente.')


signal.signal(signal.SIGINT, _sigint)


def densidade_pixels(h_m):
    return RHO_K / h_m


def meia_largura_m(h_m):
    """Meia-largura do terreno abrangido pelo quadro, em metros."""
    return (LARGURA_QUADRO_PX / 2.0) / densidade_pixels(h_m)


# ----------------------------------------------------------------- detecção
def detectar_alvo_solo(quadro):
    """Centroide da placa de solo, ou None."""
    hsv = cv2.cvtColor(quadro, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, HSV_SOLO_MIN, HSV_SOLO_MAX)
    nucleo = np.ones((3, 3), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, nucleo)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    contornos = [c for c in contornos if cv2.contourArea(c) >= AREA_MIN_SOLO_PX]
    if not contornos:
        return None

    c = max(contornos, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    altura, largura = quadro.shape[:2]
    if x <= 0 or y <= 0 or x + w >= largura - 1 or y + h >= altura - 1:
        return None                       # alvo cortado pela borda

    m = cv2.moments(c)
    if m['m00'] == 0:
        return None
    return (int(round(m['m10'] / m['m00'])), int(round(m['m01'] / m['m00'])))


def detectar_condutor(quadro):
    """Pixel do condutor mais próximo do centro do quadro, ou None."""
    hsv = cv2.cvtColor(quadro, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, HSV_CABO_MIN, HSV_CABO_MAX)
    nucleo = np.ones((3, 3), np.uint8)
    mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_NONE)
    altura, largura = quadro.shape[:2]
    cx, cy = largura / 2.0, altura / 2.0

    melhor, melhor_d = None, float('inf')
    for c in contornos:
        if cv2.contourArea(c) < AREA_MIN_CABO_PX:
            continue
        _, _, w, h = cv2.boundingRect(c)
        if max(w, h) < COMPRIMENTO_MIN_CABO_PX:
            continue
        pts = c.reshape(-1, 2).astype(float)
        d = (pts[:, 0] - cx) ** 2 + (pts[:, 1] - cy) ** 2
        i = int(d.argmin())
        if d[i] < melhor_d:
            melhor_d, melhor = d[i], (int(pts[i, 0]), int(pts[i, 1]))
    return melhor


def detectar(quadro):
    return (detectar_alvo_solo(quadro) if MODO == 'solo'
            else detectar_condutor(quadro))


# ----------------------------------------------------------------- anotação
def anotar_quadro(quadro, ponto, rho, rotulo, extras=()):
    """Desenha o centro do quadro, o ponto detectado e o vetor de erro."""
    img = quadro.copy()
    altura, largura = img.shape[:2]
    cx, cy = largura // 2, altura // 2

    cv2.line(img, (cx - 16, cy), (cx + 16, cy), (0, 255, 255), 1)
    cv2.line(img, (cx, cy - 16), (cx, cy + 16), (0, 255, 255), 1)

    linhas = [rotulo, f'rho = {rho:.1f} px/m']
    if ponto is not None:
        mx, my = ponto
        cv2.line(img, (cx, cy), (mx, my), (0, 0, 255), 1)
        cv2.circle(img, (mx, my), 6, (0, 0, 255), -1)
        cv2.circle(img, (mx, my), 12, (0, 0, 255), 1)
        dpx = math.hypot(mx - cx, my - cy)
        linhas.append(f'erro = ({mx - cx:+d}, {my - cy:+d}) px')
        linhas.append(f'     = {dpx / rho:.3f} m')
    else:
        linhas.append('alvo nao detectado')
    linhas.extend(extras)

    fonte = cv2.FONT_HERSHEY_SIMPLEX
    for i, t in enumerate(linhas):
        pos = (12, 26 + i * 24)
        cv2.putText(img, t, pos, fonte, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, t, pos, fonte, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ----------------------------------------------------------------- telemetria
async def altitude_relativa(drone):
    async for p in drone.telemetry.position():
        return p.relative_altitude_m


async def posicao_ned(drone):
    async for pv in drone.telemetry.position_velocity_ned():
        return (round(pv.position.north_m, 4),
                round(pv.position.east_m, 4),
                round(pv.position.down_m, 4))


async def posicao_velocidade_ned(drone):
    """Posicao e velocidade do mesmo instante, em uma unica consulta."""
    async for pv in drone.telemetry.position_velocity_ned():
        return ((round(pv.position.north_m, 4),
                 round(pv.position.east_m, 4),
                 round(pv.position.down_m, 4)),
                (round(pv.velocity.north_m_s, 4),
                 round(pv.velocity.east_m_s, 4),
                 round(pv.velocity.down_m_s, 4)))


async def esta_armado(drone):
    async for a in drone.telemetry.armed():
        return a


async def no_ar(drone):
    async for v in drone.telemetry.in_air():
        return v


async def aguardar_pouso(drone):
    async for v in drone.telemetry.in_air():
        if not v:
            return


async def estado_pouso(drone):
    async for e in drone.telemetry.landed_state():
        return str(e)


async def exibir(video):
    """Mantém a janela viva para que o fluxo de vídeo seja consumido."""
    while True:
        if video.frame_available():
            cv2.imshow(JANELA, video.frame())
        cv2.waitKey(1)
        await asyncio.sleep(0.05)


# ----------------------------------------------------------------- registro
class Registro:
    CAMPOS = ['t_s', 'dt_s', 'fase', 'north_m', 'east_m', 'down_m',
              'altitude_m', 'erro_norte_m', 'erro_leste_m', 'erro_radial_m',
              'v_norte_ms', 'v_leste_ms', 'v_down_ms',
              't_telemetria_ms', 't_calculo_ms']

    def __init__(self):
        self.linhas = []
        self.t0 = time.perf_counter()
        self._ultimo = self.t0

    def anotar(self, fase, pos, erro_n, erro_e, v_n, v_e, v_d,
               t_telemetria=0.0, t_calculo=0.0):
        agora = time.perf_counter()
        dt = agora - self._ultimo
        self._ultimo = agora
        n, e, d = pos
        self.linhas.append({
            't_s': round(agora - self.t0, 4),
            'dt_s': round(dt, 4),
            'fase': fase,
            'north_m': n, 'east_m': e, 'down_m': d,
            'altitude_m': round(-d, 4),
            'erro_norte_m': round(erro_n, 4),
            'erro_leste_m': round(erro_e, 4),
            'erro_radial_m': round(math.hypot(erro_n, erro_e), 4),
            'v_norte_ms': round(v_n, 4),
            'v_leste_ms': round(v_e, 4),
            'v_down_ms': round(v_d, 4),
            # Custo de cada iteracao, decomposto. O periodo total (dt_s) e
            # imposto por PERIODO_COMANDO_S; estas duas colunas medem o que
            # de fato e gasto consultando a telemetria e calculando a lei de
            # controle, que e o que se quer caracterizar.
            't_telemetria_ms': round(t_telemetria * 1000, 3),
            't_calculo_ms': round(t_calculo * 1000, 4),
        })

    @property
    def duracao_s(self):
        return self.linhas[-1]['t_s'] if self.linhas else 0.0

    def _media(self, campo, desde=1):
        v = [l[campo] for l in self.linhas[desde:]]
        return sum(v) / len(v) if v else float('nan')

    def dt_medio_s(self):
        return self._media('dt_s')

    def t_telemetria_medio_ms(self):
        return self._media('t_telemetria_ms')

    def t_calculo_medio_ms(self):
        return self._media('t_calculo_ms')

    def salvar(self, caminho):
        with open(caminho, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=self.CAMPOS)
            w.writeheader()
            w.writerows(self.linhas)


# ----------------------------------------------------------------- manobras
async def reposicionar(drone, norte_m, leste_m, altitude_m):
    """Leva o veiculo a posicao de partida e espera que ele assente.

    A espera nao e um prazo fixo. Um prazo fixo deixa o veiculo ainda em
    movimento quando o quadro e capturado, e como a densidade de pixels
    varia com o inverso da altitude, uma altitude que ainda esta mudando
    introduz um erro de escala proporcional em toda a conversao.

    Devolve a velocidade no instante em que a espera termina, para que a
    condicao de partida de cada ensaio fique registrada.
    """
    await drone.offboard.set_position_ned(
        PositionNedYaw(norte_m, leste_m, -altitude_m, 0.0))

    inicio = time.perf_counter()
    limite = inicio + TEMPO_MAX_REPOSICIONAMENTO_S
    piso = inicio + ESPERA_MIN_REPOSICIONAMENTO_S
    vel = (0.0, 0.0, 0.0)
    estavel_desde = None

    while not parar and time.perf_counter() < limite:
        pos, vel = await posicao_velocidade_ned(drone)
        rapidez = math.sqrt(sum(v * v for v in vel))
        distancia = math.sqrt((pos[0] - norte_m) ** 2
                              + (pos[1] - leste_m) ** 2
                              + (pos[2] + altitude_m) ** 2)
        agora = time.perf_counter()
        if rapidez <= RAPIDEZ_ASSENTADO_MS and distancia <= RAIO_ASSENTADO_M:
            if estavel_desde is None:
                estavel_desde = agora
            elif (agora - estavel_desde >= PERMANENCIA_ASSENTADO_S
                  and agora >= piso):
                return vel
        else:
            estavel_desde = None
        await asyncio.sleep(0.3)
    return vel


def saturar(v_n, v_e):
    m = math.hypot(v_n, v_e)
    if m > V_MAX_MS:
        k = V_MAX_MS / m
        return v_n * k, v_e * k
    return v_n, v_e


async def fase_horizontal(drone, reg, alvo_n, alvo_e, tolerancia):
    limite = time.perf_counter() + TEMPO_MAX_HORIZONTAL_S
    while not parar and time.perf_counter() < limite:
        t_a = time.perf_counter()
        pos = await posicao_ned(drone)
        t_b = time.perf_counter()

        falta_n, falta_e = alvo_n - pos[0], alvo_e - pos[1]
        convergiu = math.hypot(falta_n, falta_e) <= tolerancia
        v_n, v_e = (0.0, 0.0) if convergiu \
            else saturar(KP * falta_n, KP * falta_e)
        t_c = time.perf_counter()

        reg.anotar('horizontal', pos, falta_n, falta_e, v_n, v_e, 0.0,
                   t_telemetria=t_b - t_a, t_calculo=t_c - t_b)
        if convergiu:
            return True
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(v_n, v_e, 0.0, 0.0))
        await asyncio.sleep(PERIODO_COMANDO_S)
    return False


async def fase_descida(drone, reg, alvo_n, alvo_e, alvo_baixo, detectar_toque):
    """Desce mantendo a correção horizontal ativa.

    Nenhuma consulta adicional de telemetria e feita dentro do laco: assinar
    um segundo fluxo custa mais de um segundo por chamada e distorceria tanto
    o periodo de iteracao medido quanto o desempenho do controle horizontal.

    A fase termina por um de dois criterios. O primeiro e a altura pedida.
    O segundo, usado apenas quando o destino e o solo, e o toque.

    O toque e reconhecido comparando a descida obtida com a comandada ao
    longo de JANELA_TOQUE_S: apoiado no solo, o veiculo deixa de responder
    ao comando vertical, e a razao entre as duas cai. O criterio e relativo
    e nao depende de um limiar absoluto de deslocamento, que teria de ser
    ajustado ao ruido da estimativa de posicao.

    Devolve (convergiu, posicao no instante da parada, criterio).
    """
    limite = time.perf_counter() + TEMPO_MAX_DESCIDA_S
    historico = []          # (t, down, v_d comandado na iteracao anterior)
    v_d_anterior = 0.0
    pos = await posicao_ned(drone)
    baixo_inicial = pos[2]

    while not parar and time.perf_counter() < limite:
        t_a = time.perf_counter()
        pos = await posicao_ned(drone)
        t_b = time.perf_counter()

        falta_n, falta_e = alvo_n - pos[0], alvo_e - pos[1]
        falta_b = alvo_baixo - pos[2]

        na_altura = abs(falta_b) <= 0.05

        historico.append((t_b, pos[2], v_d_anterior))
        historico[:] = [h for h in historico if t_b - h[0] <= JANELA_TOQUE_S]

        tocou = False
        if (detectar_toque
                and pos[2] - baixo_inicial > 0.5      # ja desceu de fato
                and abs(falta_b) <= FAIXA_TOQUE_M     # perto do solo
                and len(historico) >= 4
                and t_b - historico[0][0] >= 0.8 * JANELA_TOQUE_S):
            intervalo = t_b - historico[0][0]
            obtida = (pos[2] - historico[0][1]) / intervalo
            comandada = sum(h[2] for h in historico) / len(historico)
            if comandada > 0.03:
                # Os dois criterios sao exigidos. O relativo sozinho aceita
                # uma frenagem transitoria durante uma descida rapida como
                # se fosse apoio; o absoluto descarta esse caso, porque um
                # veiculo ainda descendo percorre bem mais que
                # DESCIDA_RESIDUAL_MS * JANELA_TOQUE_S na janela.
                tocou = (obtida < FRACAO_DESCIDA_MIN * comandada
                         and obtida < DESCIDA_RESIDUAL_MS)

        if na_altura or tocou:
            reg.anotar('descida', pos, falta_n, falta_e, 0.0, 0.0, 0.0,
                       t_telemetria=t_b - t_a,
                       t_calculo=time.perf_counter() - t_b)
            return True, pos, ('altura' if na_altura else 'toque')

        v_n, v_e = saturar(KP * falta_n, KP * falta_e)
        v_d = math.copysign(min(V_DESCIDA_MS, abs(falta_b)), falta_b)
        t_c = time.perf_counter()
        reg.anotar('descida', pos, falta_n, falta_e, v_n, v_e, v_d,
                   t_telemetria=t_b - t_a, t_calculo=t_c - t_b)
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(v_n, v_e, v_d, 0.0))
        await asyncio.sleep(PERIODO_COMANDO_S)
        v_d_anterior = v_d
    return False, pos, 'tempo esgotado'


# ----------------------------------------------------------------- ensaio
async def executar_ensaio(drone, video, ident, cond):
    """Executa um ensaio. `cond` traz as condições sorteadas ou fixas."""
    vel_partida = await reposicionar(drone, cond['norte_partida_m'],
                                     cond['leste_partida_m'],
                                     cond['altitude_cmd_m'])
    rapidez_partida = math.sqrt(sum(v * v for v in vel_partida))

    # A altitude vem da coordenada down do referencial NED, e nao do campo
    # relative_altitude_m. Os dois concordam na maior parte dos ensaios, mas
    # o segundo apresentou saltos discretos de cerca de 0,75 m em relacao a
    # posicao efetivamente ocupada, verificados contra o setpoint comandado
    # e contra o tempo de descida. Como a densidade de pixels varia com o
    # inverso da altura, esse desvio se propaga proporcionalmente a toda a
    # conversao. A coordenada NED e a mesma que realimenta o controle.
    pos_sel = await posicao_ned(drone)
    altitude = -pos_sel[2]
    altitude_telemetria = await altitude_relativa(drone)
    h_efetivo = altitude - cond['plano_m']
    if h_efetivo <= 0:
        return {'ensaio': ident, 'sucesso': 0,
                'motivo': 'altitude abaixo do plano do alvo', **cond}

    rho = densidade_pixels(h_efetivo)
    tolerancia = max(TOLERANCIA_M, TOLERANCIA_PX / rho)

    if not video.frame_available():
        return {'ensaio': ident, 'sucesso': 0,
                'motivo': 'sem quadro de video', **cond}

    quadro_inicio = video.frame().copy()
    t_det = time.perf_counter()
    ponto = detectar(quadro_inicio)
    t_deteccao_ms = (time.perf_counter() - t_det) * 1000.0
    if ponto is None:
        cv2.imwrite(os.path.join(DIR_SAIDA, f'quadro_{ident}_inicio.png'),
                    anotar_quadro(quadro_inicio, None, rho, 'selecao'))
        return {'ensaio': ident, 'sucesso': 0,
                'motivo': 'alvo nao detectado na imagem', **cond}

    altura_q, largura_q = quadro_inicio.shape[:2]
    erro_x_px = ponto[0] - largura_q / 2.0
    erro_y_px = ponto[1] - altura_q / 2.0
    erro_leste_m = SINAL_LESTE * erro_x_px / rho
    erro_norte_m = SINAL_NORTE * erro_y_px / rho

    norte0, leste0, baixo0 = await posicao_ned(drone)

    # Deslocamento do veiculo, nao da camera: ao que a imagem fornece soma-se
    # o braco de alavanca da camera em relacao a origem do corpo.
    desloc_n = erro_norte_m + CAMERA_FRENTE_M
    desloc_e = erro_leste_m + CAMERA_LATERAL_M
    alvo_n = norte0 + desloc_n
    alvo_e = leste0 + desloc_e

    # Coerência do referencial: o deslocamento deduzido da imagem deve
    # apontar para o alvo de coordenadas conhecidas.
    esp_n = cond['alvo_norte_m'] - norte0 if cond['alvo_norte_m'] is not None \
        else desloc_n
    esp_e = cond['alvo_leste_m'] - leste0
    coerente = int(abs(desloc_e - esp_e) <= 0.6
                   and abs(desloc_n - esp_n) <= 0.6)
    if not coerente:
        print(f'[{ident}] ATENCAO: deslocamento deduzido '
              f'({desloc_n:+.2f},{desloc_e:+.2f}) diverge do esperado '
              f'({esp_n:+.2f},{esp_e:+.2f}). Verifique o mapeamento de eixos.')

    cv2.imwrite(os.path.join(DIR_SAIDA, f'quadro_{ident}_inicio.png'),
                anotar_quadro(quadro_inicio, ponto, rho, 'selecao',
                              [f'h = {altitude:.2f} m',
                               f'h_ef = {h_efetivo:.2f} m']))

    print(f'[{ident}] h={altitude:.2f} m  rho={rho:.1f} px/m  '
          f'tol={tolerancia * 100:.1f} cm  '
          f'deteccao=({ponto[0]},{ponto[1]}) px')

    reg = Registro()
    conv_h = await fase_horizontal(drone, reg, alvo_n, alvo_e, tolerancia)
    t_horizontal = reg.duracao_s

    # Quadro final na altitude de operacao, na mesma escala do inicial: e
    # onde a comparacao visual tem significado. Apos a descida o alvo ocupa
    # todo o campo de visao e nada se distingue.
    pos_h = await posicao_ned(drone)
    if MODO == 'solo':
        erro_h_n = pos_h[0] - cond['alvo_norte_m']
        erro_h_e = pos_h[1] - cond['alvo_leste_m']
        erro_h = math.hypot(erro_h_n, erro_h_e)
        rotulo_erro = 'erro 2D'
    else:
        erro_h_n = float('nan')
        erro_h_e = pos_h[1] - cond['alvo_leste_m']
        erro_h = abs(erro_h_e)
        rotulo_erro = 'erro perp.'

    ponto_fim = None
    if video.frame_available():
        quadro_fim = video.frame().copy()
        ponto_fim = detectar(quadro_fim)
        h_fim = max(0.05, -pos_h[2] - cond['plano_m'])
        cv2.imwrite(os.path.join(DIR_SAIDA, f'quadro_{ident}_fim.png'),
                    anotar_quadro(quadro_fim, ponto_fim,
                                  densidade_pixels(h_fim),
                                  'alinhamento concluido',
                                  [f'h = {-pos_h[2]:.2f} m',
                                   f'{rotulo_erro} = {erro_h:.3f} m']))

    if MODO == 'solo':
        alvo_baixo = -ALTURA_POUSO_M
    else:
        alvo_baixo = -(ALTURA_CONDUTOR_M + FOLGA_PARADA_M)

    conv_v = False
    pos_toque = None
    criterio_parada = 'nao executada'
    if conv_h and not parar:
        conv_v, pos_toque, criterio_parada = await fase_descida(
            drone, reg, alvo_n, alvo_e, alvo_baixo,
            detectar_toque=(MODO == 'solo'))
    t_descida = reg.duracao_s - t_horizontal

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(1.0)

    # Leitura acomodada, 1 s apos o comando de velocidade nula. O offboard e
    # mantido ate aqui; o toque e o desarme ocorrem entre ensaios, sob
    # controle de posicao do PX4, que preserva a coordenada alcancada.
    pos_final = await posicao_ned(drone)
    erro_ref = math.hypot(alvo_n - pos_final[0], alvo_e - pos_final[1])

    # A posicao no instante da parada e a que corresponde ao ponto de pouso.
    # A leitura apos a espera de 1 s e mantida em separado para comparacao.
    pos_pouso = pos_toque if pos_toque is not None else pos_final

    def erro_contra_alvo(p):
        if MODO == 'solo':
            dn = p[0] - cond['alvo_norte_m']
            de = p[1] - cond['alvo_leste_m']
            return dn, de, math.hypot(dn, de)
        return float('nan'), p[1] - cond['alvo_leste_m'], \
            abs(p[1] - cond['alvo_leste_m'])

    erro_toque_n, erro_toque_e, erro_toque = erro_contra_alvo(pos_pouso)
    erro_verdadeiro_n, erro_verdadeiro_e, erro_verdadeiro = \
        erro_contra_alvo(pos_final)

    reg.salvar(os.path.join(DIR_SAIDA, f'ensaio_{ident}.csv'))

    sucesso = int(conv_h and conv_v)
    m = {
        'ensaio': ident,
        'modo': MODO,
        **cond,
        'altitude_med_m': round(altitude, 3),
        'h_efetivo_m': round(h_efetivo, 3),
        # Condicao de partida: quanto o veiculo ainda se movia ao capturar o
        # quadro, e quanto a altitude alcancada difere da comandada.
        'rapidez_partida_ms': round(rapidez_partida, 4),
        'desvio_altitude_m': round(altitude - cond['altitude_cmd_m'], 3),
        # As duas fontes de altitude, lado a lado, para que qualquer
        # divergencia entre elas fique registrada em vez de suposta.
        'altitude_ned_m': round(altitude, 3),
        'altitude_telemetria_m': round(altitude_telemetria, 3),
        'divergencia_altitude_m': round(altitude_telemetria - altitude, 3),
        'rho_px_m': round(rho, 2),
        'tolerancia_m': round(tolerancia, 4),
        'px_detectado_x': ponto[0],
        'px_detectado_y': ponto[1],
        # Deslocamento lido na imagem (camera) e o efetivamente comandado
        # (veiculo), que difere do primeiro pelo braco de alavanca.
        'img_norte_m': round(erro_norte_m, 4),
        'img_leste_m': round(erro_leste_m, 4),
        'ref_norte_m': round(desloc_n, 4),
        'ref_leste_m': round(desloc_e, 4),
        'referencial_coerente': coerente,
        'sucesso': sucesso,
        'convergiu_horizontal': int(conv_h),
        'convergiu_vertical': int(conv_v),
        'tempo_horizontal_s': round(t_horizontal, 3),
        'tempo_descida_s': round(t_descida, 3),
        'tempo_total_s': round(reg.duracao_s, 3),
        'criterio_parada': criterio_parada,
        'dt_medio_s': round(reg.dt_medio_s(), 5),
        't_telemetria_medio_ms': round(reg.t_telemetria_medio_ms(), 3),
        't_calculo_medio_ms': round(reg.t_calculo_medio_ms(), 4),
        't_deteccao_ms': round(t_deteccao_ms, 3),
        'iteracoes': len(reg.linhas),
        'erro_ref_m': round(erro_ref, 4),
        'erro_alinhamento_m': round(erro_h, 4),
        'erro_toque_m': round(erro_toque, 4),
        'erro_toque_norte_m': round(erro_toque_n, 4)
        if erro_toque_n == erro_toque_n else '',
        'erro_toque_leste_m': round(erro_toque_e, 4),
        'north_toque_m': pos_pouso[0],
        'east_toque_m': pos_pouso[1],
        'down_toque_m': pos_pouso[2],
        'erro_verdadeiro_m': round(erro_verdadeiro, 4),
        'erro_verdadeiro_norte_m': round(erro_verdadeiro_n, 4)
        if erro_verdadeiro_n == erro_verdadeiro_n else '',
        'erro_verdadeiro_leste_m': round(erro_verdadeiro_e, 4),
        'north_final_m': pos_final[0],
        'east_final_m': pos_final[1],
        'down_final_m': pos_final[2],
        'px_final_x': ponto_fim[0] if ponto_fim else '',
        'px_final_y': ponto_fim[1] if ponto_fim else '',
        'estado_pouso': await estado_pouso(drone),
    }

    print(f'[{ident}] sucesso={sucesso}  t={m["tempo_total_s"]:.1f} s  '
          f'{rotulo_erro}={erro_verdadeiro:.3f} m  '
          f'dt={m["dt_medio_s"] * 1000:.1f} ms')
    return m


# ----------------------------------------------------------------- condições
def sortear_condicoes():
    """Sorteia as condições da campanha estatística, com semente fixa."""
    rng = random.Random(SEMENTE)
    cond = []
    for i in range(1, N_ENSAIOS + 1):
        h = rng.uniform(ALTITUDE_MIN_M, ALTITUDE_MAX_M)
        r_max = min(RAIO_MAX_M, FRACAO_MAX_QUADRO * meia_largura_m(h))
        r = rng.uniform(RAIO_MIN_M, max(RAIO_MIN_M + 0.1, r_max))
        ang = rng.uniform(0.0, 2.0 * math.pi)
        cond.append({
            'indice': i,
            'plano_m': ALVO_SOLO['altura_m'],
            'alvo_nome': ALVO_SOLO['nome'],
            'alvo_norte_m': ALVO_SOLO['norte_m'],
            'alvo_leste_m': ALVO_SOLO['leste_m'],
            'altitude_cmd_m': round(h, 3),
            'raio_partida_m': round(r, 3),
            'angulo_partida_rad': round(ang, 4),
            'norte_partida_m': round(ALVO_SOLO['norte_m'] + r * math.sin(ang), 3),
            'leste_partida_m': round(ALVO_SOLO['leste_m'] + r * math.cos(ang), 3),
        })
    return cond


def condicoes_condutor():
    cond = []
    i = 0
    for alvo in ALVOS_CONDUTOR:
        for d in AFASTAMENTOS_M:
            for _ in range(REPETICOES_CONDUTOR):
                i += 1
                cond.append({
                    'indice': i,
                    'plano_m': ALTURA_CONDUTOR_M,
                    'alvo_nome': alvo['nome'],
                    'alvo_norte_m': None,
                    'alvo_leste_m': alvo['leste_m'],
                    'altitude_cmd_m': ALTITUDE_CONDUTOR_M,
                    'raio_partida_m': d,
                    'angulo_partida_rad': '',
                    'norte_partida_m': 0.0,
                    'leste_partida_m': round(
                        alvo['leste_m'] + alvo['lado'] * d, 3),
                })
    return cond


# ----------------------------------------------------------------- métricas
def consolidar(resultados):
    validos = [r for r in resultados if 'tempo_total_s' in r]
    for r in validos:
        if isinstance(r.get('desvio_altitude_m'), (int, float)):
            r['_desvio_abs'] = abs(r['desvio_altitude_m'])
    ok = [r for r in validos if r.get('sucesso') == 1]
    n = len(resultados)
    taxa = 100.0 * len(ok) / n if n else float('nan')

    def med(c, f):
        v = [r[c] for r in f if isinstance(r.get(c), (int, float))]
        return sum(v) / len(v) if v else float('nan')

    def dp(c, f):
        v = [r[c] for r in f if isinstance(r.get(c), (int, float))]
        if len(v) < 2:
            return float('nan')
        mu = sum(v) / len(v)
        return math.sqrt(sum((x - mu) ** 2 for x in v) / (len(v) - 1))

    def reqm(c, f):
        v = [r[c] ** 2 for r in f if isinstance(r.get(c), (int, float))]
        return math.sqrt(sum(v) / len(v)) if v else float('nan')

    def mx(c, f):
        v = [r[c] for r in f if isinstance(r.get(c), (int, float))]
        return max(v) if v else float('nan')

    L = ['=' * 72, f'METRICAS CONSOLIDADAS  --  modo: {MODO}', '=' * 72]
    if MODO == 'solo':
        L += [f'Semente da amostragem                 : {SEMENTE}',
              f'Altitude sorteada em                  : '
              f'[{ALTITUDE_MIN_M:.1f}, {ALTITUDE_MAX_M:.1f}] m (uniforme)',
              f'Afastamento inicial sorteado em       : '
              f'[{RAIO_MIN_M:.1f}, {RAIO_MAX_M:.1f}] m (uniforme, limitado '
              f'a {FRACAO_MAX_QUADRO:.2f} da meia-largura do quadro)']
    L += [
        f'Ensaios executados                    : {n}',
        f'Ensaios bem-sucedidos                 : {len(ok)}',
        f'Taxa de sucesso                       : {taxa:.1f} %',
        '',
        f'Tempo medio total                     : {med("tempo_total_s", ok):.2f}'
        f' +- {dp("tempo_total_s", ok):.2f} s',
        f'Tempo medio da fase horizontal        : '
        f'{med("tempo_horizontal_s", ok):.2f} s',
        f'Tempo medio da descida                : '
        f'{med("tempo_descida_s", ok):.2f} s',
        '',
        '-- custo de execucao do laco --',
        f'Periodo imposto por projeto           : '
        f'{PERIODO_COMANDO_S * 1000:.1f} ms',
        f'Periodo medio medido                  : '
        f'{med("dt_medio_s", validos) * 1000:.2f} ms',
        f'Consulta de telemetria, media         : '
        f'{med("t_telemetria_medio_ms", validos):.3f} ms',
        f'Calculo da lei de controle, media     : '
        f'{med("t_calculo_medio_ms", validos):.4f} ms',
        f'Deteccao na imagem (uma por selecao)  : '
        f'{med("t_deteccao_ms", validos):.3f} ms',
        '',
        '-- condicao de partida dos ensaios --',
        f'Rapidez ao capturar o quadro, media   : '
        f'{med("rapidez_partida_ms", validos):.4f} m/s',
        f'Rapidez ao capturar o quadro, maxima  : '
        f'{mx("rapidez_partida_ms", validos):.4f} m/s',
        f'Desvio de altitude, media dos modulos : '
        f'{med("_desvio_abs", validos):.3f} m',
        f'Desvio de altitude, maximo em modulo  : '
        f'{mx("_desvio_abs", validos):.3f} m',
        '',
        '-- erro horizontal, contra o alvo verdadeiro --',
        f'Ao fim do alinhamento, media          : '
        f'{med("erro_alinhamento_m", ok):.4f} +- '
        f'{dp("erro_alinhamento_m", ok):.4f} m',
        f'Ao fim do alinhamento, REQM           : '
        f'{reqm("erro_alinhamento_m", ok):.4f} m',
        f'No ponto de pouso, media              : '
        f'{med("erro_toque_m", ok):.4f} +- {dp("erro_toque_m", ok):.4f} m',
        f'No ponto de pouso, REQM               : '
        f'{reqm("erro_toque_m", ok):.4f} m',
        f'No ponto de pouso, maximo             : '
        f'{mx("erro_toque_m", ok):.4f} m',
        '',
        f'REQM contra a referencia calculada    : {reqm("erro_ref_m", ok):.4f} m',
        '',
        '-- componentes do erro no ponto de pouso --',
        f'Norte, media                          : '
        f'{med("erro_toque_norte_m", ok):+.4f} +- '
        f'{dp("erro_toque_norte_m", ok):.4f} m',
        f'Leste, media                          : '
        f'{med("erro_toque_leste_m", ok):+.4f} +- '
        f'{dp("erro_toque_leste_m", ok):.4f} m',
        '=' * 72,
        '',
        'O erro contra o alvo verdadeiro e medido em relacao as coordenadas',
        'do alvo no arquivo do mundo, e inclui projecao, calibracao e',
        'controle. O erro contra a referencia calculada compara a posicao',
        'medida com a referencia que o proprio sistema estabeleceu, isolando',
        'o desempenho do controlador. A diferenca entre os dois quantifica a',
        'parcela do erro que nao provem do controle.',
        '',
        'O periodo do laco e imposto por PERIODO_COMANDO_S e nao mede custo',
        'computacional. O custo esta decomposto nas tres linhas seguintes:',
        'a consulta de telemetria, o calculo da lei de controle e a deteccao',
        'na imagem, esta ultima executada uma vez por selecao do operador e',
        'nao a cada iteracao.',
        '',
        'As componentes norte e leste do erro sao reportadas com sinal para',
        'que um eventual vies sistematico, que a norma do erro esconde, fique',
        'visivel.',
        '',
        'A rapidez ao capturar o quadro verifica a hipotese sob a qual a',
        'conversao e valida: o veiculo parado. Como a densidade de pixels',
        'varia com o inverso da altitude, uma altitude ainda em mudanca',
        'produz um erro de escala proporcional a todo o deslocamento',
        'calculado. Valores acima de ' f'{RAPIDEZ_ASSENTADO_MS:.2f} m/s'
        ' indicam que o criterio de',
        'assentamento nao foi atendido dentro do tempo maximo.',
    ]
    if MODO == 'condutor':
        L += ['', 'O condutor e uniforme ao longo do seu comprimento e nao',
              'fornece referencia nessa direcao. O erro reportado e a',
              'distancia perpendicular ao seu eixo.']

    texto = '\n'.join(L)
    print('\n' + texto)
    with open(os.path.join(DIR_SAIDA, 'metricas.txt'), 'w') as f:
        f.write(texto + '\n')


# ----------------------------------------------------------------- principal
async def main():
    os.makedirs(DIR_SAIDA, exist_ok=True)

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

    tarefa_video = asyncio.ensure_future(exibir(video))

    condicoes = (sortear_condicoes() if MODO == 'solo'
                 else condicoes_condutor())
    print(f'\n-- modo {MODO}: {len(condicoes)} ensaios programados\n')

    resultados = []
    try:
        for i, cond in enumerate(condicoes, start=1):
            if parar:
                break

            # Cada ensaio parte do solo, desarmado: mesma condicao inicial.
            if not await esta_armado(drone):
                try:
                    await drone.action.arm()
                except ActionError as erro:
                    print(f'-- Armamento negado: {erro}')
                    break
            await drone.offboard.set_position_ned(
                PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            try:
                await drone.offboard.start()
            except OffboardError as erro:
                print(f'-- Falha ao iniciar offboard: {erro._result.result}')
                break

            ident = (f'{MODO}_{cond["indice"]:03d}_'
                     f'h{cond["altitude_cmd_m"]:.1f}_'
                     f'r{cond["raio_partida_m"]:.1f}')
            print(f'--- ensaio {i}/{len(condicoes)}: {ident}')
            resultados.append(await executar_ensaio(drone, video, ident, cond))

            # Encerra o ensaio pousando, restabelecendo a condicao inicial.
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            try:
                await drone.action.land()
                await asyncio.wait_for(aguardar_pouso(drone), timeout=90)
            except Exception as erro:
                print(f'-- Falha ao pousar entre ensaios: {erro}')
            await asyncio.sleep(3.0)

        if resultados:
            campos = sorted({k for r in resultados for k in r})
            with open(os.path.join(DIR_SAIDA, 'resumo.csv'), 'w',
                      newline='') as f:
                w = csv.DictWriter(f, fieldnames=campos)
                w.writeheader()
                w.writerows(resultados)
            consolidar(resultados)
    finally:
        tarefa_video.cancel()
        try:
            await drone.offboard.stop()
        except Exception:
            pass
        try:
            await drone.action.land()
            await asyncio.wait_for(aguardar_pouso(drone), timeout=90)
        except Exception:
            pass
        cv2.destroyAllWindows()
        print('\n-- Campanha encerrada')


if __name__ == '__main__':
    asyncio.run(main())