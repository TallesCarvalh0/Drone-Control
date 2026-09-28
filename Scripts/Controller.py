"""Controle assistido de posicionamento de VANT por seleção na imagem.

O operador seleciona um ponto no quadro de vídeo transmitido pela câmera
orientada para o solo. O erro em pixels entre o ponto selecionado e o centro
do quadro é convertido em metros pela calibração de densidade de pixels e
somado à posição corrente do veículo, estabelecendo uma posição de referência
no referencial local NED. Um controlador proporcional conduz o veículo até
essa referência.

Encerramento: tecla 'q' ou Esc na janela de vídeo. Evite Ctrl+C, que mata o
processo auxiliar do MAVSDK antes que o pouso possa ser comandado.
"""
import asyncio
import signal

import cv2
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw
from gazebo_opencv import Video

# --------------------------------------------------------------- parametros
KP = 0.3                      # ganho proporcional
V_MAX_MS = 1.5                # saturacao do modulo da velocidade horizontal
TOLERANCIA_M = 0.05           # piso absoluto da tolerancia, em metros
TOLERANCIA_PX = 2.0           # tolerancia em pixels do plano da imagem
ALTITUDE_OPERACAO_M = 10.0    # altura de operacao
ESPERA_SUBIDA_S = 20.0        # espera para estabilizar na altitude
PERIODO_COMANDO_S = 0.25      # periodo do laco de controle
ESPERA_POUSO_S = 60.0         # limite para concluir o pouso ao encerrar
ITERACOES_POR_LINHA = 4       # imprime o estado a cada N iteracoes
JANELA = 'Camera do VANT'

# Calibracao de 26/09/2026: alvo de 2,00 m, 8 alturas de 5 a 20 m, 15 amostras
# por altura. Modelo geometrico rho = k/h com k = 461,7 e R2 = 0,9998. O ajuste
# de potencia livre sobre os mesmos dados deu rho(h) = 453,0 * h^(-0,9902),
# confirmando o expoente -1 exigido pela projecao perspectiva.
RHO_K = 461.7

# Controle do eixo norte. False reproduz o comportamento de um eixo so.
CONTROLAR_NORTE = True

# Mapeamento dos eixos da imagem para o referencial NED.
# SINAL_LESTE = +1 foi deduzido do codigo anterior, que funcionava.
# SINAL_NORTE ainda NAO foi verificado experimentalmente: clique num ponto
# acima do centro do quadro e observe se o marcador converge para a cruz.
# Se ele se afastar, troque para +1.0.
SINAL_LESTE = +1.0
SINAL_NORTE = -1.0

# Braco de alavanca da camera. No modelo iris_downward_depth_camera a camera
# esta 0,108 m a frente da origem do corpo (base_link), que e o ponto cuja
# posicao a telemetria reporta. Centrar o alvo na imagem coloca a CAMERA
# sobre ele; para posicionar o VEICULO, soma-se este braco ao deslocamento
# lido na imagem. Com o yaw mantido em zero, o eixo x do corpo e o norte.
CAMERA_FRENTE_M = 0.108
CAMERA_LATERAL_M = 0.0

# Desenho na imagem (BGR)
COR_CENTRO = (0, 255, 255)
COR_ALVO = (0, 0, 255)
RAIO_ALVO_PX = 6
BRACO_CRUZ_PX = 14

# ------------------------------------------------------ encerramento ordenado
parar = False


def _sigint(sig, frame):
    global parar
    if parar:
        return
    parar = True
    print('\n-- Interrupcao por sinal; o pouso pode falhar. Prefira a tecla q.')


signal.signal(signal.SIGINT, _sigint)


async def dormir(segundos):
    """Espera que pode ser interrompida pelo pedido de encerramento."""
    laco = asyncio.get_event_loop()
    fim = laco.time() + segundos
    while not parar and laco.time() < fim:
        await asyncio.sleep(0.1)


class Selecao:
    """Guarda o ponto selecionado pelo operador, em pixels do quadro."""

    def __init__(self, video):
        self.video = video
        self.pendente = None    # consumido pelo laco de controle
        self.marcador = None    # mantido para desenho na imagem

    def ao_clicar(self, evento, x, y, flags, param):
        if evento != cv2.EVENT_LBUTTONDOWN:
            return
        if not self.video.frame_available():
            print('[selecao] sem quadro disponivel; clique ignorado')
            return
        altura, largura = self.video.frame().shape[:2]
        erro_x = x - largura / 2.0
        erro_y = y - altura / 2.0
        self.pendente = (erro_x, erro_y)
        self.marcador = (int(x), int(y))
        print(f'[selecao] clique=({x},{y}) quadro={largura}x{altura} '
              f'erro_px=({erro_x:.1f},{erro_y:.1f})')

    def consumir(self):
        p = self.pendente
        self.pendente = None
        return p

    def atualizar_por_erro(self, falta_norte, falta_leste, rho):
        """Reposiciona o marcador conforme o erro restante, em pixels.

        Usa exclusivamente telemetria e a calibracao rho. Nao ha realimentacao
        pela imagem: o desenho e consequencia do modelo, nao fonte de dados.
        """
        if not self.video.frame_available():
            return
        altura, largura = self.video.frame().shape[:2]
        self.marcador = (
            int(round(largura / 2.0 + SINAL_LESTE * falta_leste * rho)),
            int(round(altura / 2.0 + SINAL_NORTE * falta_norte * rho)),
        )


def anotar(quadro, marcador):
    """Desenha o centro da imagem, o ponto selecionado e o vetor de erro."""
    altura, largura = quadro.shape[:2]
    cx, cy = largura // 2, altura // 2

    cv2.line(quadro, (cx - BRACO_CRUZ_PX, cy),
             (cx + BRACO_CRUZ_PX, cy), COR_CENTRO, 1)
    cv2.line(quadro, (cx, cy - BRACO_CRUZ_PX),
             (cx, cy + BRACO_CRUZ_PX), COR_CENTRO, 1)

    if marcador is None:
        return

    mx, my = marcador
    cv2.line(quadro, (cx, cy), (mx, my), COR_ALVO, 1)
    cv2.circle(quadro, (mx, my), RAIO_ALVO_PX, COR_ALVO, -1)
    cv2.circle(quadro, (mx, my), RAIO_ALVO_PX * 2, COR_ALVO, 1)


async def exibir(video, selecao):
    """Mantem a janela atualizada, processa eventos e a tecla de saida."""
    global parar
    while True:
        if video.frame_available():
            quadro = video.frame().copy()
            anotar(quadro, selecao.marcador)
            cv2.imshow(JANELA, quadro)
        tecla = cv2.waitKey(1) & 0xFF
        if tecla in (27, ord('q')) and not parar:
            parar = True
            print('\n-- Encerramento solicitado; pousando...')
        await asyncio.sleep(0.03)


async def altitude_relativa(drone):
    """Altitude relativa ao ponto de decolagem, sem arredondamento."""
    async for p in drone.telemetry.position():
        return p.relative_altitude_m


async def posicao_ned(drone):
    """Posicao NED corrente (norte, leste, baixo). Sem espera: leitura atual."""
    async for pv in drone.telemetry.position_velocity_ned():
        return (round(pv.position.north_m, 4),
                round(pv.position.east_m, 4),
                round(pv.position.down_m, 4))


async def esta_armado(drone):
    async for a in drone.telemetry.armed():
        return a


async def aguardar_pouso(drone):
    async for no_ar in drone.telemetry.in_air():
        if not no_ar:
            return


def saturar(v_n, v_e):
    """Limita o MODULO do vetor de velocidade horizontal a V_MAX_MS.

    O fator de reducao e comum aos dois eixos, de modo que a direcao do
    comando e preservada. Saturar cada componente isoladamente desviaria a
    trajetoria da reta que une a posicao corrente a referencia.
    """
    m = (v_n ** 2 + v_e ** 2) ** 0.5
    if m > V_MAX_MS:
        k = V_MAX_MS / m
        return v_n * k, v_e * k
    return v_n, v_e


def densidade_pixels(altitude_m):
    """Densidade de pixels por metro no plano do solo, em px/m."""
    return RHO_K / altitude_m


async def main():
    video = Video()
    cv2.namedWindow(JANELA, cv2.WINDOW_AUTOSIZE)
    selecao = Selecao(video)
    cv2.setMouseCallback(JANELA, selecao.ao_clicar)

    drone = System()
    await drone.connect(system_address='udpin://0.0.0.0:14540')

    print('Aguardando conexao...')
    async for estado in drone.core.connection_state():
        if parar:
            return
        if estado.is_connected:
            print('-- Conectado')
            break

    print('Aguardando estimativa de posicao...')
    async for saude in drone.telemetry.health():
        if parar:
            return
        if saude.is_global_position_ok and saude.is_home_position_ok:
            print('-- Estimativa de posicao OK')
            break

    tarefa_video = asyncio.ensure_future(exibir(video, selecao))

    if await esta_armado(drone):
        print('-- Veiculo ja armado; seguindo sem rearmar')
    else:
        try:
            print('-- Armando')
            await drone.action.arm()
        except ActionError as erro:
            print(f'-- Armamento negado: {erro}')
            print('-- Diagnostico no pxh:')
            print('     listener vehicle_status   (campo nav_state)')
            print('     listener health_report    (arming_check_error_flags)')
            tarefa_video.cancel()
            cv2.destroyAllWindows()
            return

    print('-- Definindo setpoint inicial')
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print('-- Iniciando offboard')
    try:
        await drone.offboard.start()
    except OffboardError as erro:
        print(f'-- Falha ao iniciar offboard: {erro._result.result}')
        await drone.action.disarm()
        tarefa_video.cancel()
        cv2.destroyAllWindows()
        return

    try:
        print(f'-- Subindo para {ALTITUDE_OPERACAO_M:.1f} m')
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -ALTITUDE_OPERACAO_M, 0.0))
        await dormir(ESPERA_SUBIDA_S)

        while not parar:
            print('-- Aguardando selecao do operador (clique na imagem)')
            alvo = None
            while alvo is None and not parar:
                alvo = selecao.consumir()
                await asyncio.sleep(0.05)
            if parar:
                break

            erro_x_px, erro_y_px = alvo

            # A altitude vem da coordenada down do referencial NED, a mesma
            # que realimenta o laco de controle, e nao do campo
            # relative_altitude_m. Em ensaios os dois divergiram em cerca de
            # 0,75 m em parte das execucoes, e como a densidade de pixels
            # varia com o inverso da altura, esse desvio se propaga
            # proporcionalmente a todo o deslocamento convertido.
            norte0, leste0, baixo0 = await posicao_ned(drone)
            altitude = -baixo0
            if altitude <= 0:
                print(f'-- Altitude invalida para conversao ({altitude:.2f} m);'
                      ' selecao descartada')
                continue

            rho = densidade_pixels(altitude)
            img_leste_m = SINAL_LESTE * erro_x_px / rho
            img_norte_m = (SINAL_NORTE * erro_y_px / rho
                           if CONTROLAR_NORTE else 0.0)

            # Braco de alavanca da camera: a imagem informa onde o alvo esta
            # em relacao a CAMERA, e a telemetria reporta a origem do corpo.
            # O deslocamento a comandar ao veiculo e a soma dos dois.
            erro_leste_m = img_leste_m + CAMERA_LATERAL_M
            erro_norte_m = (img_norte_m + CAMERA_FRENTE_M
                            if CONTROLAR_NORTE else 0.0)

            erro_radial_m = (erro_norte_m ** 2 + erro_leste_m ** 2) ** 0.5
            tolerancia = max(TOLERANCIA_M, TOLERANCIA_PX / rho)

            print(f'-- altitude={altitude:.2f} m   rho={rho:.2f} px/m')
            print(f'-- imagem:     norte={img_norte_m:.4f} m  '
                  f'leste={img_leste_m:.4f} m')
            print(f'-- referencia: norte={erro_norte_m:.4f} m  '
                  f'leste={erro_leste_m:.4f} m  radial={erro_radial_m:.4f} m')
            print(f'-- tolerancia: {tolerancia:.4f} m '
                  f'({TOLERANCIA_PX:.0f} px)')

            # O marcador na tela acompanha o ponto clicado, que e o que o
            # operador ve; o braco de alavanca nao entra aqui.
            selecao.atualizar_por_erro(img_norte_m, img_leste_m, rho)

            # Referencia absoluta: posicao no instante da selecao mais o
            # deslocamento desejado. O erro passa a ser medido a cada
            # iteracao, e nao acumulado a partir de diferencas. A posicao ja
            # foi lida acima, no mesmo instante em que a altitude usada na
            # conversao, de modo que as duas sao consistentes entre si.
            alvo_norte = norte0 + erro_norte_m
            alvo_leste = leste0 + erro_leste_m

            falta_radial = erro_radial_m
            interrompido = False
            n = 0

            while True:
                norte, leste, _ = await posicao_ned(drone)
                falta_norte = alvo_norte - norte
                falta_leste = alvo_leste - leste
                falta_radial = (falta_norte ** 2 + falta_leste ** 2) ** 0.5

                if parar or selecao.pendente is not None:
                    interrompido = True
                    break

                selecao.atualizar_por_erro(falta_norte, falta_leste, rho)

                if falta_radial <= tolerancia:
                    break

                v_norte, v_leste = saturar(KP * falta_norte,
                                           KP * falta_leste)

                if n % ITERACOES_POR_LINHA == 0:
                    print(f'   erro=({falta_norte:.4f}, {falta_leste:.4f}) m  '
                          f'radial={falta_radial:.4f} m  '
                          f'v=({v_norte:.4f}, {v_leste:.4f}) m/s')
                n += 1

                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(v_norte, v_leste, 0.0, 0.0))
                await asyncio.sleep(PERIODO_COMANDO_S)

            if interrompido:
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
                if parar:
                    break
                print('-- Correcao abortada: nova selecao do operador')
            else:
                # Setpoint de posicao, e nao de velocidade nula: velocidade
                # nula apenas impede deslocamento, sem corrigir deriva.
                await drone.offboard.set_position_ned(
                    PositionNedYaw(alvo_norte, alvo_leste, baixo0, 0.0))
                print(f'-- Alvo alcancado: erro radial residual '
                      f'{falta_radial:.4f} m   (mantendo posicao)')
    finally:
        tarefa_video.cancel()

        try:
            await drone.offboard.stop()
        except OffboardError as erro:
            print(f'-- Falha ao parar offboard: {erro._result.result}')
        except Exception as erro:
            print(f'-- Falha ao parar offboard: {erro}')

        try:
            print('-- Pousando')
            await drone.action.land()
            await asyncio.wait_for(aguardar_pouso(drone),
                                   timeout=ESPERA_POUSO_S)
            print('-- Pousado e desarmado')
        except asyncio.TimeoutError:
            print('-- Tempo esgotado aguardando o pouso')
        except Exception as erro:
            print(f'-- Falha ao pousar: {erro}')

        cv2.destroyAllWindows()


if __name__ == '__main__':
    asyncio.run(main())