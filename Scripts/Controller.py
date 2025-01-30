import asyncio
import threading
import cv2
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw, PositionNedYaw)
from gazebo_opencv import Video
import customtkinter as ctk
from PIL import Image, ImageTk
import os

class VideoStream(ctk.CTk):
    """
    Classe para gerenciar o stream de vídeo e a interface gráfica.
    """
    def __init__(self):
        super().__init__()
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        self.geometry(f"{screen_width}x{screen_height}")
        self.maxsize(width=screen_width, height=screen_height)
        self.minsize(width=1080, height=720)
        self.title('LLC - LineLynx Commander')
        self._set_appearance_mode('light')
        
        self.start_video_loop()  # Inicia o loop de vídeo
        self.mainloop()

    def start_video_loop(self):
        """
        Inicia a captura de vídeo e configura o canvas para exibir o vídeo.
        """
        self.after(1000)

        self.video = Video()  # Inicializa o objeto de captura de vídeo
        self.video.start_gst()  # Inicia o pipeline de vídeo
        self.video_canvas = ctk.CTkCanvas(self, bg='black')
        self.video_canvas.place(relx=0.05, rely=0.1, anchor='nw', relwidth=0.7, relheight=0.8)
        self.video_canvas.bind("<Button-1>", self.on_canvas_click)  # Adiciona um evento de clique
        self.update_video()

    def on_canvas_click(self, event):
        """
        Manipula o clique no vídeo.
        """
        click_y = event.y  # Obtém a posição Y do clique no canvas
        self.save_image_on_click(event)  # Chama função para salvar a imagem ao clicar

    def save_image_on_click(self, event):
        """
        Salva o frame completo do vídeo quando o usuário clica na tela.
        """
        if self.video.frame_available():
            frame = self.video.frame()  # Captura o frame atual
            
            # Obtém as dimensões do frame
            frame_height, frame_width, _ = frame.shape
            click_x = (event.x / self.video_canvas.winfo_width()) * 640  # Calcula a posição X do clique
            center_x = frame_width / 2  # Posição X do centro do frame
            center_y = frame_height / 2  # Posição Y do centro do frame
            x_initial = center_x - click_x  # Calcula a diferença em X
            
            try:
                # Grava a diferença calculada em um arquivo de saída
                with open('MAVSDK-Python/scripts/output.txt', 'w') as file:
                    file.write(str(x_initial))
            except Exception as e:
                print(f"Erro ao escrever: {e}")
            
            # Redimensiona o frame para caber no canvas
            frame = cv2.resize(frame, (self.video_canvas.winfo_width(), self.video_canvas.winfo_height()))
        
    def update_video(self):
        """
        Atualiza o frame do vídeo no canvas.
        """
        if self.video.frame_available():
            frame = self.video.frame()

            # Redimensiona o frame
            frame = cv2.resize(frame, (self.video_canvas.winfo_width(), self.video_canvas.winfo_height()))
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Converte o frame para RGB
            img = Image.fromarray(img)  # Converte o array NumPy em uma imagem PIL
            img = ImageTk.PhotoImage(img)  # Converte a imagem PIL para um objeto utilizável no tkinter
            self.video_canvas.create_image(0, 0, anchor='nw', image=img)  # Exibe a imagem no canvas
            self.video_canvas.image = img  # Mantém uma referência para a imagem
            
        self.video_canvas.after(10, self.update_video)  # Atualiza a cada 10 milissegundos
            
def run_video_stream():
    """
    Função para rodar o stream de vídeo em uma thread separada.
    """
    VideoStream()

async def pixel2meters(x_initial, drone):
    """
    Converte a distância inicial em pixels para metros com base na altitude do drone.
    """
    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)  # Obtém a altitude do drone
        print(altitude)
        await asyncio.sleep(1)
        
        # Calcula a densidade de pixels por metro com base na altitude
        pixeldensity = 231.71 * (altitude ** (-0.813))  # Fórmula de densidade de pixels
        
        # Converte a distância inicial de pixels para metros
        x_initial_m = float(x_initial) / pixeldensity
        break
    return x_initial_m

async def get_position(east_m, drone):
    """
    Obtém a posição atual do drone na direção leste.
    """
    async for positionnow in drone.telemetry.position_velocity_ned():
        east_m = round(positionnow.position.east_m, 4)  # Obtém a posição leste do drone
        await asyncio.sleep(1)
        break
    return east_m
        
async def main():
    """
    Função principal que inicializa o drone e o sistema de controle.
    """
    drone = System()
    await drone.connect(system_address="udp://:14540")  # Conecta ao drone

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()  # Arma o drone

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))  # Define a posição inicial

    print("-- Starting offboard")
    
    try:
        await drone.offboard.start()  # Inicia o modo offboard
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()  # Desarma o drone em caso de falha
        return
    
    # Captura do ponto de clique
    video_stream_thread = threading.Thread(target=run_video_stream)
    video_stream_thread.start()

    # Leva o drone a uma posição inicial
    print("-- Go up 2 m")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(7)

    tamanho = 0
    # Espera a captura do ponto clicado
    while tamanho == 0:
        with open('MAVSDK-Python/scripts/output.txt', 'r') as file:
            x_initial = file.read()
            tamanho = os.fstat(file.fileno()).st_size
            
            if tamanho != 0:
                print("sai aqui")
                break
    
    # Converte a distância em pixels para metros
    print(f"o valor do inicio é {x_initial}")
    x_initial_m = await pixel2meters(x_initial, drone)
    print(f"o valor em metros é {x_initial_m}")
    
    # Controlador proporcional para movimentação do drone
    x_meters = 0
    kp = 0.1  # Coeficiente proporcional
    east_m = 0
    
    # Controla o movimento do drone até a posição desejada
    while abs(x_meters - x_initial_m) > 0.01:
        erro = x_initial_m - x_meters  # Calcula o erro
        print(f"O erro em metros é: {erro}")
        velocidade = kp * erro  # Calcula a velocidade proporcional
        print(f"A velocidade é: {velocidade}")
        
        # Obtém a posição inicial do drone
        east_initial = await get_position(east_m, drone)
        print(f'o valor em east initial: {east_initial}')
        
        # Envia o comando de velocidade para o drone
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -1 * (velocidade), 0.0, 0.0))
        await asyncio.sleep(0.25)

        # Obtém a posição do drone após o movimento
        east_now = await get_position(east_m, drone)
        print(f'o valor em east now: {east_now}')
        
        # Calcula a distância percorrida
        east_moved = -east_now + east_initial
        print(f'o valor em east moved: {east_moved}')
        
        # Atualiza a distância total em metros
        x_meters = x_meters + east_moved
        print(f'o valor de x_meters é {x_meters}')
        
    # Limpa o arquivo de saída
    with open('MAVSDK-Python/scripts/output.txt', 'w') as file:
        print("limpado")
    
    # Desliga o modo offboard
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

if __name__ == "__main__":
    asyncio.run(main())  # Executa a função principal
