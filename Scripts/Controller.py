
#e_pixel_x = pixel_x_camera - pixel_x_operador
#v_x = Kp * erro_pixel_x


import asyncio

import threading
import cv2
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw,PositionNedYaw)
from gazebo_opencv import Video
import customtkinter as ctk
from PIL import Image,ImageTk
import os

class VideoStream(ctk.CTk):
    
    def __init__(self):
        super().__init__()
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        self.geometry(f"{screen_width}x{screen_height}")
        self.maxsize(width=screen_width, height=screen_height)
        self.minsize(width=1080, height=720)
        self.title('LLC - LineLynx Commander')
        self._set_appearance_mode('light')
        
        self.start_video_loop()
        self.mainloop() 

    def start_video_loop(self):
        # Aguarda
        self.after(1000)

        self.video = Video()
        self.video.start_gst()
        self.video_canvas = ctk.CTkCanvas(self, bg='black')
        self.video_canvas.place(relx=0.05, rely=0.1, anchor='nw', relwidth=0.7, relheight=0.8)
        self.video_canvas = ctk.CTkCanvas(self, bg='black')
        self.video_canvas.place(relx=0.05, rely=0.1, anchor='nw', relwidth=0.7, relheight=0.8)
        self.video_canvas.bind("<Button-1>", self.on_canvas_click)  # Adiciona um evento de clique
        self.update_video()

    def on_canvas_click(self, event):
        """Manipula o clique no vídeo"""

        click_y=event.y   
        self.save_image_on_click(event)
        
            
    def save_image_on_click(self,event):
        """Salva o frame completo ao clicar no vídeo"""
        if self.video.frame_available():
            frame = self.video.frame()
            
            # Obtem as dimensões do frame
            frame_height, frame_width, _ = frame.shape
            click_x = (event.x / self.video_canvas.winfo_width())*640
            # Coordenadas do centro do frame
            center_x = frame_width / 2
            center_y = frame_height / 2
            x_initial = center_x - click_x
            try:
                with open('/home/talles/UFU/TCC/MAVSDK-Python/scripts/output.txt','w') as file:
                    file.write(str(x_initial))
            except Exception as e: 
                print(f"Erro ao escrever: {e}")
            # Redimensiona o frame
            frame = cv2.resize(frame, (self.video_canvas.winfo_width(), self.video_canvas.winfo_height()))
        
    def update_video(self):
        """Atualiza o frame do vídeo no Canvas"""
        if self.video.frame_available():
            frame = self.video.frame()

            frame = cv2.resize(frame, (self.video_canvas.winfo_width(), self.video_canvas.winfo_height()))
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(img)
            img = ImageTk.PhotoImage(img)
            self.video_canvas.create_image(0, 0, anchor='nw', image=img)
            self.video_canvas.image = img
            

        self.video_canvas.after(10, self.update_video)  # Atualiza a cada 10 milissegundos
            
def run_video_stream():
    VideoStream()

async def pixel2meters(x_initial,drone):
    async for position in drone.telemetry.position():
        altitude = round(position.relative_altitude_m)
        print(altitude)
        await asyncio.sleep(1)
        pixeldensity = 231.71*(altitude**(-0.813)) # Quantos pixels equivale a 1 metro
        
        x_initial_m = float(x_initial)/pixeldensity                     
        break
    return x_initial_m

async def get_position(east_m,drone):
    async for positionnow in drone.telemetry.position_velocity_ned():
        east_m = round(positionnow.position.east_m,4)
        await asyncio.sleep(1)
        break
    return east_m
        
async def main():

    drone = System()
    await drone.connect(system_address="udp://:14540")

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
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    
    #Captura do ponto 
    video_stream_thread = threading.Thread(target=run_video_stream)
    video_stream_thread.start()

    #Leva para uma posição inicial
    print("-- Go up 2 m")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(7)

    tamanho = 0
    #Distancia do ponto clicado para o centro
    while tamanho == 0:
        with open('/home/talles/UFU/TCC/MAVSDK-Python/scripts/output.txt','r') as file:
            x_initial = file.read()
            tamanho = os.fstat(file.fileno()).st_size
            
            if tamanho != 0:
                print("sai aqui")
                break
            
    #Captura a altura do drone e converte a distancia em pixels para metros
    print(f"o valor do inicio é{x_initial}")
                                
    x_initial_m= await pixel2meters(x_initial,drone)
    
    print(f"o valor em metros é {x_initial_m}")
    
    #Controlador proporcional
    x_meters = 0
    kp = 0.1
    east_m = 0
    
    while abs(x_meters - x_initial_m) > 0.01:
        
        erro = x_initial_m - x_meters
        print(f"O erro em metros é : {erro}")
        velocidade = kp * erro
        print(f"A velocidade é: {velocidade}") 
        east_initial = await get_position(east_m,drone)
        print (f'o valor em east initial: {east_initial}')
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -1* (velocidade), 0.0, 0.0))
        await asyncio.sleep(0.25)

        east_now = await get_position(east_m,drone)
        print(f'o valor em east now: {east_now}')
        east_moved = -east_now +east_initial
        print (f'o valor em east moved: {east_moved}')
        x_meters =  x_meters + east_moved
        print(f'o valor de x_meters é {x_meters}')
        
    with open('/home/talles/UFU/TCC/MAVSDK-Python/scripts/output.txt','w') as file:
        print("limpado")
    #desligar offboard mode
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

if __name__ == "__main__":
    
    asyncio.run(main())