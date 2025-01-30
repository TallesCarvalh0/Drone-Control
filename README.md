# Simulação com Gazebo Classic e Integração com PX4 e Câmera

## Tabela de Conteúdo

- [Instruções de Configuração](#instrucoes-de-configuracao)
- [Iniciando a Simulação](#iniciando-a-simulacao)
- [Acessando o Feed da Câmera](#acessando-o-feed-da-camera)
- [Integração com OpenCV](#integracao-com-opencv)
- [Controlador do Drone com MAVSDK](#controlador-do-drone-com-mavsdk)

## Instruções de Configuração

1. **Configurar o Mundo no Gazebo:**
   - Vá até o seguinte diretório na sua instalação do PX4:
     ```bash
     PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds
     ```
   - Substitua o arquivo `empty.world` pelo arquivo de mundo desejado que está no diretório `worlds`.
   - Renomeie o arquivo de mundo para `empty.world` para que ele seja reconhecido como padrão na simulação.
   - Certifique-se também de configurar a textura do alvo na pasta `source`.

## Iniciando a Simulação

Para iniciar a simulação com uma câmera de profundidade voltada para baixo, rode o seguinte comando no terminal:

```bash
make px4_sitl gazebo-classic_iris_downward_depth_camera
```

## Acessando o Feed da Câmera

### QGroundControl

O feed da câmera pode ser acessado direto pelo QGroundControl, sem precisar configurar nada.

### Integração com OpenCV

Você também pode acessar o feed usando a biblioteca OpenCV com um script disponível no projeto.

## Integração com OpenCV

### Detalhes do Script

O script que acessa a câmera está no diretório `SCRIPTS` e se chama `gazebo_opencv.py`.

### Como Funciona

O script usa GStreamer para capturar e processar os frames de vídeo da simulação no Gazebo. Abaixo, um resumo do que ele faz:

1. **Inicialização:**
   - Configura o pipeline do GStreamer para receber o vídeo na porta `5600`.
   - Define codecs para decodificar o vídeo bruto no formato BGR compatível com OpenCV.

2. **Pipeline do GStreamer:**
   - Captura vídeo por UDP.
   - Faz parsing e decodificação H.264.
   - Converte para BGR.
   - Configura um sink para processar os frames.

3. **Captura de Frames:**
   - Extrai os frames com a função `callback` conectada ao evento `new-sample`.
   - Converte os dados do GStreamer para um array numpy compatível com OpenCV.

4. **Salvamento de Frames:**
   - A função `save_frame()` permite salvar o frame atual com um nome único.

5. **Interação com o Usuário:**
   - Mostra o feed de vídeo em uma janela.
   - Pressione `Esc` para sair ou `s` para salvar o frame atual.

### Executando o Script

Para rodar o script, use:

```bash
python gazebo_opencv.py
```

### Funções Principais
- **`gst_to_opencv(sample)`**: Converte o buffer do GStreamer para um array numpy compatível com OpenCV.
- **`frame()`**: Retorna o frame de vídeo atual.
- **`frame_available()`**: Verifica se há um frame disponível.
- **`save_frame()`**: Salva o frame atual no caminho especificado.

### Dependências

Instale as bibliotecas necessárias com:

```bash
pip install opencv-python PyGObject
```

### Notas de Uso
- Pressione `Esc` para sair.
- Pressione `s` para salvar o frame atual.

Essa configuração permite integrar os feeds do Gazebo com OpenCV para processamento e análise avançados de visão computacional.

- No caso de não funcionamento da imagem, verificar se o modelo iris_downward_depth_camera localizado no caminho abaixo é o mesmo do que está no diretório `models`.
```bash
PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_downward_depth_camera
```


## Controlador do Drone com MAVSDK

### Visão Geral do Script

O script `controller.py` controla um drone com a biblioteca MAVSDK, permitindo navegação autônoma com base em cliques no vídeo transmitido pela câmera.

### Estrutura do Script

1. **Classe `VideoStream`:**
   - Mostra o feed de vídeo em uma interface gráfica com `customtkinter`.
   - Permite capturar coordenadas de pixels clicadas na tela e salvar a posição relativa ao centro do frame.

2. **Função `pixel2meters()`:**
   - Converte a posição em pixels para metros com base na altura atual do drone.

3. **Função `get_position()`:**
   - Obtém a posição atual do drone em coordenadas NED (North, East, Down).

4. **Função `main()`:**
   - Conecta com o drone e inicia o controle offboard.
   - Captura a posição clicada pelo usuário e converte para distância em metros.
   - Implementa um controlador proporcional para mover o drone até o ponto desejado.

### Explicação do Controlador Proporcional

O controlador ajusta a posição do drone com base no erro entre a posição atual e a desejada. A fórmula básica é:

```python
v_x = Kp * erro_pixel_x
```

Onde:
- `Kp` é o ganho proporcional.
- `erro_pixel_x` é a diferença entre a posição atual e a posição alvo.

### Executando o Script

Para rodar o controlador, use:

```bash
python controller.py
```

### Dependências

Instale os pacotes necessários com:

```bash
pip install asyncio mavsdk opencv-python Pillow customtkinter
```

### Observações

- O controlador ajusta continuamente a velocidade do drone até que a diferença entre a posição atual e a desejada seja menor que 0.01 metros.
- As funções de captura de vídeo e controle do drone são executadas em threads paralelas, garantindo a responsividade do sistema.

Essa solução complementa a simulação, permitindo controlar o drone de forma precisa em ambientes simulados.

