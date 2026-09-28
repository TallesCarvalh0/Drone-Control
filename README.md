# Drone-Control

Posicionamento assistido por visão computacional para VANTs: conversão de
seleção do operador em referência de deslocamento para inspeção de linhas
de transmissão.

Código e material de simulação do Trabalho de Conclusão de Curso em
Engenharia Mecatrônica (Universidade Federal de Uberlândia, 2026).

## Conteúdo

- [O que o sistema faz](#o-que-o-sistema-faz)
- [Resultados](#resultados)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Ambiente](#ambiente)
- [Configuração da simulação](#configuração-da-simulação)
- [Dependências](#dependências)
- [Execução](#execução)
- [Recepção do vídeo](#recepção-do-vídeo)
- [Parâmetros do controlador](#parâmetros-do-controlador)
- [Limitações](#limitações)

## O que o sistema faz

O operador clica sobre um ponto na imagem de uma câmera orientada para o
solo. O desvio medido em pixels entre esse ponto e o centro do quadro é
convertido em deslocamento métrico por uma relação calibrada entre a
densidade de pixels e a altura de observação. Esse deslocamento é somado à
posição corrente do veículo, estabelecendo uma referência absoluta no
referencial local NED, e um controlador proporcional com saturação conduz a
aeronave até ela.

A referência de deslocamento é obtida da imagem, e não de coordenadas
fornecidas por sistemas de navegação por satélite. O GNSS permanece em uso
na realimentação da malha, por meio da estimativa de posição do controlador
de voo.

## Resultados

Campanha de 30 ensaios em simulação, com altitudes de operação e
afastamentos iniciais sorteados de distribuições uniformes declaradas, sob
semente fixa. Vinte e nove concluíram as duas fases.

| Métrica | Valor |
|---|---|
| REQM no ponto de pouso | 0,158 m |
| REQM ao final do alinhamento | 0,098 m |
| Erro máximo no ponto de pouso | 0,357 m |
| Tempo mediano por operação | 30,3 s |

A decomposição do erro atribui 0,098 m ao método proposto e 0,148 m à
manobra de descida acrescentada ao roteiro de ensaio, que não integra o
método.

## Estrutura do repositório

    Scripts/
        Controller.py      interface do operador e laço de controle
        ensaios.py         campanha automatizada (modos 'solo' e 'condutor')
        graficos.py        gera as figuras e a tabela de métricas
        calibracao.py      procedimento de calibração da densidade de pixels
        gazebo_opencv.py   recepção do fluxo de vídeo (GStreamer/RTP)
    Models/                modelo do veículo com a câmera acrescentada
    Worlds/                cenário de simulação da linha de transmissão
    ensaios/               dados brutos da campanha
    calibracao.csv         medidas do ensaio de calibração
    calibracao_imgs/       quadros registrados durante a calibração

## Ambiente

Versões empregadas nos ensaios reportados:

| Componente | Versão |
|---|---|
| Sistema operacional | Ubuntu 22.04.5 LTS |
| Plataforma de execução | WSL2 sobre Windows |
| Kernel | 6.18.33.2-microsoft-standard-WSL2 |
| Simulador | Gazebo Classic 11.10.2 |
| Firmware | PX4-Autopilot v1.18.0-beta1-817-g7c8d6daca9 |
| Biblioteca de controle | MAVSDK-Python 3.17.4 |

### Pontos que não decorrem da instalação padrão

**Distribuição.** A documentação do PX4 indica o Ubuntu 20.04 para uso com
o Gazebo Classic, versão não mais disponível no catálogo do WSL. A
instalação foi feita sobre o Ubuntu 22.04, cujo script de configuração do
PX4 instala por padrão o Gazebo Harmonic. O Gazebo Classic foi instalado
manualmente em seguida.

**MAVSDK.** A partir da versão 4 o pacote `mavsdk` passou a designar a
ligação nativa, cuja interface difere da utilizada aqui. A versão deve ser
fixada abaixo de 4.

## Configuração da simulação

### Cenário

O sistema de compilação do PX4 não gera automaticamente um alvo de execução
para arquivos de cenário adicionados ao diretório correspondente. Por isso o
cenário deste repositório substitui o arquivo vazio distribuído com a
plataforma:

1. Vá até o diretório de cenários da sua instalação do PX4:

       PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds

2. Substitua o `empty.world` pelo arquivo de `Worlds/` deste repositório,
   mantendo o nome `empty.world`, para que seja carregado como padrão.

3. Configure a textura do alvo na pasta `source`.

### Modelo do veículo

O modelo distribuído com o PX4 não serve: o sensor de profundidade nele
declarado publica as imagens apenas no barramento interno do simulador, sem
transmissão para consumo externo. O modelo deste repositório acrescenta um
sensor de câmera em cores com transmissão por RTP, com o mesmo campo de
visão e a mesma resolução do original: 1,5010 rad, 848 x 480 pixels, 10 Hz,
H.264 sobre RTP.

Se a imagem não funcionar, verifique se o modelo em

    PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_downward_depth_camera

é o mesmo que está em `Models/`.

## Dependências

    pip install 'mavsdk<4' opencv-python numpy matplotlib PyGObject

O `matplotlib` é necessário apenas para `graficos.py`, e o `numpy` apenas
para `ensaios.py`. O `PyGObject` atende ao GStreamer, usado por
`gazebo_opencv.py`.

## Execução

### Iniciar a simulação

    make px4_sitl gazebo-classic_iris_downward_depth_camera

### Operação assistida

Abre a janela de vídeo e aguarda o clique do operador. O controlador
conecta-se em `udpin://0.0.0.0:14540`.

    python3 Scripts/Controller.py

Encerre com `q` ou `Esc` na janela de vídeo. Evite Ctrl+C: o sinal encerra o
processo auxiliar do MAVSDK antes que o pouso possa ser comandado.

### Campanha de ensaios

Ajuste `MODO` no topo do arquivo para `'solo'` (campanha estatística, com
pouso sobre o alvo) ou `'condutor'` (aproximação a um condutor, com descida
até uma folga acima do cabo).

    python3 Scripts/ensaios.py

Em ambos os modos a seleção do operador é obtida por segmentação da imagem,
e não calculada a partir das coordenadas do mundo. A distinção é essencial:
calcular o pixel a partir da geometria conhecida usaria a calibração para
gerar a seleção e novamente para convertê-la, cancelando-a.

### Figuras e tabela de métricas

Lê os dados dos ensaios e escreve PDFs vetoriais prontos para inclusão em
LaTeX. As figuras são legíveis em impressão monocromática: as séries se
distinguem por estilo de traço e marcador, nunca por cor.

    python3 Scripts/graficos.py                  # usa ./ensaios e ./figuras
    python3 Scripts/graficos.py ensaios figuras

## Recepção do vídeo

O módulo `gazebo_opencv.py` monta um pipeline GStreamer que recebe o fluxo
transmitido pelo simulador na porta 5600, faz o parsing e a decodificação
H.264 e converte os quadros para BGR, formato consumido pelo OpenCV. Cada
quadro decodificado é disponibilizado à aplicação como uma matriz de dados
de imagem.

Funções principais:

- `frame()` — retorna o quadro de vídeo atual
- `frame_available()` — informa se há quadro disponível
- `save_frame()` — salva o quadro atual

O fluxo também pode ser visualizado diretamente pelo QGroundControl, sem
configuração adicional.

## Parâmetros do controlador

| Parâmetro | Valor | Origem |
|---|---|---|
| Ganho proporcional | 0,3 | ajuste empírico em simulação |
| Saturação da velocidade horizontal | 1,5 m/s | ajuste empírico em simulação |
| Tolerância de convergência | 2 pixels, com piso de 0,05 m | ajuste empírico em simulação |
| Período do laço | 0,25 s | projeto |
| Calibração | rho(h) = 461,7 / h | ensaio de calibração |

O comando de velocidade é proporcional ao erro **em metros**, e não ao erro
em pixels: a conversão pela calibração ocorre antes da lei de controle. A
saturação limita o módulo do vetor de velocidade horizontal, com fator de
redução comum aos dois eixos, de modo que a direção do comando é preservada.

A calibração foi determinada experimentalmente com alvo de 2,00 m de lado,
em oito alturas entre 5 e 20 m, com quinze amostras por altura. O ajuste de
potência livre sobre os mesmos dados forneceu rho(h) = 453,0 h^(-0,9902),
confirmando o expoente unitário exigido pela projeção perspectiva. A
constante obtida difere em 1,5 % da prevista analiticamente a partir dos
parâmetros ópticos declarados da câmera.

## Limitações

A validação foi conduzida integralmente em simulação. O modelo da câmera não
incorpora distorção de lente e o cenário não reproduz perturbações
atmosféricas nem os campos eletromagnéticos de uma linha energizada.

A conversão pressupõe que o ponto selecionado está em um plano cuja altura
em relação à câmera é conhecida. Nos ensaios esse plano é o solo.

O pouso não integra o método: a descida empregada nos ensaios é um comando
de velocidade vertical em modo offboard, sem projeto ou sintonia, adotado
apenas para tornar mensurável o erro no ponto de pouso.
