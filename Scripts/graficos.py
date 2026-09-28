"""Gera as figuras do Capítulo 4 a partir dos dados dos ensaios.

Lê os arquivos produzidos por ensaios.py e escreve figuras em PDF vetorial,
prontas para inclusão no documento LaTeX, e uma tabela de métricas em LaTeX.

As figuras são legíveis em impressão monocromática: as séries se distinguem
por estilo de traço e marcador, nunca por cor.

Uso:
    python3 graficos.py                  # usa ./ensaios e escreve ./figuras
    python3 graficos.py ensaios figuras
"""
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator

# --------------------------------------------------------------- aparência
# Largura de texto típica de um documento A4 com margens de 3 e 2 cm: 16 cm.
LARGURA_CM = 15.0
CM = 1 / 2.54

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 9,
    'axes.titlesize': 9,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'axes.linewidth': 0.6,
    'axes.edgecolor': '0.35',
    'axes.grid': True,
    'grid.color': '0.85',
    'grid.linewidth': 0.5,
    'grid.linestyle': '-',
    'xtick.direction': 'out',
    'ytick.direction': 'out',
    'xtick.major.width': 0.6,
    'ytick.major.width': 0.6,
    'lines.linewidth': 1.1,
    'legend.frameon': True,
    'legend.framealpha': 1.0,
    'legend.edgecolor': '0.7',
    'legend.fancybox': False,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.02,
    'pdf.fonttype': 42,
})

# Séries distinguidas por traço, não por cor.
TRACOS = ['-', '--', ':', '-.']
TONS = ['0.10', '0.40', '0.60', '0.25']


def estilo(i):
    return {'linestyle': TRACOS[i % len(TRACOS)],
            'color': TONS[i % len(TONS)]}


def limpar(ax):
    """Eixos recessivos: sem molduras superior e direita."""
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.tick_params(which='minor', length=2, color='0.6')
    ax.tick_params(which='major', length=3.5, color='0.5')
    ax.set_axisbelow(True)


def vb(x, casas=3):
    """Número com vírgula decimal, como exige a norma."""
    return f'{x:.{casas}f}'.replace('.', ',')


def virgula(ax, eixos='xy', casas=None):
    """Troca o ponto decimal por vírgula nos rótulos dos eixos."""
    from matplotlib.ticker import FuncFormatter

    def formatar(v, _pos):
        if casas is None:
            s = f'{v:g}'
        else:
            s = f'{v:.{casas}f}'
        return s.replace('.', ',')

    if 'x' in eixos:
        ax.xaxis.set_major_formatter(FuncFormatter(formatar))
    if 'y' in eixos:
        ax.yaxis.set_major_formatter(FuncFormatter(formatar))


# --------------------------------------------------------------- leitura
def ler_csv(caminho):
    with open(caminho, newline='') as f:
        return list(csv.DictReader(f))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def carregar_serie(caminho):
    """Série temporal de um ensaio, como dicionário de listas."""
    linhas = ler_csv(caminho)
    d = defaultdict(list)
    for l in linhas:
        for k, v in l.items():
            d[k].append(v if k == 'fase' else num(v))
    return d


# --------------------------------------------------------------- figuras
def fig_estados(serie, destino, titulo=None):
    """Norte, leste e altitude ao longo do tempo, em painéis empilhados.

    Painéis separados, e não um eixo com duas escalas: grandezas de faixas
    diferentes nunca compartilham eixo.
    """
    t = serie['t_s']
    fig, eixos = plt.subplots(3, 1, sharex=True,
                              figsize=(LARGURA_CM * CM, 11.0 * CM))

    dados = [('north_m', 'Norte (m)'),
             ('east_m', 'Leste (m)'),
             ('altitude_m', 'Altitude (m)')]

    for ax, (campo, rotulo) in zip(eixos, dados):
        ax.plot(t, serie[campo], color='0.15', linewidth=1.2)
        ax.set_ylabel(rotulo)
        limpar(ax)
        virgula(ax)

    # Marca a transição entre as fases.
    troca = None
    for i, f in enumerate(serie['fase']):
        if f == 'descida':
            troca = t[i]
            break
    if troca is not None:
        for ax in eixos:
            ax.axvline(troca, color='0.55', linewidth=0.8, linestyle='--')
        eixos[0].annotate('início da descida', xy=(troca, 1.0),
                          xycoords=('data', 'axes fraction'),
                          xytext=(4, -10), textcoords='offset points',
                          fontsize=7.5, color='0.35', ha='left', va='top')

    eixos[-1].set_xlabel('Tempo (s)')
    if titulo:
        eixos[0].set_title(titulo, loc='left', pad=6)
    fig.align_ylabels(eixos)
    fig.savefig(destino)
    plt.close(fig)


def fig_trajetoria3d(serie, destino, alvo=None, titulo=None):
    """Trajetória tridimensional do veículo."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(LARGURA_CM * CM, 11.5 * CM))
    ax = fig.add_subplot(111, projection='3d')

    n, e, h = serie['north_m'], serie['east_m'], serie['altitude_m']
    ax.plot(e, n, h, color='0.15', linewidth=1.2, label='trajetória')

    # Projeção no plano do solo: dá a leitura horizontal sem um segundo gráfico.
    ax.plot(e, n, [0.0] * len(h), color='0.70', linewidth=0.8,
            linestyle=(0, (4, 2)), label='projeção no solo')

    # O alvo vai primeiro e com marcador maior; o ponto final, menor e por
    # cima. Assim os dois continuam visíveis quando coincidem em projeção.
    if alvo is not None:
        ax.scatter([alvo[1]], [alvo[0]], [0.0], marker='x', s=110,
                   color='0.10', linewidth=1.6, depthshade=False,
                   zorder=4, label='alvo selecionado')
        ax.plot([alvo[1], alvo[1]], [alvo[0], alvo[0]], [0.0, max(h)],
                color='0.7', linewidth=0.7, linestyle=':')

    ax.scatter([e[0]], [n[0]], [h[0]], marker='o', s=34,
               facecolor='white', edgecolor='0.10', linewidth=1.1,
               depthshade=False, zorder=5, label='início')
    ax.scatter([e[-1]], [n[-1]], [h[-1]], marker='s', s=26,
               facecolor='0.10', edgecolor='white', linewidth=0.8,
               depthshade=False, zorder=6, label='fim da descida')

    ax.set_xlabel('Leste (m)', labelpad=6)
    ax.set_ylabel('Norte (m)', labelpad=6)
    ax.set_zlabel('Altitude (m)', labelpad=6)
    for eixo in (ax.xaxis, ax.yaxis, ax.zaxis):
        eixo.set_major_formatter(
            plt.FuncFormatter(lambda v, _p: f'{v:g}'.replace('.', ',')))

    ax.view_init(elev=22, azim=-125)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_alpha(0.0)
        pane.set_edgecolor('0.85')
    ax.grid(True, color='0.88', linewidth=0.4)
    ax.set_box_aspect((1.0, 1.0, 0.72), zoom=1.02)

    # A legenda é ancorada à figura, e não aos eixos: a caixa de um eixo
    # tridimensional é bem maior que a região desenhada, e uma legenda
    # ancorada a ela sai do quadro.
    handles, rotulos = ax.get_legend_handles_labels()
    fig.legend(handles, rotulos, loc='upper center',
               bbox_to_anchor=(0.5, 0.995), ncol=2, frameon=False,
               handletextpad=0.5, columnspacing=1.4)
    if titulo:
        fig.suptitle(titulo, x=0.03, y=0.995, ha='left', fontsize=9)

    # O recorte automático ('tight') descarta o rótulo do eixo vertical em
    # eixos tridimensionais; aqui ele é desligado e as margens são fixadas.
    fig.subplots_adjust(left=0.10, right=0.97, bottom=0.03, top=0.86)
    with plt.rc_context({'savefig.bbox': None}):
        fig.savefig(destino)
    plt.close(fig)


def fig_acoes(serie, destino, titulo=None):
    """Velocidades comandadas ao longo do tempo."""
    t = serie['t_s']
    fig, ax = plt.subplots(figsize=(LARGURA_CM * CM, 6.2 * CM))

    series = [('v_norte_ms', 'norte'),
              ('v_leste_ms', 'leste'),
              ('v_down_ms', 'vertical')]
    for i, (campo, rotulo) in enumerate(series):
        ax.plot(t, serie[campo], label=rotulo, **estilo(i))

    ax.axhline(0.0, color='0.75', linewidth=0.6)
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Velocidade comandada (m/s)')
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.01), ncol=3,
              frameon=False, handletextpad=0.5, columnspacing=1.6)
    limpar(ax)
    virgula(ax)
    if titulo:
        ax.set_title(titulo, loc='left', pad=6)
    fig.savefig(destino)
    plt.close(fig)


def fig_erro_no_tempo(serie, destino, tolerancia=None, titulo=None):
    """Erro radial em relação à referência ao longo do tempo."""
    t = serie['t_s']
    fig, ax = plt.subplots(figsize=(LARGURA_CM * CM, 6.2 * CM))
    ax.plot(t, serie['erro_radial_m'], color='0.15', linewidth=1.2)
    if tolerancia:
        ax.axhline(tolerancia, color='0.45', linewidth=0.9, linestyle='--')
        ax.annotate(f'tolerância = {vb(tolerancia * 100, 1)} cm',
                    xy=(1.0, tolerancia), xycoords=('axes fraction', 'data'),
                    xytext=(-4, 4), textcoords='offset points',
                    fontsize=7.5, color='0.35', ha='right', va='bottom')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Erro radial (m)')
    ax.set_ylim(bottom=0)
    limpar(ax)
    virgula(ax)
    if titulo:
        ax.set_title(titulo, loc='left', pad=6)
    fig.savefig(destino)
    plt.close(fig)


def fig_erro_vs_altitude(resumo, destino):
    """Erro contra a altitude de operação, nas duas etapas.

    A figura testa se resta erro de escala na conversão. Como a densidade de
    pixels varia com o inverso da altitude, um erro de escala apareceria
    como tendência do erro com a altitude. O teste cabe ao erro ao final do
    alinhamento, que é o que a conversão determina; o erro ao final da
    descida carrega, além dele, a dispersão da manobra de ensaio, que aqui
    é mais que o dobro e encobriria a tendência procurada.
    """
    ok = [r for r in resumo if r.get('sucesso') == '1']
    h = [num(r['altitude_med_m']) for r in ok]
    pouso = [num(r['erro_toque_m']) for r in ok]
    tol = [num(r['tolerancia_m']) for r in ok]
    alinh = ([num(r['erro_alinhamento_m']) for r in ok]
             if all(num(r.get('erro_alinhamento_m')) is not None for r in ok)
             else None)

    fig, ax = plt.subplots(figsize=(LARGURA_CM * CM, 7.6 * CM))

    if alinh:
        ax.scatter(h, alinh, s=28, marker='o', facecolor='white',
                   edgecolor='0.10', linewidth=1.0, zorder=4,
                   label='ao final do alinhamento')
    ax.scatter(h, pouso, s=24, marker='^', facecolor='0.45',
               edgecolor='0.15', linewidth=0.6, zorder=3,
               label='ao final da descida')

    ordem = sorted(range(len(h)), key=lambda i: h[i])
    ax.plot([h[i] for i in ordem], [tol[i] for i in ordem],
            linestyle='--', color='0.5', linewidth=0.9,
            label='tolerância de convergência')

    ax.set_xlabel('Altitude de operação (m)')
    ax.set_ylabel('Erro em relação ao alvo (m)')
    ax.set_ylim(bottom=0)
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.01), ncol=3,
              frameon=False, handletextpad=0.4, columnspacing=1.2)
    limpar(ax)
    virgula(ax)
    fig.savefig(destino)
    plt.close(fig)


def fig_histograma(resumo, destino):
    """Distribuição do erro em duas etapas, em painéis empilhados.

    O painel superior mostra o erro ao final do alinhamento, que é o que o
    método estabelece. O inferior mostra o erro ao final da descida, manobra
    acrescentada ao roteiro de ensaio. Os dois compartilham eixo horizontal
    e a mesma divisão de classes, de modo que o alargamento introduzido pela
    descida seja lido por comparação direta, e não por confronto de números.
    """
    ok = [r for r in resumo if r.get('sucesso') == '1']
    alinh = [num(r['erro_alinhamento_m']) for r in ok
             if num(r.get('erro_alinhamento_m')) is not None]
    pouso = [num(r['erro_toque_m']) for r in ok
             if num(r.get('erro_toque_m')) is not None]
    if not pouso:
        return
    if not alinh:                      # campanhas antigas, sem a coluna
        alinh = None

    import numpy as np
    topo = max(pouso + (alinh or []))
    classes = np.linspace(0.0, topo * 1.02, 13)

    n = 2 if alinh else 1
    fig, eixos = plt.subplots(n, 1, sharex=True,
                              figsize=(LARGURA_CM * CM, (8.4 if n == 2 else 6.6) * CM))
    if n == 1:
        eixos = [eixos]

    dados = ([(alinh, 'Ao final do alinhamento (método proposto)')]
             if alinh else []) + \
            [(pouso, 'Ao final da descida (inclui a manobra de ensaio)')]

    for ax, (v, rotulo) in zip(eixos, dados):
        ax.hist(v, bins=classes, facecolor='0.80', edgecolor='0.25',
                linewidth=0.8)
        media = sum(v) / len(v)
        ax.axvline(media, color='0.15', linewidth=1.0, linestyle='--')
        ax.annotate(f'média = {vb(media)} m', xy=(media, 1.0),
                    xycoords=('data', 'axes fraction'),
                    xytext=(5, -3), textcoords='offset points',
                    fontsize=7.5, color='0.25', ha='left', va='top')
        ax.set_title(rotulo, loc='left', pad=4, fontsize=8.5)
        ax.set_ylabel('Ensaios')
        limpar(ax)
        virgula(ax)

    eixos[-1].set_xlabel('Erro em relação às coordenadas do alvo (m)')
    fig.align_ylabels(eixos)
    fig.savefig(destino)
    plt.close(fig)


def fig_convergencia_todos(dir_ensaios, resumo, destino, n_max=30):
    """Erro radial de todos os ensaios sobrepostos, na fase horizontal."""
    fig, ax = plt.subplots(figsize=(LARGURA_CM * CM, 7.0 * CM))
    traçados = 0
    for r in resumo:
        caminho = os.path.join(dir_ensaios, f"ensaio_{r['ensaio']}.csv")
        if not os.path.exists(caminho) or traçados >= n_max:
            continue
        s = carregar_serie(caminho)
        t, e, fases = s['t_s'], s['erro_radial_m'], s['fase']
        tt = [t[i] for i in range(len(t)) if fases[i] == 'horizontal']
        ee = [e[i] for i in range(len(e)) if fases[i] == 'horizontal']
        if tt:
            ax.plot(tt, ee, color='0.35', linewidth=0.7, alpha=0.75)
            traçados += 1

    ax.set_xlabel('Tempo desde a seleção (s)')
    ax.set_ylabel('Erro radial (m)')
    ax.set_ylim(bottom=0)
    ax.set_title(f'{traçados} ensaios sobrepostos', loc='left', pad=6)
    limpar(ax)
    virgula(ax)
    fig.savefig(destino)
    plt.close(fig)


# --------------------------------------------------------------- tabela
def tabela_latex(resumo, destino):
    """Tabela de métricas consolidadas, em LaTeX, pronta para inclusão."""
    ok = [r for r in resumo if r.get('sucesso') == '1']
    n = len(resumo)

    def col(c, fonte=ok):
        return [num(r[c]) for r in fonte
                if c in r and num(r[c]) is not None]

    def med(c, fonte=ok):
        v = col(c, fonte)
        return sum(v) / len(v) if v else float('nan')

    def dp(c, fonte=ok):
        v = col(c, fonte)
        if len(v) < 2:
            return float('nan')
        mu = sum(v) / len(v)
        return math.sqrt(sum((x - mu) ** 2 for x in v) / (len(v) - 1))

    def reqm(c, fonte=ok):
        v = col(c, fonte)
        return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')

    def mx(c, fonte=ok):
        v = col(c, fonte)
        return max(v) if v else float('nan')

    def br(x, casas=3):
        return f'{x:.{casas}f}'.replace('.', '{,}')

    linhas = [
        ('Ensaios executados', f'{n}'),
        ('Ensaios bem-sucedidos', f'{len(ok)}'),
        ('Taxa de sucesso', f'{100.0 * len(ok) / n:.1f}'.replace('.', '{,}')
         + r'\,\%'),
        (r'Tempo médio até a conclusão', br(med('tempo_total_s'), 1)
         + r' $\pm$ ' + br(dp('tempo_total_s'), 1) + r'\,s'),
        (r'Tempo médio da fase horizontal',
         br(med('tempo_horizontal_s'), 1) + r'\,s'),
        (r'Período médio do laço de controle',
         br(med('dt_medio_s', resumo) * 1000, 1) + r'\,ms'),
        (r'Erro médio em relação ao alvo', br(med('erro_verdadeiro_m'))
         + r' $\pm$ ' + br(dp('erro_verdadeiro_m')) + r'\,m'),
        (r'REQM em relação ao alvo',
         br(reqm('erro_verdadeiro_m')) + r'\,m'),
        (r'Erro máximo em relação ao alvo',
         br(mx('erro_verdadeiro_m')) + r'\,m'),
        (r'REQM em relação à referência calculada',
         br(reqm('erro_ref_m')) + r'\,m'),
    ]

    corpo = '\n'.join(f'\t\t{a} & {b} \\\\ \\hline' for a, b in linhas)
    tex = f"""\\begin{{table}}[H]
\t\\centering
\t\\caption{{Métricas consolidadas da campanha de ensaios.}}
\t\\label{{tab:metricas}}

\t\\renewcommand{{\\arraystretch}}{{1.2}}
\t\\normalsize

\t\\begin{{tabular}}{{|p{{8.4cm}}|p{{5.0cm}}|}}
\t\t\\hline
\t\t\\centering\\arraybackslash\\textbf{{MÉTRICA}} &
\t\t\\centering\\arraybackslash\\textbf{{VALOR}} \\\\ \\hline

{corpo}

\t\\end{{tabular}}
\t\\\\
\t\\vspace{{0.3cm}}
\t\\footnotesize{{Elaboração: Autor (2026)}}
\\end{{table}}
"""
    with open(destino, 'w') as f:
        f.write(tex)


# --------------------------------------------------------------- principal
def main():
    dir_ensaios = sys.argv[1] if len(sys.argv) > 1 else 'ensaios'
    dir_figuras = sys.argv[2] if len(sys.argv) > 2 else 'figuras'
    os.makedirs(dir_figuras, exist_ok=True)

    caminho_resumo = os.path.join(dir_ensaios, 'resumo.csv')
    if not os.path.exists(caminho_resumo):
        sys.exit(f'Nao encontrei {caminho_resumo}')
    resumo = ler_csv(caminho_resumo)
    print(f'{len(resumo)} ensaios no resumo')

    # ---- figuras agregadas
    fig_erro_vs_altitude(resumo, os.path.join(dir_figuras,
                                              'erro_vs_altitude.pdf'))
    fig_histograma(resumo, os.path.join(dir_figuras, 'erro_histograma.pdf'))
    fig_convergencia_todos(dir_ensaios, resumo,
                           os.path.join(dir_figuras, 'convergencia.pdf'))
    tabela_latex(resumo, os.path.join(dir_figuras, 'tabela_metricas.tex'))
    print('  figuras agregadas e tabela de metricas')

    # ---- ensaios representativos: o de menor e o de maior erro
    ok = [r for r in resumo if r.get('sucesso') == '1'
          and num(r.get('erro_verdadeiro_m')) is not None]
    if not ok:
        print('  nenhum ensaio bem-sucedido; figuras individuais omitidas')
        return

    ok.sort(key=lambda r: num(r['erro_verdadeiro_m']))
    representativos = [('melhor', ok[0]), ('pior', ok[-1])]

    for apelido, r in representativos:
        ident = r['ensaio']
        caminho = os.path.join(dir_ensaios, f'ensaio_{ident}.csv')
        if not os.path.exists(caminho):
            print(f'  {ident}: serie nao encontrada')
            continue
        s = carregar_serie(caminho)
        alvo = (num(r.get('alvo_norte_m')), num(r.get('alvo_leste_m')))
        tol = num(r.get('tolerancia_m'))

        fig_estados(s, os.path.join(dir_figuras, f'estados_{apelido}.pdf'))
        fig_trajetoria3d(s, os.path.join(dir_figuras,
                                         f'trajetoria3d_{apelido}.pdf'),
                         alvo=alvo if alvo[0] is not None else None)
        fig_acoes(s, os.path.join(dir_figuras, f'acoes_{apelido}.pdf'))
        fig_erro_no_tempo(s, os.path.join(dir_figuras, f'erro_{apelido}.pdf'),
                          tolerancia=tol)
        print(f'  {apelido}: {ident}  '
              f'(erro {r["erro_verdadeiro_m"]} m, '
              f'h {r["altitude_med_m"]} m)')

    print(f'\nFiguras em {dir_figuras}/')


if __name__ == '__main__':
    main()