
import sys
import json
from pathlib import Path

import numpy as np
import matplotlib

# Backend no interactivo para poder guardar sin entorno grafico.
# Si se quiere ver en pantalla, comentar la linea siguiente.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Patch


# -----------------------------------------------------------------------------
#   Parametros por defecto (deben coincidir con el simulador original)
# -----------------------------------------------------------------------------
DEFAULT_AREA_SIZE_X = 2000
DEFAULT_AREA_SIZE_Y = 2000
DEFAULT_CELL_SIZE = 100

# Escala de letra (>= 1.0 amplia respecto al original). Modificable.
FONT_SCALE = 1.25

# Tamanos base tomados de la funcion original plot_coverage_holes,
# multiplicados por FONT_SCALE.
FS_TITLE       = int(round(22 * FONT_SCALE))
FS_AXIS_LABEL  = int(round(18 * FONT_SCALE))
FS_CBAR_LABEL  = int(round(18 * FONT_SCALE))
FS_TICK        = int(round(14 * FONT_SCALE))
FS_LEGEND      = int(round(14 * FONT_SCALE))
FS_TEXTBOX     = int(round(16 * FONT_SCALE))
FS_ANNOTATION  = int(round(11 * FONT_SCALE))

# Colores de severidad: identicos al original.
SEVERITY_COLORS = {
    'critical': '#FF0000',
    'severe':   '#FF6600',
    'moderate': '#FFCC00',
    'mild':     '#99CC00',
}


# -----------------------------------------------------------------------------
#   Reconstruccion del mapa de SINR (si esta disponible en el JSON)
# -----------------------------------------------------------------------------
def build_sinr_display(report):
    """Devuelve (sinr_display, area_x, area_y, has_grid).

    Si el JSON trae 'coverage_grid' con 'sinr_map', se reconstruye el mapa de
    SINR tal cual; las celdas no medidas (-999) se ponen a NaN (transparentes),
    igual que en la funcion original.
    """
    cg = report.get('coverage_grid')
    if cg and 'sinr_map' in cg:
        sinr_map = np.array(cg['sinr_map'], dtype=float)
        area_x = cg.get('area_size_x', DEFAULT_AREA_SIZE_X)
        area_y = cg.get('area_size_y', DEFAULT_AREA_SIZE_Y)
        sinr_display = np.where(sinr_map == -999, np.nan, sinr_map)
        return sinr_display, area_x, area_y, True

    # Sin grid: fondo vacio del tamano del area conocido.
    area_x = DEFAULT_AREA_SIZE_X
    area_y = DEFAULT_AREA_SIZE_Y
    rows = int(np.ceil(area_y / DEFAULT_CELL_SIZE))
    cols = int(np.ceil(area_x / DEFAULT_CELL_SIZE))
    sinr_display = np.full((rows, cols), np.nan)
    return sinr_display, area_x, area_y, False


# -----------------------------------------------------------------------------
#   Funcion de ploteo (fiel a plot_coverage_holes, con letra mas grande)
# -----------------------------------------------------------------------------
def plot_coverage_holes_from_report(report, title="Coverage Holes", save_path=None):
    holes_summary = report.get('coverage_holes', {})
    regions = holes_summary.get('regions', [])

    sinr_display, area_x, area_y, has_grid = build_sinr_display(report)

    # Figura con dos zonas: el grid a la izquierda y un panel lateral estrecho
    # a la derecha donde van el recuento de holes y la leyenda de colores.
    fig, (ax, ax_info) = plt.subplots(
        1, 2, figsize=(18, 11),
        gridspec_kw={'width_ratios': [3.4, 1]})

    # Fondo de SINR (identico al original cuando hay grid disponible).
    im = ax.imshow(sinr_display, cmap='RdYlGn', origin='upper',
                   extent=[0, area_x, area_y, 0],
                   vmin=-20, vmax=30, alpha=0.6)
    cbar = plt.colorbar(im, ax=ax, shrink=0.8)
    cbar.set_label('SINR (dB)', fontsize=FS_CBAR_LABEL)
    cbar.ax.tick_params(labelsize=FS_TICK)

    # Dibujo de cada hueco de cobertura, con su etiqueta de area/SINR encima
    # (en su posicion original, igual que la funcion original).
    for hole in regions:
        color = SEVERITY_COLORS.get(hole['severity'], '#888888')
        bb = hole['bounding_box']
        if hole['type'] == 'cluster':
            rect = plt.Rectangle(
                (bb['x_min'], bb['y_min']),
                bb['x_max'] - bb['x_min'], bb['y_max'] - bb['y_min'],
                linewidth=2, edgecolor=color, facecolor=color, alpha=0.25)
            ax.add_patch(rect)
            ax.plot(hole['center']['x'], hole['center']['y'], 'x', color=color,
                    markersize=10, markeredgewidth=2)
            ax.annotate(
                f"{hole['severity']}\n{hole['area_m2']:.0f}m2\n{hole['avg_sinr']:.1f}dB",
                (hole['center']['x'], hole['center']['y']), fontsize=FS_ANNOTATION,
                ha='center', va='bottom', color='white', fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', facecolor=color, alpha=0.8))
        else:
            ax.plot(hole['center']['x'], hole['center']['y'], 's', color=color,
                    markersize=6, alpha=0.7)

    ax.grid(True, alpha=0.2, linestyle='--')
    ax.set_xlabel('X (m)', fontsize=FS_AXIS_LABEL)
    ax.set_ylabel('Y (m)', fontsize=FS_AXIS_LABEL)
    ax.tick_params(axis='both', labelsize=FS_TICK)
    ax.set_title(title, fontsize=FS_TITLE, fontweight='bold')

    # ----------------------------------------------------------------------
    #   Panel lateral: solo recuento de holes y leyenda de colores.
    # ----------------------------------------------------------------------
    ax_info.axis('off')

    # Resumen general (cuadro negro, igual estilo que el original).
    textstr = (f"Coverage Holes: {holes_summary.get('total_holes', 0)}\n"
               f"Cells affected: {holes_summary.get('total_cells_affected', 0)}\n"
               f"Area affected: {holes_summary.get('total_area_m2', 0):.0f} m2")
    ax_info.text(0.0, 0.985, textstr, transform=ax_info.transAxes, fontsize=FS_TEXTBOX,
                 va='top', ha='left', color='white',
                 bbox=dict(boxstyle='round', facecolor='black', alpha=0.85))

    # Leyenda de severidades, agrupada debajo, en el mismo panel.
    legend_elements = [
        Patch(facecolor='#FF0000', alpha=0.5, label='Critical (SINR < -10 dB)'),
        Patch(facecolor='#FF6600', alpha=0.5, label='Severe (-10 <= SINR < 0 dB)'),
        Patch(facecolor='#FFCC00', alpha=0.5, label='Moderate (0 <= SINR < 13 dB)'),
        Patch(facecolor='#99CC00', alpha=0.5, label='Mild (13 <= SINR < 20 dB)')]
    ax_info.legend(handles=legend_elements, loc='upper left', fontsize=FS_LEGEND,
                   bbox_to_anchor=(0.0, 0.80), frameon=True)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, format='pdf', bbox_inches='tight')
        print(f"[Visualization] Guardado en {save_path}")

    plt.close(fig)


# -----------------------------------------------------------------------------
#   Main
# -----------------------------------------------------------------------------
def main():
    args = sys.argv[1:]
    if args:
        json_files = [Path(a) for a in args]
    else:
        json_files = sorted(Path('.').glob('*coverage_report*.json'))

    if not json_files:
        print("No se encontraron archivos JSON. Pasa rutas como argumento o "
              "ejecuta en un directorio con *coverage_report*.json")
        sys.exit(1)

    for jf in json_files:
        if not jf.exists():
            print(f"[Aviso] No existe: {jf}")
            continue
        with open(jf, 'r', encoding='utf-8') as f:
            report = json.load(f)

        title = "Coverage Holes"

        out_pdf = jf.with_name(jf.stem + "_coverage_holes.pdf")
        plot_coverage_holes_from_report(report, title=title, save_path=str(out_pdf))


if __name__ == "__main__":
    main()
