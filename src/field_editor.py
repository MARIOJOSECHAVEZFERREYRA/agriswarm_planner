import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from shapely.geometry import Polygon
import field_io  # Asegúrate de que este import funcione relativo a tu estructura

class InteractiveEditor:
    def __init__(self, load_filename=None):
        self.filename = load_filename
        self.final_polygon = None
        self.points = []
        
        # Configurar la ventana
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('AgriSwarm - Editor de Campo')
        plt.subplots_adjust(bottom=0.2) # Dejar espacio abajo para botones

        # Configurar el lienzo
        self.ax.set_xlim(-50, 250)
        self.ax.set_ylim(-50, 250)
        self.ax.grid(True, linestyle=':', alpha=0.6)
        self.ax.set_title("DIBUJA TU CAMPO\nClick Izq: Poner punto | Click Der: Quitar último", fontsize=12)
        
        # Línea de estado (para mensajes de error)
        self.status_text = self.ax.text(0.5, 0.95, "", transform=self.ax.transAxes, 
                                        ha='center', va='top', fontsize=12, 
                                        bbox=dict(boxstyle="round", fc="white", alpha=0.8))

        # Objeto gráfico de la línea que se dibuja
        self.line, = self.ax.plot([], [], 'b.-', markerfacecolor='k', linewidth=2)
        self.preview_line, = self.ax.plot([], [], 'b--', linewidth=1, alpha=0.5) # Cierra el polígono visualmente

        # --- BOTONES (Widgets) ---
        # Definir posiciones [left, bottom, width, height]
        ax_reset = plt.axes([0.15, 0.05, 0.2, 0.075])
        ax_load = plt.axes([0.4, 0.05, 0.2, 0.075])
        ax_save = plt.axes([0.65, 0.05, 0.2, 0.075])

        self.btn_reset = Button(ax_reset, 'Reiniciar', color='lightcoral', hovercolor='red')
        self.btn_load = Button(ax_load, 'Cargar Ultimo', color='lightyellow', hovercolor='gold')
        self.btn_save = Button(ax_save, 'Terminar', color='lightgreen', hovercolor='lime')

        # Conectar eventos
        self.btn_reset.on_clicked(self.reset)
        self.btn_load.on_clicked(self.load_previous)
        self.btn_save.on_clicked(self.finish)
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self.on_click)

    def on_click(self, event):
        """Maneja los clics en el mapa"""
        if event.inaxes != self.ax: return # Ignorar clics fuera del gráfico (en botones)

        if event.button == 1: # Click Izquierdo (Añadir)
            self.points.append((event.xdata, event.ydata))
            self.show_message("") # Limpiar errores
        elif event.button == 3: # Click Derecho (Borrar último)
            if self.points: self.points.pop()
        
        self.update_plot()

    def update_plot(self, color='b'):
        """Redibuja las líneas"""
        if not self.points:
            self.line.set_data([], [])
            self.preview_line.set_data([], [])
            self.fig.canvas.draw()
            return

        xs, ys = zip(*self.points)
        self.line.set_data(xs, ys)
        self.line.set_color(color)
        
        # Dibujar línea de cierre (del último al primero) para visualizar el polígono cerrado
        if len(self.points) > 2:
            self.preview_line.set_data([xs[-1], xs[0]], [ys[-1], ys[0]])
            self.preview_line.set_color(color)
        else:
            self.preview_line.set_data([], [])

        self.fig.canvas.draw()

    def reset(self, event):
        """Borra todo el dibujo"""
        self.points = []
        self.update_plot('b')
        self.show_message("Canvas reiniciado", color="black")
        self.ax.set_title("DIBUJA TU CAMPO")

    def load_previous(self, event):
        """Carga desde archivo JSON usando tu módulo existente"""
        if not self.filename: return
        try:
            from field_io import FieldIO # Import local para evitar ciclos
            poly = FieldIO.load_field(self.filename)
            self.points = list(poly.exterior.coords)[:-1] # Quitamos el punto repetido final
            self.update_plot('b')
            self.show_message("Campo cargado. Puedes editarlo.", color="green")
        except Exception as e:
            self.show_message(f"Error al cargar: {str(e)}", color="red")

    def finish(self, event):
        """Valida y guarda"""
        if len(self.points) < 3:
            self.show_message("Necesitas al menos 3 puntos", color="orange")
            return

        # Intentar crear polígono
        candidate_poly = Polygon(self.points)

        # --- VALIDACIÓN CRÍTICA (Detectar el "8") ---
        if not candidate_poly.is_valid:
            # ERROR DETECTADO
            self.show_message("ERROR: Geometria Invalida (Lineas cruzadas)\nReinicia o corrige los puntos.", color="red")
            self.update_plot('r') # Dibujar en ROJO para mostrar el error
            self.ax.set_title("¡ERROR! EL POLÍGONO SE CRUZA A SÍ MISMO", color='red', fontweight='bold')
            # NO cerramos la ventana. Dejamos que el usuario corrija.
        else:
            # ÉXITO
            self.final_polygon = candidate_poly
            self.show_message("Poligono Valido. Procesando...", color="green")
            plt.pause(0.5) # Pequeña pausa para ver el mensaje verde
            plt.close(self.fig)

    def show_message(self, text, color='black'):
        self.status_text.set_text(text)
        self.status_text.set_color(color)
        self.fig.canvas.draw()

    def show(self):
        """Muestra la ventana y bloquea hasta que se cierre"""
        plt.show()
        return self.final_polygon