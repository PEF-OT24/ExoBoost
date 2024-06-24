# Se establece el tamaño y posición de la pantalla
from kivy.config import Config
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '726')
Config.set('graphics', 'fullscreen', '0')

# Módulos internos
from ColorManager import ColorManager

# Importar librerías
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField
from kivy.uix.popup import Popup
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from kivy.clock import Clock
from kivymd.uix.label import MDLabel
from kivy.uix.image import Image
from kivy.uix.gridlayout import GridLayout as Grid
from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDFlatButton

# Librerías adicionales
import platform
import webbrowser

Clock.max_iteration = 1000  # Aumentar de ser necesario

# Se crean múltiples ventanas, el código se encontrará en la ventana principal
class MainWindow(Screen):
    def search_devices(self):
        device_list = self.ids.device_list
        devices = [{'text': f'DISPOSITIVO {i}'} for i in range(1,4)]
        try:
            device_list.data = devices
        except:
            print('no devices aun')
    
    def connect_disconnect(self):
        device_list = self.ids.device_list
        selected_devices = [child for child in device_list.children[0].children if child.selected]
        try:
            if selected_devices:
                print(f"Connecting/Disconnecting {selected_devices[0].text}")
            else:
                print("No device selected")
        except:
            print("boton no funciona aun")

class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass
class CustomLabelRoboto(MDLabel): pass # Case predefinida para los subtítulos con formato
class CustomLabelAD(MDLabel): pass # Case predefinida para los títulos con formato
class CustomTextEntry(MDTextField): pass # Case predefinida para las entradas de texto con formato
class InfoPopUp(Popup): pass
class ImageTeam(Image): pass
class LabelTeam(MDLabel): pass

class TestDesignApp(MDApp):  
    #------------------------ Métodos de inicio ------------------------#
    def __init__(self, **kwargs):
        '''Se inicilizan todos los métodos, el set up de la lógica y se definen atributos'''
        super().__init__(**kwargs)
        self.kv_loaded: bool = False

        # Detecta el sistema operativo
        self.os_name = self.detect_os()
        self.pos_screen(0)

        # Modo de trabajo: {"assistance", "tuning"}
        self.mode: str = None

        # Diccionario de colores
        self.colors: dict = ColorManager()._get_colors()
        '''
        Available colors:
        Cyan, Dark Blue, Light Orange, Light Gray, Black, White.
        '''
        
        # Diccionario de etiquetas 
        self.limb: str = ""
        self.motors_labels: dict[str] = {
            "Right leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Left leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Right arm": ["motor1", "motor2", "motor3"],
            "Left arm": ["motor1", "motor2", "motor3"],
            
        }

        # Diccionario de valores de los parámetros de los motores
        # Todos se inicializan con un valor arbitrario
        self.motor_parameters: dict[dict[dict[str]]] =  {
            "Right leg": {
                "motor1": {"kc": "100", "ti": "50", "sp": "0", "pv": "0"},
                "motor2": {"kc": "100", "ti": "50", "sp": "0", "pv": "0"},
                "motor3": {"kc": "100", "ti": "50", "sp": "0", "pv": "0"},
            },
            "Left leg": {
                "motor1": {"kc": "50", "ti": "100", "sp": "0", "pv": "0"},
                "motor2": {"kc": "50", "ti": "100", "sp": "0", "pv": "0"},
                "motor3": {"kc": "50", "ti": "100", "sp": "0", "pv": "0"},
            },
            "Right arm": {
                "motor1": {"kc": "1", "ti": "1", "sp": "0", "pv": "0"},
                "motor2": {"kc": "1", "ti": "1", "sp": "0", "pv": "0"},
                "motor3": {"kc": "1", "ti": "1", "sp": "0", "pv": "0"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "5", "sp": "0", "pv": "0"},
                "motor2": {"kc": "10", "ti": "5", "sp": "0", "pv": "0"},
                "motor3": {"kc": "10", "ti": "5", "sp": "0", "pv": "0"},
            }
        }

        # Define un sistema de diccionarios para establecer los límites de los parámetros
        self.motor_params_lims: dict[dict[dict[str]]] =  {
            "Right leg": {
                "motor1": {"kc": "100", "ti": "50", "sp": "0"},
                "motor2": {"kc": "100", "ti": "50", "sp": "0"},
                "motor3": {"kc": "100", "ti": "50", "sp": "0"},
            },
            "Left leg": {
                "motor1": {"kc": "50", "ti": "100", "sp": "0"},
                "motor2": {"kc": "50", "ti": "100", "sp": "0"},
                "motor3": {"kc": "50", "ti": "100", "sp": "0"},
            },
            "Right arm": {
                "motor1": {"kc": "1", "ti": "1", "sp": "0"},
                "motor2": {"kc": "1", "ti": "1", "sp": "0"},
                "motor3": {"kc": "1", "ti": "1", "sp": "0"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "5", "sp": "0"},
                "motor2": {"kc": "10", "ti": "5", "sp": "0"},
                "motor3": {"kc": "10", "ti": "5", "sp": "0"},
            }
        }

    def build(self):
        """Carga kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True

        # Diccionario de TextFields de sintonización para accceso rápido 
        self.param_entries: dict[dict] = {
            "motor1": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor1,
                "ti": self.root.get_screen('Main Window').ids.ti_motor1,
                "sp": self.root.get_screen('Main Window').ids.sp_motor1,
                "pv": self.root.get_screen('Main Window').ids.pv_motor1
            },
            "motor2": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor2,
                "ti": self.root.get_screen('Main Window').ids.ti_motor2,
                "sp": self.root.get_screen('Main Window').ids.sp_motor2,
                "pv": self.root.get_screen('Main Window').ids.pv_motor2
            },
            "motor3": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor3,
                "ti": self.root.get_screen('Main Window').ids.ti_motor3,
                "sp": self.root.get_screen('Main Window').ids.sp_motor3,
                "pv": self.root.get_screen('Main Window').ids.pv_motor3
            },
        }

        return self.root
    
    def on_start(self):
        self.root.current = "Main Window"
        self.limb_dropdown_clicked("Right leg")

        # Se lee el archivo de texto incluyendo la información del proyecto
        with open('info_proyecto.txt', 'r', encoding='utf-8') as file:
            self.info_project = file.read()
        
        # --------- Información de descripción del equipo --------
        self.team_info = [
            {"image": "images/TeresaHernandez.jpeg", "info": "Teresa Hernández\n\nIngeniería en Mecatrónica"},
            {"image": "images/CarlosReyes.jpeg", "info": "Carlos Reyes\n\nIngeniería en Mecatrónica"},
            {"image": "images/DavidVillanueva.jpeg", "info": "David Villanueva\n\nIngeniería en Mecatrónica"},
            {"image": "images/ItzelMartinez.jpeg", "info": "Itzel Martínez\n\nIngeniería en Mecatrónica & Biomédica"},
            {"image": "images/EduardoMartinez.jpeg", "info": "Eduardo Martínez\n\nIngeniería en Mecatrónica & Diseño Automotriz"}
        ]
    
    # ------------------------ Administrador de ventanas ------------------------#
    def detect_os(self) -> str:
        '''Detecta el OS y lo devuelve como string'''
        os_name = platform.system()
        if os_name == 'Linux':
            return "Linux"
        elif os_name == 'Windows':
            return "Windows"
        elif os_name == 'Darwin':
            return "MacOS"
        elif os_name.startswith('Java'):
            if 'android' in platform.java_ver()[3][0]:
                return "Android"
        else:
            return "Unknown"

    def pos_screen(self, screen: int):
        '''Establece modo pantalla completa dependiendo del OS'''
        os_name = self.detect_os()
        if os_name == 'Linux' or os_name == 'Windows':
            # Se maximiza/minimiza en windows de acuerdo al parámetro
            if screen == 0:
                Window.fullscreen = False
            else:
                Window.fullscreen = True
        elif os_name == 'Android':
            # Pantalla completa en Android
            Window.fullscreen = True

    #------------------------ Métodos generales ------------------------

    def on_tab_select(self, tab: str): 
        '''Método que establece el modo de funcionamiento en función de la tab seleccionada'''
        if tab == "Assistance mode": 
            self.mode = "assistance"
        elif tab == "Bluetooth settings" or tab == "Tuning mode": 
            self.mode = "tuning"
        
        print(self.mode)
    #------------------------ Métodos de menú de blutooth ------------------------

    def bluetooth_connection(self): pass
    def send_params(self): raise NotImplementedError("Not implemented function")

    #------------------------ Métodos del menú de asistencia ------------------------

    def on_slider_value(self, value):
        '''Handle the slider value change'''
        print(f"Slider value: {value}")

    def sit_down_stand_up(self):
        print("Sit down/stand up action triggered")

    def walk(self):
        print("Walk action triggered")

    def stop(self):
        print("Stop action triggered")

    #------------------------ Métodos del menú de sintonizción ------------------------

    def limb_dropdown_clicked(self, limb: str) -> None: 
        '''
        Función para actualizar los datos de los motores al seleccionar otra extremidad

        Entrada: Entrada seleccionada (str)
        '''
        # Se obtiene la selección
        self.limb = limb

        # Se cambian las etiquetas de los motores
        new_labels: list[str] = self.motors_labels[self.limb]
        self.root.get_screen('Main Window').ids.motor1_label.text = new_labels[0]
        self.root.get_screen('Main Window').ids.motor2_label.text = new_labels[1]
        self.root.get_screen('Main Window').ids.motor3_label.text = new_labels[2]

        # Se cambian los valores de los parámetros PI de los motores
        new_params: dict[dict[str]]= self.motor_parameters[self.limb]
        # Motor 1
        self.root.get_screen('Main Window').ids.kc_motor1.text = new_params["motor1"]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor1.text = new_params["motor1"]["ti"]
        # Motor 2
        self.root.get_screen('Main Window').ids.kc_motor2.text = new_params["motor2"]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor2.text = new_params["motor2"]["ti"]
        # Motor 3
        self.root.get_screen('Main Window').ids.kc_motor3.text = new_params["motor3"]["kc"]
        self.root.get_screen('Main Window').ids.ti_motor3.text = new_params["motor3"]["ti"]

    def on_entry_text(self, param: str, motor: str, value: str) -> None: 
        """
        Método que valida si el texto ingresado por el usuario es válido, tanto de clase como de rango.
        Entradas: param -> parámetro {'kc', 'ti', 'sp'}
                  motor -> número de motor {'motor1', 'motor2', 'motor3'}
                  value -> valor ingresado
        """
        old_params: dict[dict[str]] = self.motor_parameters[self.limb]

        if self.is_valid(value, 1): # Validación de dato como int
            if param in ["kc", "ti", "sp"]:
                max_value = self.motor_params_lims[self.limb][motor][param]
                if int(value) <= int(max_value) and int(value) >= 0: # Validación de rango válido
                    # Si es válido, se actualiza el diccionario de parámetros
                    self.motor_parameters[self.limb][motor][param] = value
                else: # Valor no válido
                    self.param_entries[motor][param].text = old_params[motor][param]
        else: # Tipo no válido
            self.param_entries[motor][param].text = old_params[motor][param]
    
    # --------------------------- Métodos del menú Pop Up -------------------------
    def show_popup(self):
        self.popup = InfoPopUp()
        self.popup.open()
        
        # Se agrega la información de cada miembro al layout
        grid: Grid = self.popup.ids.team_grid
        for person in self.team_info: 
            grid.add_widget(ImageTeam(source = person["image"]))
            grid.add_widget(LabelTeam(text = person["info"]))

    def open_repo(self) -> None: 
        '''
        Función para abrir el repositorio del código fuente
        
        Nota: el usuario necesita acceso al repo para poder abrirlo
        '''
        url: str = "https://github.com/PEF-OT24/ExoBoost"
        try: 
            webbrowser.open(url)
        except:
            dialog = MDDialog(
            title="Error",
            text="Ha ocurrido un error.",
            buttons=[
                MDFlatButton(
                    text="Cerrar", on_release=lambda *args: dialog.dismiss()
                )
            ],
            )
            dialog.open()

    def close_popup(self):
        '''Función para cerra el pop up de información'''
        self.popup.dismiss()

    # --------------------------- Funciones de uso general -------------------------

    def is_valid(self, var: str, tipo) -> bool:
        """
        Valida si un dato en formato de string pertenece a otro tipo de dato. 
        Función para validación de informaicón

        Entrada: Dato a validar tipo string
        Salida: Booleano indicando si el dato pertenece a la clase indicada
        """
        try:
            if isinstance(tipo, int):
                int(var)
            elif isinstance(tipo, float):
                float(var)
            else:
                raise TypeError("La clase de 'tipo' no está considerada")
            return True
        except (ValueError, TypeError):
            return False
        

def main():
    '''Se inicializa la app y ajusta la pantalla de acuerdo al sistema operativo'''
    TestDesignApp().run()

if __name__ == '__main__':
    main()