# Se establece el tamaño y posición de la pantalla
from kivy.config import Config
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '726')
Config.set('graphics', 'fullscreen', '0')

# Módulos internos
from ColorManager import ColorManager

# Importar librerías de kivy
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.uix.image import Image
from kivy.uix.gridlayout import GridLayout as Grid
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.popup import Popup

# Importar librerías de kivymd
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField
from kivymd.uix.label import MDLabel
from kivymd.uix.dialog import MDDialog
from kivymd.uix.button import MDFlatButton

# GNZHT-CXNFD-H982D-WE7H3-29YL3

Clock.max_iteration = 1000  # Increase this value if necessary

# Importar librerías secundarias
from threading import Thread, Timer
import platform
import json 
import uuid
import webbrowser
from time import sleep
from UUIDManager import UUIDManager

class SplashScreen(Screen):
    '''Clase para mostrar la pantalla de inicio'''

    def on_enter(self, *args):
        '''Método que agenda el cambio'''
        Clock.schedule_once(self.switch_to_main,3)

    def switch_to_main(self, dt):
        '''Método que realiza el cambio a la pantalla principal'''
        self.manager.current = 'Main Window'

# Clase para el manejo de pantallas
class WindowManager(ScreenManager): pass # Administrador de pantallas
class MainWindow(Screen): pass           # Pantalla principal
class SecundaryWindow(Screen): pass      # Pantalla secundaria

# Clases predefinidas reciclables
# El formato específico y definido se encuentra en el archivo test.kv
class CustomLabelRoboto(MDLabel): pass   # Case predefinida para los subtítulos con formato
class CustomLabelAD(MDLabel): pass       # Case predefinida para los títulos con formato
class CustomTextEntry(MDTextField): pass # Case predefinida para las entradas de texto con formato
class InfoPopUp(Popup): pass             # Clase para mostrar un pop up de información
class ImageTeam(Image): pass             # Clase para formatear las imágenes del equipo
class LabelTeam(MDLabel): pass           # Clase para formatear los datos del equipo
class ButtonDevices(MDFlatButton): pass  # Clase para crear botones de los dispositivos encontrados

class ErrorPopup(Popup):
    '''Clase para mostrar un error genérico'''
    def __init__(self, **kwargs):
        super(ErrorPopup, self).__init__(**kwargs)

class ExoBoostApp(MDApp):  
    #------------------------------------------------------- Métodos de inicio -----------------------------------------------------#
    def __init__(self, **kwargs):
        '''Se inicilizan todos los métodos, el set up de la lógica y se definen atributos'''
        super().__init__(**kwargs)

        # -------------------------- Atributos internos --------------------------
        self.kv_loaded: bool = False
        self.mode: str = None # Modo de operación la app: {assistance, tuning}
        
        # Detecta el sistema operativo
        self.os_name = self.detect_os()
        
        # Diccionario de etiquetas para la sintonización
        self.selected_limb: str = "Right leg"
        self.motors_labels: dict[str] = {
            "Right leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Left leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Right arm": ["motor1", "motor2", "motor3"],
            "Left arm": ["motor1", "motor2", "motor3"],
            
        }
        
        # Límites de los parámetros PI de los motores
        self.motor_params_lims =  {
            "Right leg": {
                "motor1": {"kc": "100", "ti": "50", "sp": "99999"},
                "motor2": {"kc": "100", "ti": "50", "sp": "99999"},
                "motor3": {"kc": "100", "ti": "50", "sp": "99999"},
            },
            "Left leg": {
                "motor1": {"kc": "50", "ti": "100", "sp": "99999"},
                "motor2": {"kc": "50", "ti": "100", "sp": "99999"},
                "motor3": {"kc": "50", "ti": "100", "sp": "99999"},
            },
            "Right arm": {
                "motor1": {"kc": "1", "ti": "1", "sp": "99999"},
                "motor2": {"kc": "1", "ti": "1", "sp": "99999"},
                "motor3": {"kc": "1", "ti": "1", "sp": "99999"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "5", "sp": "99999"},
                "motor2": {"kc": "10", "ti": "5", "sp": "99999"},
                "motor3": {"kc": "10", "ti": "5", "sp": "99999"},
            }
        }

        # -------------------------- Parámetros de BLE --------------------------
        if self.os_name == "Android" or self.os_name == "Linux":
            print("Importando librería de BLE")
            # Librería de BLE
            from BLE2 import BluetoothManager_App
            self.ble_found = True
        else: self.ble_found = False

        # Instancia de Bluetooth
        if self.ble_found: self.ble = BluetoothManager_App()

        # Atributos de lógica BLE
        self.selected_device: str = None # Almacena el nombre del dispositivo seleccionado
        self.connection_successful: bool = False # Almacena si la conexión fue exitosa

        # -------- Manejo de los UUID según la ESP32 ---------
        self.uuid_manager = UUIDManager()
        # Nombres de los servicios para manejo interno
        names = ["Parameters", "Commands", "Process"]
        values = [0x0001, 0x0002, 0x0003]
        # Se generan los UUIDs para los servicios
        self.uuid_manager.generate_uuids_services(names, values)

        # Se genera una característica por servicio
        self.uuid_manager.generate_uuids_chars(names[0], ["PI"], [0x000a])
        self.uuid_manager.generate_uuids_chars(names[1], ["PV"], [0x000b])
        self.uuid_manager.generate_uuids_chars(names[2], ["Mode"], [0x000c])
        # -------------------------- Atributos externos --------------------------
        # Diccionario de valores de los parámetros de los motores de sintonización y control
        # Todos se inicializan con un valor arbitrario
        self.motor_parameters_pi =  {
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
                "motor1": {"kc": "10", "ti": "100", "sp": "0"},
                "motor2": {"kc": "10", "ti": "100", "sp": "0"},
                "motor3": {"kc": "10", "ti": "100", "sp": "0"},
            },
            "Left arm": {
                "motor1": {"kc": "10", "ti": "500", "sp": "0"},
                "motor2": {"kc": "10", "ti": "500", "sp": "0"},
                "motor3": {"kc": "10", "ti": "500", "sp": "0"},
            }
        }

        # Diccionario de valores de la variable de proceso de los motores
        # Todos se inicializan con un valor arbitrario
        self.motor_parameters_pv =  {
            "Right leg": {
                "motor1": {"pv": "0"},
                "motor2": {"pv": "0"},
                "motor3": {"pv": "0"},
            },
            "Left leg": {
                "motor1": {"pv": "0"},
                "motor2": {"pv": "0"},
                "motor3": {"pv": "0"},
            },
            "Right arm": {
                "motor1": {"pv": "0"},
                "motor2": {"pv": "0"},
                "motor3": {"pv": "0"},
            },
            "Left arm": {
                "motor1": {"pv": "0"},
                "motor2": {"pv": "0"},
                "motor3": {"pv": "0"},
            }
        }

        # Pantalla para desplegar app (no. de monitor)
        self.pos_screen(0)

        # Diccionario de colores
        self.colors: dict = ColorManager()._get_colors()
        '''
        Colores disponibles:
        Cyan, Dark Blue, Light Orange, Light Gray, Black, White.
        '''

    def build(self):
        """Carga kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True

        # Diccionario de TextFields de sintonización para accceso rápido 
        self.param_pi_entries: dict[dict] = {
            "motor1": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor1,
                "ti": self.root.get_screen('Main Window').ids.ti_motor1,
                "sp": self.root.get_screen('Main Window').ids.sp_motor1
            },
            "motor2": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor2,
                "ti": self.root.get_screen('Main Window').ids.ti_motor2,
                "sp": self.root.get_screen('Main Window').ids.sp_motor2
            },
            "motor3": {
                "kc": self.root.get_screen('Main Window').ids.kc_motor3,
                "ti": self.root.get_screen('Main Window').ids.ti_motor3,
                "sp": self.root.get_screen('Main Window').ids.sp_motor3
            },
        }

        # Diccionario de TextFields de variables de proceso para accceso rápido
        self.param_pv_entries: dict = {
            "motor1": self.root.get_screen('Main Window').ids.pv_motor1,
            "motor2": self.root.get_screen('Main Window').ids.pv_motor2,
            "motor3": self.root.get_screen('Main Window').ids.pv_motor3
        }

        # Se retorna la pantalla de inicio
        return self.root
    
    def on_start(self):
        self.root.current = "Splash Screen"

        # --------- Información de descripción del proyecto y equipo --------
        try:
            self.info_project = open("info_proyecto.txt", 'r', encoding='utf-8').read()
            self.team_info = [
                {"image": "images/TeresaHernandez.jpeg", "info": "Teresa Hernández\n\nIngeniería en Mecatrónica"},
                {"image": "images/CarlosReyes.jpeg", "info": "Carlos Reyes\n\nIngeniería en Mecatrónica"},
                {"image": "images/DavidVillanueva.jpeg", "info": "David Villanueva\n\nIngeniería en Mecatrónica"},
                {"image": "images/ItzelMartinez.jpeg", "info": "Itzel Martínez\n\nIngeniería en Mecatrónica & Biomédica"},
                {"image": "images/EduardoMartinez.jpeg", "info": "Eduardo Martínez\n\nIngeniería en Mecatrónica & Diseño Automotriz"}
            ]
        except: 
            # Información no encontrada
            self.team_info = self.info_project = "No info found"

        self.device_widgets_list: Grid = self.root.get_screen("Main Window").ids.device_list
        
        # Se configura como la extremidad inicial la pierna derecha
        self.limb_dropdown_clicked("Right leg")
    
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

    #----------------------------------------------------- Métodos generales ----------------------------------------------------

    def on_tab_select(self, tab: str): 
        '''Método que establece el modo de funcionamiento en función de la tab seleccionada'''
        if tab == "Assistance mode": 
            self.mode = "assistance"
        elif tab == "Bluetooth settings": 
            self.mode = "bluetooth"
        elif tab == "Tuning mode": 
            self.mode = "tuning"
        
        print(self.mode)

    def is_valid(self, var: str, tipo) -> bool:
        """
        Valida si un dato en formato de string pertenece a otro tipo de dato. 
        Función para validación de informaicón

        Entradas: var  - Dato a validar tipo string
                  tipo - valor correspondiente a la clase para validación
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
        
    #----------------------------------------------------- Métodos de menú de Bluetooth -----------------------------------------
    def search_devices(self):
        '''Busca dispositivos Bluetooth, los almacena y los muestra en el ScrollView'''
        # PONER EL SPINNER PARA ESPERAR AL ESCANEO, SE PUEDE HACER EN UN THREAD SEPARADO SECUNDARIO
        if self.ble_found:
            # Estado del bluetooth
            print(f"Bluetooth habilitado: {self.ble.is_bluetooth_enabled()}\n")  

            # Se inicia el escaneo durante 5 segundos y se obtiene la lista de dispositivos
            self.ble.resetBLE() # Se reinica el escaneo
            self.ble.start_ble_scan()
            scanning = Thread(target=self.perfom_scanning, args=(5.0,))
            scanning.start()
        else: 
            print("Bluetooth no disponible")
            devices = ["Dispositivo 3", "Dispositivo 2", "Dispositivo 1"]
            # Se muestran los resutlados en la pantalla
            self.show_devices(devices)

    def show_devices(self, devices: list[str]):
        '''
        Método que muestra los elementos de una lista en el ScrollView
        Entrada: devices list[str] -> lista de dispositivos
        '''
        # --------- Lógica de la lista ---------------
        self.device_widgets_list.clear_widgets()  # Limpiar widgets anteriores
        print("show devices method")
        for dev in devices:
            btn = ButtonDevices(text=dev)
            btn.bind(on_release=self.on_device_select)
            self.device_widgets_list.add_widget(btn)
        
    def on_device_select(self, instance: ButtonDevices): 
        print("on device select method")
        '''Método que imprime dispositivo seleccionado'''
        self.selected_device = instance.text
        print(f'{instance.text} fue presionado')

        for widget in self.device_widgets_list.children:
            # Se cambia el color de fondo de los botones
            if widget.text == instance.text:
                widget.md_bg_color = self.colors["Light Orange"]
            else: 
                widget.md_bg_color = self.colors["Dark Blue"]

    def perfom_scanning(self, time: float = 5.0):
        print("perfom scanning method")
        '''Método para iniciar escaneo de dispositivos''' 
        def stop_scanning(): 
            '''Detiene el escaneo y muestra los resultados'''
            devices = self.ble.stop_ble_scan()
            # Se actualiza el ScrollView en el main Thread
            Clock.schedule_once(lambda x: self.show_devices(devices))
        # Acción que se ejecuta después de time segundos
        timer_ble = Timer(time, stop_scanning)
        timer_ble.start()

    def connect_disconnect(self): 
        '''Método para conectar/disconectar dispositivo'''

        def perform_connection():
            '''Método para realizar la conexión'''
            self.connection_successful = self.ble.connect(self.selected_device)
        # LÓGICA PARA CAMBIAR EL 
        # COLOR DE FONDO DEL BOTON
        # TEXTO DEL BOTON

        # No hace ninguna acción si no hay un dispositivo seleccionado o si el BLE no está disponible
        if not self.selected_device or not self.ble_found: return

        if not self.ble.connected:
            t = Thread(target=perform_connection)
            t.start()
            # self.connection_successful = self.ble.connect(self.selected_device) # SE DEBERÍA DE PONER EN OTRO THREAD
            print(f"Dispositivo conectado: {self.connection_successful}")
            self.root.get_screen('Main Window').ids.bluetooth_connect.text = "Disconnect"

        else:
            # Se realiza desconexión y se limpia el dispositivo seleccionado
            self.ble.disconnect()
            self.selected_device = None
            self.root.get_screen('Main Window').ids.bluetooth_connect.text = "Connect"

    def send_params(self): 
        '''Método para enviar parámetros al dispositivo conectado'''
        pass

    #----------------------------------------------------- Métodos del menú de asistencia -----------------------------------------------------
    # ---------------- Imprime valor del slider ----------------
    def on_slider_value(self, value):
        '''Handle the slider value change'''
        print(f"Assitance Level: {value}")

    #------- Imprimen acciones en botones de asistencia -----
    # Pararse/Sentarse
    def sit_down_stand_up(self):
        print("Sit down/stand up action triggered")

        # PRUEBAS DE MANDAR DATOS, MOVER DESPUÉS A UN BOTÓN DE SUBMIT
        # Acción de submit parámetros
        if not self.ble_found: return

        json_data = self.motor_parameters_pi[self.selected_limb]

        # Se definen los UUIDs y los datos a mandar para la parámetros de control 
        service_uuid = str(self.uuid_manager.uuids_services["Parameters"]) # Se convierte a string
        char_uuid = str(self.uuid_manager.uuids_chars["Parameters"]["PI"]) # Se convierte a string

        # Se mandan los datos
        self.ble.write_json(service_uuid, char_uuid, json_data)   

    #Caminar
    def walk(self):
        print("Walk action triggered")

    #Detenerse
    def stop(self):
        print("Stop action triggered")

    def assitance_method(self): pass

    #----------------------------------------------------- Métodos del menú de sintonizción -----------------------------------------------------
    
    #Método para desplegar valores de PI en cada motor de acuerdo a la extremidad seleccionada
    def limb_dropdown_clicked(self, limb: str) -> None: 
        '''
        Método para actualizar en la app los datos de los motores al seleccionar otra extremidad
        Entrada: Entrada seleccionada (str)
        '''

        # Se obtiene la selección
        self.selected_limb = limb

        # Se cambian las etiquetas de los motores
        new_labels: list[str] = self.motors_labels[self.selected_limb]
        self.root.get_screen('Main Window').ids.motor1_label.text = new_labels[0]
        self.root.get_screen('Main Window').ids.motor2_label.text = new_labels[1]
        self.root.get_screen('Main Window').ids.motor3_label.text = new_labels[2]

        # Se cambian los valores de los parámetros PI de los motores
        new_params: dict[dict[str]]= self.motor_parameters_pi[self.selected_limb]
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
        old_params: dict[dict[str]] = self.motor_parameters_pi[self.selected_limb]

        if self.is_valid(value, 1): # Validación de dato como int
            if param in ["kc", "ti", "sp"]:
                max_value = self.motor_params_lims[self.selected_limb][motor][param]
                if int(value) <= int(max_value) and int(value) >= 0: # Validación de rango válido
                    # Si es válido, se actualiza el diccionario de parámetros
                    self.motor_parameters_pi[self.selected_limb][motor][param] = value
                else: # Valor no válido
                    self.param_pi_entries[motor][param].text = old_params[motor][param]
        else: # Tipo no válido
            self.param_pi_entries[motor][param].text = old_params[motor][param]
    
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
                buttons=[MDFlatButton(text="Cerrar", on_release=lambda *args: dialog.dismiss())]
            )
            dialog.open()

    def close_popup(self):
        '''Función para cerra el pop up de información'''
        self.popup.dismiss()



if __name__ == '__main__':
    """Función principal que lanza la aplicación"""
    ExoBoostApp = ExoBoostApp()
    ExoBoostApp.run()