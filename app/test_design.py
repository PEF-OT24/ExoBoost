# Set the screen size and mode
from kivy.config import Config
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '726')
Config.set('graphics', 'fullscreen', '0')

# Import kivy and kivymd libraries 
from kivymd.app import MDApp
from kivymd.uix.label import MDLabel
from kivymd.uix.textfield import MDTextField
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from ColorManager import ColorManager
import platform

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain differente or new functions to the app
class MainWindow(Screen):
    
    def search_devices(self):
        device_list = self.ids.device_list
        devices = [{'text': f'DISPOSITIVO {i}'} for i in range(1,4)]
        device_list.data = devices
    
    def connect_disconnect(self):
        device_list = self.ids.device_list
        selected_devices = [child for child in device_list.children[0].children if child.selected]
        if selected_devices:
            print(f"Connecting/Disconnecting {selected_devices[0].text}")
        else:
            print("No device selected")

class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass
class CustomLabelRoboto(MDLabel): pass # Pre-made class to define default settings to titles and subtitles labels with MDLabel
class CustomLabelAD(MDLabel): pass # Pre-made class to define default settings to titles and subtitles labels with MDLabel
class CustomTextEntry(MDTextField): pass # Pre-made class to define defaultl settings to entry text boxes

class TestDesignApp(MDApp):  
    #------------------------ Init Methods ------------------------#
    def __init__(self, **kwargs):
        '''Initializes all methods, initial logical setup and define attributes'''
        super().__init__(**kwargs)
        self.kv_loaded: bool = False

        # Detects OS running
        self.os_name = self.detect_os()
        self.pos_screen(0)

        # Colors dictionary used on the design file
        self.colors: dict = ColorManager()._get_colors()
        '''
        Available colors:
        Cyan, Dark Blue, Light Orange, Light Gray, Black, White.
        '''

        self.limb: str = ""
        self.motors_labels: dict[str] = {
            "Right leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Left leg": ["Hip Motor", "Knee Motor", "Ankle Motor"],
            "Right arm": ["motor1", "motor2", "motor3"],
            "Left arm": ["motor1", "motor2", "motor3"],
            
        }
 
    def build(self):
        """Loads kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True
        return self.root

    def on_start(self): # Method called at the begnnin of the class, just like __init__ and build. 
        self.root.current = "Main Window"
    
    #------------------------ Window Management ------------------------#
    def detect_os(self) -> str:
        '''Detects current OS of device and returns it as string'''
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
        
    def toggle_screen(self, state: bool) -> None:
        '''Toggle between fullscreen and windowed mode depending on OS or selected mode'''
        if self.os_name == "Linux" or self.os_name == "Windows" or not(state):
            Window.fullscreen = False
        if self.os_name == "Android" or state:
            print("Fullscreen")
            Window.fullscreen = True
    
    def pos_screen(self, screen: int) -> None:
        '''Position screen on desired window for debugging'''
        if screen == 0: # Main screen
            Window.left = 0
        elif screen == 1: # Secondary screen
            Window.left = 1920
        Window.top = 30 # Slightly under screen top

    #------------------------ Bluetooth tab methods ------------------------

    def bluetooth_connection(self): pass

    #------------------------ Assitance tab methods ------------------------

    def assitance_method(self): pass

    #------------------------ Tuning tab methods ------------------------

    def limb_dropdown_clicked(self, limb: str) -> None: 
        self.limb = limb
        print(self.limb)

    def on_entry_text(self, value: str) -> None: 
        print(value)
        # self.root.ids.bottom_nav.ids.tuning_tab.ids.
    
def main():
    '''Initializes the app indicating the current OS'''
    TestDesignApp().run()

if __name__ == '__main__':
    main()