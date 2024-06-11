# Set the screen size and mode
from kivy.config import Config
Config.set('graphics', 'width', '540')
Config.set('graphics', 'height', '926')
Config.set('graphics', 'fullscreen', '0')

# Import kivy and kivymd libraries 
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
from ColorManager import ColorManager
import platform

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain differente or new functions to the app
class MainWindow(Screen): pass 
class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass

class TestDesignApp(MDApp):  
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
 
    def build(self):
        """Loads kivy design file"""
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True
        return self.root

    def on_start(self): # Method called at the begnnin of the class, just like __init__ and build. 
        self.root.current = "Main Window"

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

def main():
    '''Initializes the app indicating the current OS'''
    TestDesignApp().run()

if __name__ == '__main__':
    main()