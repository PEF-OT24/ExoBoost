# Import kivy and kivymd libraries 
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from ColorManager import ColorManager

# Create multiple windows, main code will be located in main window
# SecundaryWindow (as well as new created) might contain differente or new functions to the app
class MainWindow(Screen): pass 
class SecundaryWindow(Screen): pass
class WindowManager(ScreenManager): pass

class TestDesignApp(MDApp):  
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.kv_loaded = False
        
        self.colors = ColorManager()._get_colors()
 
    def build(self):
        """ Configura diseño inicial de la aplicación, obtiene permisos para gps y ble y carga archivo de diseño kv"""
        # Loads kivy design file 
        if not(self.kv_loaded):
            self.root = Builder.load_file("test.kv")
            self.kv_loaded = True
        return self.root

    def on_start(self): # Equivalent to __init__ method on a normal class
        self.root.current = "Main Window"

def main():
    TestDesignApp().run()

if __name__ == '__main__':
    main()