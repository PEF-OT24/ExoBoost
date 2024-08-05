class ColorManager():
    '''Class that manages colors used on the ExoBoost App for Android.'''
    def __init__(self):
        # Transparency values go from 0 to 255 (hex to binary)
        self.HexColors: dict = {
            "Cyan": {
                "color": "4FBDBA",
                "transparency": "255"
            },
            "Light Cyan":{
                "color": "80D6D4",
                "transparency": "255"
            },
            "Dark Cyan":{
                "color": "296362",
                "transparency": "255"
            },
            "Dark Blue": {
                "color": "1F3B4D",
                "transparency": "255"
            },
            "Blue":{
                "color": "5B7789",
                "transparency": "255"
            },
            "Light Orange": {
                "color": "FF7A59",
                "transparency": "255"
            },
            "Light Gray": {
                "color": "E5E5E5",
                "transparency": "255"
            },
            "Black": {
                "color": "000000",
                "transparency": "255"
            },
            "White": {
                "color": "FFFFFF",
                "transparency": "255"
            },
            "Red": {
                "color": "FF0000",
                "transparency": "255"
            },
            "Green": {
                "color": "50C878",
                "transparency": "255"
            },
        }

        # Creates a list of available colors names
        self.colors: list = list(self.HexColors.keys())
        
        # Normalizes all hex and transparency values to be usable by Kivy
        self.KivyColors = {}
        self.Convert_Colors()

    def hex_to_rgb_normalized(self, hex_value: list[str]) -> float:
        # Gets HEX value of each channel and converts them to decimal
        r = int(hex_value[0:2], 16)
        g = int(hex_value[2:4], 16)
        b = int(hex_value[4:6], 16)

        # Normalizes the values
        r_normalized = r / 255.0
        g_normalized = g / 255.0
        b_normalized = b / 255.0

        # Returns the normalized values
        return r_normalized, g_normalized, b_normalized

    def Convert_Colors(self) -> None:
        '''Converts all hex values to color tuples for kivy'''
        for col in self.colors:
            red, green, blue = self.hex_to_rgb_normalized(self.HexColors[col]["color"])
            transparency = int(self.HexColors[col]["transparency"])/255
            self.KivyColors[col] = (red, green, blue, transparency)
    
    def _get_colors(self) -> dict:
        '''Return the kivy colors dic'''
        return self.KivyColors
    
def main(): 
    test = ColorManager()._get_colors()
    print(test["Light Orange"])

if __name__ == '__main__':
    main()
