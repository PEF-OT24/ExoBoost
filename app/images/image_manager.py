from PIL import Image
import os

def convertir_png_a_bmp_monocromatica_sin_ruido(ruta_imagen_png, umbral=128):
    # Abre la imagen en PNG
    imagen = Image.open(ruta_imagen_png)
    
    # Convierte la imagen a escala de grises
    imagen_gris = imagen.convert('L')  # 'L' es para escala de grises (de 0 a 255)
    
    # Aplica un umbral para eliminar el ruido (binarización)
    imagen_binaria = imagen_gris.point(lambda p: 255 if p > umbral else 0, '1')
    
    # Obtiene el nombre base del archivo sin la extensión
    nombre_base = os.path.splitext(ruta_imagen_png)[0]
    
    # Genera el nombre del archivo de salida en formato BMP
    ruta_imagen_bmp = f"{nombre_base}.bmp"
    
    # Guarda la imagen en formato BMP
    imagen_binaria.save(ruta_imagen_bmp, format='BMP')
    
    print(f"Imagen convertida y guardada como: {ruta_imagen_bmp}")

# Ejemplo de uso
ruta_imagen_png = "LogoAppBB.png"
convertir_png_a_bmp_monocromatica_sin_ruido(ruta_imagen_png, umbral=128)
