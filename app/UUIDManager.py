import uuid

class UUIDManager:
    '''Clase para manejar los UUID de la ESP32. Maneja los UUIDs personalizados tanto en servicios como en características. 
    Para guardar los UUIDs se utiliza el formato de diccionario'''
    def __init__(self):

        # Se define la base para los UUIDs de los servicios
        self._base_uuid_services = uuid.UUID('00000000-0000-1000-8000-00805f9b34fb')

        # Se define la base para los UUIDs de las características
        self._base_uuid_chars = uuid.UUID('00000000-0000-1000-8000-00805f9b34fa')

        # Inicializa las variables
        self.uuids_services = {}
        self.uuids_chars = {}

    def generate_custom_uuid(self, base: uuid.UUID, short_uuid) -> uuid.UUID:
        '''
        Método para generar UUIDs personalizados
        Entrada: base UUID -> base para el UUID a generar
                 short_uuid -> caracterización para el UUID en los bytes 4 a 7

        Salida: UUID personalizado
        '''
        # Convertir el UUID base a una lista de bytes
        base_uuid_bytes = bytearray(base.bytes)

        # Convertir el short_uuid a bytes y reemplazar los bytes en la posición adecuada
        short_uuid_bytes = short_uuid.to_bytes(2, byteorder='big')
        base_uuid_bytes[2:4] = short_uuid_bytes

        # Convertir los bytes de nuevo a un UUID
        custom_uuid = uuid.UUID(bytes=bytes(base_uuid_bytes))
        return custom_uuid

    def generate_uuids_services(self, names: list[str], values: list) -> None:
        '''
        Método que genera las UUIDs de los servicios y los guarda en un diccionario
        Entrada: names -> lista de los nombres nombres de los servicios
                 values -> lista de los valores de los UUIDs de los servicios
        '''
        if not (len(names) == len(values)): 
            print("Los nombres y valores no tienen la misma longitud")
            return
        
        # Se generan los UUIDs según fueron pasados como parámetros
        # Se guardan como objetos de uuid.UUID
        for i in range(len(names)): 
            self.uuids_services[names[i]] = self.generate_custom_uuid(self._base_uuid_services, values[i])

    def generate_uuids_chars(self, service_uuid_name: str, names: list[str], values: list):
        '''
        Método que genera las UUIDs de las características de un servicio y las guarda en un diccionario
        Entrada: service_uuid_name -> UUID del servicio que contendrá las características
                 names -> lista de los nombres nombres de las características
                 values -> lista de los valores de los UUIDs de las características
        '''
        if not (len(names) == len(values)): 
            print("Los nombres y valores no tienen la misma longitud")
            return
        
        uuids_dict = {}
        # Se generan los UUIDs de las características para el servicio indicado 
        for i in range(len(names)): 
            uuids_dict[names[i]] = self.generate_custom_uuid(self._base_uuid_chars, values[i])
        self.uuids_chars[service_uuid_name] = uuids_dict

if __name__ == '__main__':
    '''Se generan UUIDs de prueba'''
    uuid_manager = UUIDManager()
    # Nombres de los servicios para manejo interno
    names = ["Parameters", "Commands", "Process"]
    values = [0x0001, 0x0002, 0x0003]
    # Se generan los UUIDs
    uuid_manager.generate_uuids_services(names, values)

    # --- Servicio de Parameters ---
    uuid_manager.generate_uuids_chars(names[0], ["PI"], [0x000a])
    uuid_manager.generate_uuids_chars(names[0], ["LEVEL"], [0x000d])
    # --- Servicio de Process ---
    uuid_manager.generate_uuids_chars(names[1], ["PV"], [0x000b])
    # --- Servicio de Commands ---
    uuid_manager.generate_uuids_chars(names[2], ["Mode"], [0x000c])

    print("Servicios")
    print(uuid_manager.uuids_services)
    print("\n")
    print("Características")
    print(uuid_manager.uuids_chars["Parameters"]["P"])