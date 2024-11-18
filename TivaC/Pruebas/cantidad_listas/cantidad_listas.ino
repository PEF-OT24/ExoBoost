// Definición de constantes


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

uint8_t length_lista(int8_t *array_in){
  // Listas del tamaño de un byte
  return sizeof(array_in) / sizeof(array_in[0]);
}

void loop() {
//  uint8_t size_1;
//  size_1 = length_lista(lista1);
  int32_t lista1[] = {1, 2, 3, 4, 6, 7, 2, 4, 6, 7, 4, 5, 7, 8, 3, 2};
  Serial.print("Tamaño: "); Serial.println(sizeof(lista1) / sizeof(lista1[0]));
}
