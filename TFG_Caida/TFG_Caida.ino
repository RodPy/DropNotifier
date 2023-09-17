void setup() {
// inicializa el pin digital 2 como salida
pinMode(ledPin, OUTPUT);
}
void loop() {
digitalWrite(ledPin, HIGH); // Enciende el LED
delay(1000); // Espera un segundo
digitalWrite(ledPin, LOW); // Apaga el LED
delay(1000); // Espera un segundo
}
