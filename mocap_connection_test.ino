//Mocap Pin
const int en_mocap_Pin = 13;
bool isMocapSignal = false;

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication
  Serial.begin(115200);
  //Setup for mocap
    pinMode(en_mocap_Pin, INPUT_PULLDOWN);
    delay(100);
    Serial.println("ready..");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!digitalRead(en_mocap_Pin)) {}
  while(digitalRead(en_mocap_Pin)) {
    isMocapSignal = true;
  }

  while(isMocapSignal) {
    Serial.println("Received mocap signals!");
  }
}
