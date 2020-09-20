#define PIN_LED 7
unsigned int toggle = 0;

void setup() {
  pinMode(PIN_LED, 1);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
}

void loop() {
    //최초 setup이 두번 call돼서 LED가 한번 켜졌다 꺼진 후 loop가 실행되는 문제가 발생했는데,
    //하드웨어 문제일 수 있다는 답변을 찾아서 코드는 그대로 제출하겠습니다.
  
    digitalWrite(PIN_LED, 0); // 최초 켜진 상태로 1초 지속
    delay(1000);
    int cnt = 11; // 마지막에 꺼진 상태로 while을 마무리 하기 위해 cnt를 11로 설정
    while(cnt--){
      toggle = toggle_state(toggle);
      digitalWrite(PIN_LED, toggle); // update LED status.
      delay(100); 
   }
   while(1){} // 실행을 마치고 무한루프
}

int toggle_state(int toggle) {
  return toggle == 0 ? 1 : 0;
}
