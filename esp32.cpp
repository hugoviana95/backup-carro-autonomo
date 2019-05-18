#include<Arduino.h>

#define PWM_IN_CH2 34
#define PWM_OUT_CH2 32
#define PWM_IN_CH1 33
#define PWM_OUT_CH1 26

//Variaveis do sinal de saída PWM do canal 1 (direção)
int pwm_tempo_ch1;
int freq_ch1 = 100;
int canal_ch1 = 0;
int resolucao_ch1 = 12;
int dutyCycle_ch1 = 0;

//Variaveis do sinal de saída PWM do canal 2 (velocidade)
int pwm_tempo_ch2;
int freq_ch2 = 100;
int canal_ch2 = 1;
int resolucao_ch2 = 12;
int dutyCycle_ch2 = 0;

void ch1(){
  pwm_tempo_ch1 = pulseIn(PWM_IN_CH1, HIGH);
  Serial.print("tON1:");
  Serial.println(pwm_tempo_ch1);
  dutyCycle_ch1 = (pwm_tempo_ch1*0.000001*freq_ch1*4095);
  Serial.print("dutycycle1:");
  Serial.println(dutyCycle_ch1);
  ledcWrite(canal_ch1, dutyCycle_ch1);
}

void ch2(){
  pwm_tempo_ch2 = pulseIn(PWM_IN_CH2, HIGH);
  Serial.print("tON2:");
  Serial.println(pwm_tempo_ch2);
  dutyCycle_ch2 = (pwm_tempo_ch2*0.000001*freq_ch2*4095);
  Serial.print("dutycycle2:");
  Serial.println(dutyCycle_ch2);
  ledcWrite(canal_ch2, dutyCycle_ch2);
}

void setup() {
  pinMode(PWM_IN_CH1, INPUT);
  pinMode(PWM_IN_CH2, INPUT);
  pinMode(PWM_OUT_CH1, OUTPUT);
  pinMode(PWM_OUT_CH2, OUTPUT);

  ledcSetup(canal_ch1, freq_ch1, resolucao_ch1);
  ledcAttachPin(PWM_OUT_CH1, canal_ch1);
  ledcWrite(canal_ch1, dutyCycle_ch1);

  ledcSetup(canal_ch2, freq_ch2, resolucao_ch2);
  ledcAttachPin(PWM_OUT_CH2, canal_ch2);
  ledcWrite(canal_ch2, dutyCycle_ch2);

  Serial.begin(115200);
}

void loop() {
  ch1();
  ch2();
}

