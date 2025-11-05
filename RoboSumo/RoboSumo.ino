#include <AFMotor.h>
#include <Ultrasonic.h>

const int trigPin = A4;
const int echoPin = A5;
const int sensorFrontalIR = A0;
const int sensorTraseiroIR = A1;

const int DELAY_INICIO = 5000;
const int DIST_DETECCAO_OPONENTE = 40;
const int VELOCIDADE_BUSCA = 240;
const int VELOCIDADE_NORMAL = 200;
const int VELOCIDADE_ATAQUE = 255;

Ultrasonic ultrasonic(trigPin, echoPin);
AF_DCMotor mFE(1), mFD(2), mTE(3), mTD(4);

boolean bordaDetetada = false;
boolean oponenteDetectado = false;
int distanciaOponente = 0;

unsigned long tempoInicio = 0;
unsigned long tempoRecuo = 0;
unsigned long tempoAtaque = 0;
unsigned long tempoGiro = 0;
boolean sistemaIniciado = false;

void setup() {
  tempoInicio = millis();
}

void loop() {
  if (!sistemaIniciado && (millis() - tempoInicio) >= DELAY_INICIO) {
    sistemaIniciado = true;
  }
  
  if (!sistemaIniciado) {
    return;
  }
  
  verificarBordas();
  
  if (bordaDetetada) {
    recuarDaArena();
    if (tempoRecuo == 0) {
      bordaDetetada = false;
    }
  }
  else {
    distanciaOponente = medirDistancia();
    
    if (distanciaOponente < DIST_DETECCAO_OPONENTE && distanciaOponente > 0) {
      atacarOponente();
    } else {
      buscarOponente();
    }
  }
}

void verificarBordas() {
  int leituraTraseiraIR = digitalRead(sensorTraseiroIR);
  int leituraFrontalIR = digitalRead(sensorFrontalIR);
  
  if (leituraFrontalIR == 1) {
    bordaDetetada = true;
  }
  
  if (leituraTraseiraIR == 1) {
    bordaDetetada = true;
  }
}

int medirDistancia() {
  return ultrasonic.read();
}

void moveFrente(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  mFE.run(FORWARD);
  mFD.run(FORWARD);
  mTE.run(FORWARD);
  mTD.run(FORWARD);
}

void moveRe(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  mFE.run(BACKWARD);
  mFD.run(BACKWARD);
  mTE.run(BACKWARD);
  mTD.run(BACKWARD);
}

void giraEsquerda(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  mFE.run(BACKWARD);
  mFD.run(BACKWARD);
  mTE.run(FORWARD);
  mTD.run(FORWARD);
}

void giraDireita(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  mFE.run(FORWARD);
  mFD.run(FORWARD);
  mTE.run(BACKWARD);
  mTD.run(BACKWARD);
}

void parar() {
  mFE.run(RELEASE);
  mFD.run(RELEASE);
  mTE.run(RELEASE);
  mTD.run(RELEASE);
}

void recuarDaArena() {
  unsigned long tempoAtualRecuo = millis();
  
  if (tempoRecuo == 0) {
    tempoRecuo = tempoAtualRecuo;
    moveRe(VELOCIDADE_NORMAL);
  }
  
  if ((tempoAtualRecuo - tempoRecuo) < 400) {
    moveRe(VELOCIDADE_NORMAL);
  }
  else if ((tempoAtualRecuo - tempoRecuo) < 700) {
    giraEsquerda(VELOCIDADE_NORMAL);
  }
  else {
    parar();
    tempoRecuo = 0;
  }
}

void atacarOponente() {
  verificarBordas();
  
  if(bordaDetetada) {
    parar();
    tempoAtaque = 0;
    return;
  }
  
  if (tempoAtaque == 0) {
    tempoAtaque = millis();
  }
  
  moveFrente(VELOCIDADE_ATAQUE);
}

void buscarOponente() {
  unsigned long tempoAtualGiro = millis();
  
  if (tempoGiro == 0) {
    tempoGiro = tempoAtualGiro;
  }
  
  if ((tempoAtualGiro - tempoGiro) < 150) {
    giraDireita(VELOCIDADE_BUSCA);
  }
  else if ((tempoAtualGiro - tempoGiro) < 200) {
    parar();
  }
  else {
    tempoGiro = 0;
  }
}
