// ===============================================
// Programa: Robô Sumô - Estrutura Base
// Objetivo: Básico - Permanecer na arena e buscar oponente
// ===============================================

#include <AFMotor.h>
#include <Ultrasonic.h>

// ==== Pinos ====
const int trigPin = A4;
const int echoPin = A5;
const int sensorFrontalIR = A0;      // Digital
const int sensorTraseiroIR = A1;     // Digital

// ==== Constantes de Comportamento ====
const int DELAY_INICIO = 5000;           // 5 segundos de espera inicial
const int DIST_DETECCAO_OPONENTE = 40;   // 20 cm para detectar oponente// Leitura elevada = fora da arena
const int VELOCIDADE_NORMAL = 200;       // Velocidade padrão dos motores
const int VELOCIDADE_ATAQUE = 255;       // Velocidade máxima para ataque

// ==== Objetos ====
Ultrasonic ultrasonic(trigPin, echoPin);
AF_DCMotor mFE(1), mFD(2), mTE(3), mTD(4);  // Frente E, Frente D, Trás E, Trás D

// ==== Variáveis de Estado ====
boolean bordaDetetada = false;
boolean oponenteDetectado = false;
int distanciaOponente = 0;

// ==== Variáveis de Tempo (millis) ====
unsigned long tempoInicio = 0;
unsigned long tempoRecuo = 0;
unsigned long tempoAtaque = 0;
unsigned long tempoGiro = 0;
boolean sistemaIniciado = false;

// ==== SETUP ====
void setup() {
  Serial.begin(9600);
  
  // Inicia contagem para aguardar 5 segundos
  tempoInicio = millis();
  Serial.println("Sistema iniciando em 5 segundos...");
}

// ==== LOOP PRINCIPAL ====
void loop() {
  // Verifica se atingiu 5 segundos para iniciar
  if (!sistemaIniciado && (millis() - tempoInicio) >= DELAY_INICIO) {
    sistemaIniciado = true;
    Serial.println("Robô ativo!");
  }
  
  // Sistema desligado até completar 5 segundos
  if (!sistemaIniciado) {
    return;
  }
  
  // Verifica bordas (sensores IR traseiros)
  verificarBordas();
  
  // Se detectou borda, recua para dentro da arena
  if (bordaDetetada) {
    recuarDaArena();
    // Só zera após recuo completo
    if (tempoRecuo == 0) {
      bordaDetetada = false;
    }
  }
  // Caso contrário, busca oponente
  else {
    distanciaOponente = medirDistancia();
    
    if (distanciaOponente < DIST_DETECCAO_OPONENTE && distanciaOponente > 0) {
      // Oponente detectado: atacar
      atacarOponente();
    } else {
      // Nenhum oponente: buscar girando
      buscarOponente();
    }
  }
}

// ================================================
// FUNÇÕES DE DETECÇÃO
// ================================================

/**
* Verifica se o robô está chegando perto da borda (faixa preta)
* Sensores IR traseiros detectam contraste com a superfície branca
*/
void verificarBordas() {
  int leituraTraseiraIR = digitalRead(sensorTraseiroIR);
  
  int leituraFrontalIR = digitalRead(sensorFrontalIR);
  
  // Valor 1 = borda detectada (superfície preta absorve IR)
  if (leituraFrontalIR == 1) {
    bordaDetetada = true;
    Serial.println("⚠️ BORDA FRONTAL DETECTADA!");
  }
  
  if (leituraTraseiraIR == 1) {
    bordaDetetada = true;
    Serial.println("⚠️ BORDA TRASEIRA DETECTADA!");
  }
}

/**
* Mede a distância do oponente usando sensor ultrassônico
* Retorna distância em cm, ou -1 se não conseguir medir
*/
int medirDistancia() {
  int distancia = ultrasonic.read();
  
  // Debug: mostrar leitura a cada 10 ciclos
  static int contador = 0;
  if (++contador >= 10) {
    Serial.print("Distância: ");
    Serial.print(distancia);
    Serial.println(" cm");
    contador = 0;
  }
  
  return distancia;
}

// ================================================
// FUNÇÕES DE MOVIMENTO
// ================================================

/**
* Move o robô para frente na velocidade normal
*/
void moveFrente(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  mFE.run(FORWARD); // MOTOR FRENTE ESQUERDA
  mFD.run(FORWARD); // MOTOR FRENTE DIREITA
  mTE.run(FORWARD); // MOTOR TRASEIRA ESQUERDA
  mTD.run(FORWARD); // MOTOR TRASEIRA DIREITA
}

/**
* Recua o robô para trás
*/
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

/**
* Gira o robô para a esquerda no próprio eixo
* Lado esquerdo recua, lado direito avança
*/
void giraEsquerda(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  // Direita recua, esquerda avança
  mFE.run(BACKWARD);
  mFD.run(BACKWARD);
  mTE.run(FORWARD);
  mTD.run(FORWARD);
}

/**
* Gira o robô para a direita no próprio eixo
* Lado direito recua, lado esquerdo avança
*/
void giraDireita(int velocidade = VELOCIDADE_NORMAL) {
  mFE.setSpeed(velocidade);
  mTE.setSpeed(velocidade);
  mFD.setSpeed(velocidade);
  mTD.setSpeed(velocidade);
  
  // Direita recua, esquerda avança
  mFE.run(FORWARD);
  mFD.run(FORWARD);
  mTE.run(BACKWARD);
  mTD.run(BACKWARD);
}

/**
* Para todos os motores
*/
void parar() {
  mFE.run(RELEASE);
  mFD.run(RELEASE);
  mTE.run(RELEASE);
  mTD.run(RELEASE);
}

// ================================================
// FUNÇÕES DE COMPORTAMENTO
// ================================================

/**
* Quando detecta borda, recua e gira para voltar à arena
*/
void recuarDaArena() {
  unsigned long tempoAtualRecuo = millis();
  
  // Se não iniciou o recuo, inicializa
  if (tempoRecuo == 0) {
    Serial.println("🔄 Reculando para dentro da arena...");
    tempoRecuo = tempoAtualRecuo;
    moveRe(VELOCIDADE_NORMAL);
  }
  
  // Recua por 400ms
  if ((tempoAtualRecuo - tempoRecuo) < 400) {
    moveRe(VELOCIDADE_NORMAL);
  }
  // Gira por 300ms
  else if ((tempoAtualRecuo - tempoRecuo) < 700) {
    giraEsquerda(VELOCIDADE_NORMAL);
  }
  // Finaliza
  else {
    parar();
    tempoRecuo = 0;  // Reset para próxima detecção
  }
}

/**
* Detectou oponente: ataca em linha reta com máxima velocidade
* Verifica borda continuamente durante o ataque
*/
void atacarOponente() {
  unsigned long tempoAtualAtaque = millis();
  
  // ⭐ Verifica borda SEMPRE durante o ataque
  verificarBordas();
  
  if(bordaDetetada) {
    Serial.println("🚨 BORDA DETECTADA DURANTE ATAQUE!");
    parar();
    tempoAtaque = 0;  // Cancela o ataque
    // O loop principal cuidará do recuo
    return;
  }
  
  // Se não iniciou o ataque, inicializa
  if (tempoAtaque == 0) {
    Serial.println("🎯 ATACANDO OPONENTE!");
    tempoAtaque = tempoAtualAtaque;
  }
  // Ataca por 500ms
  else if ((tempoAtualAtaque - tempoAtaque) < 500) {
    moveFrente(VELOCIDADE_ATAQUE);
  }
  else {
    parar();
    tempoAtaque = 0;  // Reset para próximo ataque
  }
}

/**
* Nenhum oponente detectado: gira e busca
*/
void buscarOponente() {
  unsigned long tempoAtualGiro = millis();
  
  // Se não iniciou o giro, inicializa
  if (tempoGiro == 0) {
    Serial.println("🔍 Buscando oponente...");
    tempoGiro = tempoAtualGiro;
  }
  
  // Gira por 200ms
  if ((tempoAtualGiro - tempoGiro) < 200) {
    giraDireita(VELOCIDADE_NORMAL);
  }
  // Para por 100ms
  else if ((tempoAtualGiro - tempoGiro) < 300) {
    parar();
  }
  // Finaliza ciclo
  else {
    tempoGiro = 0;  // Reset para próximo giro
  }
}
