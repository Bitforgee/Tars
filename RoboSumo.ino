// ===============================================
// Programa: Robô Sumô - Versão Estratégica 4.1
// Melhorias: IA de combate, memória tática, anti-travamento
// IMPORTANTE: 100% não-bloqueante usando millis()
// Arena: Branca com listras pretas (borda escura)
// ===============================================

#include <AFMotor.h>
#include <Ultrasonic.h>

// ==== Sensores ====
const int sensorFrontalIR = A0;
const int sensorTraseiroIR = A1;
const int trigPin = A4;
const int echoPin = A5;

// ==== Controle e Status ====
const int ledBusca = 8;
const int ledAtaque = 9;
const int ledRecuo = 10;
const int botaoStart = 7;

// ==== Objetos ====
Ultrasonic ultrasonic(trigPin, echoPin);
AF_DCMotor mFE(1), mFD(2), mTE(3), mTD(4);

// ==== Constantes de Combate ====
const int VEL_PADRAO = 255;
const int VEL_ATAQUE_TURBO = 255;
const int VEL_BUSCA_RAPIDA = 200;
const int DIST_ATAQUE = 30;
const int DIST_ATAQUE_TURBO = 15;
const unsigned long INTERVALO_SENSOR = 50;
const unsigned long TEMPO_MOV = 250;
const unsigned long TEMPO_GIRO_RAPIDO = 200;
const unsigned long TEMPO_RECUO_BORDA = 300;
const unsigned long TEMPO_INATIVIDADE = 8000;
const unsigned long TEMPO_ANTI_TRAVAMENTO = 2000;
const unsigned long INTERVALO_LEITURA_DIST = 10;  // Para filtro não-bloqueante
const float CORR_DIST = 0.98;

// ==== Calibração ====
int refFrente = 0;
int refTras = 0;
int margemFrente = 100;
int margemTras = 100;

// ==== Controle temporal ====
unsigned long tUltSensor = 0;
unsigned long tInicioAcao = 0;
unsigned long tUltMov = 0;
unsigned long tUltDeteccao = 0;
unsigned long tUltMudancaEstado = 0;
unsigned long tInicioLeituraDistancia = 0;
bool acaoEmAndamento = false;
bool roboAtivo = false;

// ==== Filtro de distância não-bloqueante ====
long distancias[3] = { 0, 0, 0 };
int indiceDistancia = 0;
int leiturasValidas = 0;
bool filtroCompleto = false;

// ==== Memória Tática ====
enum DirecaoOponente { FRENTE,
                       ESQUERDA,
                       DIREITA,
                       DESCONHECIDA };
DirecaoOponente ultimaDirecaoOponente = DESCONHECIDA;
int contadorBuscaSemSucesso = 0;
bool modoAgressivo = false;

// ==== Padrão de busca inteligente ====
int padraoAtual = 0;
unsigned long tempoBusca = 0;

// ==== Sub-estados para ações sequenciais não-bloqueantes ====
enum SubEstado {
  SUB_INICIO,
  SUB_RECUANDO,
  SUB_GIRANDO,
  SUB_FINALIZADO
};
SubEstado subEstadoAtual = SUB_INICIO;

// ==== Estados ====
enum EstadoRobo {
  AGUARDANDO,
  PROCURANDO,
  ATACANDO,
  ATAQUE_TURBO,
  RECUANDO,
  AJUSTANDO,
  ECONOMIA,
  BUSCA_AGRESSIVA,
  CORRIGINDO_TRAVAMENTO  // Novo: estado específico para anti-travamento
};
EstadoRobo estadoAtual = AGUARDANDO;

// ==== UTILITÁRIOS ====
void setVel(int vE, int vD) {
  vE = constrain(vE, 0, 255);
  vD = constrain(vD, 0, 255);
  mFE.setSpeed(vE);
  mTE.setSpeed(vE);
  mFD.setSpeed(vD);
  mTD.setSpeed(vD);
}

void pararMotores() {
  mFE.run(RELEASE);
  mFD.run(RELEASE);
  mTE.run(RELEASE);
  mTD.run(RELEASE);
}

void frente(int velocidade = VEL_PADRAO) {
  setVel(velocidade, velocidade);
  mFE.run(FORWARD);
  mFD.run(FORWARD);
  mTE.run(FORWARD);
  mTD.run(FORWARD);
  tUltMov = millis();
}

void tras(int velocidade = VEL_PADRAO) {
  setVel(velocidade, velocidade);
  mFE.run(BACKWARD);
  mFD.run(BACKWARD);
  mTE.run(BACKWARD);
  mTD.run(BACKWARD);
  tUltMov = millis();
}

void esquerda(int velocidade = VEL_PADRAO) {
  setVel(velocidade, velocidade);
  mFE.run(FORWARD);
  mTE.run(FORWARD);
  mFD.run(BACKWARD);
  mTD.run(BACKWARD);
  tUltMov = millis();
}

void direita(int velocidade = VEL_PADRAO) {
  setVel(velocidade, velocidade);
  mFE.run(BACKWARD);
  mTE.run(BACKWARD);
  mFD.run(FORWARD);
  mTD.run(FORWARD);
  tUltMov = millis();
}

// ==== FILTRO DE DISTÂNCIA NÃO-BLOQUEANTE ====
// Função para iniciar uma nova leitura de distância
void iniciarLeituraDistancia() {
  indiceDistancia = 0;
  leiturasValidas = 0;
  filtroCompleto = false;
  tInicioLeituraDistancia = millis();
}

// Função que coleta leituras de forma não-bloqueante
bool atualizarLeituraDistancia(unsigned long agora) {
  // Se ainda não completou as 3 leituras
  if (indiceDistancia < 3) {
    // Verifica se passou tempo suficiente desde a última leitura
    if (agora - tInicioLeituraDistancia >= (indiceDistancia * INTERVALO_LEITURA_DIST)) {
      long d = ultrasonic.read();
      if (d > 0 && d < 200) {
        distancias[indiceDistancia] = d;
        leiturasValidas++;
      } else {
        distancias[indiceDistancia] = 0;
      }
      indiceDistancia++;

      // Se completou as 3 leituras
      if (indiceDistancia >= 3) {
        filtroCompleto = true;
      }
    }
  }
  return filtroCompleto;
}

// Retorna a distância filtrada (média das leituras válidas)
long obterDistanciaFiltrada() {
  if (!filtroCompleto || leiturasValidas == 0) {
    return 0;
  }

  long soma = 0;
  for (int i = 0; i < 3; i++) {
    if (distancias[i] > 0) {
      soma += distancias[i];
    }
  }

  return (soma / leiturasValidas) * CORR_DIST;
}

// ==== Funções de Borda ====
bool bordaFrente() {
  return analogRead(sensorFrontalIR) < (refFrente - margemFrente);
}

bool bordaTras() {
  return analogRead(sensorTraseiroIR) < (refTras - margemTras);
}

// ==== LEDs ====
void leds() {
  digitalWrite(ledBusca, estadoAtual == PROCURANDO || estadoAtual == BUSCA_AGRESSIVA);
  digitalWrite(ledAtaque, estadoAtual == ATACANDO || estadoAtual == ATAQUE_TURBO);
  digitalWrite(ledRecuo, (estadoAtual == RECUANDO || estadoAtual == AJUSTANDO || estadoAtual == CORRIGINDO_TRAVAMENTO));
}

// ==== Calibração (único lugar onde delay() é aceitável - setup inicial) ====
void CalibrarSensores() {
  Serial.println(F("Calibrando sensores... mantenha o robô sobre a arena branca."));
  delay(2000);  // OK aqui: é só no setup, antes da luta começar

  long somaF = 0, somaT = 0;
  int menorF = 1023, maiorF = 0;
  int menorT = 1023, maiorT = 0;

  for (int i = 0; i < 50; i++) {
    int valF = analogRead(sensorFrontalIR);
    int valT = analogRead(sensorTraseiroIR);

    somaF += valF;
    somaT += valT;

    if (valF < menorF) menorF = valF;
    if (valF > maiorF) maiorF = valF;
    if (valT < menorT) menorT = valT;
    if (valT > maiorT) maiorT = valT;

    delay(20);  // OK aqui: é calibração inicial
  }

  refFrente = somaF / 50;
  refTras = somaT / 50;

  margemFrente = max(80, (maiorF - menorF) / 2);
  margemTras = max(80, (maiorT - menorT) / 2);

  Serial.println(F("==== Calibração concluída ===="));
  Serial.print(F("Ref. Frente: "));
  Serial.println(refFrente);
  Serial.print(F("Margem Frente: "));
  Serial.println(margemFrente);
  Serial.print(F("Ref. Trás: "));
  Serial.println(refTras);
  Serial.print(F("Margem Trás: "));
  Serial.println(margemTras);
  Serial.println(F("=============================="));
}

// ==== Estratégia de busca inteligente ====
void executarBuscaInteligente(unsigned long agora) {
  unsigned long tempoNaBusca = agora - tempoBusca;

  // Se perdeu o oponente recentemente, busca na última direção
  if (ultimaDirecaoOponente != DESCONHECIDA && (agora - tUltDeteccao) < 1500) {
    switch (ultimaDirecaoOponente) {
      case ESQUERDA:
        esquerda(VEL_BUSCA_RAPIDA);
        break;
      case DIREITA:
        direita(VEL_BUSCA_RAPIDA);
        break;
      case FRENTE:
        frente(VEL_BUSCA_RAPIDA);
        break;
    }
    return;
  }

  // Busca em espiral com variação de velocidade
  if (padraoAtual == 0) {
    int fator = map(tempoNaBusca % 3000, 0, 3000, 60, 150);
    setVel(VEL_BUSCA_RAPIDA, VEL_BUSCA_RAPIDA - fator);
    esquerda();

    if (tempoNaBusca > 4000) {
      padraoAtual = 1;
      tempoBusca = agora;
    }
  }
  // Varredura rápida
  else if (padraoAtual == 1) {
    direita(VEL_PADRAO);

    if (tempoNaBusca > 2000) {
      padraoAtual = 0;
      tempoBusca = agora;
    }
  }

  contadorBuscaSemSucesso++;
}

// ==== Detecta travamento ====
bool detectarTravamento(unsigned long agora) {
  return (agora - tUltMudancaEstado) > TEMPO_ANTI_TRAVAMENTO
         && (estadoAtual == PROCURANDO || estadoAtual == BUSCA_AGRESSIVA);
}

// ==== Executa correção de travamento (NÃO-BLOQUEANTE) ====
void executarCorrecaoTravamento(unsigned long agora) {
  switch (subEstadoAtual) {
    case SUB_INICIO:
      tras(VEL_PADRAO);
      tInicioAcao = agora;
      subEstadoAtual = SUB_RECUANDO;
      Serial.println(F("Anti-travamento: recuando"));
      break;

    case SUB_RECUANDO:
      if (agora - tInicioAcao >= TEMPO_RECUO_BORDA) {
        direita(VEL_PADRAO);
        tInicioAcao = agora;
        subEstadoAtual = SUB_GIRANDO;
        Serial.println(F("Anti-travamento: girando"));
      }
      break;

    case SUB_GIRANDO:
      if (agora - tInicioAcao >= 400) {
        padraoAtual = (padraoAtual + 1) % 2;
        subEstadoAtual = SUB_FINALIZADO;
        Serial.println(F("Anti-travamento: concluído"));
      }
      break;

    case SUB_FINALIZADO:
      estadoAtual = modoAgressivo ? BUSCA_AGRESSIVA : PROCURANDO;
      tempoBusca = agora;
      tUltMudancaEstado = agora;
      subEstadoAtual = SUB_INICIO;
      break;
  }
}

// ==== Setup ====
void setup() {
  Serial.begin(9600);

  pinMode(sensorFrontalIR, INPUT);
  pinMode(sensorTraseiroIR, INPUT);
  //pinMode(botaoStart, INPUT_PULLUP);
  pinMode(ledBusca, OUTPUT);
  pinMode(ledAtaque, OUTPUT);
  pinMode(ledRecuo, OUTPUT);

  setVel(VEL_PADRAO, VEL_PADRAO);
  CalibrarSensores();

  Serial.println(F("Pressione o botão para iniciar..."));
  Serial.println(F("=== MODO ESTRATÉGICO 4.1 (100% NÃO-BLOQUEANTE) ==="));
}

// ==== Loop Principal ====
void loop() {
  unsigned long agora = millis();

  // ==== START ====
  // if (!roboAtivo && digitalRead(botaoStart) == LOW) {
  if (!roboAtivo) {
    // Debounce não-bloqueante simples
    static unsigned long tUltBotao = 0;
    if (agora - tUltBotao > 50) {
      //if (digitalRead(botaoStart) == LOW) {
      roboAtivo = true;
      estadoAtual = PROCURANDO;
      tempoBusca = agora;
      tUltMudancaEstado = agora;
      iniciarLeituraDistancia();
      Serial.println(F("=== COMBATE INICIADO ==="));
      // }
      tUltBotao = agora;
    }
  }

  if (!roboAtivo) {
    digitalWrite(ledBusca, (millis() / 300) % 2);
    return;
  }

  // ==== ECONOMIA ====
  if ((agora - tUltMov) > TEMPO_INATIVIDADE && estadoAtual != ECONOMIA) {
    pararMotores();
    estadoAtual = ECONOMIA;
    Serial.println(F("Modo economia ativado"));
  }

  // ==== Anti-travamento ====
  if (detectarTravamento(agora) && estadoAtual != CORRIGINDO_TRAVAMENTO) {
    estadoAtual = CORRIGINDO_TRAVAMENTO;
    subEstadoAtual = SUB_INICIO;
    Serial.println(F("Travamento detectado!"));
  }

  // ==== Atualiza leitura de distância não-bloqueante ====
  atualizarLeituraDistancia(agora);

  // ==== LOOP PRINCIPAL ====
  if (agora - tUltSensor >= INTERVALO_SENSOR) {
    tUltSensor = agora;

    // Se o filtro está completo, obtém a distância e reinicia
    long d = 0;
    if (filtroCompleto) {
      d = obterDistanciaFiltrada();
      iniciarLeituraDistancia();  // Inicia novo ciclo de leituras
    }

    bool bF = bordaFrente();
    bool bT = bordaTras();
    leds();

    // Debug
    if (contadorBuscaSemSucesso % 50 == 0) {
      Serial.print(F("Dist: "));
      Serial.print(d);
      Serial.print(F(" | Estado: "));
      Serial.println(estadoAtual);
    }

    switch (estadoAtual) {
      case PROCURANDO:
      case BUSCA_AGRESSIVA:
        if (d > 0 && d < DIST_ATAQUE) {
          tUltDeteccao = agora;
          ultimaDirecaoOponente = FRENTE;
          modoAgressivo = true;
          contadorBuscaSemSucesso = 0;

          if (d < DIST_ATAQUE_TURBO) {
            estadoAtual = ATAQUE_TURBO;
            Serial.println(F(">>> ATAQUE TURBO <<<"));
          } else {
            estadoAtual = ATACANDO;
            Serial.println(F(">>> Oponente detectado! <<<"));
          }
          tUltMudancaEstado = agora;
        } else if (bF || bT) {
          estadoAtual = AJUSTANDO;
          subEstadoAtual = SUB_INICIO;
          tUltMudancaEstado = agora;
        } else {
          executarBuscaInteligente(agora);
        }
        break;

      case ATACANDO:
        if (d > DIST_ATAQUE || d == 0) {
          Serial.println(F("Oponente perdido - busca agressiva"));
          estadoAtual = BUSCA_AGRESSIVA;
          tempoBusca = agora;
          tUltMudancaEstado = agora;
        } else if (bF || bT) {
          estadoAtual = RECUANDO;
          subEstadoAtual = SUB_INICIO;
          tUltMudancaEstado = agora;
        } else {
          if (d < DIST_ATAQUE_TURBO) {
            estadoAtual = ATAQUE_TURBO;
          } else {
            int vel = map(d, 0, DIST_ATAQUE, VEL_PADRAO, VEL_PADRAO - 50);
            frente(vel);
          }
        }
        break;

      case ATAQUE_TURBO:
        if (d > DIST_ATAQUE_TURBO && d < DIST_ATAQUE) {
          estadoAtual = ATACANDO;
        } else if (d > DIST_ATAQUE || d == 0) {
          estadoAtual = BUSCA_AGRESSIVA;
          tempoBusca = agora;
          tUltMudancaEstado = agora;
        } else if (bF || bT) {
          estadoAtual = RECUANDO;
          subEstadoAtual = SUB_INICIO;
          tUltMudancaEstado = agora;
        } else {
          frente(VEL_ATAQUE_TURBO);
          tUltDeteccao = agora;
        }
        break;

      case RECUANDO:
        switch (subEstadoAtual) {
          case SUB_INICIO:
            tras(VEL_PADRAO);
            tInicioAcao = agora;
            subEstadoAtual = SUB_RECUANDO;
            break;

          case SUB_RECUANDO:
            if (agora - tInicioAcao >= TEMPO_RECUO_BORDA) {
              // Gira para o lado oposto da última detecção
              if (ultimaDirecaoOponente == ESQUERDA) {
                direita(VEL_PADRAO);
              } else {
                esquerda(VEL_PADRAO);
              }
              tInicioAcao = agora;
              subEstadoAtual = SUB_GIRANDO;
            }
            break;

          case SUB_GIRANDO:
            if (agora - tInicioAcao >= TEMPO_GIRO_RAPIDO) {
              estadoAtual = modoAgressivo ? BUSCA_AGRESSIVA : PROCURANDO;
              tempoBusca = agora;
              tUltMudancaEstado = agora;
              subEstadoAtual = SUB_INICIO;
            }
            break;
        }
        break;

      case AJUSTANDO:
        switch (subEstadoAtual) {
          case SUB_INICIO:
            if (bF) {
              tras(VEL_PADRAO);
            } else if (bT) {
              frente(VEL_PADRAO);
            }
            tInicioAcao = agora;
            subEstadoAtual = SUB_RECUANDO;
            break;

          case SUB_RECUANDO:
            if (agora - tInicioAcao >= TEMPO_MOV) {
              estadoAtual = modoAgressivo ? BUSCA_AGRESSIVA : PROCURANDO;
              tempoBusca = agora;
              tUltMudancaEstado = agora;
              subEstadoAtual = SUB_INICIO;
            }
            break;
        }
        break;

      case CORRIGINDO_TRAVAMENTO:
        executarCorrecaoTravamento(agora);
        break;

      case ECONOMIA:
        pararMotores();
        //if (digitalRead(botaoStart) == LOW) {
        static unsigned long tUltBotaoEconomia = 0;
        if (agora - tUltBotaoEconomia > 50) {
          //if (digitalRead(botaoStart) == LOW) {
          estadoAtual = PROCURANDO;
          tempoBusca = agora;
          tUltMudancaEstado = agora;
          modoAgressivo = false;
          iniciarLeituraDistancia();
          Serial.println(F("Reativado do modo economia"));
          //}
          tUltBotaoEconomia = agora;
        }
        // }
        break;

      case AGUARDANDO:
      default:
        pararMotores();
        break;
    }
  }
}