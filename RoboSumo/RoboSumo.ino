// ===============================================
// Programa: Robô Sumô - Versão Estratégica 4.1
// Melhorias: IA de combate, memória tática, anti-travamento
// IMPORTANTE: 100% não-bloqueante usando millis()
// Arena: Branca com listras pretas (borda escura)
// CORREÇÕES: Detecção de borda e calibração não-bloqueante
// ===============================================

#include <AFMotor.h>
#include <Ultrasonic.h>

// ==== Sensores ====
const int sensorFrontalIR = A0;   // Pino digital para sensor IR frontal
const int sensorTraseiroIR = A1;  // Pino digital para sensor IR traseiro
const int trigPin = A4;
const int echoPin = A5;
const int botaoStart = 7;

// ==== Objetos ====
Ultrasonic ultrasonic(trigPin, echoPin);
AF_DCMotor mFE(1), mFD(2), mTE(3), mTD(4);

// ==== Constantes de Combate ====
const int VEL_PADRAO = 255;
const int VEL_ATAQUE_TURBO = 255;
const int VEL_BUSCA_RAPIDA = 200;
const int DIST_ATAQUE = 35;
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

// ==== Variáveis para calibração não-bloqueante ====
enum EstadoCalibracao {
  CALIB_INICIO,
  CALIB_AGUARDANDO,
  CALIB_COLETANDO,
  CALIB_CONCLUIDA
};
EstadoCalibracao estadoCalibracao = CALIB_INICIO;
unsigned long tInicioCalibracao = 0;
int contagemLeituras = 0;
long somaF = 0, somaT = 0;
int menorF = 1023, maiorF = 0;
int menorT = 1023, maiorT = 0;

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
  CORRIGINDO_TRAVAMENTO
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
void iniciarLeituraDistancia() {
  indiceDistancia = 0;
  leiturasValidas = 0;
  filtroCompleto = false;
  tInicioLeituraDistancia = millis();
}

bool atualizarLeituraDistancia(unsigned long agora) {
  if (indiceDistancia < 3) {
    if (agora - tInicioLeituraDistancia >= (indiceDistancia * INTERVALO_LEITURA_DIST)) {
      long d = ultrasonic.read();
      if (d > 0 && d < 200) {
        distancias[indiceDistancia] = d;
        leiturasValidas++;
      } else {
        distancias[indiceDistancia] = 0;
      }
      indiceDistancia++;

      if (indiceDistancia >= 3) {
        filtroCompleto = true;
      }
    }
  }
  return filtroCompleto;
}

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

// ==== FUNÇÕES DE BORDA CORRIGIDAS (DIGITAL) ====
// Sensores IR digitais:
// Arena BRANCA = HIGH (1) - reflete luz
// Borda PRETA = LOW (0) - absorve luz
bool bordaFrente() {
  // Lê estado digital: LOW = detectou preto (borda)
  bool detectou = (digitalRead(sensorFrontalIR) == 1);

  // Debug - descomente se necessário
  Serial.print("BORDA FRONTAL: ");
  Serial.print(digitalRead(sensorFrontalIR));
  Serial.print("BORDA TRASEIRA: ");
  Serial.print(digitalRead(sensorTraseiroIR));
  Serial.print(" | Borda FRONTAL: ");
  Serial.println(detectou ? "SIM" : "NAO");

  return detectou;
}

bool bordaTras() {
  // Lê estado digital: LOW = detectou preto (borda)
  bool detectou = (digitalRead(sensorTraseiroIR) == 1);

  Serial.print("BORDA FRONTAL: ");
  Serial.print(digitalRead(sensorFrontalIR));
  Serial.print("BORDA TRASEIRA: ");
  Serial.print(digitalRead(sensorTraseiroIR));
  Serial.print(" | Borda TRASEIRA: ");
  Serial.println(detectou ? "SIM" : "NAO");
  return detectou;
}

// ==== CALIBRAÇÃO NÃO NECESSÁRIA (SENSORES DIGITAIS) ====
// Sensores digitais já retornam HIGH/LOW, não precisam calibração
bool CalibrarSensores() {
  unsigned long agora = millis();

  switch (estadoCalibracao) {
    case CALIB_INICIO:
      Serial.println(F("Testando sensores digitais IR..."));
      tInicioCalibracao = agora;
      estadoCalibracao = CALIB_AGUARDANDO;
      return false;

    case CALIB_AGUARDANDO:
      if (agora - tInicioCalibracao >= 1000) {
        estadoCalibracao = CALIB_COLETANDO;
        tInicioCalibracao = agora;
        Serial.println(F("Verificando sensores..."));
      }
      return false;

    case CALIB_COLETANDO:
      if (agora - tInicioCalibracao >= 1000) {
        // Lê estado atual dos sensores
        int valF = digitalRead(sensorFrontalIR);
        int valT = digitalRead(sensorTraseiroIR);

        Serial.println(F("==== Teste de Sensores ===="));
        Serial.print(F("IR Frontal: "));
        Serial.println(valF == HIGH ? "BRANCO" : "PRETO");
        Serial.print(F("IR Traseiro: "));
        Serial.println(valT == HIGH ? "BRANCO" : "PRETO");
        Serial.println(F("==========================="));
        Serial.println(F("IMPORTANTE: Sensores devem estar em BRANCO na arena!"));
        Serial.println(F("Se estiverem em PRETO, ajuste a sensibilidade dos sensores."));

        estadoCalibracao = CALIB_CONCLUIDA;
      }
      return false;

    case CALIB_CONCLUIDA:
      return true;
  }

  return false;
}

// ==== Estratégia de busca inteligente ====
void executarBuscaInteligente(unsigned long agora) {
  unsigned long tempoNaBusca = agora - tempoBusca;

  // 1. Se viu oponente recentemente, tenta reencontrar rapidamente
  if (ultimaDirecaoOponente != DESCONHECIDA && (agora - tUltDeteccao) < 1000) {  // agora 1 segundo
    switch (ultimaDirecaoOponente) {
      case ESQUERDA: esquerda(VEL_BUSCA_RAPIDA); break;
      case DIREITA: direita(VEL_BUSCA_RAPIDA); break;
      case FRENTE: frente(VEL_BUSCA_RAPIDA); break;
    }
    return;
  }

  // 2. Alternância mais rápida dos padrões
  if (padraoAtual == 0) {
    int fator = map(tempoNaBusca % 1200, 0, 1200, 60, 140);
    setVel(VEL_BUSCA_RAPIDA, VEL_BUSCA_RAPIDA - fator);
    esquerda();

    // Troca para padrão 1 mais rápido
    if (tempoNaBusca > 1200) {
      padraoAtual = 1;
      tempoBusca = agora;
    }
  } else if (padraoAtual == 1) {
    direita(VEL_PADRAO);

    // Troca para padrão 0 mais rápido
    if (tempoNaBusca > 600) {
      padraoAtual = 2;  // Adiciona novo padrão: avanço rápido
      tempoBusca = agora;
    }
  }
  // 3. Padrão de avanço (curto e reativo)
  else if (padraoAtual == 2) {
    frente(VEL_BUSCA_RAPIDA);
    // Avança por 400ms, depois volta a girar
    if (tempoNaBusca > 400) {
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

// ==== Executa correção de travamento ====
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

// ==== SETUP ====
void setup() {
  Serial.begin(9600);

  pinMode(sensorFrontalIR, INPUT);
  pinMode(sensorTraseiroIR, INPUT);

  setVel(VEL_PADRAO, VEL_PADRAO);
  estadoCalibracao = CALIB_INICIO;

  Serial.println(F("=== ROBÔ SUMÔ 4.1 - MODO NÃO-BLOQUEANTE ==="));
  Serial.println(F("Iniciando calibração..."));
}

bool Iniciar() {
  static bool aguardandoBotao = true;
  static bool esperando = false;
  static unsigned long tInicio = 0;

  // Espera o botão ser pressionado (LOW = pressionado, se for INPUT_PULLUP)
  if (aguardandoBotao) {
    if (digitalRead(botaoStart) == 1) {
      aguardandoBotao = false;
      esperando = true;
      tInicio = millis();  // Marca o início da espera de 5 segundos
    }
    return false;
  }

  // Espera 5 segundos depois do botão
  if (esperando) {
    if (millis() - tInicio >= 5000) {
      esperando = false;  // Finaliza espera
      return true;        // Pronto para começar!
    }
    return false;  // Ainda esperando os 5 segundos
  }

  // Já iniciou
  return true;
}

// ==== LOOP PRINCIPAL ====
void loop() {
  unsigned long agora = millis();

  // ==== FASE 1: CALIBRAÇÃO ====
  if (estadoCalibracao != CALIB_CONCLUIDA) {
    CalibrarSensores();
    return;
  }

  // ==== FASE 2: AGUARDANDO BOTÃO E 5 SEGUNDOS ====
  static bool iniciouCombate = false; // Marca se já liberou o combate

  if (!iniciouCombate) {
    if (Iniciar()) { // Só libera quando apertou botão e passaram 5s
      iniciouCombate = true;
      roboAtivo = true;
      estadoAtual = PROCURANDO;
      tempoBusca = agora;
      tUltMudancaEstado = agora;
      iniciarLeituraDistancia();
      Serial.println(F("=== COMBATE INICIADO ==="));
    } else {
      // Aqui pode piscar LED, printar algo, etc, enquanto aguarda botão/start
      return; // Não faz nada até liberar
    }
  }

  // ==== FASE 3: COMBATE ====

  // Modo economia
  if ((agora - tUltMov) > TEMPO_INATIVIDADE && estadoAtual != ECONOMIA) {
    pararMotores();
    estadoAtual = ECONOMIA;
    Serial.println(F("Modo economia ativado"));
  }

  // Anti-travamento
  if (detectarTravamento(agora) && estadoAtual != CORRIGINDO_TRAVAMENTO) {
    estadoAtual = CORRIGINDO_TRAVAMENTO;
    subEstadoAtual = SUB_INICIO;
    Serial.println(F("Travamento detectado!"));
  }

  // Atualiza leitura de distância
  atualizarLeituraDistancia(agora);

  // Loop principal de sensores
  if (agora - tUltSensor >= INTERVALO_SENSOR) {
    tUltSensor = agora;

    long d = 0;
    if (filtroCompleto) {
      d = obterDistanciaFiltrada();
      iniciarLeituraDistancia();
    }

    bool bF = bordaFrente();
    bool bT = bordaTras();

    // Debug periódico
    if (contadorBuscaSemSucesso % 50 == 0) {
      Serial.print(F("Dist: "));
      Serial.print(d);
      Serial.print(F(" | Estado: "));
      Serial.println(estadoAtual);
    }

    switch (estadoAtual) {
      case PROCURANDO:
      case BUSCA_AGRESSIVA:
        if (bF || bT) {
          estadoAtual = AJUSTANDO;
          subEstadoAtual = SUB_INICIO;
          tUltMudancaEstado = agora;
        } else if (d > 0 && d < DIST_ATAQUE) {
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
        } else {
          executarBuscaInteligente(agora);
        }
        break;
      case ATACANDO:
        if (bF || bT) {
          estadoAtual = RECUANDO;
          subEstadoAtual = SUB_INICIO;
          tUltMudancaEstado = agora;
          break;
        }
        if (d > DIST_ATAQUE || d == 0) {
          Serial.println(F("Oponente perdido - busca agressiva"));
          estadoAtual = BUSCA_AGRESSIVA;
          tempoBusca = agora;
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
        if (bF || bT) {
          estadoAtual = RECUANDO;
          subEstadoAtual = SUB_INICIO;
          tUltMudancaEstado = agora;
          break;
        }
        if (d > DIST_ATAQUE_TURBO && d < DIST_ATAQUE) {
          estadoAtual = ATACANDO;
        } else if (d > DIST_ATAQUE || d == 0) {
          estadoAtual = BUSCA_AGRESSIVA;
          tempoBusca = agora;
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
        static unsigned long tUltBotaoEconomia = 0;
        if (agora - tUltBotaoEconomia > 50) {
          // Descomente para usar botão:
          // if (digitalRead(botaoStart) == 1) {
          estadoAtual = PROCURANDO;
          tempoBusca = agora;
          tUltMudancaEstado = agora;
          modoAgressivo = false;
          iniciarLeituraDistancia();
          Serial.println(F("Reativado do modo economia"));
          // }
          tUltBotaoEconomia = agora;
        }
        break;
      case AGUARDANDO:
      default:
        pararMotores();
        break;
    }
  }
}
