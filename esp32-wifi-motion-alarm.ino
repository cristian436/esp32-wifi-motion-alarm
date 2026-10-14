// Ho dovuto mettere degli asterischi al posto dei dati sensibili (secret data)
// in seguito alla pubblicazione di questo file, quindi non sarà eseguibile così com'è.

/*
  ======================================================================
  ALLARME WIFI (RSSI sensing) per ESP32 - versione robusta
  ----------------------------------------------------------------------
  Obiettivo: rilevare movimento dalle variazioni del segnale WiFi
  SENZA falsi allarmi quando non si muove nulla, MANTENENDO la
  sensibilita' al movimento reale.

  Differenze chiave rispetto alla versione originale:
   1) Validazione del campione + filtro di Hampel (mediana/MAD):
      i glitch (es. il vecchio fallback a -100) NON entrano piu' nel calcolo.
   2) Nessuna soglia "magica": la calibrazione misura media e rumore
      reale dell'ambiente. La metrica e' un PUNTEGGIO NORMALIZZATO
      (spread / sigma_rumore). In ambiente quieto resti sensibile;
      in ambiente rumoroso non scatti, perche' tutto e' relativo al
      rumore misurato.
   3) Baseline adattiva (EMA) aggiornata SOLO quando e' quieto:
      segue la deriva lenta (temperatura, gestione potenza dell'AP,
      interferenze 2.4 GHz) senza perdere sensibilita'.
   4) WiFi.setSleep(false) + micro-traffico UDP al gateway ad ogni
      campione: l'RSSI viene aggiornato davvero (niente valori "vecchi"
      / aliasing legato ai beacon).
   5) DUAL-CORE: il campionamento gira in un task FreeRTOS ad alta
      priorita' su core 1 con cadenza rigida (vTaskDelayUntil); il
      Telegram/TLS (bloccante) gira nel loop su core 0 -> il blocco di
      rete non corrompe piu' la cadenza delle misure.
   6) Telegram: gestione corretta di piu' messaggi, comandi accettati
      SOLO dal CHAT_ID autorizzato, riconnessione WiFi automatica.

  NOTA SICUREZZA: il token del bot e' un segreto. Se lo hai condiviso,
  rigeneralo con @BotFather.
  ======================================================================
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// ===== CONFIGURAZIONE =====
const char* WIFI_SSID     = "********";
const char* WIFI_PASSWORD = "********";
const String BOT_TOKEN    = "********";
const String CHAT_ID      = "********";

// ===== PARAMETRI DI CAMPIONAMENTO =====
const int      SAMPLE_MS              = 50;     // 20 Hz: campiona ogni 50 ms (cadenza rigida)
const int      CAMPIONI_CALIBRAZIONE  = 200;    // ~10 s di calibrazione
const int      WINDOW                 = 20;     // finestra di analisi (~1 s)
const int      HAMPEL_WIN             = 7;      // finestra del filtro anti-outlier (dispari)

// ===== PARAMETRI DI RILEVAMENTO =====
// Punteggio = spread_corrente / sigma_rumore. In quiete vale ~1.
const float    K_ENTER                = 3.0;    // soglia di allarme (alza = meno sensibile)
const float    K_QUIET                = 1.8;    // sotto questo valore l'ambiente e' "quieto"
const int      CONFERME_NECESSARIE    = 6;      // campioni FILTRATI consecutivi sopra soglia
const float    SIGMA_FLOOR            = 0.6;    // limite minimo del rumore (RSSI e' quantizzato a 1 dB)
const float    MAX_JUMP_DB            = 6.0;    // salto massimo plausibile tra campioni "veri"
const float    EMA_ALPHA             = 0.01;   // velocita' di adattamento baseline (lenta)

// ===== PARAMETRI NOTIFICA/TELEGRAM =====
const unsigned long COOLDOWN_NOTIFICA_MS   = 10000;
const unsigned long INTERVALLO_TELEGRAM_MS = 2000;

// ===== OGGETTI RETE =====
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);
WiFiUDP udp;

// ===== STATO CONDIVISO TRA I CORE =====
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool          g_allarmAttivo = true;
volatile bool          g_linkUp       = false;
volatile bool          g_motionPending = false;
volatile float         g_lastRssi     = -100;
volatile float         g_lastScore    = 1.0;
volatile float         g_sigma0       = SIGMA_FLOOR;
volatile float         g_baseline     = -60;

// ===== BUFFER (usati solo dal task di sensing, core 1) =====
float hraw[HAMPEL_WIN];      // storico campioni validati (per Hampel)
int   hrawCount = 0;
int   hrawIdx   = 0;

float win[WINDOW];           // finestra di analisi (campioni filtrati)
int   winIdx   = 0;
bool  winFull  = false;

float lastValid = -100;      // ultimo RSSI valido (per sostituire i campioni invalidi)
int   conferme  = 0;
unsigned long ultimaNotifica = 0;

// ---------------------------------------------------------------------
// Utility statistiche
// ---------------------------------------------------------------------
float medianOf(const float* src, int n) {
  float b[HAMPEL_WIN];
  for (int i = 0; i < n; i++) b[i] = src[i];
  for (int i = 1; i < n; i++) {          // insertion sort (n piccolo)
    float k = b[i]; int j = i - 1;
    while (j >= 0 && b[j] > k) { b[j + 1] = b[j]; j--; }
    b[j + 1] = k;
  }
  if (n & 1) return b[n / 2];
  return 0.5f * (b[n / 2 - 1] + b[n / 2]);
}

// Filtro di Hampel: se il nuovo campione e' un outlier rispetto alla
// mediana locale (oltre 3*MAD oppure oltre MAX_JUMP_DB), lo sostituisce
// con la mediana. Cosi' un singolo glitch (es. -100) non inquina nulla.
float hampelClean(float v) {
  hraw[hrawIdx] = v;
  hrawIdx = (hrawIdx + 1) % HAMPEL_WIN;
  if (hrawCount < HAMPEL_WIN) hrawCount++;

  if (hrawCount < 3) return v;           // non abbastanza storia: passa cosi'

  float med = medianOf(hraw, hrawCount);
  float devs[HAMPEL_WIN];
  for (int i = 0; i < hrawCount; i++) devs[i] = fabsf(hraw[i] - med);
  float mad = medianOf(devs, hrawCount);

  float allowed = 3.0f * 1.4826f * mad;  // ~3 sigma robusti
  if (allowed < MAX_JUMP_DB) allowed = MAX_JUMP_DB;

  if (fabsf(v - med) > allowed) return med;  // outlier -> sostituito
  return v;
}

// Restituisce media e deviazione standard della finestra di analisi
void windowStats(float &mean, float &stddev) {
  int n = winFull ? WINDOW : winIdx;
  if (n < 2) { mean = win[0]; stddev = 0; return; }
  float s = 0;
  for (int i = 0; i < n; i++) s += win[i];
  mean = s / n;
  float v = 0;
  for (int i = 0; i < n; i++) { float d = win[i] - mean; v += d * d; }
  stddev = sqrtf(v / n);
}

void resetBuffers() {
  hrawCount = 0; hrawIdx = 0;
  winIdx = 0; winFull = false;
  conferme = 0;
}

// Lettura RSSI valida (scarta 0 e valori implausibili). Niente fallback
// "killer" a -100: in caso di lettura non valida riusa l'ultimo valido.
float readRssiValid() {
  int r = WiFi.RSSI();
  if (r >= -1 || r < -120) return lastValid;  // 0 o assurdo -> ultimo valido
  lastValid = (float)r;
  return (float)r;
}

// Micro-pacchetto verso il gateway: provoca l'ACK 802.11 dell'AP, cosi'
// l'RSSI restituito da WiFi.RSSI() e' realmente aggiornato ad ogni ciclo.
void keepAlive() {
  IPAddress gw = WiFi.gatewayIP();
  if (gw == IPAddress(0, 0, 0, 0)) return;
  udp.beginPacket(gw, 9);   // porta 9 = discard
  udp.write((uint8_t)0);
  udp.endPacket();
}

// ---------------------------------------------------------------------
// TASK DI SENSING (core 1, alta priorita', cadenza rigida)
// ---------------------------------------------------------------------
void sensingTask(void* pv) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (!g_linkUp) {            // WiFi giu': sospendi il rilevamento
      resetBuffers();
      vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_MS));
      continue;
    }

    keepAlive();
    float raw = readRssiValid();
    float clean = hampelClean(raw);

    win[winIdx] = clean;
    winIdx = (winIdx + 1) % WINDOW;
    if (winIdx == 0) winFull = true;

    float mean, spread;
    windowStats(mean, spread);

    float sigma = g_sigma0;
    if (sigma < SIGMA_FLOOR) sigma = SIGMA_FLOOR;
    float score = spread / sigma;

    bool ready = winFull;       // valuta solo a finestra piena

    if (ready) {
      // Adattamento lento della baseline SOLO quando e' quieto.
      if (score < K_QUIET) {
        g_sigma0   = (1.0f - EMA_ALPHA) * g_sigma0   + EMA_ALPHA * spread;
        g_baseline = (1.0f - EMA_ALPHA) * g_baseline + EMA_ALPHA * mean;
      }

      bool attivo;
      portENTER_CRITICAL(&mux);
      attivo = g_allarmAttivo;
      portEXIT_CRITICAL(&mux);

      if (attivo && score > K_ENTER) {
        conferme++;
        if (conferme >= CONFERME_NECESSARIE) {
          unsigned long ora = millis();
          if (ora - ultimaNotifica > COOLDOWN_NOTIFICA_MS) {
            portENTER_CRITICAL(&mux);
            g_motionPending = true;     // il loop si occupera' di inviare
            portEXIT_CRITICAL(&mux);
            ultimaNotifica = ora;
          }
          conferme = 0;
        }
      } else {
        conferme = 0;
      }
    }

    // Pubblica diagnostica
    portENTER_CRITICAL(&mux);
    g_lastRssi  = clean;
    g_lastScore = score;
    portEXIT_CRITICAL(&mux);

    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_MS));
  }
}

// ---------------------------------------------------------------------
// CALIBRAZIONE (misura media e rumore reale dell'ambiente)
// ---------------------------------------------------------------------
void calibra() {
  Serial.println("Calibrazione (stai fermo)...");
  resetBuffers();
  static float calBuf[CAMPIONI_CALIBRAZIONE];
  int n = 0;

  for (int i = 0; i < CAMPIONI_CALIBRAZIONE; i++) {
    keepAlive();
    float raw = readRssiValid();
    float clean = hampelClean(raw);
    calBuf[n++] = clean;
    delay(SAMPLE_MS);
  }

  float s = 0;
  for (int i = 0; i < n; i++) s += calBuf[i];
  float mean = s / n;
  float v = 0;
  for (int i = 0; i < n; i++) { float d = calBuf[i] - mean; v += d * d; }
  float sigma = sqrtf(v / n);
  if (sigma < SIGMA_FLOOR) sigma = SIGMA_FLOOR;

  // Pre-carica la finestra con la coda della calibrazione (stato "quieto")
  int start = (n >= WINDOW) ? n - WINDOW : 0;
  winIdx = 0; winFull = false;
  for (int i = start; i < n; i++) {
    win[winIdx] = calBuf[i];
    winIdx = (winIdx + 1) % WINDOW;
    if (winIdx == 0) winFull = true;
  }

  portENTER_CRITICAL(&mux);
  g_baseline = mean;
  g_sigma0   = sigma;
  portEXIT_CRITICAL(&mux);

  Serial.printf("Baseline: %.1f dBm | sigma rumore: %.2f dB\n", mean, sigma);
}

// ---------------------------------------------------------------------
// WIFI
// ---------------------------------------------------------------------
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);              // <-- niente modem sleep: misure stabili
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connessione WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnesso! IP: " + WiFi.localIP().toString());
    g_linkUp = true;
  } else {
    Serial.println("\nConnessione fallita, riprovo nel loop.");
    g_linkUp = false;
  }
}

// ---------------------------------------------------------------------
// TELEGRAM
// ---------------------------------------------------------------------
void gestisciMessaggio(int i) {
  // Sicurezza: ignora chiunque non sia il CHAT_ID autorizzato
  if (bot.messages[i].chat_id != CHAT_ID) return;

  String testo = bot.messages[i].text;
  testo.toLowerCase();

  if (testo == "/attiva") {
    portENTER_CRITICAL(&mux); g_allarmAttivo = true; portEXIT_CRITICAL(&mux);
    bot.sendMessage(CHAT_ID, "✅ Allarme attivato", "");
  } else if (testo == "/disattiva") {
    portENTER_CRITICAL(&mux); g_allarmAttivo = false; portEXIT_CRITICAL(&mux);
    bot.sendMessage(CHAT_ID, "🔕 Allarme disattivato", "");
  } else if (testo == "/stato") {
    bool attivo; float rssi, score, sigma, base;
    portENTER_CRITICAL(&mux);
    attivo = g_allarmAttivo; rssi = g_lastRssi; score = g_lastScore;
    sigma = g_sigma0; base = g_baseline;
    portEXIT_CRITICAL(&mux);
    String s = attivo ? "✅ Attivo" : "🔕 Disattivato";
    bot.sendMessage(CHAT_ID,
      "Stato allarme: " + s + "\n" +
      "RSSI: " + String(rssi, 0) + " dBm\n" +
      "Baseline: " + String(base, 1) + " dBm\n" +
      "Rumore (sigma): " + String(sigma, 2) + " dB\n" +
      "Punteggio: " + String(score, 2) + " (soglia " + String(K_ENTER, 1) + ")",
      "");
  } else if (testo == "/start" || testo == "/help") {
    bot.sendMessage(CHAT_ID,
      "🏠 *Allarme Casa*\n\n"
      "/attiva — attiva il rilevamento\n"
      "/disattiva — disattiva il rilevamento\n"
      "/stato — diagnostica in tempo reale",
      "Markdown");
  }
}

void pollTelegram() {
  int n = bot.getUpdates(bot.last_message_received + 1);
  while (n) {
    for (int i = 0; i < n; i++) gestisciMessaggio(i);
    n = bot.getUpdates(bot.last_message_received + 1);
  }
}

// ---------------------------------------------------------------------
// SETUP / LOOP (core 0/loopTask: rete + Telegram, NON time-critical)
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  connectWiFi();
  client.setInsecure();
  client.setTimeout(5000);
  udp.begin(0);

  calibra();

  bot.sendMessage(CHAT_ID,
    "✅ Sistema attivo: calibrazione completata", "");

  // Task di sensing ad alta priorita' su core 1
  xTaskCreatePinnedToCore(sensingTask, "sensing", 8192, NULL, 3, NULL, 1);
}

unsigned long ultimoCheckTelegram = 0;

void loop() {
  // 1) Gestione link WiFi
  if (WiFi.status() != WL_CONNECTED) {
    g_linkUp = false;
    Serial.println("WiFi perso, riconnessione...");
    WiFi.disconnect();
    connectWiFi();
    if (g_linkUp) calibra();   // ricalibra dopo la riconnessione
    return;
  } else {
    g_linkUp = true;
  }

  // 2) Allarme in attesa di invio?
  bool pending = false;
  portENTER_CRITICAL(&mux);
  if (g_motionPending) { g_motionPending = false; pending = true; }
  portEXIT_CRITICAL(&mux);
  if (pending) {
    Serial.println(">>> MOVIMENTO RILEVATO! <<<");
    bot.sendMessage(CHAT_ID, "🚨 Movimento rilevato!", "");
  }

  // 3) Polling Telegram (bloccante, ma fuori dal time-critical)
  if (millis() - ultimoCheckTelegram > INTERVALLO_TELEGRAM_MS) {
    pollTelegram();
    ultimoCheckTelegram = millis();
  }

  delay(20);
}
