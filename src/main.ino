#include <SoftwareSerial.h>
//-------- порты для rs 485
#define SSerialRx        D2  // Serial Receive pin RO
#define SSerialTx        D3   // Serial Transmit pin DI
//-------- инициализация 
SoftwareSerial RS485Serial(SSerialRx, SSerialTx); // Rx, Tx

//// линия управления передачи приема
#define SerialControl D4   // RS485 
/////// флаг приема передачи
#define RS485Transmit    HIGH
#define RS485Receive     LOW
/////// байтовые последовательности запроса инфы
byte testConnect[] = { 0x00, 0x00 };
byte Access[]      = { 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
byte Sn[]          = { 0x00, 0x08, 0x00 }; // серийный номер
byte Freq[]        = { 0x00, 0x08, 0x16, 0x40 }; // частота
byte Current[]     = { 0x00, 0x08, 0x16, 0x21 };//  ток
byte Suply[]       = { 0x00, 0x08, 0x16, 0x11 }; // напряжение
byte Power[]       = { 0x00, 0x08, 0x16, 0x00 };// мощность p
byte PowerQ[]       = { 0x00, 0x08, 0x16, 0x08 };// мощность Q
byte PowerS[]       = { 0x00, 0x08, 0x16, 0x04 };// мощность S

byte CosF[]       = { 0x00, 0x08, 0x16, 0x30 };// cosf


byte Angle[]       = { 0x00, 0x08, 0x16, 0x51 }; // углы
byte energyT0[]  =   { 0x00, 0x05, 0x00, 0x00 };///  суммарная энергия прямая + обратная + активная + реактивная
byte energyT1[]  =   { 0x00, 0x05, 0x00, 0x01 };///  суммарная энергия прямая + обратная + активная + реактивная
byte energyT2[]  =   { 0x00, 0x05, 0x00, 0x02 };///  суммарная энергия прямая + обратная + активная + реактивная
byte energyT3[]  =   { 0x00, 0x05, 0x00, 0x03 };///  суммарная энергия прямая + обратная + активная + реактивная
byte energyT4[]  =   { 0x00, 0x05, 0x00, 0x04 };///  суммарная энергия прямая + обратная + активная + реактивная
//-------------------------------------------------------Переменые хранения данных с счетчика НАЧАЛО
String serNum;
String ARPower;
String valFreq;
String U;
String A;
String Anglle;
String PowerNow;
String Tarif1;
String Tarif2;
String Tarif3;
String Tarif4;
String PQ;
String PS;
String CSF; 
//-------------------------------------------------------Переменые хранения данных с счетчика КОНЕЦ

byte response[19]; 
int byteReceived;
int byteSend;
int netAdr=000;//адрес

//---------------------------------------------------------Рабочие переменные
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include <TimeLib.h>
#include <NtpClientLib.h>
#include <ArduinoJson.h>

#define ssid "kamikadze"        // WiFi SSID
#define password "18041996"    // WiFi password
#define HISTORY_FILE "/history.json"
const uint8_t GPIOPIN[4] = {D5,D6,D7,D8};  // Led
float   t = 0 ;
float   h = 0 ;
float   pa = 0;
int     sizeHist = 84 ;        // рамер (7h x 12pts) 

int var=0;
const long intervalHist = 1000 * 60 * 5;  // 5 mesures / heure - 5 measures / hours
unsigned long previousMillis = intervalHist;  // интервал измерений


ESP8266WebServer server ( 80 );

StaticJsonBuffer<10000> jsonBuffer;                 // Buffer static contenant le JSON courant - Current JSON static buffer
JsonObject& root = jsonBuffer.createObject();
JsonArray& timestamp = root.createNestedArray("timestamp");
JsonArray& hist_t = root.createNestedArray("t");
JsonArray& hist_h = root.createNestedArray("h");
JsonArray& hist_pa = root.createNestedArray("pa");
JsonArray& bart = root.createNestedArray("bart");   // Clé historgramme (temp/humidité) - Key histogramm (temp/humidity)
JsonArray& barh = root.createNestedArray("barh");   // Clé historgramme (temp/humidité) - Key histogramm (temp/humidity)

char json[10000];                                   // Buffer pour export du JSON - JSON export buffer

void updateGpio(){
  String gpio = server.arg("id");
  String etat = server.arg("etat");
  String success = "1";
  int pin = D5;
 if ( gpio == "D5" ) {
      pin = D5;
 } else if ( gpio == "D7" ) {
     pin = D7;
 } else if ( gpio == "D8" ) {
     pin = D8;  
 } else {   
      pin = D6;
  }
  Serial.println(pin);
  if ( etat == "1" ) {
    digitalWrite(pin, HIGH);
  } else if ( etat == "0" ) {
    digitalWrite(pin, LOW);
  } else {
    success = "1";
    Serial.println("Err Led Value");
  }
  
  String json = "{\"gpio\":\"" + String(gpio) + "\",";
  json += "\"etat\":\"" + String(etat) + "\",";
  json += "\"success\":\"" + String(success) + "\"}";
    
  server.send(200, "application/json", json);
  Serial.println("GPIO updated");
}

void sendMesures() {
  String json = "{\"t\":\"" + String(t) + "\",";
  json += "\"h\":\"" + String(h) + "\",";
  json += "\"pa\":\"" + String(pa) + "\"}";

  server.send(200, "application/json", json);
  Serial.println("Send measures");
}

void calcStat(){
  float statTemp[7] = {-999,-999,-999,-999,-999,-999,-999};
  float statHumi[7] = {-999,-999,-999,-999,-999,-999,-999};
  int nbClass = 7;  // Nombre de classes - Number of classes                         
  int currentClass = 0;
  int sizeClass = hist_t.size() / nbClass;  // 2
  double temp;
  //
  if ( hist_t.size() >= sizeHist ) {
    //Serial.print("taille classe ");Serial.println(sizeClass);
    //Serial.print("taille historique ");Serial.println(hist_t.size());
    for ( int k = 0 ; k < hist_t.size() ; k++ ) {
      temp = root["t"][k];
      if ( statTemp[currentClass] == -999 ) {
        statTemp[ currentClass ] = temp;
      } else {
        statTemp[ currentClass ] = ( statTemp[ currentClass ] + temp ) / 2;
      }
      temp = root["h"][k];
      if ( statHumi[currentClass] == -999 ) {
        statHumi[ currentClass ] = temp;
      } else {
        statHumi[ currentClass ] = ( statHumi[ currentClass ] + temp ) / 2;
      }
         
      if ( ( k + 1 ) > sizeClass * ( currentClass + 1 ) ) {
        //Serial.print("k ");Serial.print(k + 1);Serial.print(" Cellule statTemp = ");Serial.println(statTemp[ currentClass ]);
        currentClass++;
      } else {
        //Serial.print("k ");Serial.print(k + 1);Serial.print(" < ");Serial.println(sizeClass * currentClass);
      }
    }
    
    Serial.println("Histogram - Temperature"); 
    for ( int i = 0 ; i < nbClass ; i++ ) {
      Serial.print(statTemp[i]);Serial.print('|');
    }
    Serial.println("Histogram - Humidity "); 
    for ( int i = 0 ; i < nbClass ; i++ ) {
      Serial.print(statHumi[i]);Serial.print('|');
    }
    Serial.print("");
    if ( bart.size() == 0 ) {
      for ( int k = 0 ; k < nbClass ; k++ ) { 
        bart.add(statTemp[k]);
        barh.add(statHumi[k]);
      }  
    } else {
      for ( int k = 0 ; k < nbClass ; k++ ) { 
        bart.set(k, statTemp[k]);
        barh.set(k, statHumi[k]);
      }  
    }
  }
}

void sendTabMesures() {
  double temp = root["t"][0];      // get oldest record (temperature)
  String json = "[";
  json += "{\"mesure\":\"Серийный номер\",\"valeur\":\"" + String(serNum) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-indent-left\",\"precedente\":\"" /*+ String(temp) + */"\"},"; //1
  temp = root["h"][0];             //  get oldest record (humidity)
  json += "{\"mesure\":\"Потребляемая мощность\",\"valeur\":\"" + String(ARPower) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//2
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Частота сети\",\"valeur\":\"" + String(valFreq) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//4
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Напряжение сети\",\"valeur\":\"" + String(U) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//5
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Потребляемый ток\",\"valeur\":\"" + String(A) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//6
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Потрябляемая мощность\",\"valeur\":\"" + String(Anglle) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//7
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Потрябляемая мощность в данный момент\",\"valeur\":\"" + String(PowerNow) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) +*/ "\"},";//8
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"По тарифу 1\",\"valeur\":\"" + String( Tarif1) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) +*/ "\"},";//9
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"По тарифу 2\",\"valeur\":\"" + String( Tarif2) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//10
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"По тарифу 3\",\"valeur\":\"" + String( Tarif3) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//11
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"По тарифу 4\",\"valeur\":\"" + String( Tarif4) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) + */"\"},";//12
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Мощность Q\",\"valeur\":\"" + String(PQ) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) +*/ "\"},";//13
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Мощность S\",\"valeur\":\"" + String(PS) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-tint\",\"precedente\":\"" + /*String(temp) +*/ "\"},";//13
  temp = root["pa"][0];             // get oldest record (Atmospheric Pressure)
  json += "{\"mesure\":\"Всего потребленно за мясяц\",\"valeur\":\"" + String(CSF) + "\",\"unite\":\"\",\"glyph\":\"glyphicon-dashboard\",\"precedente\":\"" + /*String(temp) + */"\"}"; //14
  json += "]";
  server.send(200, "application/json", json);
  Serial.println("Send data tab");
}

void sendHistory(){  
  root.printTo(json, sizeof(json));             // Экспортировать объект JSON в виде строки
  server.send(200, "application/json", json);   // Отправка данных истории веб-клиенту
  Serial.println("Send History");   
}

void loadHistory(){
  File file = SPIFFS.open(HISTORY_FILE, "r");
  if (!file){
    Serial.println("Нет истории");
  } else {
    size_t size = file.size();
    if ( size == 0 ) {
      Serial.println("Файл истории пуст!");
    } else {
      std::unique_ptr<char[]> buf (new char[size]);
      file.readBytes(buf.get(), size);
      JsonObject& root = jsonBuffer.parseObject(buf.get());
      if (!root.success()) {
        Serial.println("Невозможно прочитать файл JSON");
      } else {
        Serial.println("История загружена");
        root.prettyPrintTo(Serial);  
      }
    }
    file.close();
  }
}

void saveHistory(){
  Serial.println("Сохранение истории");            
  File historyFile = SPIFFS.open(HISTORY_FILE, "w");
  root.printTo(historyFile); // Экспорт и сохранение объекта JSON в область SPIFFS
  historyFile.close();  
}

void setup() {
  RS485Serial.begin(9600);
  pinMode(SerialControl, OUTPUT);
  digitalWrite(SerialControl, RS485Receive);
  //-----------------------------------------
  NTP.onNTPSyncEvent([](NTPSyncEvent_t error) {
    if (error) {
      Serial.print("Ошибка синхронизации времени:");
      if (error == noResponse)
        Serial.println("NTP-сервер недоступен");
      else if (error == invalidAddress)
        Serial.println("Недопустимый адрес сервера NTP");
      }
    else {
      Serial.print("Получено время NTP:");
      Serial.println(NTP.getTimeDateString(NTP.getLastNTPSync()));
    }
  });
  // NTP-сервер, разница во времени, летнее время 
  NTP.begin("pool.ntp.org", 0, true); 
  NTP.setInterval(60000);
  delay(500);
     
  for ( int x = 0 ; x < 5 ; x++ ) {
    pinMode(GPIOPIN[x], OUTPUT);
  }
  
  Serial.begin ( 115200 );
 
  WiFi.begin ( ssid, password );
  int tentativeWiFi = 0;
  //  Ожидание подключения
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 ); Serial.print ( "." );
    tentativeWiFi++;
    if ( tentativeWiFi > 20 ) {
      ESP.reset();
      while(true)
        delay(1);
    }
  }
  // Соединение Wi-Fi в порядке
  Serial.println ( "" );
  Serial.print ( "Connected to " ); Serial.println ( ssid );
  Serial.print ( "IP address: " ); Serial.println ( WiFi.localIP() );
  
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount failed");        // Проблема с хранилищем SPIFFS
  } else { 
    Serial.println("SPIFFS Mount succesfull");
    loadHistory();
  }
  delay(50);
  
  server.on("/tabmesures.json", sendTabMesures);
  server.on("/mesures.json", sendMesures);
  server.on("/gpio", updateGpio);
  server.on("/graph_temp.json", sendHistory);

  server.serveStatic("/js", SPIFFS, "/js");
  server.serveStatic("/css", SPIFFS, "/css");
  server.serveStatic("/img", SPIFFS, "/img");
  server.serveStatic("/", SPIFFS, "/index.html");

  server.begin();
  Serial.println ( "HTTP server started" );

  Serial.print("Uptime :");
  Serial.println(NTP.getUptime());
  Serial.print("LastBootTime :");
  Serial.println(NTP.getLastBootTime());
}

void mercurii()
{
  testConnect[0] = netAdr;
  response[0] = 0;
  send(testConnect, sizeof(testConnect), response);
  Access[0] = netAdr;
  response[0] = 0;
  send(Access, sizeof(Access), response);
          

   
         serNum = getSerialNumber(netAdr);
         //Serial.print("s:"+ serNum +"\r\n");   
         ARPower = getEnergyT0(netAdr);
         //Serial.print("p:"+ ARPower+"\r\n");   
          valFreq = getFreq(netAdr);
        // Serial.print("f:"+ valFreq+"\r\n");   
          U = getSuply(netAdr);
         //Serial.print("u:"+ U+"\r\n");   
         A = getCurrent(netAdr);
        // Serial.print("a:"+ A+"\r\n");   
         Anglle = getAngle(netAdr);
        // Serial.print("g:"+ Angle+"\r\n");   
         PowerNow = getPowerNow(netAdr);
        // Serial.print("e:"+ PowerNow+"\r\n");  
         Tarif1 = getEnergyT1(netAdr);
        // Serial.print("t1:"+ Tarif1+"\r\n");  
        Tarif2 = getEnergyT2(netAdr);
       //  Serial.print("t2:"+ Tarif2+"\r\n");  
          Tarif3 = getEnergyT3(netAdr);
      //   Serial.print("t3:"+ Tarif3+"\r\n");  
          Tarif4 = getEnergyT4(netAdr);
       //  Serial.print("t4:"+ Tarif4+"\r\n");  
          PQ = getPowerQ(netAdr);
       //  Serial.print("q:"+ PQ+"\r\n");  
          PS = getPowerS(netAdr);
        // Serial.print("c:"+ PS+"\r\n");  
          CSF = getCosF(netAdr);
        // Serial.print("k:"+ CSF+"\r\n");  
}
void loop() {
server.handleClient();
mercurii();
 
}

String getSerialNumber(int netAdr)
{
  String s1,s2,s3,s4;
  response[0]=0;
  Sn[0] = netAdr;
  send(Sn, sizeof(Sn),response);
  if((int)response[1] < 10) { s1="0" + String((int)response[1]); } else {s1=String((int)response[1]);}
  if((int)response[2] < 10) { s2="0" + String((int)response[2]); } else {s2=String((int)response[2]);}
  if((int)response[3] < 10) { s3="0" + String((int)response[3]); } else {s3=String((int)response[3]);}
  if((int)response[4] < 10) { s4="0" + String((int)response[4]); } else {s4=String((int)response[4]);}
  String n = s1+s2+s3+s4;
  
  return String(response[0])+";"+n;
}


String getPowerNow(int netAdr)
{
 
  response[0]=0;
  Power[0] = netAdr;
  send(Power, sizeof(Power),response);
  long r = 0;
  int dir_U0=0;
  int dir_U1=0;
  int dir_U2=0;
  int dir_U3=0;

  if((long)response[1]<<16 == 0x40) dir_U0=1;
  if((long)response[1]<<16 == 0x80) dir_U0=-1;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U0= String(r * dir_U0);
  r = 0;

  if((long)response[4]<<16 == 0x40) dir_U1=1;
  if((long)response[4]<<16 == 0x80) dir_U1=-1;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U1= String(r * dir_U1);
  r=0;

  if((long)response[7]<<16 == 0x40) dir_U2=1;
  if((long)response[7]<<16 == 0x80) dir_U2=-1;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U2= String(r * dir_U2);
  r = 0;

  if((long)response[10]<<16 == 0x40) dir_U3=1;
  if((long)response[10]<<16 == 0x80) dir_U3=-1;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String U3= String(r * dir_U3);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U0+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
 
 
}

String getPowerQ(int netAdr)
{
 
  response[0]=0;
  PowerQ[0] = netAdr;
  send(PowerQ, sizeof(PowerQ),response);
  long r = 0;
  int dir_U0=0;
  int dir_U1=0;
  int dir_U2=0;
  int dir_U3=0;

  if((long)response[1]<<16 == 0x40) dir_U0=1;
  if((long)response[1]<<16 == 0x80) dir_U0=-1;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U0= String(r * dir_U0);
  r = 0;

  if((long)response[4]<<16 == 0x40) dir_U1=1;
  if((long)response[4]<<16 == 0x80) dir_U1=-1;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U1= String(r * dir_U1);
  r=0;

  if((long)response[7]<<16 == 0x40) dir_U2=1;
  if((long)response[7]<<16 == 0x80) dir_U2=-1;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U2= String(r * dir_U2);
  r = 0;

  if((long)response[10]<<16 == 0x40) dir_U3=1;
  if((long)response[10]<<16 == 0x80) dir_U3=-1;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String U3= String(r * dir_U3);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U0+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
 

}

String getPowerS(int netAdr)
{
 
  response[0]=0;
  PowerS[0] = netAdr;
  send(PowerS, sizeof(PowerS),response);
  long r = 0;
  int dir_U0=0;
  int dir_U1=0;
  int dir_U2=0;
  int dir_U3=0;

  if((long)response[1]<<16 == 0x40) dir_U0=1;
  if((long)response[1]<<16 == 0x80) dir_U0=-1;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U0= String(r * dir_U0);
  r = 0;

  if((long)response[4]<<16 == 0x40) dir_U1=1;
  if((long)response[4]<<16 == 0x80) dir_U1=-1;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U1= String(r * dir_U1);
  r=0;

  if((long)response[7]<<16 == 0x40) dir_U2=1;
  if((long)response[7]<<16 == 0x80) dir_U2=-1;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U2= String(r * dir_U2);
  r = 0;

  if((long)response[10]<<16 == 0x40) dir_U3=1;
  if((long)response[10]<<16 == 0x80) dir_U3=-1;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String U3= String(r * dir_U3);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U0+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
 
 
}

String getCosF(int netAdr)
{
  response[0]=0;
  CosF[0] = netAdr;
  send(CosF, sizeof(CosF),response);
  long r = 0;

  int dir_U0=0;
  int dir_U1=0;
  int dir_U2=0;
  int dir_U3=0;

  if((long)response[1]<<16 == 0x40) dir_U0=1;
  if((long)response[1]<<16 == 0x80) dir_U0=-1;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U0= String(r * dir_U0);
  r = 0;

  if((long)response[4]<<16 == 0x40) dir_U1=1;
  if((long)response[4]<<16 == 0x80) dir_U1=-1;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U1= String(r * dir_U1);
  r=0;

  if((long)response[7]<<16 == 0x40) dir_U2=1;
  if((long)response[7]<<16 == 0x80) dir_U2=-1;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U2= String(r * dir_U2);
  r = 0;

  if((long)response[10]<<16 == 0x40) dir_U3=1;
  if((long)response[10]<<16 == 0x80) dir_U3=-1;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String U3= String(r * dir_U3);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U0+";"+U1+";"+U2+";"+U3);
  else   return String("Error");
 
}



String getAngle(int netAdr)
{
 
  response[0]=0;
  Angle[0] = netAdr;
  send(Angle, sizeof(Angle),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");

}


String getCurrent(int netAdr)
{
  response[0]=0;
  Current[0] = netAdr;
  send(Current, sizeof(Current),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");

}

String getSuply(int netAdr)
{
 
  response[0]=0;
  Suply[0] = netAdr;
  send(Suply, sizeof(Suply),response);
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String U1= String(r);
  r = 0;
  r |= (long)response[4]<<16;
  r |= (long)response[6]<<8;
  r |= (long)response[5];
  String U2= String(r);
  r=0;
  r |= (long)response[7]<<16;
  r |= (long)response[9]<<8;
  r |= (long)response[8];
  String U3= String(r);
  if(response[0] == netAdr)   return String(String(response[0])+";"+U1+";"+U2+";"+U3);
  else   return String("Error");

}


String getFreq(int netAdr)
{
 
  response[0]=0;
  Freq[0] = netAdr;
  send(Freq, sizeof(Freq),response);
 
  long r = 0;
  r |= (long)response[1]<<16;
  r |= (long)response[3]<<8;
  r |= (long)response[2];
  String fr= String(r);
  //return fr;
  if(response[0] == netAdr)   return String(response[0])+";"+fr;
  else   return String("Error");

}

String getEnergyT0(int netAdr)
{
  response[0]=0;
  energyT0[0] = netAdr;
  send(energyT0, sizeof(energyT0),response);
  if(response[0] == netAdr) 
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }

  else   return String("Error");

  
}



String getEnergyT1(int netAdr)
{
 
  response[0]=0;
  energyT1[0] = netAdr;
  send(energyT1, sizeof(energyT1),response);
  if(response[0] == netAdr) 
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }

  else   return String("Error");
 
}



String getEnergyT2(int netAdr)
{
 
  response[0]=0;
  energyT2[0] = netAdr;
  send(energyT2, sizeof(energyT2),response);
  if(response[0] == netAdr) 
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }

  else   return String("Error");
 
}

String getEnergyT3(int netAdr)
{
 
  response[0]=0;
  energyT3[0] = netAdr;
  send(energyT3, sizeof(energyT3),response);
  if(response[0] == netAdr) 
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }

  else   return String("Error");
 
}


String getEnergyT4(int netAdr)
{
 
  response[0]=0;
  energyT4[0] = netAdr;
  send(energyT4, sizeof(energyT4),response);
  if(response[0] == netAdr) 
  {
  long r = 0;
  r |= (long)response[2]<<24;
  r |= (long)response[1]<<16;
  r |= (long)response[4]<<8;
  r |= (long)response[3];
  String A_plus= String(r);
  r=0;
  r |= (long)response[6]<<24;
  r |= (long)response[5]<<16;
  r |= (long)response[8]<<8;
  r |= (long)response[7];
  String A_minus= String(r);
  r = 0;
  r |= (long)response[10]<<24;
  r |= (long)response[9]<<16;
  r |= (long)response[12]<<8;
  r |= (long)response[11];
  String R_plus= String(r);
  r = 0;
  r |= (long)response[14]<<24;
  r |= (long)response[13]<<16;
  r |= (long)response[16]<<8;
  r |= (long)response[15];
  String R_minus= String(r);
  return String(String(response[0])+";"+A_plus+";"+A_minus+";"+R_plus+";"+R_minus);
  }

  else   return String("Error");
 
}




//////////////////////////////////////////////////////////////////////////////////
void send(byte *cmd, int s, byte *response) {
 // Serial.print("sending...");

  unsigned int crc = crc16MODBUS(cmd, s);

  unsigned int crc1 = crc & 0xFF;
  unsigned int crc2 = (crc>>8) & 0xFF;
  delay(10);
  digitalWrite(SerialControl, RS485Transmit);  // Init Transceiver   
       for(int i=0; i<s; i++) 
       {
              RS485Serial.write(cmd[i]);
       }
  RS485Serial.write(crc1);
  RS485Serial.write(crc2);
  byte i = 0;
  digitalWrite(SerialControl, RS485Receive);  // Init Transceiver   
  delay(200);
         if (RS485Serial.available()) 
           {
             while (RS485Serial.available()) 
               {
                byteReceived= RS485Serial.read();    // Read received byte             
                delay(10);  
                response[i++] = byteReceived;
                }
           }

  
  delay(20);
}




unsigned int crc16MODBUS(byte *s, int count) {
  unsigned int crcTable[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    unsigned int crc = 0xFFFF;

    for(int i = 0; i < count; i++) {
        crc = ((crc >> 8) ^ crcTable[(crc ^ s[i]) & 0xFF]);
    }

    return crc;
}

//-------------------------
void addPtToHist(){
  unsigned long currentMillis = millis();
  
  //Serial.println(currentMillis - previousMillis);
  if ( currentMillis - previousMillis > intervalHist ) {
    long int tps = NTP.getTime();
    previousMillis = currentMillis;
    //Serial.println(NTP.getTime());
    if ( tps > 0 ) {
      timestamp.add(tps);
      hist_t.add(double_with_n_digits(t, 1));
      hist_h.add(double_with_n_digits(h, 1));
      hist_pa.add(double_with_n_digits(pa, 1));

      //root.printTo(Serial);
      if ( hist_t.size() > sizeHist ) {
        //Serial.println("efface anciennes mesures");
        timestamp.removeAt(0);
        hist_t.removeAt(0);
        hist_h.removeAt(0);
        hist_pa.removeAt(0);
      }
      //Serial.print("size hist_t ");Serial.println(hist_t.size());
      calcStat();
      //delay(100);
      saveHistory();
      //root.printTo(Serial);  
    }  
  }
}
