//   Gerador de sinal de RF 2.4GHz com Arduino
//   código base de Alain Fort F1CJN feb 2,2016
//   modificado por Arthur Liraneto 21/03/2022 (ROBOT V1.1 and V1.0)
//
//  *************************************************** PORTUGUÊS ***********************************************************
//
//  O código base de Alain Fort utiliza Arduino Uno, um shield de display LCD padrão da ROBOT com botões, e um sintetizador ADF4351 facilmente encontrado
//  na internet. A modificação inserida por Arthur Liraneto adiciona um Switch de RF HMC241, 3 filtros Chebyshev próprios e 1 combinador de RF 2.4GHz comercial
//  de baixo custo encontrado em lojas de antenas.
//  A frequência pode ser ajustada com resolução de 10kHz de 35MHz até 2.4 GHz com máxima performance, porém, acima desta frequência, o projeto precisa
//  de um combinador de mais alta frequência e ajustes respectivos de nível de potência de saída.
//  Vinte frequências podem ser memorizadas na EEPROM do Arduino. Ao ligar, a memória zero é sempre selecionada
//  O cursor pode se mover com os botões de esquerda e direita. Os números podem ser alterados com os botões para cima e para baixo.
//  - Para ler ou escrever a frequência na memória, coloque o cursor mais a esquerda na primeira tela com REE (leitura) ou WEE (escrita).
//  - O cursor desaparece após alguns segundos e é reativado automaticamente ao pressionar um botão.
//
//  MEMORIZAÇÃO
//  - Para a frequência, selecione WEE, e depois selecione o número de memória e em seguida pressione o botão select por 1 segundo.
//  A palavra memorização aparece na tela. Após isso o cursor pode ir para outro lugar.
//  - Para a frequência de referência, mova o cursor para os números 10 ou 25 e pressione select por 1 segundo.

//*************************************************************************************************************************
// ATENÇÃO: se vc estiver utilizando o shield ROBOT 1.1, é necessário alterar a função read_lcd_buttons.

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SPI.h>

#define ADF4351_LE 3

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte poscursor = 0; //posição corrente do cursor 0 à 15
byte line = 0; // linha do display LCD em curso 0 ou 1
byte memoire,RWtemp; // numero da memória EEPROM

uint32_t registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ; // 437 MHz com referência à 25 MHz
//uint32_t registers[6] =  {0, 0, 0, 0, 0xBC803C, 0x580005} ; // 437 com referência à 25 MHz
int address,modif=0,WEE=0;
int lcd_key = 0;
int adc_key_in  = 0;
int timer = 0,timer2=0; // utilizado para medir a duração do aperto de um botão
unsigned int i = 0;


double RFout, REFin, INT, OutputChannelSpacing, FRACF;
unsigned int PFDRFout, powerprint=4, powerlevel=1, sweepflag=0, submenuflag=0, sweepcount=0;
double RFoutMin = 35, RFoutMax = 4400, REFinMax = 250, PDFMax = 32;
unsigned int long RFint,RFintold,INTA,RFcalc, sweepcalc,sweepcalc2,PDRFout, MOD, FRAC, sweepf1=10000, sweepf2=240000;
byte OutputDivider;byte lock=2;
unsigned int long reg0, reg1;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

//**************************** SP LEITURA DOS BOTOES ********************************************
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // lê o valor dos botões
  if (adc_key_in < 790)lcd.blink();
  
  if (adc_key_in < 50)return btnRIGHT;  // para o display ROBOT V1.0
  if (adc_key_in < 195)return btnUP;
  if (adc_key_in < 380)return btnDOWN;
  if (adc_key_in < 555)return btnLEFT;
  if (adc_key_in < 790)return btnSELECT; // Fim do display ROBOT1.1

  //if (adc_key_in < 50)return btnRIGHT; // para o display ROBOT 1.1
  //if (adc_key_in < 250)return btnUP;
  //if (adc_key_in < 450)return btnDOWN;
  //if (adc_key_in < 650)return btnLEFT;
  //if (adc_key_in < 850)return btnSELECT; // Fim do display ROBOT 1.1

  return btnNONE;  // sem aperto de botão
}

//***************************** SP Mostradores de frequência no display LCD ********************************
void printAll ()
{
  //RFout=1001.10 // test
  lcd.setCursor(0, 0);
  lcd.print("RF=");
  if (RFint < 100000) lcd.print(" ");
  if (RFint < 10000)  lcd.print(" ");
  lcd.print(RFint/100);
  lcd.print(".");
  RFcalc=RFint-((RFint/100)*100);
  if (RFcalc<10)lcd.print("0");
  lcd.print(RFcalc);
  lcd.print("MHz");
  lcd.print(" up");
  lcd.setCursor(0,1);
  if (WEE==0) {lcd.print("REE=");}
  else {lcd.print("WEE=");}
  if (memoire<10)lcd.print(" ");
  lcd.print(memoire,DEC);
  if  ((digitalRead(2)==1))lcd.print(" LKD ");
  else lcd.print(" NLK ");
  lcd.print(PFDRFout,DEC);
  lcd.print(" P");
  lcd.print(powerprint,DEC);
  lcd.setCursor(poscursor,line);
  
}

void printsubmenu ()
{
  lcd.setCursor(0, 0);
  lcd.print("F1=");
  if (sweepf1 < 100000) lcd.print(" ");
  if (sweepf1 < 10000)  lcd.print(" ");
  lcd.print(sweepf1/100);
  lcd.print(".");
  sweepcalc=sweepf1-((sweepf1/100)*100);
  if (sweepcalc<10)lcd.print("0");
  lcd.print(sweepcalc);
  lcd.print("MHz ");
  lcd.print("S");
  lcd.print(sweepflag,DEC);

  lcd.setCursor(0,1);
  lcd.print("F2=");
  if (sweepf2 < 100000) lcd.print(" ");
  if (sweepf2 < 10000)  lcd.print(" ");
  lcd.print(sweepf2/100);
  lcd.print(".");
  sweepcalc2=sweepf2-((sweepf2/100)*100);
  if (sweepcalc2<10)lcd.print("0");
  lcd.print(sweepcalc2);
  lcd.print("MHz ");
  lcd.print("dn");
  lcd.setCursor(poscursor,line);
    
}

void WriteRegister32(const uint32_t value)   //Programa um registrador 32bits
{
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)          // quebra em 4 x 8bits
  SPI.transfer((value >> 8 * i) & 0xFF); // deslocamento, máscara de envio de byte via SPI
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

void SetADF4351()  // programa todos os registradores do ADF4351
{ for (int i = 5; i >= 0; i--)  // programação ADF4351 comença pelo R5
    WriteRegister32(registers[i]);
}

void processbuttons(){

lcd_key = read_LCD_buttons();  // lê os botões

  switch (lcd_key)               // Seleciona a ação
  {
    case btnRIGHT: //Direita
      poscursor++; // cursor para a direita
      if (line == 0) {
        if(submenuflag==1){
        if (poscursor == 7 ) { poscursor = 8;line = 0; } //se o cursor está no ponto "."
        if (poscursor == 10 ) { poscursor = 15;line = 0; }
        if (poscursor == 16 ) {poscursor = 3; line = 1; }; //se o cursor está a direita
      }else{
        if (poscursor == 7 ) { poscursor = 8;line = 0; } //se o cursor está no ponto "."
        if (poscursor == 10 ) { poscursor = 15;line = 0; }
        if (poscursor == 16 ) {poscursor = 5; line = 1; }; //se o cursor está a direita
      }
      }
     if (line == 1) {
     if(submenuflag==1){
        if (poscursor == 7 ) {poscursor = 8; line = 1; } 
        if (poscursor == 10 ) {poscursor = 15; line = 1; }
        if (poscursor==16) {poscursor=3; line=0;};     
      }else{
        if (poscursor == 1 ) {poscursor = 5; line = 1; } //se o cursor está nos números de memória
        if (poscursor == 6 ) {poscursor = 12; line = 1; } //se o cursor está nos números de memória
        if (poscursor == 13 ) {poscursor = 15; line = 1; }
        if (poscursor==16) {poscursor=3; line=0;};   
      }
     }
      lcd.setCursor(poscursor, line);
      break;
      
    case btnLEFT: //Gauche
      poscursor--; // décalage curseur
      if (line == 0) {
        if (poscursor == 2) {poscursor = 15; line = 1;  };
        if (poscursor == 7) {   poscursor = 6; line=0;}
        if (poscursor == 14) {   poscursor = 9; line=0;}
      }
       if(line==1){
        if(submenuflag==1){
          if (poscursor==2) {poscursor=15; line=0;};
          if (poscursor==7) {poscursor=6; line=1;};
          if (poscursor==14) {poscursor=9; line=1;};
      }else{
          if (poscursor==14) {poscursor=12; line=1;};
          if (poscursor==11) {poscursor=5; line=1;};
          if (poscursor==4) {poscursor=15; line=0;};
      }
       }
      //Serial.print(poscursor,DEC);  
      lcd.setCursor(poscursor, line);
      break;
    
    case btnUP: //Haut
      if (line == 0){ 
        if(submenuflag==1){
        if (poscursor == 3) sweepf1 = sweepf1 + 100000 ;
        if (poscursor == 4) sweepf1 = sweepf1 + 10000 ;
        if (poscursor == 5) sweepf1 = sweepf1 + 1000 ;
        if (poscursor == 6) sweepf1 = sweepf1 + 100 ;
        if (poscursor == 8) sweepf1 = sweepf1 + 10 ;
        if (poscursor == 9) sweepf1 = sweepf1 + 1 ;
        if (poscursor == 15) sweepflag = 1; 
        if (sweepf1 > 440000) sweepf1 = 440000;
       printsubmenu();
       }else{
        if (poscursor == 3) RFint = RFint + 100000 ;
        if (poscursor == 4) RFint = RFint + 10000 ;
        if (poscursor == 5) RFint = RFint + 1000 ;
        if (poscursor == 6) RFint = RFint + 100 ;
        if (poscursor == 8) RFint = RFint + 10 ;
        if (poscursor == 9) RFint = RFint + 1 ;
        if (RFint > 440000) RFint = RFintold;
        if (poscursor ==15) {
          submenuflag=1; 
          line=1;
          }
       printAll();
       }
      }
      if (line == 1){
        if(submenuflag==1){
        if (poscursor == 3) sweepf2 = sweepf2 + 100000 ;
        if (poscursor == 4) sweepf2 = sweepf2 + 10000 ;
        if (poscursor == 5) sweepf2 = sweepf2 + 1000 ;
        if (poscursor == 6) sweepf2 = sweepf2 + 100 ;
        if (poscursor == 8) sweepf2 = sweepf2 + 10 ;
        if (poscursor == 9) sweepf2 = sweepf2 + 1 ;
        if (sweepf2 > 440000) sweepf2 = 440000;
      printsubmenu();
      }else{
        if (poscursor == 5){ memoire++; 
        if (memoire==20)memoire=0;
        if (WEE==0){RFint=EEPROMReadlong(memoire*4); // Leitura EEPROM e mostrador
           if (RFint>440000) RFint=440000; 
           } 
        }  
        if (poscursor==12){ 
        if( PFDRFout==10){PFDRFout=25;} //reglage FREF
        else if ( PFDRFout==25){PFDRFout=10;}
        else PFDRFout=25;// au cas ou PFDRF different de 10 et 25
        modif=1;  }

        if (poscursor==15){
          if(powerprint==4) break;
          else powerprint=powerprint+1;
          }
                    
      if( (poscursor==0) && (WEE==1))WEE=0;
      else if ((poscursor==0) && (WEE==0))WEE=1;                  
      printAll();
      }
      }      
     
      break; // fin bouton up
    
    case btnDOWN: //bas
      if (line == 0) {
        if(submenuflag==1){
        if (poscursor == 3) sweepf1 = sweepf1 - 100000 ;
        if (poscursor == 4) sweepf1 = sweepf1 - 10000 ;
        if (poscursor == 5) sweepf1 = sweepf1 - 1000 ;
        if (poscursor == 6) sweepf1 = sweepf1 - 100 ;
        if (poscursor == 8) sweepf1 = sweepf1 - 10 ;
        if (poscursor == 9) sweepf1 = sweepf1 - 1 ;
        if (poscursor == 15) {
          sweepflag=0;
          sweepcount=0;
          }
        if (RFint < 3450) sweepf1 = 3450;
        if (RFint > 440000)  sweepf1 = 440000;
        printsubmenu();
        }else{
        if (poscursor == 3) RFint = RFint - 100000 ;
        if (poscursor == 4) RFint = RFint - 10000 ;
        if (poscursor == 5) RFint = RFint - 1000 ;
        if (poscursor == 6) RFint = RFint - 100 ;
        if (poscursor == 8) RFint = RFint - 10 ;
        if (poscursor == 9) RFint = RFint - 1 ;
        if (RFint < 3450) RFint = RFintold;
        if (RFint > 440000)  RFint = RFintold;
        printAll();
      }
      }
     if (line == 1){ 
      if(submenuflag==1){
      if (poscursor == 3) sweepf2 = sweepf2 - 100000 ;
        if (poscursor == 4) sweepf2 = sweepf2 - 10000 ;
        if (poscursor == 5) sweepf2 = sweepf2 - 1000 ;
        if (poscursor == 6) sweepf2 = sweepf2 - 100 ;
        if (poscursor == 8) sweepf2 = sweepf2 - 10 ;
        if (poscursor == 9) sweepf2 = sweepf2 - 1 ;
        if (RFint < 3450) sweepf2 = 3450;
        if (RFint > 440000)  sweepf2 = 440000;

        if (poscursor == 15) {
          submenuflag=0; 
          line=0;
          printAll();
          break;
        }
        printsubmenu();
      }else{
        if (poscursor == 5){memoire--; 
        if (memoire==255)memoire=19;
        if (WEE==0){RFint=EEPROMReadlong(memoire*4); // leitura EEPROM e mostrador
           if (RFint>440000) RFint=440000;
          } 
        } // fin poscursor =5 

       if (poscursor==12){ 
       if( PFDRFout==10){PFDRFout=25;} //regulagem de frequência
       else if ( PFDRFout==25){PFDRFout=10;}
       else PFDRFout=25;// no caso de PFDRF ser diferente de 10 e 25
       modif=1;
       }

       if (poscursor==15){
          if(powerprint==0) break;
          else powerprint=powerprint-1;
          }
                   
       if( (poscursor==0) && (WEE==1))WEE=0;
       else if ((poscursor==0)&&(WEE==0))WEE=1; 
       printAll();
      }
     }
      break; // fin bouton bas
 
    case btnSELECT:
      do {
        adc_key_in = analogRead(0);      // Testa soltar o botão
        delay(1); timer2++;        // increcementa o timer a cada 1 milisegundo
        if (timer2 > 600) { //aguarda 600 milisegundos
         if (WEE==1 || poscursor==15){ 
         if (line==1 && poscursor==15){ EEPROMWritelong(20*4,PFDRFout);EEPROM.write(100,55);} // escrita FREF
         else if (WEE==1) {EEPROMWritelong(memoire*4,RFint);EEPROM.write(101,55);}// escrita RF na EEPROM no endereço de memória*4
          lcd.setCursor(0,1); lcd.print("  MEMORISATION  ");}
          lcd.setCursor(poscursor,line);
          delay(500);timer2=0;
          printsubmenu();
        }; // mes

        } 
      while (adc_key_in < 900); // espera soltar o botão
      break;  // fim do botão select
      
     case btnNONE: {
      break;
      };
      break;
  }

   do { adc_key_in = analogRead(0); delay(1);} while (adc_key_in < 900);
  
}

void powerleveladjust(){
  
  if(powerprint!=powerlevel){

  powerlevel=powerprint;
  
if(powerlevel==4){
      bitWrite (registers[4], 5, 1);//bit0 output power
      bitWrite (registers[4], 3, 1);//bit0 output power
      bitWrite (registers[4], 4, 1);//bit1 output power
      }

     if(powerlevel==3){
      bitWrite (registers[4], 5, 1);//bit0 output power
      bitWrite (registers[4], 3, 0);//bit0 output power
      bitWrite (registers[4], 4, 1);//bit1 output power
      }

     if(powerlevel==2){
      bitWrite (registers[4], 5, 1);//bit0 output power
      bitWrite (registers[4], 3, 1);//bit0 output power
      bitWrite (registers[4], 4, 0);//bit1 output power
      }

     if(powerlevel==1){
      bitWrite (registers[4], 5, 1);//bit0 output power
      bitWrite (registers[4], 3, 0);//bit0 output power
      bitWrite (registers[4], 4, 0);//bit1 output power
      }

     if(powerlevel==0){
      bitWrite (registers[4], 5, 0);//bit0 output power
     }
SetADF4351();
}
}

void sweepfunction(){

while(sweepflag==1){
      RFout=RFint;
      RFout=RFout/100;
      powerleveladjust();
      outputcontrol();
    
    if(sweepcount==0){
      RFint=sweepf1;
      sweepcount=1;
    }else{
    if(RFint>(sweepf2)) {
      RFint=sweepf1;
      sweepcount=0;
      }else{
      RFint=RFint+1000; //precisa ser baseado no RFint com ele mesmo
      }
    }

    processbuttons();
    delay(300);
}
  
}

void outputcontrol(){
  
  if ((RFint != RFintold)|| (modif==1) || (sweepflag==1)) {

    if (RFout > 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
      bitWrite (registers[4], 5, 1);//bit0 output power P1
      bitWrite (registers[4], 3, 0);//bit0 output power P1
      bitWrite (registers[4], 4, 0);//bit1 output power P1
      digitalWrite(0, HIGH);
      digitalWrite(1, LOW);
    }
    if (RFout <= 2200) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
      digitalWrite(0, HIGH);
      digitalWrite(1, LOW);
    }

    if (RFout <=2050){
      bitWrite (registers[4], 5, 1);//bit0 output power P3
      bitWrite (registers[4], 3, 0);//bit0 output power P3
      bitWrite (registers[4], 4, 1);//bit1 output power P3
      }

    if(RFout <=1960){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
    }

    if(RFout <=1850){
      bitWrite (registers[4], 5, 1);//bit0 output power P1
      bitWrite (registers[4], 3, 0);//bit0 output power P1
      bitWrite (registers[4], 4, 0);//bit1 output power P1
    }

    if(RFout <=1650){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
    }    

    if(RFout <=1570){
      bitWrite (registers[4], 5, 1);//bit0 output power P3
      bitWrite (registers[4], 3, 0);//bit0 output power P3
      bitWrite (registers[4], 4, 1);//bit1 output power P3
    }        
    
    if(RFout <=1450){
      bitWrite (registers[4], 5, 1);//bit0 output power P3
      bitWrite (registers[4], 3, 0);//bit0 output power P3
      bitWrite (registers[4], 4, 1);//bit1 output power P3
      digitalWrite(0, HIGH);
      digitalWrite(1, HIGH);      
    }   

    if(RFout <=1400){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
    }   

    if(RFout <=1300){
      bitWrite (registers[4], 5, 1);//bit0 output power P1
      bitWrite (registers[4], 3, 0);//bit0 output power P1
      bitWrite (registers[4], 4, 0);//bit1 output power P1
    }         

    if (RFout <=1250){
      bitWrite (registers[4], 5, 1);//bit0 output power P1
      bitWrite (registers[4], 3, 0);//bit0 output power P1
      bitWrite (registers[4], 4, 0);//bit1 output power P1
      }   

    if (RFout <=1230){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
      }      

    if (RFout <= 1100) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);    
      }            

    if (RFout <=1100){
      bitWrite (registers[4], 5, 1);//bit0 output power P3
      bitWrite (registers[4], 3, 0);//bit0 output power P3
      bitWrite (registers[4], 4, 1);//bit1 output power P3
      }  

    if (RFout <=930){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
      }        

    if (RFout <=900){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
      }  

    if (RFout <=770){
      bitWrite (registers[4], 5, 1);//bit0 output power P3
      bitWrite (registers[4], 3, 0);//bit0 output power P3
      bitWrite (registers[4], 4, 1);//bit1 output power P3
      }          

    if (RFout <=700){
      bitWrite (registers[4], 5, 1);//bit0 output power P3
      bitWrite (registers[4], 3, 0);//bit0 output power P3
      bitWrite (registers[4], 4, 1);//bit1 output power P3
      }                       

    if (RFout < 660)  {
      bitWrite (registers[4], 5, 1);//bit0 output power P4
      bitWrite (registers[4], 3, 1);//bit0 output power P4
      bitWrite (registers[4], 4, 1);//bit1 output power P4 
      digitalWrite(0, LOW);
      digitalWrite(1, HIGH);           
    }
    
    if (RFout < 550)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);   
      bitWrite (registers[4], 5, 1);//bit0 output power P1
      bitWrite (registers[4], 3, 0);//bit0 output power P1
      bitWrite (registers[4], 4, 0);//bit1 output power P1 
      digitalWrite(0, LOW);
      digitalWrite(1, HIGH);
    }

    if (RFout <=540){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
      } 
    
    if (RFout < 275)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
      bitWrite (registers[4], 5, 1);//bit0 output power P4
      bitWrite (registers[4], 3, 1);//bit0 output power P4
      bitWrite (registers[4], 4, 1);//bit1 output power P4       
      digitalWrite(0, LOW);
      digitalWrite(1, LOW);
    }

    if (RFout <=230){
      bitWrite (registers[4], 5, 1);//bit0 output power P2
      bitWrite (registers[4], 3, 1);//bit0 output power P2
      bitWrite (registers[4], 4, 0);//bit1 output power P2
      }     
    
    if (RFout < 137.5) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
      digitalWrite(0, LOW);
      digitalWrite(1, LOW);
    }
    if (RFout < 68.75) {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
      digitalWrite(0, LOW);
      digitalWrite(1, LOW);
    }

    INTA = (RFout * OutputDivider) / PFDRFout;
    MOD = (PFDRFout / OutputChannelSpacing);
    FRACF = (((RFout * OutputDivider) / PFDRFout) - INTA) * MOD;
    FRAC = round(FRACF); // arrendonda o resultado

    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;

    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // adiciona o endereço "001"
    bitSet (registers[1], 27); // Prescaler em 8/9

    bitSet (registers[2], 28); // Travado digitalmente == "110" nos b28 b27 b26
    bitSet (registers[2], 27); // Travado digitalmente
    bitClear (registers[2], 26); // Travado digitalmente
   
    SetADF4351();  // Programa todos os registros do ADF4351
    
    RFintold=RFint;
    
    modif=0;
    
    if(submenuflag==0)    printAll();  // Affichage LCD
    if(submenuflag==1)    printsubmenu();
  }
}

// *************** SP escrita de palavra long (32bits) na EEPROM entre endereço e endereço+3 **************
void EEPROMWritelong(int address, long value)
      {
      //Decomposition du long (32bits) en 4 bytes
      //trois = MSB -> quatre = lsb
      byte quatre = (value & 0xFF);
      byte trois = ((value >> 8) & 0xFF);
      byte deux = ((value >> 16) & 0xFF);
      byte un = ((value >> 24) & 0xFF);

      //Ecrit 4 bytes dans la memoire EEPROM
      EEPROM.write(address, quatre);
      EEPROM.write(address + 1, trois);
      EEPROM.write(address + 2, deux);
      EEPROM.write(address + 3, un);
      }

// *************** SP leitura de palavra long (32bits) na EEPROM entre endereço e endereço+3 **************
long EEPROMReadlong(long address)
      {
      //Read the 4 bytes from the eeprom memory.
      long quatre = EEPROM.read(address);
      long trois = EEPROM.read(address + 1);
      long deux = EEPROM.read(address + 2);
      long un = EEPROM.read(address + 3);

      //Retourne le long(32bits) utilizando o deslocamento de 0, 8, 16 e 24 bits e as máscaras
      return ((quatre << 0) & 0xFF) + ((trois << 8) & 0xFFFF) + ((deux << 16) & 0xFFFFFF) + ((un << 24) & 0xFFFFFFFF);
      }
//************************************ Setup ****************************************
void setup() {
  lcd.begin(16, 2); // duas linhas de 16 caracteres
  lcd.display();
  analogWrite(10,255); //Luminosidade LCD

  //Serial.begin (115200); //  Serial para o PC via Arduino "Serial Monitor"  com taxa 19200
  lcd.print("    GERADOR   ");
  lcd.setCursor(0, 1);
  lcd.print("    ADF4351     ");
  poscursor = 7; line = 0; 
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("Mod por Liraneto");
   delay(1000);

  pinMode(2, INPUT);  // PIN 2 na entrada para travamento
  pinMode(ADF4351_LE, OUTPUT);          // pinos setup
  pinMode(0,OUTPUT); // RF switch control
  pinMode(1,OUTPUT); // RF switch control
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();                          // Inicia barramento SPI
  SPI.setDataMode(SPI_MODE0);           // CPHA = 0 e Clock positivos
  SPI.setBitOrder(MSBFIRST);            // peso forte na frente

  if (EEPROM.read(100)==55){PFDRFout=EEPROM.read(20*4);} // se a referência é escrita na EEPROM, nós a ligamos
  else {PFDRFout=25;}

  if (EEPROM.read(101)==55){RFint=EEPROMReadlong(memoire*4);} // se a referência é escrita na EEPROM, nós a ligamos
  else {RFint=7000;}

  RFintold=1234;//para que RFintold seja diferente de RFout assim que é iniciado
  RFout = RFint/100 ; // frequência de saída
  OutputChannelSpacing = 0.01; // Sem frequência = 10kHz

  WEE=0;  address=0;
  lcd.blink();
  printAll();
  delay(500);


} // Fim setup

//*************************************Loop***********************************
void loop()
{

  sweepfunction();

  RFout=RFint;
  RFout=RFout/100;
  
  powerleveladjust();
  outputcontrol();

  processbuttons();

   //cursor ocioso desliga 
   delay (10);
   timer++; // inc timer
   if (timer>1000){
    lcd.noBlink();timer=0;
    }

}   // fim loop
