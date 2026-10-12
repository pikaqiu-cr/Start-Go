#ifndef _PRINT_H
#define _PRINT_H

//#define DEBUG // Faut-il passer en mode débogage ?
//#define RLEDEBUG // Faut-il activer le débogage de la librairie rle ?

// Choix des ports séries : entre 0 et 3 
#define SPOUT 0 // Où vont les affichages lors de l'utilisation des macros printL et printLN ?
#define SPIN 0 // D'où viennent les données lors de l'utilisation des macros READ() READSTRING() et AVAIL() ? 

/////  NE PAS EDITER SOUS CETTE LIGNE ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MYAVAIL(sp) ((sp==0)?(Serial.available()):((sp==1)?(Serial1.available()):((sp==2)?(Serial2.available()):(Serial3.available()))))
#define MYREAD(sp) ((sp==0)?(Serial.read()):((sp==1)?(Serial1.read()):((sp==2)?(Serial2.read()):(Serial3.read()))))
#define MYREADSTRING(sp) ((sp==0)?(Serial.readString()):((sp==1)?(Serial1.readString()):((sp==2)?(Serial2.readString()):(Serial3.readString()))))

#define AVAIL() MYAVAIL(SPIN)
#define READ() MYREAD(SPIN)
#define READSTRING() MYREADSTRING(SPIN)

#if(SPOUT==0)
  #define NL() Serial.println("");
  #define PRINT(s) Serial.print(s);
  #define PRINTLN(s) Serial.println(s);
  #define LPRINT(lbl,val) Serial.print(lbl);Serial.print(": ");Serial.print(val);
  #define LPRINTLN(lbl,val) Serial.print(lbl);Serial.print(": ");Serial.println(val);
#endif 
#if(SPOUT==1)
  #define NL() Serial1.println("");
  #define PRINT(s) Serial1.print(s);
  #define PRINTLN(s) Serial1.println(s);
  #define LPRINT(lbl,val) Serial1.print(lbl);Serial1.print(": ");Serial1.print(val);
  #define LPRINTLN(lbl,val) Serial1.print(lbl);Serial1.print(": ");Serial1.println(val);
#endif 
#if(SPOUT==2)
  #define NL() Serial2.println("");
  #define PRINT(s) Serial2.print(s);
  #define PRINTLN(s) Serial2.println(s);
  #define LPRINT(lbl,val) Serial2.print(lbl);Serial2.print(": ");Serial2.print(val);
  #define LPRINTLN(lbl,val) Serial2.print(lbl);Serial2.print(": ");Serial2.println(val);
#endif 
#if(SPOUT==3)
  #define NL() Serial3.println("");
  #define PRINT(s) Serial3.print(s);
  #define PRINTLN(s) Serial3.println(s);
  #define LPRINT(lbl,val) Serial3.print(lbl);Serial3.print(": ");Serial3.print(val);
  #define LPRINTLN(lbl,val) Serial3.print(lbl);Serial3.print(": ");Serial3.println(val);
#endif 

#ifdef DEBUG
	#define DPRINT(s) PRINT(s)
	#define DPRINTLN(s) PRINTLN(s)	 
  #define DLPRINT(lbl,val) LPRINT(lbl,val)
  #define DLPRINTLN(lbl,val) LPRINTLN(lbl,val)
#else 
	#define DPRINT(s) 
	#define DPRINTLN(s)
  #define DLPRINT(lbl,val)
  #define DLPRINTLN(lbl,val) 
#endif

#ifdef RLEDEBUG
	#define RDPRINT(s) PRINT(s)
	#define RDPRINTLN(s) PRINTLN(s)	 
  #define RDLPRINT(lbl,val) LPRINT(lbl,val)
  #define RDLPRINTLN(lbl,val) LPRINTLN(lbl,val)
#else 
	#define RDPRINT(s) 
	#define RDPRINTLN(s)
  #define RDLPRINT(lbl,val)
  #define RDLPRINTLN(lbl,val) 
#endif

#endif
