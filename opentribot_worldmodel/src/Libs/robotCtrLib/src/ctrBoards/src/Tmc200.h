#ifndef _TMC200_H_
#define _TMC200_H_

#include "SingleDrive.h"
#include "sercom.h"
#include <queue>

#define  TMC200_CHAPTER    "TMC200"
#define  TMC_COM_BUF_SIZE  512
#define  MAX_ANSW_PARAMS   15
#define  MAX_NUM_DATA      10

#define TV_DIFF(__TV1__, __TV2__) (__TV2__.tv_sec-__TV1__.tv_sec)*1000.0 + (__TV2__.tv_usec-__TV1__.tv_usec)/1000.


typedef struct {
  float   data;
  timeval tv;
} TRealLabeledData;

typedef struct {
    int     data;
    timeval tv;
  } TLabeledData;

typedef struct tmc_real_data_s{
  tmc_real_data_s()
  {
    for (int i=0; i<3; i++) {
      V[i].data=0; V[i].tv.tv_sec=-1; V[i].tv.tv_usec=-1; 
      C[i].data=0; C[i].tv.tv_sec=-1; C[i].tv.tv_usec=-1;
      D[i].data=0; D[i].tv.tv_sec=-1; D[i].tv.tv_usec=-1;
      O[i].data=0; O[i].tv.tv_sec=-1; O[i].tv.tv_usec=-1;
      T[i].data=0; T[i].tv.tv_sec=-1; T[i].tv.tv_usec=-1;
      SS[i].data=0; SS[i].tv.tv_sec=-1; SS[i].tv.tv_usec=-1;
    } 
    n.data=-1;   n.tv.tv_sec=-1;  n.tv.tv_usec=-1;
    dt.data=-1;  dt.tv.tv_sec=-1; dt.tv.tv_usec=-1;
    BE.data=0;  BE.tv.tv_sec=-1; BE.tv.tv_usec=-1;
    UB.data=0;  UB.tv.tv_sec=-1; UB.tv.tv_usec=-1;
  };
  
  void print(std::ostream &out)
  {
    out << "[V:(";
    for (int i=0; i<3; i++) out << " " << V[i].data << " ";
    out << ")" << " t: " << V[0].tv.tv_sec << "s " <<  V[0].tv.tv_usec << "us ]\n";
    
    out << "[C:(";
    for (int i=0; i<3; i++) out << " " << C[i].data << " ";
    out << ")" << " t: " << C[0].tv.tv_sec << "s " <<  C[0].tv.tv_usec << "us ]\n";
    
    out << "[n:(" << n.data << ") t: " << n.tv.tv_sec<< "s " <<  n.tv.tv_usec << "us ]\n";
    
    out << "[UB: (" << UB.data << ") t: " << UB.tv.tv_sec<< "s " <<  UB.tv.tv_usec << "us ]\n";
    
    out << "[T:(";
    for (int i=0; i<3; i++) out << " " << T[i].data << " ";
    out << ")" << " t: " << T[0].tv.tv_sec << "s " <<  T[0].tv.tv_usec << "us ]\n";
    
    out << "[SS Motor:  1(" << SS[0].data << ") 2(" << SS[1].data << ") 3(" << SS[2].data <<") t: " << SS[0].tv.tv_sec<< "s " <<  SS[0].tv.tv_usec << "us ]\n";
  };

  // Timestamp für alle daten hängt von der verwendeten Kommunikation ab! 
  // Entspricht Zeit zu der das Datum eingelesen wurde, nicht unbedingt der Zeit an dem das Datum empfangen wurde!!
  TRealLabeledData V[3];   // Geschwindigkeit für Getriebeausgang in rad/s
  TRealLabeledData C[3];   // Ströme in A
  TRealLabeledData n;      // Nachrichtenzähler
  TRealLabeledData dt;     // Zykluszeit in ms
  TRealLabeledData D[3];   // Distanz ??
  TRealLabeledData O[3];   // PWM output in percent
  TLabeledData     BE;     // flag für Motorendstufe (3bit)
  TRealLabeledData UB;     // Versorgungsspannung in  V
  TRealLabeledData T[3];   // Temperatur Motorwicklungen in Grad Cels
  TLabeledData     SS[3];     // Flag für Motortemperaturabschaltung für Motor 1,2,3
} tmc_real_data_t;





class Tmc200
{

 public:
  typedef struct tmc_data_s{
    tmc_data_s()
    {
      for (int i=0; i<3; i++) {
	V[i].data=0; V[i].tv.tv_sec=-1; V[i].tv.tv_usec=-1; 
	C[i].data=0; C[i].tv.tv_sec=-1; C[i].tv.tv_usec=-1;
	D[i].data=0; D[i].tv.tv_sec=-1; D[i].tv.tv_usec=-1;
	O[i].data=0; O[i].tv.tv_sec=-1; O[i].tv.tv_usec=-1;
	T[i].data=0; T[i].tv.tv_sec=-1; T[i].tv.tv_usec=-1;
	} 
      n.data=-1;   n.tv.tv_sec=-1;  n.tv.tv_usec=-1;
      dt.data=-1;  dt.tv.tv_sec=-1; dt.tv.tv_usec=-1;
      BE.data=0;  BE.tv.tv_sec=-1; BE.tv.tv_usec=-1;
      UB.data=0;  UB.tv.tv_sec=-1; UB.tv.tv_usec=-1;
      SS.data=0;  SS.tv.tv_sec=-1; SS.tv.tv_usec=-1;
    };
    // Timestamp für alle daten hängt von der verwendeten Kommunikation ab! 
    // Entspricht Zeit zu der das Datum eingelesen wurde, nicht unbedingt der Zeit an dem das Datum empfangen wurde!!
    TLabeledData V[3];   // Geschwindigkeit für Motorausgang 1024 entspricht Leerlaufdrehzahl
    TLabeledData C[3];   // Ströme 1024 entspricht 10A
    TLabeledData n;      // Nachrichtenzähler
    TLabeledData dt;     // Zykluszeit in ms
    TLabeledData D[3];   // Distanz
    TLabeledData O[3];   // PWM
    TLabeledData BE;     // Fehler bit der Motorendstufe (SW Vers >= 2)
    TLabeledData UB;     // Versorgungsspannung in ticks; Umrechnung V= (ticks *1/23) + 1.5 (SW Vers >= 2)
    TLabeledData T[3];   // Temperatur der Motorwicklungen direkt in Grad Cels (SW Vers >= 2)
    TLabeledData SS;     // Boolscher Wert für die Temperaturschranke (SW Vers >= 2)
  } tmc_data_t;


 protected:
  static const int workingmode_default = 2;
  static const int sendmode_default    = 1;
  static const int tw_default          = 27;
  static const int ktn_default         = 2;
  static const int ktz_default         = 16;
  static const int tmc_version_default = 0;
  static const int c_limit_mode_default =  0;
  static const int max_temp_out_default = 155;
  static const int max_temp_in_default  =  50;
  static const int c_measure_mode_default = 0;

  bool lost_connection; // flag for status of connection to board
  int       cmd_counter;
  long int  loopcounter;

  
  std::ostream *errout, *infoout;
  
  bool    tmc_com_res;                   // globale Variable benutzt von Makro TALK
  char    tmc_command[TMC_COM_BUF_SIZE]; // globale Variable benutzt von Makro TALK
  char    tmc_answer[TMC_COM_BUF_SIZE];  // globale Variable benutzt von Makro TALK
  timeval tmc_send_time;
  timeval tmc_recv_time;
  int     answ_params[MAX_ANSW_PARAMS];
  tmc_data_t tmc_data[MAX_NUM_DATA];


  char config_fname[200];
  SerCom * sercom;
  

  std::queue<TLabeledData> not_received_async_cmds;
  static const int ASYNC_CMD_VEL    = 0;
  static const int ASYNC_CMD_PORT1L = 1;
  static const int ASYNC_CMD_WM     = 2;

  // tmc params
  struct {
    int tmc_version;
    int workingmode;
    int sendmode;
    int c_limit_mode;
    int c_measure_mode;
    SingleDrive * Drive[3];
    int tw, ktn, ktz; // parameter des thermischen Modells (für alle Motoren gleich)
    int max_temp_out, max_temp_in; // parameter der thermischen hysterese Abschaltung
    float vcc;
  } params;

  void init_tmc_params();
  bool read_tmc_params(const char *_f_name, const char *_chapter);
  
  bool cmpt_radvel_to_tmcvel(float  _v1_rad, float  _v2_rad, float  _v3_rad,
			     int   &_v1_tmc, int   &_v2_tmc, int   &_v3_tmc);
  bool cmpt_tmcvel_to_radvel(int    _v1_tmc, int    _v2_tmc, int    _v3_tmc,
			     float &_v1_rad, float &_v2_rad, float &_v3_rad);
  bool cmpt_real_data       (tmc_data_t *tmc_data, tmc_real_data_t &tmc_real_data);

  bool parse_answer_string(char * _answer, tmc_data_t &_data, timeval *tstmp);

 public:
  // blocking version of communication
  bool setVelocity   (int v1,    int v2,    int v3,    tmc_data_t &_data,      timeval *tstmp = NULL);
  // non blocking version of communication
  bool setVelocity   (int v1,    int v2,    int v3,    timeval *tstmp = NULL);
  // non blocking version of communication, you will receive nothing here with the blocking setVelocity
  int  getTmcData(tmc_data_t      * _data_list, int _max_num, bool _all=false);

 protected:
  bool ping();
  bool reinit();

 public:
  Tmc200(const char* _conf_fname, std::ostream *_errout = &std::cerr, std::ostream *_infoout = &std::cout);
  ~Tmc200();

  static const int WM_DO    = 0; // working mode direct output
  static const int WM_DO_C  = 1; // working mode direct output with current sensing
  static const int WM_PID_C = 2; // working mode pid control with current sensing
  
  static const int SM_NOTHING    = 0;
  static const int SM_V          = 1;
  static const int SM_C          = 2;
  static const int SM_N          = 3;
  static const int SM_D          = 4;
  static const int SM_P          = 5;
 
  // TMC Version: OLD: old Boards used in DO; 16: New Board OS SW1.16; 116: New Boards OS SW1.16 (newest)
  static const int VERSION_OLD  = 0;
  static const int VERSION_16   = 1;
  static const int VERSION_116  = 2;

  static const int CLIMIT_MODE_IND   = 0;
  static const int CLIMIT_MODE_SIM   = 1;

  static const int C_MEASURE_MODE_LIN = 0;
  static const int C_MEASURE_MODE_KOR = 1;

  bool  init();

  int   get_tmc_version(){return params.tmc_version;};

  // what maximal motor velocity is possible for drive i? in [rad/s] at gear output i from (0,1,2)
  float get_max_motor_vel(int i=0);

  void print_params(std::ostream &_out);

  bool setWorkingMode(int _mode);
  bool setWorkingMode2(int _mode);
  bool getWorkingMode();
  bool setWorkingMode_async(int _mode);
  bool getWorkingMode(int &_mode);


  bool setSendMode(int _mode);
  bool setSendMode2(int _mode);
  bool getSendMode();
  bool getSendMode(int &_mode);

  // _motor: motor number 1-3
  // _imp  : number of impulses of encoder
  // _max_v: max vel of motor [turns/min]
  // _dist : distance moving in one encoder impulse (gearfactor)
  // the ...2 funktions are with param check
  bool setEncParam(int _motor, int _imp, int _max_v, int _dist);
  bool setEncParam2(int _motor, int _imp, int _max_v, int _dist);
  bool getEncParam(int _motor, int &_imp, int &_max_v, int &_dist);
  bool getEncParam2(int _motor, int &_imp, int &_max_v, int &_dist);
  bool getEncParam(int _motor);
  bool getEncParam2(int _motor);

  bool setPIDParam2(int _motor, int _kp, int _ki, int _kd);
  bool setPIDParam(int _motor, int _kp, int _ki, int _kd);
  bool getPIDParam2(int _motor, int &_kp, int &_ki, int &_kd);
  bool getPIDParam(int _motor, int &_kp, int &_ki, int &_kd);
  bool getPIDParam2(int _motor);
  bool getPIDParam(int _motor);

  bool setCLimitParam2(int _motor, int _c_nom, int _c_max);
  bool setCLimitParam(int _motor, int _c_nom, int _c_max);
  bool getCLimitParam2(int _motor);
  bool getCLimitParam2(int _motor, int &_c_nom, int &_c_max);
  bool getCLimitParam(int _motor);
  bool getCLimitParam(int _motor, int &_c_nom, int &_c_max);

  bool setCLimitMode(int _mode);
  bool getCLimitMode(int &_mode);
  bool getCLimitMode();

  bool setCMeasureMode(int _mode);

  bool setTModelParams2(int _motor, int _tw, int _ktn, int _ktz);
  bool setTModelParams(int _motor, int _tw, int _ktn, int _ktz);
  bool getTModelParams2(int _motor);
  bool getTModelParams(int _motor);
  bool getTModelParams2(int _motor, int &_tw, int &_ktn, int &_ktz);
  bool getTModelParams(int _motor, int &_tw, int &_ktn, int &_ktz);

  bool setTHystParams2(int _motor, int _max_out, int _min_in);
  bool setTHystParams(int _motor, int _max_out, int _min_in);
  bool getTHystParams2(int _motor);
  bool getTHystParams(int _motor);
  bool getTHystParams2(int _motor, int &_max_out, int &_min_in);
  bool getTHystParams(int _motor, int &_max_out, int &_min_in);

  /* read power supply voltage of tmc, value in volt */
  bool getPowerSupplyVoltage(float &_v);

  /* read the motor temperature model values */
  bool getMotorTemp(int &t1, int &t2, int &t3);

  /* perform a boot on the tmc, all params set on TMC are lost!! 
     this functionality is only available in TMC version > 1.16 */
  bool rebootTMC();

  /* function to set output of digital IO 1.6 and 1.7 of tmc (only outputs)
   * values 0,1,2,3 are possible */
  bool set_port1L(int _value, timeval *tstmp = NULL);
  bool set_port1L_async(int _value, timeval *tstmp = NULL);

  // blocking communication
  bool setVelocityRPS(float _v1, float _v2, float _v3, tmc_real_data_t &_data, timeval *tstmp = NULL);
  bool setVelocityU(float _v1, float _v2, float _v3, tmc_real_data_t &_data, timeval *tstmp=NULL);

  // non blocking communication
  bool setVelocityRPS(float _v1, float _v2, float _v3, timeval *tstmp = NULL);
  bool setVelocityU(float _v1, float _v2, float _v3, timeval *tstmp=NULL);

  int  getTmcData(tmc_real_data_t * _data_list, int _max_num, bool _all=false);

};

#endif
