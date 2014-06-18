
#ifndef Tribots_GameState_h
#define Tribots_GameState_h

#include "../Fundamental/Time.h"

namespace Tribots {

  /** Liste der moeglichen Spielphasen */
  enum RefereeState {
    stopRobot,               ///< stop, kein Roboter bewegt sich
    freePlay,                ///< freies Spiel, keine Einschraenkungen
    preOwnKickOff,           ///< vor dem Anstoss, Roboter positionieren sich
    preOpponentKickOff,      ///< vor dem Anstoss, Roboter positionieren sich
    postOpponentKickOff,     ///< nach der Ballfreigabe fuer den Gegner, eigenes Team darf Ball noch nicht beruehren
    preOwnGoalKick,          ///< vor dem Torabschlag
    preOpponentGoalKick,     ///< vor dem Torabschlag
    postOpponentGoalKick,    ///< nach der Ballfreigabe fuer den Gegner, eigenes Team darf noch nicht in den gegnerischen Strafraum
    preOwnCornerKick,        ///< vor dem Eckball
    preOpponentCornerKick,   ///< vor dem Eckball
    postOpponentCornerKick,  ///< nach der Ballfreigabe fuer den Gegner, eigenes Team muss 2m Abstand zum Ball halten
    preOwnThrowIn,           ///< vor dem Ein"wurf"
    preOpponentThrowIn,      ///< vor dem Ein"wurf"
    postOpponentThrowIn,     ///< nach der Ballfreigabe fuer den Gegner, eigenes Team darf Ball noch nicht beruehren
    preOwnFreeKick,          ///< vor dem Freistoss
    preOpponentFreeKick,     ///< vor dem Freistoss
    postOpponentFreeKick,    ///< nach der Ballfreigabe fuer den Gegner, eigenes Team muss 2m Abstand zum Ball halten
    preOwnPenalty,           ///< vor dem Strafstoss
    preOpponentPenalty,      ///< vor dem Strafstoss
    postOpponentPenalty,     ///< nach der Ballfreigabe fuer den Gegner, Torwart darf noch nicht an den Ball
    ownPenalty,              ///< Strafstossdurchfuehrung des Gegners
    opponentPenalty,         ///< Strafstossdurchfuehrung des eigenen Teams
    preDroppedBall,          ///< "DroppedBall"
    errorState,              ///< irgendein Fehlerzustand
    testState1,              ///< Testzustaende
    testState2,
    testState3,
    testState4,
    testState5,
    testState6,
    testState7,
    testState8,
  };


  /** Liste der moeglichen Refereebox-Signale */
  enum RefboxSignal {
    SIGnop,                  ///< ohne Bedeutung, veraendert nichts 
    SIGstop,
    SIGhalt,
    SIGstart,
    SIGready,
    SIGcyanKickOff,
    SIGmagentaKickOff,
    SIGownKickOff,
    SIGopponentKickOff,
    SIGcyanFreeKick,
    SIGmagentaFreeKick,
    SIGownFreeKick,
    SIGopponentFreeKick,
    SIGcyanGoalKick,
    SIGmagentaGoalKick,
    SIGownGoalKick,
    SIGopponentGoalKick,
    SIGcyanCornerKick,
    SIGmagentaCornerKick,
    SIGownCornerKick,
    SIGopponentCornerKick,
    SIGcyanThrowIn,
    SIGmagentaThrowIn,
    SIGownThrowIn,
    SIGopponentThrowIn,
    SIGcyanPenalty,
    SIGmagentaPenalty,
    SIGownPenalty,
    SIGopponentPenalty,
    SIGcyanGoalScored,
    SIGmagentaGoalScored,
    SIGownGoalScored,
    SIGopponentGoalScored,
    SIGdroppedBall,
    SIGtest1,
    SIGtest2,
    SIGtest3,
    SIGtest4,
    SIGtest5,
    SIGtest6,
    SIGtest7,
    SIGtest8,
  };



  enum TeamSignal {
    TeamSIGnop,                  ///< ohne Bedeutung, veraendert nichts
    TeamSIGstop,
    TeamSIGstayaway,
    TeamSIGgivePass,
    TeamSIGgoForIt,
    TeamSIGhelpDefend,
    TeamSIGreceivePass,
  };


  static const int num_team_signals = 8;                  ///< Anzahl moeglicher Signale
  static const char teamsignal_names [8][23] = {         ///< Bezeichner (strings) der verschiedenen Signale
    "TeamSIGnop           ",
    "TeamSIGstop          ",
    "TeamSIGstayAway      ",
    "TeamSIGgivePass      ",
    "TeamSIGgoForIt       ",
    "TeamSIGhelpDefend    ",
    "TeamSIGreceivePass   ",

  };





  /** Struktur, um die momentane Spielphase zu erfassen */
  struct GameState {
    RefereeState refstate;           ///< aktueller Refereestate
    int teamsignal;

    Time latest_update;              ///< zeitpunkt der letzten Aktualisierung von refstate
    bool in_game;                    ///< true, wenn der Roboter im Spiel ist, false sonst

    unsigned int own_score;          ///< Torerfolge eigenes Team
    unsigned int opponent_score;     ///< Torerfolge gegnerisches Team
    unsigned int yellow_cards;       ///< Anzahl gelbe Karten

    double intended_cycle_time;      ///< vorgesehene Zykluszeit
    double actual_cycle_time;        ///< tatsaechliche Zykluszeit
    unsigned long int cycle_num;     ///< Zyklusnummer
    Time cycle_start_time;           ///< Zeit, zu der Zyklus gestartet
    Time expected_execution_time;    ///< Zeit, zu der die Umsetzung des Fahrtbefehls erwartet wird

    GameState () throw ();
    GameState (const GameState&) throw ();
    const GameState& operator= (const GameState&) throw ();
  };


  static const int num_referee_states = 32;                  ///< Anzahl moeglicher Spielsituationen
  static const char referee_state_names [32][23] = {         ///< Bezeichner (strings) der verschiedenen Spielsituationen
    "Stop                  ",
    "Freies Spiel          ",
    "vor eigenem Anstoss   ",
    "vor Anstoss Gegner    ",
    "nach Anstoss Gegner   ",
    "vor eigenem Torabstoss",
    "vor Torabstoss Gegner ",
    "nach Torabstoss Gegner",
    "vor eigenem Eckball   ",
    "vor Eckball Gegner    ",
    "nach Eckball Gegner   ",
    "vor eigenem Einwurf   ",
    "vor Einwurf Gegner    ",
    "nach Einwurf Gegner   ",
    "vor eigenem Freistoss ",
    "vor Freistoss Gegner  ",
    "nach Freistoss Gegner ",
    "vor eigenem Strafstoss",
    "vor Strafstoss Gegner ",
    "nach Strafstoss Gegner",
    "eigener Strafstoss    ",
    "Strafstoss Gegner     ",
    "vor DroppedBall       ",
    "Fehler                ",
    "Testzustand 1         ",
    "Testzustand 2         ",
    "Testzustand 3         ",
    "Testzustand 4         ",
    "Testzustand 5         ",
    "Testzustand 6         ",
    "Testzustand 7         ",
    "Testzustand 8         ",
  };


  static const int num_refbox_signals = 42;                  ///< Anzahl moeglicher Signale
  static const char refbox_signal_names [42][23] = {         ///< Bezeichner (strings) der verschiedenen Signale
    "SIGnop                ",
    "SIGstop               ",
    "SIGhalt               ",
    "SIGstart              ",
    "SIGready              ",
    "SIGcyanKickOff        ",
    "SIGmagentaKickOff     ",
    "SIGownKickOff         ",
    "SIGopponentKickOff    ",
    "SIGcyanFreeKick       ",
    "SIGmagentaFreeKick    ",
    "SIGownFreeKick        ",
    "SIGopponentFreeKick   ",
    "SIGcyanGoalKick       ",
    "SIGmagentaGoalKick    ",
    "SIGownGoalKick        ",
    "SIGopponentGoalKick   ",
    "SIGcyanCornerKick     ",
    "SIGmagentaCornerKick  ",
    "SIGownCornerKick      ",
    "SIGopponentCornerKick ",
    "SIGcyanThrowIn        ",
    "SIGmagentaThrowIn     ",
    "SIGownThrowIn         ",
    "SIGopponentThrowIn    ",
    "SIGcyanPenalty        ",
    "SIGmagentaPenalty     ",
    "SIGownPenalty         ",
    "SIGopponentPenalty    ",
    "SIGcyanGoalScored     ",
    "SIGmagentaGoalScored  ",
    "SIGownGoalScored      ",
    "SIGopponentGoalScored ",
    "SIGdroppedBall        ",
    "SIGtest1              ",
    "SIGtest2              ",
    "SIGtest3              ",
    "SIGtest4              ",
    "SIGtest5              ",
    "SIGtest6              ",
    "SIGtest7              ",
    "SIGtest8              ",
  };

}

#endif

