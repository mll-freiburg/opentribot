#include <cmath>
#include "HeikoPlayer.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Fundamental/geometry.h"
#include "../WorldModel/WorldModel.h"
#include "../WorldModel/FataMorgana.h"
#include "../Behavior/Skills/WithoutBall/SPatrol.h"
#include "../Behavior/SPBehavior.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallDirectly.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"

#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"

#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"


// Innerhalb dieser Datei wird die spielertypspezifische Strategie des 
// Feldspielers festgelegt. Die vom Feldspieler verwendeten Behaviors und
// Skills sollten soweit wie m�lich generisch gehalten werden, d.h. 
// Strategiespezifische Einstellungen wie Aktionsbereiche und 
// Rollenspezifikationen sollten aussschlie�ich hier ber das setzen von 
// allgemeinen Parametern der Behaviors und durch Ableiten und �erschreiben 
// ihrer Aktivierungsbedingungen vorgenommen werden.

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("HeikoPlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, 
			    PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new HeikoPlayer (reader);
    }
  };
  Builder the_builder;
}

namespace Tribots {   // unbenannter Namespace notwenig, um Verwechslungen beim Linken mit FieldPlayer zu vermeiden

static string FieldPlayer07_role = "ballL";

static const char *_tribots_fp07_roles[5] = { "ballL", "ballR", "left", "right", "safety" };

bool
HeikoPlayer::set_role(const char* role) throw ()
{
  if (MultiRolePlayer::set_role(role)) {
    FieldPlayer07_role = std::string(role);      // fuer "interne" behaviors
    WBOARD->setZonePressureRole(std::string(role));
    return true;
  }
  return false;
}



} // Ende unbenannter Namespace

HeikoPlayer::HeikoPlayer (const ConfigReader& cfg) throw () 
  : BehaviorPlayer ("HeikoPlayer", _tribots_fp07_roles, 5) 
{
    
  set_role("ballL");  // anfaengliche Rolle setzen
    
  // Das WhiteBoard veranlassen, die Configs auzuwerten ///////////////////////
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Area, die der Roboter nicht verlassen darf festlegen (Training) //////////
  // ACHTUNG: Am 23.4.08 auf kleinere Seitenbanden eingestellt, da zu oft in Banden gefahren.
  XYRectangle area(Vec( fgeom.field_width /2. - 500., 
                        fgeom.field_length/2. ),
                   Vec(-fgeom.field_width /2. + 500.,
                       -fgeom.field_length/2. + 500)); 
  
  // Aktivitaetsbereiche fuer Rollen festlegen ////////////////////////////////
  Vec target(0., fgeom.field_length/4.);

  XYRectangle ballLeftActiveArea(Vec(-fgeom.field_width   /2. - 2000., 
                                     fgeom.field_length   /2. + 2000.), 
                                 Vec(fgeom.field_width    /2. -  500.,
                                     -fgeom.field_length  /2. - 2000.));
  XYRectangle ballRightActiveArea(Vec(fgeom.field_width   /2. + 2000., 
                                      fgeom.field_length  /2. + 2000.),
                                  Vec(-fgeom.field_width  /2. +  500.,
                                      -fgeom.field_length /2. - 2000.));
  
  // von gegn. grundlinie bis 2000.mm hinter grundlinie, von 2000.mm ausserhalb
  // der Seitenlinie bis 750.mm ber die tor-tor-linie (1,5m ueberlappung)
  XYRectangle leftActiveArea(Vec(-fgeom.field_width/2. - 2000., 
                                 fgeom.field_length/2.), 
                             Vec(3250., 
                                 -fgeom.field_length/2.-2000.));
  XYRectangle rightActiveArea(Vec(fgeom.field_width/2. + 2000., 
                                  fgeom.field_length/2),
                              Vec(-3250., 
                                  -fgeom.field_length/2.-2000.));
  // Bis etwas vor dem Torraum, nicht bis ganz zur seitenlinie
  XYRectangle safetyActiveArea(Vec(-fgeom.penalty_area_width/2.-1500., 
                                   -fgeom.field_length/2.-1000.),  // nicht sonderlich weit rausfahren
                               Vec(+fgeom.penalty_area_width/2.+1500., 
                                   -fgeom.field_length/2.+fgeom.penalty_area_length+2000.));

  // Kickdauern und Passwahrscheinlichkeiten einlesen /////////////////////////
  int longPassKickDuration = 40;
  int shortPassKickDuration = 20;
  int throwInKickDuration = 7; // nur fuer roboter mit harting kicker relevant
  int shotKickDuration = 34; // nur fuer roboter mit harting kicker relevant // GESCHWINDIGKEITSHACK
  double standardPassProbability = 0.75;
  double standardDribbling = 0.25;
  int standardPositioning = 0;
  
  if (cfg.get(("FieldPlayer07::shortPassKickDuration"), shortPassKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::shortPassKickDuration");
  }    
  if (cfg.get(("FieldPlayer07::longPassKickDuration"), longPassKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::longPassKickDuration");
  }   
  if (cfg.get(("FieldPlayer07::throwInKickDuration"), throwInKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::throwInKickDuration");
  } 
  if (cfg.get(("FieldPlayer07::shotKickDuration"), shotKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::shotKickDuration");
  } 
  if (cfg.get(("FieldPlayer07::standardPassProbability"), standardPassProbability) < 0) {
    throw InvalidConfigurationException("FieldPlayer07:standardPassProbability");
  }   
  if (cfg.get(("FieldPlayer07::standardDribbling"), standardDribbling) < 0) {
    throw InvalidConfigurationException("FieldPlayer07:standardDribbling");
  }   
  if (cfg.get(("FieldPlayer07::standardPositioning"), standardPositioning) < 0) {
    throw InvalidConfigurationException("FieldPlayer07:standardPositioning");
  } 

  // Optionsstack fuellen /////////////////////////////////////////////////////
  
  addOption (new BGameStopped());
int hackKickLength = 60;
    addOption (new BShoot(hackKickLength));
  addOption (new BDribbleBallStraightToGoalEvadeSidewards());

  addOption (new BApproachBallDirectly());
  addOption (new BEmergencyStop());
}

HeikoPlayer::~HeikoPlayer () throw () 
{}
