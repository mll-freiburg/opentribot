### TARGET: Dateiname der Zieldatei
TARGET = ../../../../bin/teamcontrol


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = 


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = \
Widgets/UI/CommunicationLoadWidget.ui \
Widgets/UI/RefboxWidget.ui \
Widgets/UI/CoachWidget.ui \
Widgets/UI/JoystickWidget.ui \
Widgets/UI/RobotWidget.ui \
Widgets/UI/RobotDataWidget.ui \
Widgets/UI/TeamcontrolFieldWidget.ui \
Widgets/UI/TeamcontrolMainWidget.ui \


### QTHEADERS: Header, die QT-Makros enthalten
HEADERS = \
../components/FieldOfPlay.h \
../components/ValPlotWidget.h \
../components/IDQObjects.h \
Widgets/TacticsWidget.h \
Widgets/MouseJoyWidget.h \
Widgets/CommunicationLoadWidget.h \
Widgets/RefboxWidget.h \
Widgets/CoachWidget.h \
Widgets/JoystickWidget.h \
Widgets/RobotWidget.h \
Widgets/RobotDataWidget.h \
Widgets/TeamcontrolFieldWidget.h \
Widgets/TeamcontrolMainWidget.h \


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES = \
Widgets/TacticsWidget.cpp \
Widgets/MouseJoyWidget.cpp \
Widgets/CommunicationLoadWidget.cpp \
Widgets/RefboxWidget.cpp \
Widgets/CoachWidget.cpp \
Widgets/JoystickWidget.cpp \
Widgets/RobotWidget.cpp \
Widgets/RobotDataWidget.cpp \
Widgets/TeamcontrolFieldWidget.cpp \
Widgets/TeamcontrolMainWidget.cpp \
Logic/main.cpp \
Logic/Coach.cpp \
Logic/JoystickControl.cpp \
Logic/RefboxClient.cpp \
Logic/RefboxControl.cpp \
Logic/RemoteRobot.cpp \
Logic/TeamcontrolMasterServer.cc \
Logic/TeamcontrolSlaveClient.cc \
Logic/TeamcontrolUDPCommunication.cc \
States/RemoteBlackboard.cpp \
Policies/PolicyFactory.cpp \
Policies/PolicyDummy.cpp \
Policies/StaticPolicy.cpp \
Policies/FieldPlayerPolicy.cpp \
Policies/FieldPlayer07Policy.cpp \
Policies/DynamicDefendOffendPolicy.cpp \
Policies/RCPlayer05Policies.cpp \
Policies/ChallengePlayerPolicy.cpp \
../components/FieldOfPlay.cpp \
../components/CycleInfo.cpp \
../components/ValPlotWidget.cpp \
../components/IDQObjects.cpp \
../../../Fundamental/Time.cc \
../../../Fundamental/Vec.cc \
../../../Fundamental/Angle.cc \
../../../Fundamental/Vec3D.cc \
../../../Fundamental/ConfigReader.cc \
../../../Fundamental/geometry.cc \
../../../Fundamental/stringconvert.cc \
../../../Fundamental/Joystick.cc \
../../../Fundamental/Frame2D.cc \
../../../Structures/DriveVector.cc \
../../../Structures/FieldGeometry.cc \
../../../Structures/GameState.cc \
../../../Structures/RobotProperties.cc \
../../../Structures/RobotData.cc \
../../../Structures/TacticsBoard.cc \
../../../Structures/TacticsBoardDescription.cc \
../../../Structures/TeammateLocation.cc \
../../../Structures/TribotsException.cc \
../../../Structures/ObstacleLocation.cc \
../../../Structures/VisibleObject.cc \
../../../Structures/MessageBoard.cc \
../../../Structures/Journal.cc \
../../../Structures/default_exception_handler.cc \
../../../WorldModel/Orga/RefereeStateMachine.cc \
../../../Communication/MultiPacketUDPCommunication.cc \
../../../Communication/PriorityUDPCommunication.cc \
../../../Communication/TaggedUDPCommunication.cc \
../../../Communication/NonspecificTaggedUDPCommunication.cc \
../../../Communication/TribotsUDPCommunication.cc \
../../../Communication/UDPSocket.cc \
../../../Communication/encoding.cc \
../../../ImageProcessing/ObjectAnalysis/Regions.cc \
../../../ImageProcessing/ObjectAnalysis/ChainCoding.cc \
