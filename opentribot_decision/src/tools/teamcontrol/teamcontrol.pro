### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/teamcontrol


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release thread


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
Widgets/TeamcontrolMainWidget.ui \
Widgets/CoachWidget.ui \
Widgets/JoystickWidget.ui \
Widgets/RefboxWidget.ui \
Widgets/RobotWidget.ui \
Widgets/RobotDataWidget.ui \
Widgets/TeamcontrolFieldWidget.ui \
Widgets/CommunicationLoadWidget.ui \


### QTHEADERS: Header, die QT-Makros enthalten
HEADERS = \
../components/FieldOfPlay.h \
../components/ValPlotWidget.h \
../components/IDQObjects.h \
Widgets/TacticsWidget.h \
Widgets/MouseJoyWidget.h \


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= \
Widgets/TacticsWidget.cpp \
Widgets/MouseJoyWidget.cpp \
Logic/main.cpp \
Logic/Coach.cpp \
Logic/CoachLogger.cpp \
Logic/JoystickControl.cpp \
Logic/PingClient.cpp \
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
Policies/FieldPlayer07Policy.cpp \
../components/FieldOfPlay.cpp \
../components/CycleInfo.cpp \
../components/ValPlotWidget.cpp \
../components/IDQObjects.cpp \
../../Fundamental/Time.cc \
../../Fundamental/RolePreferences.cpp \
../../Fundamental/Vec.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Vec3D.cc \
../../Fundamental/ConfigReader.cc \
../../Fundamental/geometry.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/binary_encoding.cc \
../../Fundamental/Joystick.cc \
../../Fundamental/Frame2D.cc \
../../Fundamental/POSIXThread.cc \
../../Structures/DriveVector.cc \
../../Structures/FieldGeometry.cc \
../../Structures/GameState.cc \
../../Structures/RobotProperties.cc \
../../Structures/RobotData.cc \
../../Structures/TacticsBoard.cc \
../../Structures/TacticsBoardDescription.cc \
../../Structures/TeammateLocation.cc \
../../Structures/TribotsException.cc \
../../Structures/ObstacleLocation.cc \
../../Structures/VisibleObject.cc \
../../Structures/MessageBoard.cc \
../../Structures/RobotLocationReadWriter.cc \
../../Structures/BallLocationReadWriter.cc \
../../Structures/GameStateReadWriter.cc \
../../Structures/MessageBoardReadWriter.cc \
../../Structures/Journal.cc \
../../Structures/default_exception_handler.cc \
../../WorldModel/Orga/RefereeStateMachine.cc \
../../Communication/MultiPacketUDPCommunication.cc \
../../Communication/PriorityUDPCommunication.cc \
../../Communication/TaggedUDPCommunication.cc \
../../Communication/NonspecificTaggedUDPCommunication.cc \
../../Communication/TribotsUDPCommunication.cc \
../../Communication/UDPSocket.cc \
../../Communication/encoding.cc \
../../ImageProcessing/ObjectAnalysis/Regions.cc \
../../ImageProcessing/ObjectAnalysis/ChainCoding.cc \
