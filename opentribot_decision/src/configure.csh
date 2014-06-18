#! /bin/tcsh -f
# Konfigurationsskript fuer robotcontrol

set use_camera = 1
set use_serrobot = 1
set use_canrobot = 0
set use_odesim = 0
set use_odesim2 = 0
set use_odesimpack = 0
set odepath = ""

if ( $#argv >= 1 ) then
# Kommandozeilenparameter einlesen
  set argvcount = 1
  while ( $argvcount <= $#argv )
    if ( "$argv[$argvcount]" == "+camera" ) then 
      set use_camera = 1
    else if ( "$argv[$argvcount]" == "-camera" ) then 
      set use_camera = 0
    else if ( "$argv[$argvcount]" == "+serrobot" ) then 
      set use_serrobot = 1
    else if ( "$argv[$argvcount]" == "-serrobot" ) then 
      set use_serrobot = 0
    else if ( "$argv[$argvcount]" == "+canrobot" ) then 
      set use_canrobot = 1
    else if ( "$argv[$argvcount]" == "-canrobot" ) then 
      set use_canrobot = 0
    else if ( "$argv[$argvcount]" == "+odesim" ) then 
      set use_odesim = 1
    else if ( "$argv[$argvcount]" == "-odesim" ) then 
      set use_odesim = 0
    else if ( "$argv[$argvcount]" == "+odesim2" ) then 
      set use_odesim2 = 1
    else if ( "$argv[$argvcount]" == "-odesim2" ) then 
      set use_odesim2 = 0
    else if ( "$argv[$argvcount]" == "+odesimpack" ) then 
      set use_odesimpack = 1
    else if ( "$argv[$argvcount]" == "-odesimpack" ) then 
      set use_odesimpack = 0
    else if ( "$argv[$argvcount]" == "default") then
      set use_camera = 1
      set use_serrobot = 1
      set use_canrobot = 0
      set use_odesim = 0
      set use_odesim2 = 0
      set use_odesimpack = 0
    else
      if ( -e "$argv[$argvcount]" ) then
        set odepath = $argv[$argvcount]
      else
        echo skipping unknown argument \"${argv[$argvcount]}\"
      endif
    endif
    @ argvcount++
  end
  
  # problematische Dateien loeschen
  set okay = 1
  foreach file ( Simulator/SimClient.o Simulator/SimClient.d Behavior/Skills/BallHandling/SDribbleBallToPos.o )
    if ( -e $file ) then
      \rm $file   # Files die vom Simulator abhaengen loeschen
    endif
  end
 
  # ggf. Bibliotheken uebersetzen
  set serrobot_lib = 0
  set canrobot_lib = 0
  set npp_lib = 0
  set jpeg_lib = 0
  set nfq_lib = 0
  set la_lib = 0

  if ( -e Libs/robotCtrLib/librobotCtr.a ) then
    set serrobot_lib = 1
  endif
  if ( -e Libs/NFQ/lib/libNFQ.a ) then
    set nfq_lib = 1
  endif
  if ( -e Libs/RobotCtr2/lib/libRobotCtr2.a ) then
    set canrobot_lib = 1
  endif
  if ( -e Libs/n++/lib/libn++.a ) then
    set npp_lib = 1
  endif
  if ( -e Libs/LA/lib/libLA.a ) then
    set la_lib = 1
  endif
  if ( -e Libs/jpeg-6b/libjpeg.a ) then
    set jpeg_lib = 1
  endif
  
  if ( ! $npp_lib ) then
    echo "did not find n++ library; try to generate it..."
    cd Libs/n++/src
    \make 
    cd ../../..
    if ( -e Libs/n++/lib/libn++.a ) then 
      set npp_lib = 1
      echo "...done"
    else
      echo "...unsuccessful. Stop."
      set okay = 0
    endif
  endif

  if ( ! $nfq_lib ) then
    echo "did not find NFQ library; try to generate it..."
    cd Libs/NFQ/src
    \make 
    cd ../../..
    if ( -e Libs/NFQ/lib/libNFQ.a ) then 
      set nfq_lib = 1
      echo "...done"
    else
      echo "...unsuccessful. Stop."
      set okay = 0
    endif
  endif
  
  if ( ! $la_lib ) then
    echo "did not find LA library; try to generate it..."
    cd Libs/LA/src
    \make 
    cd ../../..
    if ( -e Libs/LA/lib/libLA.a ) then 
      set nfq_la = 1
      echo "...done"
    else
      echo "...unsuccessful. Stop."
      set okay = 0
    endif
  endif
  
  if ( ! $jpeg_lib ) then
    echo "did not find jpeg library; try to generate it..."
    cd Libs/jpeg-6b
    sh ./configure
    \make
    cd ../..
    if ( -e Libs/jpeg-6b/libjpeg.a ) then
      set jpeg_lib = 1
      echo "...done"
    else
      echo "...unsuccessful. Stop."
      set okay = 0
    endif
  endif
  
  if ( $okay && $use_serrobot && ! $serrobot_lib ) then
    echo "did not find robotCtr library; try to generate it..."
    cd Libs/robotCtrLib
    \make 
    cd ../..
    if ( -e Libs/robotCtrLib/librobotCtr.a ) then 
      set serrobot_lib = 1
      echo "...done"
    else
      echo "...unsuccessful. Stop."
      set okay = 0
    endif
  endif

  if ( $okay && $use_canrobot && ! $canrobot_lib ) then
    echo "did not find RobotCtr2 library; try to generate it..."
    cd Libs/RobotCtr2/src
    \make 
    cd ../../..
    if ( -e Libs/RobotCtr2/lib/libRobotCtr2.a ) then 
      set canrobot_lib = 1
      echo "...done"
    else
      echo "...unsuccessful. Stop."
      set okay = 0
    endif
  endif

  if ( $okay && $use_odesim ) then
    if ( $odepath == "" && ! -e Simulator/odeserver ) then
      echo "No path to Stefans simulator found and none was given. Stop. Call:"
      echo "    $0 +odesim PATH"
      echo "to help this problem where PATH is the absolute path to Stefans simulator"
      set okay = 0
    else if ( $odepath != "" ) then
      if ( -e Simulator/odeserver ) then
        \rm Simulator/odeserver
      endif
      \ln -s $odepath Simulator/odeserver
      echo "set link to odeserver: $odepath"
    endif
  endif

  # bin-Verzeichnis ggf. erzeugen
  if ( ! -e ../bin ) then
    echo "generating bin-directory"
    \mkdir ../bin
  endif
    
  # Verzeichnis .robotcontrol ggf. erzeugen und mit default-Werten fuellen
  if ( ! -e $HOME/.robotcontrol ) then
    echo "generating .robotcontrol-directory"
    \mkdir $HOME/.robotcontrol
  endif
  foreach speccfg ( colors.lut colors.ranges dist_marker.cfg image_mask.ppm Sony_DFW.cfg Basler.cfg BaslerSpecific.cfg File.cfg playerspecific.cfg robotspecific.cfg )
    if ( ! -e $HOME/.robotcontrol/$speccfg ) then
      echo "generating default config file ${speccfg}"
      \cp ../config_files/default/default_$speccfg $HOME/.robotcontrol/$speccfg
    endif
  end
  
  # robotcontrol.pro schreiben
  if ( $okay ) then
    echo "# File automatically generated by configure.csh." > robotcontrol.pro
    echo "# Do not change by hand. Instead edit the *.addpro files" >> robotcontrol.pro
    echo "include robotcontrol_base.addpro" >> robotcontrol.pro
    if ( $use_camera ) then
      echo "include robotcontrol_camera.addpro" >> robotcontrol.pro
    endif
    if ( $use_serrobot ) then
      echo "include robotcontrol_serrobot.addpro" >> robotcontrol.pro
    endif
    if ( $use_canrobot ) then
      echo "include robotcontrol_canrobot.addpro" >> robotcontrol.pro
    endif
    if ( $use_odesim ) then
      echo "include robotcontrol_odesim.addpro" >> robotcontrol.pro
    endif
    if ( $use_odesim2 ) then
      echo "include robotcontrol_odesim2.addpro" >> robotcontrol.pro
    endif
    if ( $use_odesimpack ) then
      echo "include robotcontrol_odesimpack.addpro" >> robotcontrol.pro
    endif
    echo "everything done. Type >make< to compile main program."
  else
    echo "did not change configuration due to errors in generating relevant libraries"
  endif

else
  # keine Kommandozeilenparameter
  echo "Skript um eine individuelle Zusammenstellung der Hardware-nahen"
  echo "Komponenten zu erreichen. Aufruf mit folgenden Parametern moeglich:"
  echo "+camera, -camera: Kameraanbindung hinzunehmen/weglassen"
  echo "+serrobot, -serrobot: Seriellport-Roboteransteuerung hinzunehmen/weglassen"
  echo "+canrobot, -canrobot: CAN-Roboteransteuerung hinzunehmen/weglassen"
  echo "+odesim, -odesim: Anbindung an Stefans Simulator einrichten/weglassen"
  echo "+odesim2, -odesim2: Anbindung an modifizierten Stefans Simulator einrichten/weglassen"
  echo "+odesimpack, -odesimpack: Anbindung an Rolands Simulator einrichten/weglassen"
  echo "default: entspricht +camera +serrobot -canrobot -odesim"
  echo "zudem kann der Pfad(absolut!) zu Stefans Simulator angegeben werden"
endif
