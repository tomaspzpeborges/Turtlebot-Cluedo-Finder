if [[ "$1" == "Daniel" ]]
then
  if [[ "$2" == "web" ]]
  then
    if [ $# -eq 2 ]
    then
      vglrun roslaunch setups/setupDanLinux.launch
    else
      vglrun roslaunch setups/setupDanLinux.launch "$3"
    fi
  else
    if [ $# -eq 2 ]
    then
      roslaunch src/setups/setupDan.launch
    else
      roslaunch src/setups/setupDan.launch "$3"
    fi
  fi

elif [[ "$1" == "Tanvir" ]]
then
   if [[ "$2" == "web" ]]
  then
    if [ $# -eq 2 ]
    then
      vglrun roslaunch setups/setup_tanvir.launch
    else
      vglrun roslaunch setups/setup_tanvir.launch "$3"
    fi
  else
    if [ $# -eq 2 ]
    then
      vglrun roslaunch setups/setup_tanvir.launch
    else
      vglrun roslaunch setups/setup_tanvir.launch "$3"
    fi
  fi

elif [[ "$1" == "Andrezj" ]]
then
  if [ $# -eq 1 ]
  then
    roslaunch src/setups/setupDan.launch
  else
    roslaunch src/setups/setupDan.launch "$2"
  fi

elif [[ "$1" == "Thomas" ]]
then
  if [ $# -eq 1 ]
  then
    roslaunch src/setups/setupDan.launch
  else
    roslaunch src/setups/setupDan.launch "$2"
  fi

elif [[ "$1" == "Main" ]]
then
  if [ $# -eq 1 ]
  then
    roslaunch src/setups/setupDan.launch
  else
    roslaunch src/setups/setupDan.launch "$2"
  fi

fi
