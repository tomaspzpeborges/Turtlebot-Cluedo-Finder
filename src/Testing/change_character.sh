#!/bin/sh

#warning: moving this file will mess up the paths

#assumptions:
# $HOME/.gazebo/models/cluedo_character/materials/textures/ holds all possible characters with standardnames.png

# relaunch gazebo after running script
read -p "enter character name for green room (lowercase) : " name


cd $HOME/.gazebo/models/cluedo_character/materials/textures/

if [ \( "$name" != "plum" \) -a \( "$name" != "scarlet" \) -a \( "$name" != "mustard" \) -a \( "$name" != "peacock" \) ]
then
   echo "Please enter a valid character"
else
    cp $name.png Cluedo_character.png
fi