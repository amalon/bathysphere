/*
 * main.h
 *
 * Copyright (C) 2007-2014 James Hogan <james@albanarts.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 * (in the file called COPYING).
 *
 *
 * Main glut and setup functions.
 *
 */

#ifndef _FILE_main_h
#define _FILE_main_h

/// Main program entry point.
int main(int argc, char **argv);

// World handling.

/// Initialise the environment.
void SetupEnvironment();
/// Cleanup after the environment.
void CleanupEnvironment();
/// Get the time interval since the last call to GetElapsedTime().
float GetElapsedTime();
/// Advance the environment.
/**
 * @param dt Seconds since last call to Advance.
 */
void Advance(float dt);

// GLUT callback functions.

/// GLUT display callback.
void DisplayCallback();
/// GLUT idle callback.
void IdleCallback();
/// GLUT reshape callback.
void ReshapeCallback(int w, int h);
/// GLUT keybaord button callback.
void KeyboardCallback(unsigned char key, int x, int y);
/// GLUT special button callback.
void SpecialCallback(int key, int x, int y);
/// GLUT special button up callback.
void SpecialUpCallback(int key, int x, int y);
/// GLUT main menu callback.
void MainMenuCallback(int value);
/// GLUT sub menu callback.
/**
 * Finds which configuration switch was changed and what it was changed to.
 * Sets the new value by calling SetSwitch.
 */
void SubMenuCallback(int value);

// Configuration menu management.

/// Comfiguration menu switch data item.
struct ConfigurationMenu {
  int          value;         ///< current value.
  int          numValues;     ///< number of values.
  const char * name;          ///< property name.
  const char * values;        ///< separated by ascii 0 ('\0').
  void (*callback)(int, int); ///< callback function (oldValue, newValue).
};

/// Set the value of a configuration switch
/**
 * @param switchId The index of the switch.
 * @param value The new value of the switch.
 * 
 * Changes the value of the switch and if its changed, calls the callback function.
 */
void SetSwitch(int switchId, int value);

// Menu callback functions.

/// Camera view menu callback.
void SwitchView(int from, int to);
/// Debug mode menu callback.
void SwitchDebugMode(int from, int to);
/// Octree debug menu callback.
void SwitchDebugOctree(int from, int to);
/// Light ray menu callback.
void SwitchLightRays(int from, int to);

#endif // _FILE_code_h
