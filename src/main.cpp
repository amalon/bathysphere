/*
 * main.cpp
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

#include "main.h"

// headers from the rest of the program.
#include "environment.h"
#include "observer.h"
#include "bathysphere.h"
#include "cord.h"
#include "seabed.h"
#include "seasurface.h"
#include "reef.h"
#include "debug.h"
#include "boat.h"

// system headers
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

// include openGL, glUtility and utility toolkit headers
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

// available views.
#define VIEW_PANORAMIC    0
#define VIEW_BATHYSPHERE  1
#define VIEW_PLAN         2
#define VIEW_THIRD_PERSON 3
#define VIEW_MAX          4

// key states for motion.
bool keyLeft     = false;
bool keyRight    = false;
bool keyUp       = false;
bool keyDown     = false;
bool keyPageUp   = false;
bool keyPageDown = false;

// third person orientation + distance target values (in radians).
float thirdPersonAzimuth   = 0.0f;
float thirdPersonElevation = 0.0f;
float thirdPersonDistance  = 3.0f;

/// List of observers, indexed by VIEW_* definitions.
Observer      observers[VIEW_MAX];
/// The current observer.
Observer *    currentObserver = NULL;
/// The environment which manages most of the running of the simulation.
Environment * env = NULL;

/// The boat object.
Boat *        boat = NULL;
/// The bathysphere object.
Bathysphere * bathysphere = NULL;
/// The coral reef object.
Reef *        reef = NULL;
/// The sea surface object.
SeaSurface *  seaSurface = NULL;

/// Configuration menu switch ids.
#define SWITCH_VIEW       0
#define SWITCH_WIREFRAME  1
#define SWITCH_DEBUG      2
#define SWITCH_DEBUG_OCTREE 3
#define SWITCH_LIGHT_RAYS 4
#define SWITCH_MAX        5

/// Gets the value of a configuration switch.
/**
 * @param X The configuration switch name e.g. VIEW, WIREFRAME etc.
 * @return The value of the configuration switch.
 */
#define SWITCH(X) switches[SWITCH_##X].value

/// Comfiguration menu switch data.
ConfigurationMenu switches[SWITCH_MAX] = {
  // Changes the view that the scene is displayed from.
  { -1, 4, "Observer",      "Panoramic\0Bathysphere\0Plan\0003rd Person", SwitchView },
  // Toggles the polygon mode between fill and line.
  {  0, 2, "Polygon Mode",  "Solid\0Wireframe", NULL },
  // Toggles debug reaction force arrows and potential collision triangle highlighting.
  {  0, 2, "Debug Physics", "Off\0On", SwitchDebugMode },
  // Toggles debug octree display.
  {  0, 2, "Debug Octree",  "Off\0On", SwitchDebugOctree },
  // Toggles light rays near the surface of the water.
  {  1, 2, "Light Rays",    "Off\0On", SwitchLightRays }
};

/// Set the value of a configuration switch.
void SetSwitch(int switchId, int value)
{
  int oldValue = switches[switchId].value;
  if (oldValue != value) {
    switches[switchId].value = value;
    if (switches[switchId].callback) {
      switches[switchId].callback(oldValue, value);
    }
  }
}

/// Main program entry point.
int main(int argc, char **argv)
{
  // Print a copyright notice to the console
  std::cout << " Bathysphere Sim 1.0  Copyright (C) 2007  James Hogan\n";
  std::cout << " This program comes with ABSOLUTELY NO WARRANTY.\n";
  std::cout << " This is free software, and you are welcome to redistribute it\n";
  std::cout << " under certain conditions." << std::endl;
  
  // Initialise the glut display.
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  
  // Create a window to draw in.
  glutCreateWindow("Bathysphere Sim 1.0");

  // Register the glut callback functions.
  glutReshapeFunc(ReshapeCallback);
  glutDisplayFunc(DisplayCallback);
  glutIdleFunc(IdleCallback);
  glutKeyboardFunc(KeyboardCallback);
  glutSpecialFunc(SpecialCallback);
  glutSpecialUpFunc(SpecialUpCallback);

  // Set up the configuration menus.
  int menuIds[SWITCH_MAX];
  int switchValue = 0;
  for (int i = 0; i < SWITCH_MAX; ++i) {
    menuIds[i] = glutCreateMenu(SubMenuCallback);
    const char * values = switches[i].values;
    for (int j = 0; j < switches[i].numValues; ++j) {
      glutAddMenuEntry(values, switchValue++);
      // skip past next null.
      while (*(values++) != '\0');
    }
  }
  // Set up the left click menu, add submenus etc.
  glutCreateMenu(MainMenuCallback);
  for (int i = 0; i < SWITCH_MAX; ++i) {
    glutAddSubMenu(switches[i].name, menuIds[i]);
  }
  glutAddMenuEntry("Exit", -1);
  glutAttachMenu(GLUT_LEFT_BUTTON);
  
  // Initialise configuration switches.
  SetSwitch(SWITCH_VIEW, VIEW_BATHYSPHERE);
  
  // Seed the standard random number generater using the time so that the scene
  // is different each time it is viewed.
  srand(time(NULL));
  
  // Because of GLUT's infinite main loop, any cleanup needs to be done using
  // exit callback functions. This will allow destructors to get called and
  // in future close any files safely etc.
  atexit(CleanupEnvironment);
  // Initialise the objects in the environment.
  SetupEnvironment();

  // And hand over control of the entire program to GLUT's infinite loop.
  // This function will never return (coz it smells)
  glutMainLoop();
  
  // Just in case.
  return 0;
}

/// Initialise the environment.
void SetupEnvironment()
{
  // Create the environment.
  env = new Environment();
  
  // Create the boat object.
  boat = new Boat();
  env->AddObject(boat);
  
  // Create the tether and attach to crane.
  Cord * tether = new Cord(3.0f, maths::Vector<3, float>(-20.0f + 20.0f*rand()/RAND_MAX, -20.0f + 20.0f*rand()/RAND_MAX, 30.0f));
  boat->cranes[0].SetTether(tether);
  
  // Create the bathysphere and attach to tether.
  bathysphere = new Bathysphere();
  tether->SetAttachee(bathysphere, bathysphere->GetTetherHookPosition());
  
  // Randomly position and orient the boat
  bool goodEnough = false;
  while (!goodEnough) {
    boat->Orientation().ApplyRotation(2.0f * PI * rand() / RAND_MAX, maths::Vector<3, float>(0.0f, 0.0f, 1.0f));
    boat->SetPosition(maths::Vector<3, float>(-25.0f + 50.0f * rand() / RAND_MAX,
                                              -25.0f + 50.0f * rand() / RAND_MAX,
                                              0.0f));
    
    // Its not good enough if the top of the tether is too close to the edges.
    maths::Vector<3, float> tetherTop = tether->GetPosition();
    if (tetherTop[0] > -10.0f && tetherTop[0] < 10.0f &&
        tetherTop[1] > -10.0f && tetherTop[1] < 10.0f) {
      goodEnough = true;
    }
  }
  
  // Create the coral reef seabed.
  reef = new Reef(150.0f);
  reef->SetPosition(maths::Vector<3, float>(0.0f, 0.0f, -100.0f));
  // Make some coral as well.
  reef->BuildCoralLibrary();
  reef->CreateCoralInstances(5);
  env->reef = reef;
  
  // Create the sea surface.
  seaSurface = new SeaSurface(150.0f);
  seaSurface->SetPosition(maths::Vector<3, float>(0.0f, 0.0f, 0.0f));
  env->seaSurface = seaSurface;
  
  // Generate some kelp.
  env->GenerateKelp(4);
  
  // initialise the observers.
  // Panoramic view.
  observers[VIEW_PANORAMIC].SetFov(1.5f);
  observers[VIEW_PANORAMIC].SetNearClip(2.0f);
  observers[VIEW_PANORAMIC].SetFarClip(150.0f);
  observers[VIEW_PANORAMIC].SetPosition(maths::Vector<3, float>(0.0f, -50.0f, -40.0f));
  
  // Plan view.
  observers[VIEW_PLAN].SetPosition(maths::Vector<3, float>(0.0f, 0.0f, -seaSurface->GetMaxAmplitude()));
  
  // Bathysphere internal observer.
  observers[VIEW_BATHYSPHERE].SetFov(1.8f);
  observers[VIEW_BATHYSPHERE].SetNearClip(0.1f);
  observers[VIEW_BATHYSPHERE].SetFarClip(300.0f);
  observers[VIEW_BATHYSPHERE].LockToObject(bathysphere, maths::Vector<3, float>(0.0f, 0.0f, 0.875f));
  observers[VIEW_BATHYSPHERE].orientation.Identity();
  observers[VIEW_BATHYSPHERE].orientation.ApplyRotation(0.5f*PI, maths::Vector<3, float>(1.0f, 0.0f, 0.0f));
  
  // Bathysphere third person observer.
  observers[VIEW_THIRD_PERSON].SetFov(1.5f);
  observers[VIEW_THIRD_PERSON].SetNearClip(0.1f);
  observers[VIEW_THIRD_PERSON].SetFarClip(300.0f);
  observers[VIEW_THIRD_PERSON].SetPosition(maths::Vector<3, float>(10.0f, 00.0f, -20.0f));
  observers[VIEW_THIRD_PERSON].SetPosition(maths::Vector<3, float>(5.0f, 00.0f, 50.0f));
}

/// Cleanup after the environment.
void CleanupEnvironment()
{
  // Delete the environment, and objects not added to the environment object update list.
  if (reef) {
    delete reef;
    reef = NULL;
  }
  if (seaSurface) {
    delete seaSurface;
    seaSurface = NULL;
  }
  if (env) {
    delete env;
    env = NULL;
  }
  
  // Clean up static octree data (list of unused nodes).
  OctreeCube::CleanupStatic();
}

/// Get the time interval since the last call to GetElapsedTime().
float GetElapsedTime() {
  // from particles practical.
  static clock_t t_old = clock();
  clock_t t_new, elapsed;

  t_new = clock();

  elapsed = t_new - t_old;

  t_old = t_new;
  
  return (float) elapsed / CLOCKS_PER_SEC;
}

/// Advance the environment.
void Advance(float dt)
{
  // Handle keyboard based movement.
  
  // Rotate left and right
  float rotate = 0.0f;
  if (keyLeft) {
    rotate += 1.0f;
  }
  if (keyRight) {
    rotate -= 1.0f;
  }
  thirdPersonAzimuth += rotate*dt;
  observers[VIEW_BATHYSPHERE].orientation.ApplyRotation(rotate*dt, maths::Vector<3, float>(0.0f, 0.0f, 1.0f));
  
  // Rotate up and down
  rotate = 0.0f;
  if (keyUp) {
    rotate += 1.0f;
  }
  if (keyDown) {
    rotate -= 1.0f;
  }
  thirdPersonElevation += rotate*dt;
  
  // Zoom in and out
  rotate = 0.0f;
  if (keyPageUp) {
    rotate -= 1.0f;
  }
  if (keyPageDown) {
    rotate += 1.0f;
  }
  thirdPersonDistance *= exp(rotate*dt);
  
  // Move the panoramic camera closer to the destination.
  // Exponential closing.
  observers[VIEW_THIRD_PERSON].SetPosition(observers[VIEW_THIRD_PERSON].GetPosition() * 0.9f
      + (bathysphere->GetPosition() + maths::Vector<3, float>(sin(thirdPersonAzimuth)   * cos(thirdPersonElevation),
                                                              cos(thirdPersonAzimuth)   * cos(thirdPersonElevation),
                                                              sin(thirdPersonElevation))* thirdPersonDistance     ) * 0.1f);
  
  // Advance the environment (inc. its objects) and the observers.
  env->Advance(dt);
  for (int i = 0; i < VIEW_MAX; ++i) {
    observers[i].Advance(dt);
  }
  
  // Stuff's changed, Redraw the scene
  glutPostRedisplay();
}

//*************************//
// GLUT CALLBACK FUNCTIONS //
//*************************//

/// GLUT display callback.
void DisplayCallback()
{
  // Apply configuration switches.
  glPolygonMode(GL_FRONT_AND_BACK, SWITCH(WIREFRAME) ? GL_LINE : GL_FILL);

  // Clear the modelview matrix.
  glLoadIdentity();

  // Transform from the camera to the center of the scene, depending on the view.
  if (SWITCH(VIEW) == VIEW_PANORAMIC) {
    // Panoramic view looks roughly in th middle of the scene from far enough that
    // the entire scene can be seen.
    maths::Vector<3, float> pos = currentObserver->GetPosition();
    maths::Vector<3, float> bpos = bathysphere->GetPosition();

    gluLookAt(pos[0], pos[1], pos[2],
              bpos[0]/2, bpos[1]/2 , (bpos[2]-50)/2,
              0.0,  0.0,  1.0);
    
  } else if (SWITCH(VIEW) == VIEW_THIRD_PERSON) {
    // Third person view is based around the bathysphere.
    maths::Vector<3, float> pos = currentObserver->GetPosition();
    maths::Vector<3, float> bpos = bathysphere->GetPosition();

    gluLookAt(pos[0], pos[1], pos[2],
              bpos[0], bpos[1], bpos[2],
              0.0, 0.0, 1.0);
    
  } else if (SWITCH(VIEW) == VIEW_BATHYSPHERE) {
    // Bathysphere view is from within the bathysphere.
    // Translate the camera forwards a bit so that we're closer to the window.
    glTranslatef(0.0f, 0.0f, 0.6f);
    // The orientation is handled by the observer in this case.
    currentObserver->SetModelView();
    
  } else if (SWITCH(VIEW) == VIEW_PLAN) {
    // Plan view is orthographic from above.
    maths::Vector<3, float> pos = currentObserver->GetPosition();
    
    gluLookAt(pos[0], pos[1], pos[2],
              0.0, 0.0, -60.0,
              0.0, 1.0,   0.0);
  }
  
  // Let the environment object do the rest of the rendering.
  env->Render(*currentObserver);

  // Display the finished image on the screen.
  glFlush();
  glutSwapBuffers();
}

/// GLUT idle callback.
void IdleCallback()
{
  // Get the change in time.
  float dt = GetElapsedTime();
  // Accumulate the time until its enough to warrent an advancement.
  static float accum = 0.0f;
  accum += dt;
  // This means every update is FRAME_TIME which helps with stbility.
#define FRAME_TIME 0.05f
  if (accum > FRAME_TIME) {
    Advance(FRAME_TIME);
    accum -= FRAME_TIME;
  }
}

/// GLUT reshape callback.
void ReshapeCallback(int w, int h)
{
  // Set the new viewport for each observer.
  for (int i = 0; i < VIEW_MAX; ++i) {
    observers[i].SetViewport(0, 0, w, h);
  }
  glViewport(0, 0, w, h);

  // Update the projection matrix.
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  /*********************************************************
  The plan view is an orthographic projection it must 
  project the project squares as squares not rectangles. 
  So when the width is greater than the height the `bottom' 
  and `top' values are scaled. This adds white space at the 
  top and bottom of the image but ensures the geometry 
  is correct. The opposite applies when h > w.
  ***********************************************************/

  if (SWITCH(VIEW) == VIEW_PLAN) {
    // Haven't got around to putting othographic projection capabilities into
    // the observer.
    if (w <= h) {
      glOrtho(-25, 25, 
              -25*(GLfloat)h/(GLfloat)w, 25*(GLfloat)h/(GLfloat)w, 
              0, 150);
    } else {
      glOrtho(-25*(GLfloat)w/(GLfloat)h, 25*(GLfloat)w/(GLfloat)h, 
              -25, 25, 
              0, 150);
    }
  } else {
    // If it isn't PLAN view, its a perspective matrix.
    // Use the observer function.
    currentObserver->SetProjection();
  }

  // Switch back to the modelview matrix so normal tranformation can take place.
  // Redraw the scene with the new window size.
  glMatrixMode(GL_MODELVIEW);
  glutPostRedisplay();
}

/// GLUT keybaord button callback.
void KeyboardCallback(unsigned char key, int x, int y)
{
  // Handle the key presses
  switch(key){
    case ' ':
      boat->cranes[0].Stop();
      break;
    
    case 'a':
    case 'A':
      boat->cranes[0].WindTo(140.0f);
      break;
    
    case 'b':
    case 'B':
      boat->cranes[0].WindIn();
      break;
      
    // ESCAPE
    case 27:
      // Reset the world.
      CleanupEnvironment();
      SetupEnvironment();
      break;
      
    default:
      fprintf(stdout, "No function attached to key %c.\n", key);
  }
}

/// GLUT special button callback.
void SpecialCallback(int key, int x, int y)
{
  // Set flags for the navigation buttons.
  switch(key){

    case GLUT_KEY_RIGHT:
      keyRight = true;
      break;

    case GLUT_KEY_LEFT:
      keyLeft = true;
      break;

    case GLUT_KEY_UP:
      keyUp = true;
      break;

    case GLUT_KEY_DOWN:
      keyDown = true;
      break;

    case GLUT_KEY_PAGE_UP:
      keyPageUp = true;
      break;

    case GLUT_KEY_PAGE_DOWN:
      keyPageDown = true;
      break;
  }
}


/// GLUT special button up callback.
void SpecialUpCallback(int key, int x, int y)
{
  // Unset flags for the navigation butons.
  switch(key){

    case GLUT_KEY_RIGHT:
      keyRight = false;
      break;

    case GLUT_KEY_LEFT:
      keyLeft = false;
      break;

    case GLUT_KEY_UP:
      keyUp = false;
      break;

    case GLUT_KEY_DOWN:
      keyDown = false;
      break;

    case GLUT_KEY_PAGE_UP:
      keyPageUp = false;
      break;

    case GLUT_KEY_PAGE_DOWN:
      keyPageDown = false;
      break;
  }
}


/// GLUT main menu callback.
void MainMenuCallback(int value)
{
  // Handle some of the stuff on the main menu.
  switch(value){
    case -1:
      // Quitting the program is the only way to break out of glut.
      // This will call CleanupEnvironment which is registered as an exit callback.
      exit(0);
      break;
  }
}

/// GLUT sub menu callback.
void SubMenuCallback(int value)
{
  int switchValue = 0;
  for (int i = 0; i < SWITCH_MAX; ++i) {
    if (value < switchValue + switches[i].numValues) {
      value -= switchValue;
      SetSwitch(i, value);
      break;
    }
    switchValue += switches[i].numValues;
  }
}

/// Camera view menu callback.
void SwitchView(int from, int to)
{
  currentObserver = &observers[to];
  currentObserver->SetActive();
  int w, h;
  w = glutGet(GLUT_WINDOW_WIDTH);
  h = glutGet(GLUT_WINDOW_HEIGHT);
  ReshapeCallback(w, h);
  glutPostRedisplay();
}

/// Debug mode menu callback.
void SwitchDebugMode(int from, int to)
{
  Debug::DebugObject::SetDebugRender((bool)to);
}

/// Octree debug menu callback.
void SwitchDebugOctree(int from, int to)
{
  env->SetRenderOptions(to ? OctreeRenderAll : OctreeRenderDefault);
}

/// Light ray menu callback.
void SwitchLightRays(int from, int to)
{
  seaSurface->SetDrawSunRays((bool)to);
}
