/*
 * octree.cpp
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
 * Object container spacial tree.
 *
 */

#include "octree.h"

// Constructors
Octree::Octree(float fSize)
  : _biggestCube(fSize)
{
}

// Add an object and return a reference to the new structure
Object * Octree::AddObject(Object * object)
{
  //return cubeMaster.AddObj(pObj);
  AddObjectToCube(&_biggestCube, object);
  return object;
}

// Remove an object and return whether successful
bool Octree::RemObject(Object * object)
{
  return false;
  //return RemObjectFromCube(&_biggestCube, object);
  //return cubeMaster.RemObj(pObj);
}

// Advance all objects in standard time units
/*void Octree::Advance(const mlDate & dClock)
{
  cubeMaster.Advance(dClock);
}*/

// Render diagramatically the octree structure down to the iMaxLevels level
void Octree::RenderDiagram(unsigned char maxLevel) const
{
  _biggestCube.RenderDiagram(maxLevel);
}

// Give all contained object a signal.
void SignalAllObjects(unsigned int signal, void * data)
{
  
}
