/*
 * octreecube.cpp
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
 * Object container spacial octree cube.
 *
 */

#include "octree.h"

#include <cassert>

// Static options
std::deque<OctreeCube*> OctreeCube::s_spareCubes;
unsigned char OctreeCube::s_maxLevel = 6;

// Constructors
OctreeCube::OctreeCube(float halfSize) // For construction of a base cube
  : _parentCube(NULL),_index(0),_level(0),_numSubcubes(0),
    _mid(0.0f),_halfSize(halfSize),_isMarkedForDeletion(false)
{
  // Initialise subcubes to NULL
  for (int i = 0; i < 8; ++i) {
    _subcubes[i] = NULL;
  }
}
OctreeCube::OctreeCube(OctreeCube * const rParentCube, unsigned char i) // For construction of a child cube
  : _parentCube(rParentCube),_index(i),_level(rParentCube->_level + 1),_numSubcubes(0),
                _halfSize(rParentCube->_halfSize * 0.5f),_isMarkedForDeletion(false)
{
  // Debug checks
  //assert(rParentCube && "Invalid parent cube pointer to constructor");; // Should throw errors by initialisation
  assert(i<8 && "Invalid subcube index (must be in range [0,7])");
  // Find new mid point
  if (i & 0x01) { // X
    _mid[0] = rParentCube->_mid[0] + _halfSize;
  } else {
    _mid[0] = rParentCube->_mid[0] - _halfSize;
  }
  if (i & 0x02) { // Y
    _mid[1] = rParentCube->_mid[1] + _halfSize;
  } else {
    _mid[1] = rParentCube->_mid[1] - _halfSize;
  }
  if (i & 0x04) { // Z
    _mid[2] = rParentCube->_mid[2] + _halfSize;
  } else {
    _mid[2] = rParentCube->_mid[2] - _halfSize;
  }
  // Initialise subcubes to NULL
  _subcubes[0] = _subcubes[1] = _subcubes[2] = _subcubes[3]
      = _subcubes[4] = _subcubes[5] = _subcubes[6] = _subcubes[7] = NULL;
}

OctreeCube & OctreeCube::Init(OctreeCube * const rParentCube, unsigned char i)
{
  // Debug checks
  assert(!_numSubcubes && "Shouldn't call Init on an OctCube with subcubes");;
  assert(rParentCube && "Invalid parent cube pointer to constructor");;
  assert(i<8 && "Invalid subcube index (must be in range [0,7])");;
  _parentCube = rParentCube;
  _index = i; _level = rParentCube->_level + 1; _numSubcubes = 0;
  _halfSize = rParentCube->_halfSize * 0.5f;
  // Find new mid point
  if (i & 0x01) { // X
    _mid[0] = rParentCube->_mid[0] + _halfSize;
  } else {
    _mid[0] = rParentCube->_mid[0] - _halfSize;
  }
  if (i & 0x02) { // Y
    _mid[1] = rParentCube->_mid[1] + _halfSize;
  } else {
    _mid[1] = rParentCube->_mid[1] - _halfSize;
  }
  if (i & 0x04) { // Z
    _mid[2] = rParentCube->_mid[2] + _halfSize;
  } else {
    _mid[2] = rParentCube->_mid[2] - _halfSize;
  }
  // Initialise subcubes to NULL
  _subcubes[0] = _subcubes[1] = _subcubes[2] = _subcubes[3]
      = _subcubes[4] = _subcubes[5] = _subcubes[6] = _subcubes[7] = NULL;
  return *this;
}

// Destructor
OctreeCube::~OctreeCube()
{
  // Go through list of object (wrappers) deleting
  while (!_objects.empty()) {
    RemoveObjectFromCube(this,*_objects.begin());
  }
  // Delete _subcubes first
  if (Subdivided()) {
    int i = 0;
    for (; i < 8; ++i) {
      if (_subcubes[i]) {
        --_numSubcubes;
        CubeDelete(_subcubes[i]);
      }
    }
  }
}

// Cleanup the cube variables
void OctreeCube::Cleanup()
{
  _parentCube = NULL;
  // Go through list of object (wrappers) deleting
  while (!_objects.empty()) {
    RemoveObjectFromCube(this,*_objects.begin());
  }
  // Delete _subcubes first
  if (Subdivided()) {
    int i = 0;
    for (; i < 8; ++i) {
      if (_subcubes[i]) {
        --_numSubcubes;
        CubeDelete(_subcubes[i]);
      }
    }
  }
  assert(!_numSubcubes && "Just cleaned up, shouldn't be any more subcubes");;
}

// Add an object and return whether trueful
bool AddObjectToCube(OctreeCube * pCube, Object * object)
{
#define FLIP(bits,flips) (unsigned char)((unsigned int)bits ^ ((unsigned int)(flips)))
  // Get some things
  unsigned char SubInd;
  //maths::Vector<3, float> ObjPos = object->GetPosition();
  float R = object->GetMaxRadius();
  float fMaxDiam = R * 2.0f;
  // Find if needs to be in a smaller cube
  bool bSmaller = (fMaxDiam * OctreeCube::MaxObjectDiametersInCube() <= pCube->_halfSize
      && pCube->_level < OctreeCube::s_maxLevel);
  if (bSmaller) {
    // Calculate which subcubes are intersected, Add each of them
    maths::Vector<3, float> V = (maths::Vector<3, float>)(object->GetPosition(pCube->_mid));
    R *= R;
    bool bXYZ = true,
    bXY = true, bYZ = true, bZX = true;
    // First cube
    pCube->DetermineOctrantRelative(V,SubInd);
    for (int i = 0; i < 3; ++i) {
      V[i] = V[i]*V[i];
    }
    AddObjectToCube(pCube->GetSub(SubInd,true),object);
    // Face cubes
    if (V[0] < R) AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x1),true),object); else bXY = bZX = false;
    if (V[1] < R) AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x2),true),object); else bXY = bYZ = false;
    if (V[2] < R) AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x4),true),object); else bYZ = bZX = false;
    // Edge cubes
    float fXY;
    if (bXY && (fXY = V[0]+V[1]) < R) AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x3),true),object); else bXYZ = false;
    if (bYZ && V[1]+V[2] < R)         AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x6),true),object); else bXYZ = false;
    if (bZX && V[2]+V[0] < R)         AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x5),true),object); else bXYZ = false;
    // Vertex cubes
    if (bXYZ && fXY+V[2] < R)        AddObjectToCube(pCube->GetSub(FLIP(SubInd,0x7),true),object);
  } else {
    // Put into this sector
    pCube->_objects.insert(object);
    if (object->IsAlpha()) {
      bool found = false;
      std::list<DistanceSortedObject>::iterator it2;
      for (it2 = pCube->_distanceSortedObjects.begin(); it2 != pCube->_distanceSortedObjects.end(); ++it2) {
        if (*it2 == object) {
          found = true;
          break;
        }
      }
      if (!found) {
        pCube->_distanceSortedObjects.push_front(object);
      }
    }
    object->_octreeCubes.insert(pCube);
  }
  return true;
}

// Remove an object and return whether trueful
bool RemoveObjectFromCube(OctreeCube * pCube, Object * object)
{
  if (object->IsAlpha()) {
    // remove from sorted object list.
    std::list<DistanceSortedObject>::iterator it;
    for (it = pCube->_distanceSortedObjects.begin(); it != pCube->_distanceSortedObjects.end(); ++it) {
      if (*it == object) {
        std::list<DistanceSortedObject>::iterator tmpid = it--;
        pCube->_distanceSortedObjects.erase(tmpid);
        break;
      }
    }
  }
  // and the main lists.
  pCube->_objects.erase(object);
  object->_octreeCubes.erase(pCube);
  pCube->RealityCheck(true);
  return true;
}

// Check whether this cube is needed
bool OctreeCube::RealityCheck(bool bRecur)
{
  if (!_numSubcubes && _objects.empty() && _parentCube) {
    assert(!_parentCube->_isMarkedForDeletion && "Parent marked for deletion");;
    // Clean self up since empty and no _subcubes
    _parentCube->RemSub(_index, bRecur);
    return true;
  }
  return false;
}


// Testers
// Return whether this cube could contain any of the sphere specified
bool OctreeCube::ContainsAnyOf(const maths::Vector<3, float> & vPos, float fRad)
{
  maths::Vector<3, float> vTo(vPos - _mid);
  float fBoundry = _halfSize + fRad;
  // If outside of cube, then no
  if (vTo[0] < -fBoundry || vTo[0] > fBoundry ||
      vTo[1] < -fBoundry || vTo[1] > fBoundry ||
      vTo[2] < -fBoundry || vTo[2] > fBoundry) {
    return false;
      }
  // Simply return true otherwise, can't be bothered with cleverness
      return true;
}

// Return whether this cube contains all of the sphere specified
bool OctreeCube::ContainsAllOf(const maths::Vector<3, float> & vPos, float fRad)
{
  maths::Vector<3, float> vTo(vPos - _mid);
  float fBoundry = _halfSize - fRad;

  // If outside of cube, then no
  if ((fBoundry < 0.0f) ||
      (vTo[0] < -fBoundry) || (vTo[0] > fBoundry) ||
      (vTo[1] < -fBoundry) || (vTo[1] > fBoundry) ||
      (vTo[2] < -fBoundry) || (vTo[2] > fBoundry)) {
    return false;
      }
  // Simply return true otherwise, can't be bothered with cleverness
      return true;
}



// Sub divisional
OctreeCube * OctreeCube::GetSub(unsigned char i, bool bCreate)
{
  assert(!_isMarkedForDeletion && "Shouldn't be accessing child cubes of cube marked for deletion");;
  // Up to a maximum of s_maxLevel+1 levels
  if (_level >= s_maxLevel)
    return NULL;
  // Debug checks
  assert(i < 8 && "Octree sub-cube index out of bounds (must be in range [0,7]");;
  if (!bCreate || _subcubes[i])
    return _subcubes[i];
  ++_numSubcubes;
  return (_subcubes[i] = CubeNew(this,i));
}

void OctreeCube::RemSub(unsigned char i, bool bRecur)
{
  // Debug checks
  assert(i < 8 && "Octree sub-cube index out of bounds (must be in range [0,7]");;
  assert(_subcubes[i] && "Octree sub-cube isn't set so cannot be removed");;
  --_numSubcubes;
  CubeDelete(_subcubes[i]);
  if (bRecur)
    RealityCheck(true);
}

// Downward efficient rendering
void OctreeCube::DownwardRender(const Observer & observer, unsigned int iFrameId, float fCamConst, OctreeRenderOptions Settings)
{
  //#define PERFORM_CUBE_BEHIND_CAMERA_CHECK
//#define PERFORM_CUBE_FRUSTUM_CHECK
    //#define PERFORM_OBJ_FRUSTUM_CHECK
  // Do occlusion culling
  if ((Settings & OctreeOcclusionCull) == OctreeOcclusionCull) {
    int visibility = observer.OcclusionCheck(_mid, _halfSize * Sqrt3);
    if (visibility < 0) {
      // If not visible, don't bother drawing
      return;
    } else if (visibility > 0) {
      // If entirely visible, don't bother checking for subcubes
      Settings = (OctreeRenderOptions)(Settings & ~OctreeOcclusionCull);
    }
  }
  
  // Render anything in this cube?
  maths::Vector<3, float> relpos(_mid-observer.GetPosition());

  if ((_halfSize*_halfSize*fCamConst > (relpos).sqr())  // Distance/size check
#ifdef PERFORM_CUBE_BEHIND_CAMERA_CHECK
       && ((relpos * observer.Loc.View) > (-Sqrt3)*_halfSize)  // Behind camera check
#endif
#ifdef PERFORM_CUBE_FRUSTUM_CHECK
       && observer.SphereInFrustum(_mid,_halfSize*Sqrt3)       // Frustum check
#endif
     ) {
    // Draw specifically objects in this cube.
    // Non alpha objects first
    std::set<Object*>::iterator it;
    for (it = _objects.begin(); it != _objects.end(); ++it) {
      Object * pPGO = *it;
      if (!pPGO->IsAlpha() && pPGO->GetLastFrameId() != iFrameId) {
        // Check in the viewing frustum
#ifdef PERFORM_OBJ_FRUSTUM_CHECK
        if (observer.SphereInFrustum(pPGO->GetPosition(),pPGO->GetMaxRadius())) {
#endif
          // Needs drawing, update frame and render
          pPGO->SetLastFrameId(iFrameId).RenderObj(observer);
#ifdef PERFORM_OBJ_FRUSTUM_CHECK
        }
#endif
      }
    }
    // Then alpha, sorted back to front.
    _distanceSortedObjects.sort();
    std::list<DistanceSortedObject>::iterator alphait;
    for (alphait = _distanceSortedObjects.begin(); alphait != _distanceSortedObjects.end(); ++alphait) {
      Object * pPGO = **alphait;
      if (pPGO->GetLastFrameId() != iFrameId) {
        // Check in the viewing frustum
#ifdef PERFORM_OBJ_FRUSTUM_CHECK
        if (observer.SphereInFrustum(pPGO->GetPosition(),pPGO->GetMaxRadius())) {
#endif
          // Needs drawing, update frame and render
          pPGO->SetLastFrameId(iFrameId).RenderObj(observer);
#ifdef PERFORM_OBJ_FRUSTUM_CHECK
        }
#endif
      }
    }
    
    // Draw debug cubes?
    if ((Settings & OctreeRenderDebugCube) == OctreeRenderDebugCube) {
      RenderDiagram(_level);
    }

    // Draw objects in sub cubes as well
    if (((Settings & OctreeRenderObjects) == OctreeRenderObjects) && Subdivided()) {
      // First Cube (nearest / farest)
      unsigned char iCubeid,iCubeidComp;
      DetermineOctrant(observer.GetPosition(),iCubeid);
      if ((Settings & OctreeRenderBackToFront) != OctreeRenderBackToFront) {
        iCubeidComp = iCubeid;
        iCubeid = ((~iCubeid) & 0x07);  // Limit to 3 lsb
      } else {
        iCubeidComp = ((~iCubeid) & 0x07); // Limit to 3 lsb
      }
      OctreeCube * pCube = GetSub(iCubeidComp);
      if (pCube) {
        pCube->DownwardRender(observer,iFrameId,fCamConst,Settings);
      }
      // Sort the other 6 Cubes
      unsigned char i;
      // The order to draw the other 6 cubes
      int order[6];
      // Number of cubes in the order
      int norder = 0;
      // The distance of the observer to each cube
      float dist[8];
      // Do an insertion sort
      for (i = 0; i < 8; ++i) {
        if (i != iCubeid && i != iCubeidComp) {
          pCube = GetSub(i);
          if (pCube) {
            dist[i] = observer.DistanceSquared(pCube->_mid);
            int j = 0;
            for (j = 0; j < norder; ++j) {
              if (dist[i] > dist[order[j]]) {
                for (int k = norder; k > j; --k) {
                  order[k] = order[k-1];
                }
                break;
              }
            }
            order[j] = i;
            ++norder;
          }
        }
      }
#if 0
      // Check that the insertion sort worked correctly
      float lastdist = -1.0f;
      for (i = 0; i < norder; ++i) {
        assert(lastdist < 0.0f || dist[order[i]] <= lastdist);
        lastdist = dist[order[i]];
      }
#endif
      for (i = 0; i < norder; ++i) {
        pCube = GetSub(order[i]);
        pCube->DownwardRender(observer,iFrameId,fCamConst,Settings);
      }
      // Last Cube (nearest / farest)
      pCube = GetSub(iCubeid);
      if (pCube) {
        pCube->DownwardRender(observer,iFrameId,fCamConst,Settings);
      }
    }
  }
}

// Render diagramatically the octree structure down to the iMaxLevels level
void OctreeCube::RenderDiagram(unsigned char iMaxLevel) const
{
  // Render self
  maths::Vector<3, float> V = _mid;

  glColor4f(0.5f, 0.0f, 1.0f, 0.1f);
  glBegin(GL_LINES);
  glVertex3(V + maths::Vector<3, float>(-_halfSize, 0.0f, 0.0f));
  glVertex3(V + maths::Vector<3, float>( _halfSize, 0.0f, 0.0f));
  glVertex3(V + maths::Vector<3, float>( 0.0f,-_halfSize, 0.0f));
  glVertex3(V + maths::Vector<3, float>( 0.0f, _halfSize, 0.0f));
  glVertex3(V + maths::Vector<3, float>( 0.0f, 0.0f,-_halfSize));
  glVertex3(V + maths::Vector<3, float>( 0.0f, 0.0f, _halfSize));
  glEnd();
  glColor4f(1.0f, 0.5f, 0.0f, 0.1f);
  glBegin(GL_LINE_LOOP);
  glVertex3(V + maths::Vector<3, float>( 0.0f, -_halfSize,-_halfSize));
  glVertex3(V + maths::Vector<3, float>( 0.0f,  _halfSize,-_halfSize));
  glVertex3(V + maths::Vector<3, float>( 0.0f,  _halfSize, _halfSize));
  glVertex3(V + maths::Vector<3, float>( 0.0f, -_halfSize, _halfSize));
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3(V + maths::Vector<3, float>( -_halfSize, 0.0f,-_halfSize));
  glVertex3(V + maths::Vector<3, float>(  _halfSize, 0.0f,-_halfSize));
  glVertex3(V + maths::Vector<3, float>(  _halfSize, 0.0f, _halfSize));
  glVertex3(V + maths::Vector<3, float>( -_halfSize, 0.0f, _halfSize));
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3(V + maths::Vector<3, float>(-_halfSize,-_halfSize, 0.0f));
  glVertex3(V + maths::Vector<3, float>( _halfSize,-_halfSize, 0.0f));
  glVertex3(V + maths::Vector<3, float>( _halfSize, _halfSize, 0.0f));
  glVertex3(V + maths::Vector<3, float>(-_halfSize, _halfSize, 0.0f));
  glEnd();

  // Render children
  if (_numSubcubes && iMaxLevel > _level) {
    int i = 0;
    for (; i < 8; ++i) {
      if (_subcubes[i]) {
        _subcubes[i]->RenderDiagram(iMaxLevel);
      }
    }
  }
}

/*void OctreeCube::Advance(const mlDate & dClock)
{
  // Advance EVERY SINGLE object
  std::set<Object*>::iterator it = _objects.begin();
  for (; it != _objects.end(); ++it) {
    (*it)->Advance(dClock);
}
  int i = 0;
  for (; i < 8; ++i) {
    if (_subcubes[i])
      _subcubes[i]->Advance(dClock);
}
  RealityCheck(false);
}*/

// Static functions/data for storage of oct cubes which have been destroyed, so that they can be reused
void OctreeCube::CubeDelete(OctreeCube *& pOtherCube)
{
  // Add a spare cube to a list of spares
  assert(!pOtherCube->_isMarkedForDeletion && "Cube already marked for deletion");;
  //assert(!pCube->_numSubcubes && "Deleting cube with subcubes");;
  OctreeCube * pCube = pOtherCube;
  pOtherCube = NULL;
  pCube->Cleanup();
  pCube->_isMarkedForDeletion = true;
  s_spareCubes.push_back(pCube);
}

OctreeCube * OctreeCube::CubeNew(OctreeCube * const rParentCube, unsigned char i)
{
  // Get a spare cube or allocate a new one
  if (s_spareCubes.empty())
    return new OctreeCube(rParentCube,i);
  // Get the item at the back of the queue and initialise it
  OctreeCube * pTmp = s_spareCubes.back();
  pTmp->_isMarkedForDeletion = false;
  s_spareCubes.pop_back();
  return &(pTmp->Init(rParentCube,i));
}

void OctreeCube::CleanupStatic()
{
  CubeListResize(0);
}

void OctreeCube::CubeListResize(unsigned int iMaxSize)
{
  // Pop elements deleting until down to iMaxSize
  while (s_spareCubes.size() > iMaxSize) {
    delete s_spareCubes.back();
    s_spareCubes.pop_back();
  }
}
