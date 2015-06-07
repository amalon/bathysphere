/*
 * octree.h
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

#ifndef _FILE_octree_h
#define _FILE_octree_h

#include "maths/Vector.h"
#include "object.h"
#include "observer.h"
#include "constants.h"

// Standard Template Library (STL)
#include <set>
#include <deque>

// Bits for render option
enum OctreeRenderOptions {
  OctreeRenderBackToFront = 0x01,
  OctreeRenderObjects = 0x02,
  OctreeRenderDebugCube = 0x04,
  OctreeOcclusionCull = 0x08,
  OctreeRenderDefault = OctreeRenderBackToFront | OctreeRenderObjects | OctreeOcclusionCull,
  OctreeRenderAll = OctreeRenderBackToFront | OctreeRenderObjects | OctreeRenderDebugCube | OctreeOcclusionCull
};

/// A cube in the octree
class OctreeCube
{
  public:
    // Constructors
    OctreeCube(float halfSize); // For construction of a base cube
    OctreeCube(OctreeCube * const parentCube, unsigned char i);  // For construction of a child cube
    OctreeCube & Init(OctreeCube * const parentCube, unsigned char i);

// Destructor
    ~OctreeCube();
    void Cleanup();
    static void CleanupStatic();

// Main functions
    friend bool AddObjectToCube(OctreeCube * cube, Object * obj); // Add an object and return whether successful
    friend bool RemoveObjectFromCube(OctreeCube * cube, Object * obj);      // Remove an object and return whether successful
    
    // for object iteration
    typedef std::set<Object*>::iterator ObjectIterator;
    typedef std::set<Object*>::const_iterator ConstObjectIterator;
    ObjectIterator ObjectsBegin()
    {
      return _objects.begin();
    }
    ConstObjectIterator ObjectsBegin() const
    {
      return _objects.begin();
    }
    ObjectIterator ObjectsEnd()
    {
      return _objects.end();
    }
    ConstObjectIterator ObjectsEnd() const
    {
      return _objects.end();
    }

    void RenderDiagram(unsigned char maxLevel) const; // Render diagramatically the octree structure down to the iMaxLevels level

    bool RealityCheck(bool recur);  // Check whether this cube is needed

// Downward efficient rendering
    void DownwardRender(const Observer & observer, unsigned int iFrameId, float fCamConst, OctreeRenderOptions Settings);
    inline static float CalcCamConst(const Observer & observer)
    {
      return maths::sqr((2.0f*observer.GetMeanResolution())/(observer.GetFov()*MaxObjectDiametersInCube()) + Sqrt3);
    }
    inline void RenderFromCam(const Observer & observer, unsigned int iFrameId, OctreeRenderOptions Settings)
    {
      DownwardRender(observer,iFrameId, CalcCamConst(observer), Settings);
    }

//omPhysicalGameObj * AddObj(omPhysicalGameObj * pObj); // Add an object and return a reference to the new structure
//bool RemObj(omPhysicalGameObj * pObj);       // Remove an object and return whether successful

// Testers
    inline bool ContainsAnyOf(Object * object)  // Return whether this cube could contain any of pObj
    {
      return ContainsAnyOf(object->GetPosition(), object->GetMaxRadius());
    }
    bool ContainsAnyOf(const maths::Vector<3, float> & vPos, float fRad);  // Return whether this cube could contain any of the sphere specified
    inline bool ContainsAllOf(Object * object)  // Return whether this cube contains all of pObj
    {
      return ContainsAllOf(object->GetPosition(), object->GetMaxRadius());
    }
    bool ContainsAllOf(const maths::Vector<3, float> & vPos, float fRad);  // Return whether this cube contains all of the sphere specified

// Accessors + mutators
    inline float MaxObjectRadius() { return _halfSize / MaxObjectDiametersInCube(); }

// Sub divisional
    inline unsigned char Subdivided() const
    {
      return _numSubcubes;
    }
    OctreeCube * GetSub(unsigned char i, bool bCreate = false);
    inline OctreeCube * GetNextSubcube(int & previous)
    {
      for (++previous; previous < 8; ++previous) {
        if (_subcubes[previous]) {
          return _subcubes[previous];
        }
      }
      return NULL;
    }
    void RemSub(unsigned char i, bool bRecur);
    inline void DetermineOctrant(const maths::Vector<3, float> & pos, unsigned char & octCubes)
    {
      octCubes = 0x0;
      if (pos[0] > _mid[0]) octCubes |= 0x1;
      if (pos[1] > _mid[1]) octCubes |= 0x2;
      if (pos[2] > _mid[2]) octCubes |= 0x4;
    }
    inline void DetermineOctrantRelative(const maths::Vector<3, float> & pos, unsigned char & octCubes)
    {
      octCubes = 0x0;
      if (pos[0] > 0.0f) octCubes |= 0x1;
      if (pos[1] > 0.0f) octCubes |= 0x2;
      if (pos[2] > 0.0f) octCubes |= 0x4;
    }

// Static accessors
    inline static unsigned char MaxLevels()
    {
      return s_maxLevel + 1;
    }
    inline static float MaxObjectDiametersInCube()
    {
      return 1.0f;
    }

  protected:
    friend class Object;

// Constant data
    OctreeCube * _parentCube;  // Parent cube
    unsigned char _index,_level, _numSubcubes;    // Cube level starting from 0 @ top level
    maths::Vector<3, float> _mid; float _halfSize;   // Centre of cube and half size

// Variable data
    /// All objects.
    std::set<Object*> _objects;
    /// Alpha objects sorted by distance to observer.
    std::list<DistanceSortedObject> _distanceSortedObjects;
    OctreeCube * _subcubes[8];
    bool _isMarkedForDeletion;

  protected:
// Static functions/data for storage of oct cubes which have been destroyed, so that they can be reused
    static std::deque<OctreeCube*> s_spareCubes;

    static void CubeDelete(OctreeCube *& pCube);
    static OctreeCube * CubeNew(OctreeCube * const parentCube, unsigned char i);
    static void CubeListResize(unsigned int maxSize = 0);

// Static options
    static unsigned char s_maxLevel;
};

/// Octree management class
class Octree
{
  public:
// Constructors
    Octree(float fSize);
// Destructor
// inline ~omOctree() {}

// Main functions
    Object * AddObject(Object * pObj);         // Add an object and return a reference to the new structure
    Object * MoveObject(Object * pObj);        // Add an object and return a reference to the new structure
    bool RemObject(Object * pObj);               // Remove an object and return whether successful
    void RenderDiagram(unsigned char iMaxLevel) const; // Render diagramatically the octree structure down to the iMaxLevels level
    inline void RenderFromCam(const Observer & observer, unsigned int frameId, OctreeRenderOptions settings)
    {
      _biggestCube.RenderFromCam(observer,frameId, settings);
    }
    
    inline OctreeCube * GetTopCube()
    {
      return &_biggestCube;
    }
    
    /// Give all contained object a signal.
    void SignalAllObjects(unsigned int signal, void * data);

  protected:
// Data
    OctreeCube _biggestCube;
};

#endif // _FILE_octree_h
