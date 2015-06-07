/*
 * reefmesh.h
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
 * Adaptive mesh specially for reefs.
 *
 */

#ifndef _FILE_reefmesh_h
#define _FILE_reefmesh_h

#include "mesh/AdaptiveMesh.h"

/// Specially adapted adaptive mesh for generating coral reefs.
class ReefMesh : public mesh::AdaptiveMesh
{
  protected:
    /// Number which determines the shape of the reef.
    unsigned int _initialHash;
    
  public:
    /// Main constructor
    ReefMesh(float halfsize, unsigned int hash);
    
    
    // From AdaptiveMesh
    virtual void interpolateEdge(maths::Vector<3,float> & pOutput, Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex);
    //virtual void simpleInterpolateEdge(maths::Vector<3,float> & pOutput, const maths::Vector<3,float> & p1, const maths::Vector<3,float> & p2);
    
    // Used by simpleInterpolateEdge
    virtual float calcPerturbation(const maths::Vector<3,float> & p, float fEdgeLengthSq);
    
    
    /// Get the index of the per triangle merge misses.
    /**
     * @return The index of the per triangle merge misses.
     */
    inline static unsigned int getIndexTriangleMergeMisses()
    {
      return AdaptiveMesh::getFirstTriangleUint();
    }
    
    /// Get the index of the per triangle plane equations.
    /**
     * @return The index of the per triangle plane equations.
     */
    inline static unsigned int getIndexTrianglePlane()
    {
      return AdaptiveMesh::getFirstTriangleFloat();
    }
    
    /// Get the index of the per triangle edge plane equations.
    /**
     * @return The index of the per triangle edge plane equations.
     */
    inline static unsigned int getIndexTriangleEdgePlanes()
    {
      return AdaptiveMesh::getFirstTriangleFloat()+1;
    }
    
    /// Get the index of the per vertex normals.
    /**
     * @return The index of the per vertex normal field.
     */
    inline static unsigned int getIndexVertexNormal()
    {
      return AdaptiveMesh::getFirstVertexFloat();
    }
    
    /// Get the index of the per vertex diffuse colour.
    /**
     * @return The index of the per vertex diffuse field.
     */
    inline static unsigned int getIndexVertexDiffuse()
    {
      return AdaptiveMesh::getFirstVertexFloat()+1;
    }
};

#endif // _FILE_reefmesh_h
