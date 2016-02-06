/*
 * maths/Mesh.cpp
 *
 * Copyright (C) 2007-2015 James Hogan <james@albanarts.com>
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
 * Static triangle mesh.
 *
 */

#include "mesh/Mesh.h"
#include "maths/Vector.h"

#include <GL/gl.h>

/// Calculate the neighbours of the permanent triangles
/**
 * @return Status code:
 *  - true (success)
 *  - false (no neighbour data exists in the mesh)
 *
 * Runs in O(num_triangles^2) time.
 * @note Finds neighbours which match in winding only.
 */
bool mesh::Mesh::calculatePermanentNeighbours()
{
  // Perform painfully exhaustive search for common sets of Vertices

  int iNeighboursIndex = getTriangleFieldUintIndex(MESH_DATA_NEIGHBOURS);
  if (iNeighboursIndex >= 0) {
    const Mesh::MeshData<unsigned int> & triangleNeighbours = getPermanentTriangleFieldUint(iNeighboursIndex);
    const MeshData<unsigned int> & triangleVerts   = getPermanentTriangleVerts();

    // Run through all the triangles
    unsigned int iTri1;
    for (iTri1 = 0; iTri1 < getNumPermanentTriangles(); ++iTri1) {
      // Run through the edges of the first triangle
      int iEdge1;
      for (iEdge1 = 0; iEdge1 < 3; ++iEdge1) {
        // Get two Vertices
        unsigned int iVerts[2] = {triangleVerts[iTri1][iEdge1], triangleVerts[iTri1][(iEdge1+1)%3]};
        // Run through the edges of the second triangle
        int iEdge2;
        for (iEdge2 = 0; iEdge2 < 3; ++iEdge2) {
          // Run through all the remaining triangles
          unsigned int iTri2;
          for (iTri2 = iTri1+1; iTri2 < getNumPermanentTriangles(); ++iTri2) {
            // If edges match, join together
            if ((triangleVerts[iTri2][iEdge2] == iVerts[1]) && (triangleVerts[iTri2][(iEdge2+1)%3] == iVerts[0])) {
              triangleNeighbours[iTri1][iEdge1] = iTri2;
              triangleNeighbours[iTri2][iEdge2] = iTri1;
              goto next_triangle;
            }
          }
        }
      next_triangle:
          ;
      }
    }
    return true;
  } else {
    return false;
  }
}

/// Calculate the planes of the permanent triangles
void mesh::Mesh::calculatePermanentPlanes()
{
  int iPlanesIndex = getTriangleFieldFloatIndex(MESH_DATA_PLANE);
  int iEdgePlanesIndex = getTriangleFieldFloatIndex(MESH_DATA_EDGEPLANES);
  if (iPlanesIndex >= 0 || iEdgePlanesIndex >= 0) {
    MeshData<float>              & trianglePlanes     = getPermanentTriangleFieldFloat(iPlanesIndex);
    MeshData<float>              & triangleEdgePlanes = getPermanentTriangleFieldFloat(iEdgePlanesIndex);
    const MeshData<unsigned int> & triangleVerts      = getPermanentTriangleVerts();
    const MeshData<float>        & vertexPositions    = getPermanentVertexPositions();

    // Run through all the triangles
    unsigned int triangleIndex;
    for (triangleIndex = 0; triangleIndex < getNumPermanentTriangles(); ++triangleIndex) {
      // find the surface normal
      const maths::Vector<3, float> * verts[3] = {
        (const maths::Vector<3,float>*)vertexPositions[triangleVerts[triangleIndex][0]],
        (const maths::Vector<3,float>*)vertexPositions[triangleVerts[triangleIndex][1]],
        (const maths::Vector<3,float>*)vertexPositions[triangleVerts[triangleIndex][2]]
      };
      maths::Vector<3, float> normal;
      maths::cross(normal,
                   *verts[1] - *verts[0],
                   *verts[2] - *verts[1]);
      if (iPlanesIndex >= 0) {
        normal.normalize();

        // find the plane equation of the triangle
        maths::Vector<4, float> * plane = (maths::Vector<4,float>*)(void*)trianglePlanes[triangleIndex];
        *plane = (maths::Vector<4, float>)normal;
        (*plane)[3] = -(*verts[0] * normal);
      }

      if (iEdgePlanesIndex >= 0) {
        // find the plane equation of the edges
        maths::Vector<4, float> * edgePlanes = (maths::Vector<4,float>*)(void*)triangleEdgePlanes[triangleIndex];
        for (int edgeId = 0; edgeId < 3; ++edgeId) {
          // tangential vectors: v[(e+1)%3]-v[e] and face normal
          // find the edge plane normal
          maths::Vector<3, float> edgeNormal;
          maths::cross(edgeNormal,
                       *verts[(edgeId+1)%3] - *verts[edgeId],
                       normal);
          edgeNormal.normalize();

          // and now the plane
          edgePlanes[edgeId] = (maths::Vector<4, float>)edgeNormal;
          edgePlanes[edgeId][3] = -(*verts[edgeId] * edgeNormal);
        }
      }
    }
  }
  return;
}

/// Render the mesh to OpenGL.
void mesh::Mesh::renderGL() const
{
  /// @note Renders permanent triangles and Vertices only.
  const Mesh::MeshData<float       > & vertexPositions = getPermanentVertexPositions ();
  const Mesh::MeshData<unsigned int> & triangleVerts   = getPermanentTriangleVerts   ();

  unsigned int i;

  //glEnable(GL_DEPTH_TEST);

  // Draw triangles
  glColor3f(1,0,1);
  glBegin(GL_TRIANGLES);
  {
    for (i = 0; i < getNumPermanentTriangles(); ++i) {
      glVertex3fv(vertexPositions[triangleVerts[i][0]]);
      glVertex3fv(vertexPositions[triangleVerts[i][1]]);
      glVertex3fv(vertexPositions[triangleVerts[i][2]]);
    }
  }
  glEnd();

  // Draw triangle outlines
  glColor3f(1,1,0);
  for (i = 0; i < getNumPermanentTriangles(); ++i) {
    glBegin(GL_LINE_LOOP);
    {
      glVertex3fv(vertexPositions[triangleVerts[i][0]]);
      glVertex3fv(vertexPositions[triangleVerts[i][1]]);
      glVertex3fv(vertexPositions[triangleVerts[i][2]]);
    }
    glEnd();
  }

  // Draw Vertices
  glColor3f(0,1,1);
  glBegin(GL_POINTS);
  {
    for (i = 0; i < getNumPermanentVertices(); ++i) {
      glVertex3fv(vertexPositions[i]);
    }
  }
  glEnd();


  int iNeighboursIndex = getTriangleFieldUintIndex(MESH_DATA_NEIGHBOURS);
  if (iNeighboursIndex >= 0) {
    const Mesh::MeshData<unsigned int> & triangleNeighbours = getPermanentTriangleFieldUint(iNeighboursIndex);
    glDisable(GL_DEPTH_TEST);
    glBegin(GL_LINES);

    for (i = 0; i < getNumPermanentTriangles(); ++i) {
      int j;
      for (j = 0; j < 3; ++j) {
        unsigned int iNeighbour = triangleNeighbours[i][j];
        if (iNeighbour != MESH_INVALID_TRIANGLE) {
          // Get mid point of neighbour triangle and biased mid point of this triangle
          maths::Vector<3,float> vNeighbourMid = maths::Vector<3,float>(vertexPositions[triangleVerts[iNeighbour][0]]) + maths::Vector<3,float>(vertexPositions[triangleVerts[iNeighbour][1]]) + maths::Vector<3,float>(vertexPositions[triangleVerts[iNeighbour][2]]);
          vNeighbourMid /= 3.0f;
          maths::Vector<3,float> vBiasedMid = (maths::Vector<3,float>(vertexPositions[triangleVerts[i][j]]) + maths::Vector<3,float>(vertexPositions[triangleVerts[i][(j+1)%3]]))*4.0f + maths::Vector<3,float>(vertexPositions[triangleVerts[i][(j+2)%3]]);
          vBiasedMid /= 9.0f;

          glColor3f(0,0,1);
          glVertex3fv(vNeighbourMid);
          glColor3f(1,1,0);
          glVertex3fv(vBiasedMid);
        }
      }
    }
    glEnd();
    //glEnable(GL_DEPTH_TEST);
  }
}

/// Perform a vertex operation on all vertices.
void mesh::Mesh::foreachVertex(VertexOperation op, void * data)
{
  for (unsigned int i = 0; i < getNumPermanentVertices(); ++i) {
    op(data, &m_PermanentVertices, i);
  }
}


