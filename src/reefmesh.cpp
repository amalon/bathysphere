/*
 * reefmesh.cpp
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

#include "reefmesh.h"
#include "constants.h"

extern "C" {
#include "random.h"
}

#include "maths/TemplateMaths.h"

/// Constructor from number of vertex & triangle fields.
/**
 */
ReefMesh::ReefMesh(float halfsize, unsigned int hash)
  : AdaptiveMesh(2, 0, 2, 1,
                 ReefMesh::getIndexVertexNormal(),
                 ReefMesh::getIndexVertexDiffuse(),
                 ReefMesh::getIndexTrianglePlane(),
                 ReefMesh::getIndexTriangleEdgePlanes(),
                 ReefMesh::getIndexTriangleMergeMisses()),
    _initialHash(hash)
{
  if (true) {
    // Initialise to a single very large triangle.
    init(3, 1);
    const Mesh::MeshData<float       > & vertexPositions    = getPermanentVertexPositions  ();
    const Mesh::MeshData<float       > & vertexNormals      = getPermanentVertexFieldFloat (getIndexVertexNormal());
    const Mesh::MeshData<unsigned int> & triangleVerts      = getPermanentTriangleVerts    ();
    
    for (int i = 0; i < 3; ++i) {
      float angle = 2.0f*PI*i/3;
      maths::setVector3(vertexPositions[ i], 5.0f * halfsize * maths::cos(angle), 5.0f * halfsize * maths::sin(angle), 0.0f);
      maths::setVector3(vertexNormals[ i], 0.0f, 0.0f, 1.0f);
    }
    maths::setVector3(triangleVerts[ 0], 0u, 1u, 2u);
    
  } else if (true) {
    // Initialise the number of triangles and vertices
    init(4, 2);
    const Mesh::MeshData<float       > & vertexPositions    = getPermanentVertexPositions  ();
    const Mesh::MeshData<float       > & vertexNormals      = getPermanentVertexFieldFloat (getIndexVertexNormal());
    const Mesh::MeshData<unsigned int> & triangleVerts      = getPermanentTriangleVerts    ();
    
    maths::setVector3(vertexPositions[ 0], -halfsize, -halfsize, 0.0f);
    maths::setVector3(vertexPositions[ 1],  halfsize, -halfsize, 0.0f);
    maths::setVector3(vertexPositions[ 2], -halfsize,  halfsize, 0.0f);
    maths::setVector3(vertexPositions[ 3],  halfsize,  halfsize, 0.0f);
    
    maths::setVector3(vertexNormals[ 0], 0.0f, 0.0f, 1.0f);
    maths::setVector3(vertexNormals[ 1], 0.0f, 0.0f, 1.0f);
    maths::setVector3(vertexNormals[ 2], 0.0f, 0.0f, 1.0f);
    maths::setVector3(vertexNormals[ 3], 0.0f, 0.0f, 1.0f);
    
    maths::setVector3(triangleVerts[ 0], 0u, 1u, 2u);
    maths::setVector3(triangleVerts[ 1], 1u, 3u, 2u);
  
  } else if (false) {
    // used for testing collision detection, produces cones, convex + concave
    // Initialise the number of triangles and vertices
    init(33, 32);
    const Mesh::MeshData<float       > & vertexPositions    = getPermanentVertexPositions  ();
    const Mesh::MeshData<float       > & vertexNormals      = getPermanentVertexFieldFloat (getIndexVertexNormal());
    const Mesh::MeshData<unsigned int> & triangleVerts      = getPermanentTriangleVerts    ();
    
    for (unsigned int i = 0; i < 32; ++i) {
      float ang = 2*PI*i/32;
      maths::setVector3(vertexPositions[ i], halfsize*maths::sin(ang), halfsize*maths::cos(ang), 1.0f);
      maths::setVector3(vertexNormals[ i], 0.0f, 0.0f, 3.0f);
      maths::setVector3(triangleVerts[ i], (i+1u)%32u, i, 32u);
    }
    maths::setVector3(vertexPositions[32],  0.0f,  0.0f, 3.0f);
    maths::setVector3(vertexNormals[32], 0.0f, 0.0f, 1.0f);
    
  }
  
  calculatePermanentNeighbours();
  calculatePermanentPlanes();
}


/// Interpolate the vertices in an edge.
/**
 * @param pOutput[
 * out] Output vector.
 * @param pTriangle Triangle block.
 * @param iTriangleIndex Triangle index within block.
 * @param iEdgeIndex Edge index.
 */
void ReefMesh::interpolateEdge(maths::Vector<3,float> & pOutput, Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex)
{
  // Use simple interpolate edge to get mid point
  const unsigned int * pTriVerts = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex];
  
  // Get first vertex
  const Mesh::MeshBlock * pVPBlock1 = getVertexBlock(pTriVerts[iEdgeIndex]);
  const unsigned int iLocalV1 = pTriVerts[iEdgeIndex] - pVPBlock1->getStartIndex();
  const maths::Vector<3,float> & p1 = *(maths::Vector<3,float>*)pVPBlock1->getFieldFloat(getIndexVertexPosition()) [iLocalV1];
  
  // Get second vertex
  const Mesh::MeshBlock * pVPBlock2 = getVertexBlock(pTriVerts[(iEdgeIndex+1)%3]);
  const unsigned int iLocalV2 = pTriVerts[(iEdgeIndex+1)%3] - pVPBlock2->getStartIndex();
  const maths::Vector<3,float> & p2 = *(maths::Vector<3,float>*)pVPBlock2->getFieldFloat(getIndexVertexPosition()) [iLocalV2];
  
  // Use simple interpolation
  //simpleInterpolateEdge(pOutput, p1, p2);
  
  // Get mid point and resize to radius.
  maths::add(pOutput,p1,p2);
  pOutput /= 2;
  maths::Vector<3,float> vNorm = pOutput;
  maths::Vector<3,float> vDir = pOutput;
  
  /// @todo ?put perturbation in adaptive mesh?
  float fLenSq = (p1-p2).sqr()*0.25f;
  float fPerturb = calcPerturbation(pOutput, fLenSq);
  
  const unsigned int iVertId = pTriVerts[3 + iEdgeIndex];
  Mesh::MeshBlock * pVertBlock = getVertexBlock(iVertId);
  if (fPerturb != 0.0f) {
    maths::Vector<3,float> * pNormal = (maths::Vector<3,float>*) pVertBlock->getFieldFloat(getIndexVertexNormal()) [iVertId - pVertBlock->getStartIndex()];
    maths::Vector<3,float> tangent = p1-p2;
    maths::Vector<3,float> binormal;
    maths::cross(binormal, maths::Vector<3, float>(0.0f, 0.0f, 1.0f), tangent);
    maths::cross(*pNormal, tangent, binormal);
    pNormal->normalize();
    pOutput += (*pNormal)*fPerturb;
  }
  
  // Diffuse colours
  const maths::Vector<4,float> & diffuse1 = *(maths::Vector<4,float>*)pVPBlock1->getFieldFloat(getIndexVertexDiffuse()) [iLocalV1];
  const maths::Vector<4,float> & diffuse2 = *(maths::Vector<4,float>*)pVPBlock2->getFieldFloat(getIndexVertexDiffuse()) [iLocalV2];
  
  maths::Vector<4,float> & diffuseResult = *(maths::Vector<4,float>*) pVertBlock->getFieldFloat(getIndexVertexDiffuse()) [iVertId - pVertBlock->getStartIndex()];
  
  // Find the midpoint
  diffuseResult = (diffuse1 + diffuse2) * 0.5f;
  
  // Hash the parameters to get some predictable numbers.
  unsigned int hash = _initialHash;
  hash = hash_32(sizeof(const maths::Vector<3,float>), (unsigned char*)(void*)&p1, hash);
  hash = hash_32(sizeof(const maths::Vector<3,float>), (unsigned char*)(void*)&p2, hash);
  hash = hash_32(sizeof(const maths::Vector<4,float>), (unsigned char*)(void*)&diffuse1, hash);
  hash = hash_32(sizeof(const maths::Vector<4,float>), (unsigned char*)(void*)&diffuse2, hash);
  unsigned int hashes[3];
  unsigned int number = 57;
  hashes[0] = hash_32(sizeof(unsigned int), (unsigned char*)(void*)&number, hash);
  number = 933;
  hashes[1] = hash_32(sizeof(unsigned int), (unsigned char*)(void*)&number, hashes[0]);
  number = 2399;
  hashes[2] = hash_32(sizeof(unsigned int), (unsigned char*)(void*)&number, hashes[1]);
  for (int i = 0; i < 3; ++i) {
    // vary depending on distance between verts
    float variance = sqrt(fLenSq) / 20.0f;
    if (variance > 0.5f) {
      variance = 0.5f;
    }
    diffuseResult[i] += variance * (float)hashes[i] / ((unsigned int)-1);
    // reflect if outside of [0,1]
    if (diffuseResult[i] > 1.0f) {
      diffuseResult[i] = 1.0f - diffuseResult[i];
    }
    if (diffuseResult[i] < 0.0f) {
      diffuseResult[i] = - diffuseResult[i];
    }
  }
  
  return;
  
  /*
  float fTexCoord = 0.5f + fNewAlt*5e-1f;
  if (fTexCoord > 0.98f) fTexCoord = 0.98f;
  if (fTexCoord < 0.02f) fTexCoord = 0.02f;
  pVertBlock->getFieldFloat(getIndexVertexTexCoord1()) [iVertId - pVertBlock->getStartIndex()] [0]
      = 0.0;
  pVertBlock->getFieldFloat(getIndexVertexTexCoord1()) [iVertId - pVertBlock->getStartIndex()] [1]
      = fTexCoord;
  */
  
  //float * pDiffuse = pVertBlock->getFieldFloat(getIndexVertexDiffuse()) [iVertId - pVertBlock->getStartIndex()];
  //maths::Vector<3,float> * pNormal = (maths::Vector<3,float>*) pVertBlock->getFieldFloat(getIndexVertexNormal()) [iVertId - pVertBlock->getStartIndex()];
  
  
  // Use altitude to determine pDiffuse
  /*maths::Vector<3,float> tangent = p1-p2;
  maths::Vector<3,float> binormal;
  maths::cross(binormal, pOutput, tangent);
  maths::cross(*pNormal, tangent, binormal);
  pNormal->normalize();*/
}

/// Calculate a perturbation distance on a new vertex.
/**
 * @param p Position before perturbation.
 * @param fEdgeLengthSq Approximate distance between p and one of the vertices from which it was created.
 * @return The ammount of perturbation.
 */
float ReefMesh::calcPerturbation(const maths::Vector<3,float> & p, float fEdgeLengthSq)
{
  // Hash the parameters to get a predictable number.
  unsigned int hash = _initialHash;
  hash = hash_32(sizeof(const maths::Vector<3,float>), (unsigned char*)(void*)&p, hash);
  hash = hash_32(sizeof(float), (unsigned char*)(void*)&fEdgeLengthSq, hash);
  float fRand = (float)hash / ((unsigned int)-1);
  // Map it to [-1,1]
  fRand = -1.0f + 2.0f*fRand;
  float fRatio = fRand*0.3f;
  if (fEdgeLengthSq < 10.0f*10.0f) {
    fRatio *= 0.8f;
  }
  if (fEdgeLengthSq > 20.0f*20.0f) {
    fRatio *= 0.5f;
  }
  if (fEdgeLengthSq > 30.0f*30.0f) {
    fRatio *= 0.0f;
  }
  return sqrt(fEdgeLengthSq) * fRatio;
}
