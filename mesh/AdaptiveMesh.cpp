/*
 * maths/AdaptiveMesh.cpp
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
 * Adaptive triangle meshes.
 *
 */

#include "mesh/AdaptiveMesh.h"

#include <GL/gl.h>
#include <cassert>
#include <iostream>


#define ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE 4
#define ADAPTIVE_MESH_VERTEX_BLOCK_SIZE 8

// #define ADAPTIVE_MESH_DRAW_NORMALS 0.01f

#define ADAPTIVE_MESH_QUALITY 1.0e-6f
#define ADAPTIVE_MESH_MERGE_FACTOR 0.1f

#ifdef _ALL_CHECKS
//#define ADAPTIVE_MESH_CONSISTENCY_CHECKS
#endif

#ifdef ADAPTIVE_MESH_CONSISTENCY_CHECKS
#define CHECK_CONSISTENCY() checkConsistency();
#else
#define CHECK_CONSISTENCY()
#endif



const char mesh::AdaptiveMesh::s_aiSplitTriVerts[7][6] =
{
  {0, 1, 2,-1,-1,-1}, // 000
  {2, 0, 3, 1,-1,-1}, // 001
  {0, 1, 4, 2,-1,-1}, // 010
  {3, 1, 4, 2, 0,-1}, // 011
  {5, 0, 1, 2,-1,-1}, // 100
  {3, 1, 2, 5, 0,-1}, // 101
  {5, 0, 1, 4, 2,-1}  // 110
};

/// Constructor from number of vertex & triangle fields.
/**
 * @param nVertexDataFieldsFloat Number of per vertex float data fields.
 * @param nVertexDataFieldsUint Number of per vertex unsigned int data fields.
 * @param nTriangleDataFieldsFloat Number of per triangle float data fields.
 * @param nTriangleDataFieldsUint Number of per triangle unsigned int data fields.
 * @param iIndexNormals Optional Index of the per vertex normals.
 * @param iIndexPlanes Optional Index of the per triangle plane equations.
 * @param iIndexEdgePlanes Optional Index of the per triangle edge plane equations.
 * @param iIndexMergeMisses Optional Index of the per triangle merge_misses.
 */
mesh::AdaptiveMesh::AdaptiveMesh(unsigned int nVertexDataFieldsFloat,
                           unsigned int nVertexDataFieldsUint,
                           unsigned int nTriangleDataFieldsFloat,
                           unsigned int nTriangleDataFieldsUint,
                           int iIndexNormals, int iIndexDiffuse,
                           int iIndexPlanes, int iIndexEdgePlanes, int iIndexMergeMisses)
  : Mesh(nVertexDataFieldsFloat   + 0,
         nVertexDataFieldsUint    + 0,
         nTriangleDataFieldsFloat + 0,
         nTriangleDataFieldsUint  + 4),
    m_VertexIndexNormal(iIndexNormals),
    m_VertexIndexDiffuse(iIndexDiffuse),
    m_TriangleIndexPlanes(iIndexPlanes),
    m_TriangleIndexEdgePlanes(iIndexEdgePlanes),
    m_TriangleIndexMergeMisses(iIndexMergeMisses)
{
  // Change the number of vertex id's per triangle to 6
  m_PermanentTriangles.getFieldUint (getIndexTriangleVerts() ).init(
      new MeshFieldInformation<unsigned int>(MESH_DATA_VERTICES, 6, MESH_INVALID_VERTEX)
                                                                   );
  // Initialise the data fields
  m_PermanentTriangles.getFieldUint(getIndexTriangleSubTriangleBlock()).init(
      new MeshFieldInformation<unsigned int>(MESH_DATA_SUBTRIBLOCK, 1, MESH_INVALID_TRIBLOCK)
                                                                            );
  m_PermanentTriangles.getFieldUint(getIndexTriangleSplitState()).init(
      new MeshFieldInformation<unsigned int>(MESH_DATA_SPLITSTATE, 1, 0x00000000)
                                                                      );
  m_PermanentTriangles.getFieldUint(getIndexTriangleNeighbours()).init(
      new MeshFieldInformation<unsigned int>(MESH_DATA_NEIGHBOURS, 3, MESH_INVALID_TRIANGLE)
                                                                      );
  m_PermanentTriangles.getFieldUint(getIndexTriangleParent()).init(
      new MeshFieldInformation<unsigned int>(MESH_DATA_PARENT, 1, MESH_INVALID_TRIANGLE)
                                                                      );

  if (iIndexNormals >= 0) {
    m_PermanentVertices.getFieldFloat(iIndexNormals).init(
                                      new MeshFieldInformation<float>(MESH_DATA_NORMAL, 3, 0.0f)
                                                         );
  }
  if (iIndexDiffuse >= 0) {
    m_PermanentVertices.getFieldFloat(iIndexDiffuse).init(
                                      new MeshFieldInformation<float>(MESH_DATA_DIFFUSE, 4, 1.0f)
                                                         );
  }
  if (iIndexPlanes >= 0) {
    m_PermanentTriangles.getFieldFloat(iIndexPlanes).init(
                                      new MeshFieldInformation<float>(MESH_DATA_PLANE, 4, 0.0f)
                                                                    );
  }
  if (iIndexEdgePlanes >= 0) {
    m_PermanentTriangles.getFieldFloat(iIndexEdgePlanes).init(
                                      new MeshFieldInformation<float>(MESH_DATA_EDGEPLANES, 4*3, 0.0f)
                                                                    );
  }
  if (iIndexMergeMisses >= 0) {
    m_PermanentTriangles.getFieldUint(iIndexMergeMisses).init(
                                      new MeshFieldInformation<unsigned int>(MESH_DATA_MERGE_MISSES, 1, 0)
                                                                         );
  }
}

/// Destructor
mesh::AdaptiveMesh::~AdaptiveMesh()
{
  std::vector<MeshBlockCounted<unsigned char> *>::iterator it1;
  for (it1 = m_TemporaryVertexBlocks.begin(); it1 != m_TemporaryVertexBlocks.end(); ++it1) {
    delete *it1;
  }
  std::vector<Mesh::MeshBlock *>::iterator it2;
  for (it2 = m_TemporaryTriangleBlocks.begin(); it2 != m_TemporaryTriangleBlocks.end(); ++it2) {
    delete *it2;
  }
  std::list<MeshBlockCounted<unsigned char> *>::iterator it3;
  for (it3 = m_SpareVertexBlocks.begin(); it3 != m_SpareVertexBlocks.end(); ++it3) {
    delete *it3;
  }
  std::list<Mesh::MeshBlock *>::iterator it4;
  for (it4 = m_SpareTriangleBlocks.begin(); it4 != m_SpareTriangleBlocks.end(); ++it4) {
    delete *it4;
  }
}

/// Calculate the triangle planes and edge planes of the triangles in a block.
/**
 * @param block The triangle block.
 */
void mesh::AdaptiveMesh::calculatePlanes(Mesh::MeshBlock * block)
{
  if (getIndexTrianglePlane() >= 0 || getIndexTriangleEdgePlanes() >= 0) {
    const MeshData<float>        & trianglePlanes     = block->getFieldFloat(getIndexTrianglePlane());
    const MeshData<float>        & triangleEdgePlanes = block->getFieldFloat(getIndexTriangleEdgePlanes());
    const MeshData<unsigned int> & triangleVerts      = block->getFieldUint(getIndexTriangleVerts());

    // Run through all the triangles
    unsigned int triangleIndex;
    for (triangleIndex = 0; triangleIndex < block->getNumItems(); ++triangleIndex) {
      // find the surface normal
      const maths::Vector<3, float> * verts[3];
      for (int i = 0; i < 3; ++i) {
        unsigned int vid = triangleVerts[triangleIndex][i];
        const MeshBlock * vertexBlock = getVertexBlock(vid);
        assert(vertexBlock && "Vertices must exist");
        verts[i] = (const maths::Vector<3,float>*) (void*) vertexBlock->getFieldFloat(getIndexVertexPosition())[vid - vertexBlock->getStartIndex()];
      }
      maths::Vector<3, float> normal;
      maths::cross(normal,
                   *verts[1] - *verts[0],
                   *verts[2] - *verts[1]);
      if (getIndexTrianglePlane() >= 0) {
        normal.normalize();

        // find the plane equation of the triangle
        maths::Vector<4, float> * plane = (maths::Vector<4,float>*)(void*)trianglePlanes[triangleIndex];
        *plane = (maths::Vector<4, float>)normal;
        (*plane)[3] = -(*verts[0] * normal);
      }

      if (getIndexTriangleEdgePlanes() >= 0) {
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

/// Check for anything that could indicate bugs in the code.
void mesh::AdaptiveMesh::checkConsistency() const
{
  /// - 1) Ensure that splits + vertices correspond
  /// - 2) Ensure that any vertices which are set do exist
  /// - 3) Ensure that any fully split triangles are subdivided
  /// - 4) Ensure that any subdivided triangle's subblocks exist
  /// - 5) Ensure that neighbours link back

  unsigned int triangleIndex;
  for (triangleIndex = 0; triangleIndex < getNumPermanentTriangles(); ++triangleIndex) {
    checkTriangleConsistency(&m_PermanentTriangles,
                              triangleIndex);
  }
}

/// Check for anything within a triangle that could indicate bugs in the code.
void mesh::AdaptiveMesh::checkTriangleConsistency(const Mesh::MeshBlock * triangle, unsigned int triangleIndex) const
{
  int i;
  unsigned char splits = getTriangleSplitBits(triangle, triangleIndex);
  assert(splits < 0x8 && "Splits value out of range");

  const maths::Vector<3, float> * verts[6];
  for (i = 0; i < 6; ++i) {
    unsigned int vid = triangle->getFieldUint(getIndexTriangleVerts()) [triangleIndex] [i];
    if (i < 3) {
      assert(vid != MESH_INVALID_VERTEX && "Core triangle vertex not set");
    } else {
      /// - 1) Ensure that splits + vertices correspond
      if (vid != MESH_INVALID_VERTEX) {
        assert((splits & (0x1<<(i-3))) && "Split - vertex id mismatch, vertex id set, split value not");
      } else {
        assert(!(splits & (0x1<<(i-3))) && "Split - vertex id mismatch, split value set, vertex id not");
      }
    }

    /// - 2) Ensure that any vertices which are set do exist
    if (vid != MESH_INVALID_VERTEX) {
      const MeshBlock * vertexBlock = getVertexBlock(vid);
      if (vid != MESH_INVALID_VERTEX) {
        assert(vertexBlock && "Triangle vertex block missing despite vertex id being set");
      }
      verts[i] = (const maths::Vector<3,float>*) (void*) vertexBlock->getFieldFloat(getIndexVertexPosition())[vid - vertexBlock->getStartIndex()];
    } else {
      verts[i] = NULL;
    }
  }

  /// - 3) Ensure that any fully split triangles are subdivided
  const unsigned int subTriBlock = triangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [triangleIndex] [0];
  if (splits == 0x7) {
    /// - 4) Ensure that any subdivided triangle's subblocks exist
    assert(subTriBlock != MESH_INVALID_TRIBLOCK && "Fully split triangle has no sub block set");
    Mesh::MeshBlock * block = m_TemporaryTriangleBlocks[subTriBlock];
    assert(block && "Sub block missing from fully split triangle with sub block id");
    for (i = 0; i < (int)block->getNumItems(); ++i) {
      checkTriangleConsistency(block, i);
    }
  } else {
    assert(subTriBlock == MESH_INVALID_TRIBLOCK && "Non fully split triangle has a sub block set");
  }

  /// - 5) Ensure that neighbours link back + share the same verticies
  const unsigned int * neighbourTriangles = triangle->getFieldUint(getIndexTriangleNeighbours())[triangleIndex];
  for (i = 0; i < 3; ++i) {
    if (neighbourTriangles[i] != MESH_INVALID_TRIANGLE) {
      const Mesh::MeshBlock * neighbourBlock = getTriangleBlock(neighbourTriangles[i]);
      assert(neighbourBlock && "Neighbour triangle id set but neighbour block missing");
      const unsigned int * neighbourNeighbourTriangles = neighbourBlock->getFieldUint(getIndexTriangleNeighbours())[neighbourTriangles[i]-neighbourBlock->getStartIndex()];
      // triangleIndex+triangle->getStartIndex() should be in neighbourNeighbourTriangles
      bool found = false;
      for (int j = 0; j < 3; ++j) {
        if (triangleIndex+triangle->getStartIndex() == neighbourNeighbourTriangles[j]) {
          // check that they also share the same vertices
          assert(triangle->getFieldUint(getIndexTriangleVerts()) [triangleIndex] [(i+1)%3] == neighbourBlock->getFieldUint(getIndexTriangleVerts()) [neighbourTriangles[i]-neighbourBlock->getStartIndex()] [j] && "Neighbour triangles dont't share the same verticies 1");
          assert(triangle->getFieldUint(getIndexTriangleVerts()) [triangleIndex] [i] == neighbourBlock->getFieldUint(getIndexTriangleVerts()) [neighbourTriangles[i]-neighbourBlock->getStartIndex()] [(j+1)%3] && "Neighbour triangles dont't share the same verticies 2");
          found = true;
          break;
        }
      }
      assert(found && "Neighbour triangle not linked back");
    }
  }
}

/// Adapt the mesh so that there is more quality closer to the observer.
/**
 * @param fObserver Pointer to three floats representing position of observer.
 * @param iOperationLimit Maximum number of operations to perform (a value of zero removes the limit).
 * @param maxDepth The maximum depth to subdivide to.
 */
void mesh::AdaptiveMesh::adaptToObserver(const maths::Observer<float> & observer, unsigned int * pOperationLimit, unsigned int maxDepth, bool clean)
{
  /*
  // Adapt each permanent triangle seperately.
  if (m_SortedPermanentTriangleList.empty()) {
    for (iTriangleIndex = 0; iTriangleIndex < getNumPermanentTriangles(); ++iTriangleIndex) {
      float distSq = observer.distanceFactor();
      m_SortedPermanentTriangleList.push_back();
    }
  }
  */

  unsigned int iOffset = 0;
  if (!m_TriangleAdaptionStack.empty()) {
    iOffset = m_TriangleAdaptionStack.back();
    m_TriangleAdaptionStack.pop_back();
  }
  unsigned int iTriangleIndex;
  for (iTriangleIndex = 0; iTriangleIndex < getNumPermanentTriangles(); ++iTriangleIndex) {
    /*
    used up ops: decrease depth
    not used up ops: incrase depth
    */
    unsigned int iOps = *pOperationLimit;
    adaptTriangleToObserver(observer,
                            &m_PermanentTriangles,
                            (iTriangleIndex+iOffset)%getNumPermanentTriangles(),
                            &iOps, clean,
                            maxDepth);

    if (pOperationLimit && !*pOperationLimit) {
      m_TriangleAdaptionStack.push_back((iTriangleIndex+iOffset+1)%getNumPermanentTriangles());
      return;
    }

  }

  /*/for (int i = 0; i < 100; ++i) {
    // For now, pick a random edge and split it
    //srand((unsigned)time(0));
    int iTriangleIndex = rand() % getUpperBoundNumTriangles();
    int iEdgeIndex = rand() % 3;

    Mesh::MeshBlock * pTriangle = getTriangleBlock(iTriangleIndex);
    // Only continue if edge isn't already split
    if (pTriangle && !isEdgeSplit(pTriangle, iTriangleIndex-pTriangle->getStartIndex(), iEdgeIndex)) {
      splitEdge(pTriangle, iTriangleIndex-pTriangle->getStartIndex(), iEdgeIndex);
      //splitAllEdges(pTriangle, iTriangleIndex-pTriangle->getStartIndex());
    }
  }//*/

  // Gradually empty out the spare block queues
  if (!m_SpareTriangleBlocks.empty()) {
    delete m_SpareTriangleBlocks.front();
    m_SpareTriangleBlocks.pop_front();
  }
  if (!m_SpareVertexBlocks.empty()) {
    delete m_SpareVertexBlocks.front();
    m_SpareVertexBlocks.pop_front();
  }
}

/// Adapt a specific triangle to an observer.
/**
 * Splits or remerges triangles to attain optimum triangle density.
 * @param vObserver Pointer to three floats representing position of observer.
 * @param pTriangle Triangle block of triangle.
 * @param iTriangleIndex Local triangle index within @a pTriangle
 *
 * @todo implement operation limit.
 */
void mesh::AdaptiveMesh::adaptTriangleToObserver(const maths::Observer<float> & observer,
                                                 Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex,
                                                 unsigned int * pOperationLimit, bool clean,
                                                 unsigned int iMaxDepth)
{
  /**
   * recursively test triangles for distance from observer
   * when a triangle is big on the screen (> ~50px)
   *  if subdivided test subtriangles
   *  if not, subdivide
   * when a triangle is small on the screen (< ~25px)
   *  remerge
   */

  // Calculate a value for the size of a triangle on the screen
  // area^2 = ((v1-v0) x (v2-v0))^2
  // mid = (v0+v1+v2)/3
  // factor = area^2 / (mid-fObserver)^2

  const unsigned int * pTriVerts = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex];
  maths::Vector<3,float> vVertPos[3];
  int i;
  for (i = 0; i < 3; ++i) {
    assert(pTriVerts[i] != MESH_INVALID_VERTEX && "Vertices not set");
    const Mesh::MeshBlock * pBlock = getVertexBlock(pTriVerts[i]);
    if (!pBlock) {
      pBlock = getVertexBlock(pTriVerts[i]);
    }
    assert(pBlock && "Vertices missing");
    vVertPos[i] = (maths::Vector<3,float>) pBlock->getFieldFloat(getIndexVertexPosition())[pTriVerts[i]-pBlock->getStartIndex()];
  }

  maths::Vector<3,float> vNorm;
  maths::cross(vNorm, vVertPos[1]-vVertPos[0], vVertPos[2]-vVertPos[0]);

  float fAreaSq = vNorm.sqr();

  maths::Vector<3,float> vVector(vVertPos[0]);
  vVector += vVertPos[1];
  vVector += vVertPos[2];
  vVector /= 3.0f;

  float fSquareDist = observer.distanceSquared(vVector);
  float fFactor = fAreaSq / (fSquareDist * fSquareDist);
  fFactor *= fFactor;
  fFactor *= fFactor;
  //fFactor *= fFactor;
  fFactor /= 100.0f;

  float fSplitBoundary = ADAPTIVE_MESH_QUALITY;
  float fMergeBoundary = fSplitBoundary*ADAPTIVE_MESH_MERGE_FACTOR;

  fSplitBoundary *= fSplitBoundary;
  fMergeBoundary *= fMergeBoundary;

  adjustTrianglePriority(pTriVerts, fSplitBoundary, fMergeBoundary);

  bool isVisible = triangleVisibility(pTriangle, iTriangleIndex, observer);

  if (!isVisible) {
    fSplitBoundary *= 200.0f;
    fMergeBoundary *= 200.0f;
  }

  // Find whether the triangle is protected
  unsigned int * merge_misses = NULL;
  int merge_misses_id = getIndexTriangleMergeMisses();
  int protection = 0;
  if (merge_misses_id >= 0) {
    merge_misses = pTriangle->getFieldUint(merge_misses_id) [iTriangleIndex];
    protection = *merge_misses;
    if (*merge_misses) {
      --*merge_misses;
    }
  }

  unsigned int j = 0;
  if (!m_TriangleAdaptionStack.empty()) {
    //j = m_TriangleAdaptionStack.back();
    m_TriangleAdaptionStack.pop_back();
  }
  // Get the split factor so we can decide what to do
  unsigned char iSplits = getTriangleSplitBits(pTriangle, iTriangleIndex);
  if (iSplits >= 0x7) {
    // fully subdivided, check children if still big on screen
    // if protected, go into sub triangles to see if can unprotect anything
    if (((clean || isVisible) && fFactor > fSplitBoundary) || protection) {
      const unsigned int iSubTriBlock = pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0];
      if (iMaxDepth) {
        if (iSubTriBlock != MESH_INVALID_TRIBLOCK) {
          Mesh::MeshBlock * pBlock = m_TemporaryTriangleBlocks[iSubTriBlock];
          if (pBlock) {
            // see which triangle is closest to cam
            int order[4] = {0,1,2,3};
            float dist2[4];
            for (i = 0; i < 3; ++i) {
              dist2[i] = observer.distanceSquared(vVertPos[i]);
            }
            dist2[3] = (dist2[0] + dist2[1] + dist2[2])/3.0f;
            for (i = 0; i < 3; ++i) {
              int k = i+1;
              int orderi = order[i];
              while (k < 4 && dist2[order[k]] < dist2[orderi]) {
                order[k-1] = order[k];
                ++k;
              }
              order[k-1] = orderi;
            }
            for (; j < 4; ++j) {
              adaptTriangleToObserver(observer, pBlock, order[j], pOperationLimit, clean, iMaxDepth-1);
              if (pOperationLimit && !*pOperationLimit) {
                //if (j != 3)
                //  m_TriangleAdaptionStack.push_back(j+1);
                return;
              }
            }
          }
        }
      } else {
        m_TriangleAdaptionStack.clear();
      }
    } else if (!protection && fFactor < fMergeBoundary) {
      joinAllEdges(pTriangle, iTriangleIndex);
      if (pOperationLimit && *pOperationLimit) {
        --*pOperationLimit;
      }
    }
  } else if (isVisible) {
    // not fully subdivided, splitAllEdges if fFactor > some value
    if (fFactor > fSplitBoundary) {
      splitAllEdges(pTriangle, iTriangleIndex);
      if (pOperationLimit && *pOperationLimit) {
        --*pOperationLimit;
      }
    }
  }
  m_TriangleAdaptionStack.clear();
}

/// Split all edges.
/**
 * Splits all edges using splitEdge.
 * @param pTriangle Pointer to the triangle block containing a triangle to be fully split.
 * @param iTriangleIndex Local index of the triangle to be split.
 */
void mesh::AdaptiveMesh::splitAllEdges(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex)
{
  unsigned char iSplits = getTriangleSplitBits(pTriangle, iTriangleIndex);
  if ((iSplits & 0x1) == 0x0)
    splitEdge(pTriangle, iTriangleIndex, 0);
  if ((iSplits & 0x2) == 0x0)
    splitEdge(pTriangle, iTriangleIndex, 1);
  if ((iSplits & 0x4) == 0x0)
    splitEdge(pTriangle, iTriangleIndex, 2);
}

/// Join all edges.
/**
 * Joins all edges using joinEdge.
 * @param pTriangle Pointer to the triangle block containing a triangle to be fully joined.
 * @param iTriangleIndex Local index of the triangle to be joined.
 */
void mesh::AdaptiveMesh::joinAllEdges(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex)
{
  unsigned char iSplits = getTriangleSplitBits(pTriangle, iTriangleIndex);
  if ((iSplits & 0x1) != 0x0)
    joinEdge(pTriangle, iTriangleIndex, 0);
  if ((iSplits & 0x2) != 0x0)
    joinEdge(pTriangle, iTriangleIndex, 1);
  if ((iSplits & 0x4) != 0x0)
    joinEdge(pTriangle, iTriangleIndex, 2);
}

/// Split an edge.
/**
 * Splits the edge by creating a vertex in between.
 * This will subdivide neighbouring triangles if their edges are all split.
 * If the edge only has one neighbouring triangle (i.e. the other one isn't subdivided enough)
 * it will be subdivided first.
 * Hence this function is recursive, it is ensured that the parent triangle's neighbours are
 * subdivided (edge split) first.
 * @param pTriangle Pointer to the triangle block containing a triangle which the edge is a part of.
 * @param iTriangleIndex Local index of the triangle which edge is a part of.
 * @param iEdgeIndex Index of the edge within the triangle.
 */
void mesh::AdaptiveMesh::splitEdge(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex)
{
  /*
  make use of:
  void interpolateEdge(maths::Vector<3,float> & pOutput, Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex);
  */


  // Create new vertex
  Mesh::MeshBlock * pVertexBlock; // Will store the block to which vertex index iVertexIndex belongs.
  unsigned int iVertexIndex = getNewVertex(&pVertexBlock);
  // Set position
  pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex] [3 + iEdgeIndex] = iVertexIndex;
  interpolateEdge(* (maths::Vector<3,float>*) pVertexBlock->getFieldFloat(getIndexVertexPosition()) [iVertexIndex-pVertexBlock->getStartIndex()],
                  pTriangle, iTriangleIndex, iEdgeIndex);

  // Find the neighbour triangle
  unsigned int iNeighbourTriangle = pTriangle->getFieldUint(getIndexTriangleNeighbours())[iTriangleIndex][iEdgeIndex];
  if (iNeighbourTriangle == MESH_INVALID_TRIANGLE) {
    // No neighbour triangle, try splitting the parent's neighbour
    unsigned int iParentTriangle = *pTriangle->getFieldUint(getIndexTriangleParent())[iTriangleIndex];
    if (iParentTriangle != MESH_INVALID_TRIANGLE) {
      Mesh::MeshBlock * pParentBlock = getTriangleBlock(iParentTriangle);
      assert(pParentBlock);
      const Mesh::MeshData<unsigned int> & pParentNeighbours = pParentBlock->getFieldUint(getIndexTriangleNeighbours());
      unsigned char iParentEdge = (iTriangleIndex + iEdgeIndex) % 3;
      // Make sure its fully split
      unsigned int iParentNeighbourTriangle = pParentNeighbours[iParentTriangle-pParentBlock->getStartIndex()][iParentEdge];
      if (iParentNeighbourTriangle != MESH_INVALID_TRIANGLE) {
        Mesh::MeshBlock * pParentNeighbourBlock = getTriangleBlock(iParentNeighbourTriangle);
        assert(pParentNeighbourBlock);
        splitAllEdges(pParentNeighbourBlock, iParentNeighbourTriangle - pParentNeighbourBlock->getStartIndex());

        // This should now have changed!
        iNeighbourTriangle = pTriangle->getFieldUint(getIndexTriangleNeighbours())[iTriangleIndex][iEdgeIndex];
      }
    }
  }

  // subdivideTriangle(pTriangle, iTriangleIndex);

  // Set subvert of triangle and split state
  *pTriangle->getFieldUint(getIndexTriangleSplitState())[iTriangleIndex] |= (0x1 << iEdgeIndex);
  unsigned char splits = getTriangleSplitBits(pTriangle, iTriangleIndex);
  if (splits >= 0x7) {
    subdivideTriangle(pTriangle, iTriangleIndex);
  }

  if (iNeighbourTriangle != MESH_INVALID_TRIANGLE) {
    unsigned int iTriangleGlobalIndex = pTriangle->getStartIndex() + iTriangleIndex;

    Mesh::MeshBlock * pTriangle2 = getTriangleBlock(iNeighbourTriangle);
    unsigned int iTriangleIndex2 = iNeighbourTriangle - pTriangle2->getStartIndex();

    // Get neighbour's neighbours to see which neighbours edge we're connected to
    unsigned int * pNeighbourNeighbours = pTriangle2->getFieldUint(getIndexTriangleNeighbours()) [iTriangleIndex2];
    unsigned char iEdgeIndex2;
    if (pNeighbourNeighbours[0] == iTriangleGlobalIndex) {
      iEdgeIndex2 = 0;
    } else if (pNeighbourNeighbours[1] == iTriangleGlobalIndex) {
      iEdgeIndex2 = 1;
    } else if (pNeighbourNeighbours[2] == iTriangleGlobalIndex) {
      iEdgeIndex2 = 2;
    } else {
      assert(false && "Triangles neighbour is not correctly linked back");
    }

    // Set subvert of triangle and split state
    pTriangle2->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex2] [3 + iEdgeIndex2] = iVertexIndex;
    *pTriangle2->getFieldUint(getIndexTriangleSplitState())[iTriangleIndex2] |= (0x1 << iEdgeIndex2);
    unsigned char splits2 = getTriangleSplitBits(pTriangle2, iTriangleIndex2);
    if (splits2 >= 0x7) {
      subdivideTriangle(pTriangle2, iTriangleIndex2);
    }
  }


  //if T.parent.neighbour[(t+e)%3].subf < 3
  //. split all those edges
  //get triangles
  //get triangle edge id's
  //create new vertex id + set to triangles
  //set vertex position to mid point or use vertex_adapt
  //increment subdivision factor of each triangle
  //if either sub(T1) or sub(T2) == 3 then
  //. create subtriangles in T
}

/// Interpolate the vertices in an edge.
/**
 * @param pOutput[out] Output vector.
 * @param pTriangle Triangle block.
 * @param iTriangleIndex Triangle index within block.
 * @param iEdgeIndex Edge index.
 */
void mesh::AdaptiveMesh::interpolateEdge(maths::Vector<3,float> & pOutput, Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex)
{
  // Use simple interpolate edge to get mid point
  const unsigned int * pTriVerts = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex];

  // Get first vertex
  const Mesh::MeshBlock * pVPBlock = getVertexBlock(pTriVerts[iEdgeIndex]);
  const maths::Vector<3,float> & p1 = *(maths::Vector<3,float>*)pVPBlock->getFieldFloat(getIndexVertexPosition()) [pTriVerts[iEdgeIndex] - pVPBlock->getStartIndex()];

  // Get second vertex
  pVPBlock = getVertexBlock(pTriVerts[(iEdgeIndex+1)%3]);
  const maths::Vector<3,float> & p2 = *(maths::Vector<3,float>*)pVPBlock->getFieldFloat(getIndexVertexPosition()) [pTriVerts[(iEdgeIndex+1)%3] - pVPBlock->getStartIndex()];

  // Use simple interpolation
  simpleInterpolateEdge(pOutput, p1, p2);
}

/// Interpolate between vertices in an edge given the vectors.
/**
 * @param pOutput[out] Output vector.
 * @param p1 First vertex vector.
 * @param p2 Second vertex vector.
 */
void mesh::AdaptiveMesh::simpleInterpolateEdge(maths::Vector<3,float> & pOutput, const maths::Vector<3,float> & p1, const maths::Vector<3,float> & p2)
{
  // Get simple mid point
  maths::add(pOutput,p1,p2) *= 0.5f;
}

/// Join an edge.
/**
 * Joins a split edge by removing the vertex in between.
 * This will remerge the neighbouring triangles if they are subdivided.
 * @param pTriangle Pointer to the triangle block containing a triangle which the edge is a part of.
 * @param iTriangleIndex Local index of the triangle which edge is a part of.
 * @param iEdgeIndex Index of the edge within the triangle.
 *
 * @pre Specified edge must already be split.
 */
void mesh::AdaptiveMesh::joinEdge(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex)
{
  /*
  T1 = pTriangle[iTriangleIndex]
  E1 = iEdgeIndex
  find neighbour T2 and neighbour edge E2
  *CHANGED
  *if both T1 and T2 are split 111b then
  *  if T1.child[E1].split[0] then
  *    joinEdge(T1.child, E1, 0)
  *  if T1.child[(E1+1)%3].split[2] then
  *    joinEdge(T1.child, (E1+1)%3, 2);
  UNSAFE{
    if T1.split == 111b then
      T1.remerge();
    if T2.split == 111b then
      T2.remerge();
    T1.split[E1] = 0b
    T2.split[E2] = 0b
  }
  cleanvert(T1.vert[3+E1]);
  T1.vert[3+E1] = T2.vert[3+E2] = MESH_INVALID_VERTEX
  */

  // Get neighbour index
  unsigned int iGlobalTriangleIndex = pTriangle->getStartIndex() + iTriangleIndex;
  unsigned int iGlobalNeighbourIndex = pTriangle->getFieldUint(getIndexTriangleNeighbours()) [iTriangleIndex] [iEdgeIndex];
  Mesh::MeshBlock * pNeighbourBlock = NULL;

  unsigned int iSplits = getTriangleSplitBits(pTriangle, iTriangleIndex);
  // initialise to work around gcc warning (uninitialised use, but in practice
  // pNeighbourBlock must be set)
  unsigned int iNeighbourSplits = 0;

  unsigned int iNeighbourIndex = MESH_INVALID_TRIANGLE;
  // again, initialise to work around gcc warning
  unsigned int iNeighbourEdgeIndex = 0;

  // Get more info about neighbour such as which of its edges are touching this triangle
  if (iGlobalNeighbourIndex != MESH_INVALID_TRIANGLE) {
    pNeighbourBlock = getTriangleBlock(iGlobalNeighbourIndex);
    if (pNeighbourBlock) {
      iNeighbourIndex = iGlobalNeighbourIndex - pNeighbourBlock->getStartIndex();
      iNeighbourSplits = getTriangleSplitBits(pNeighbourBlock, iNeighbourIndex);
      unsigned int * pNeighbourNeighbours = pNeighbourBlock->getFieldUint(getIndexTriangleNeighbours()) [iNeighbourIndex];
      if (pNeighbourNeighbours[0] == iGlobalTriangleIndex)
        iNeighbourEdgeIndex = 0;
      else if (pNeighbourNeighbours[1] == iGlobalTriangleIndex)
        iNeighbourEdgeIndex = 1;
      else
        iNeighbourEdgeIndex = 2;
    }
  }

  // Go through outer triangles of children of both triangles to ensure all smaller edges are joined
  if (iSplits >= 0x7) {
    // Get the child block
    unsigned int iChildBlock = pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0];
    Mesh::MeshBlock * pChildBlock = m_TemporaryTriangleBlocks[iChildBlock];
    //const Mesh::MeshData<unsigned int> & rChildSplits = pChildBlock->getFieldUint(getIndexTriangleSplitState());
    // Go through outer triangles joining edges
    int iTriCounter;
    for (iTriCounter = 0; iTriCounter < 3; ++iTriCounter) {
      joinAllEdges(pChildBlock, iTriCounter);
    }
  }
  if (pNeighbourBlock && iNeighbourSplits >= 0x7) {
    // Get the child block
    unsigned int iChildBlock = pNeighbourBlock->getFieldUint(getIndexTriangleSubTriangleBlock()) [iNeighbourIndex] [0];
    Mesh::MeshBlock * pChildBlock = m_TemporaryTriangleBlocks[iChildBlock];
    //const Mesh::MeshData<unsigned int> & rChildSplits = pChildBlock->getFieldUint(getIndexTriangleSplitState());
    // Go through outer triangles joining edges
    int iTriCounter;
    for (iTriCounter = 0; iTriCounter < 3; ++iTriCounter) {
      joinAllEdges(pChildBlock, iTriCounter);
    }
  }

  // The following chunk of code should be completed to remain stable.
  {
    // cleanup after subtriangles of both sides of edge
    if (iSplits >= 0x7)
      remergeTriangle(pTriangle, iTriangleIndex);
    if (pNeighbourBlock && iNeighbourSplits >= 0x7)
      remergeTriangle(pNeighbourBlock, iNeighbourIndex);
    // unset the split bit of the edge for the triangle of each side.
    pTriangle->getFieldUint(getIndexTriangleSplitState()) [iTriangleIndex] [0] &= ~(0x1 << iEdgeIndex);
    if (pNeighbourBlock) {
      pNeighbourBlock->getFieldUint(getIndexTriangleSplitState()) [iNeighbourIndex] [0] &= ~(0x1 << iNeighbourEdgeIndex);
    }

    // Clean up after the vertex
    unsigned int vid = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex] [3+iEdgeIndex];
    removeVertex(vid);
    assert(!getVertexBlock(vid) && "vertex not deleted");

    // Remove the references to the vertex from both sides of the edge
    pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex] [3+iEdgeIndex] = MESH_INVALID_VERTEX;
    if (pNeighbourBlock) {
      pNeighbourBlock->getFieldUint(getIndexTriangleVerts()) [iNeighbourIndex] [3+iNeighbourEdgeIndex] = MESH_INVALID_VERTEX;
    }
  }
}

/// Subdivide a triangle.
/**
 * Creates four child triangles within the specified triangle.
 * Links subtriangles with neighbours (subtriangles of neighbouring triangles).
 * @param pTriangle Pointer to the triangle block containing the triangle which isn't subdivided but has a split factor of 3.
 * @param iTriangleIndex Local index of the triangle.
 */
void mesh::AdaptiveMesh::subdivideTriangle(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex)
{

  // assume split_bits==111b
  // assume all 3 neighbour triangles exist

  unsigned int iTriangleGlobalIndex = iTriangleIndex + pTriangle->getStartIndex();

  // Create the new triangles
  Mesh::MeshBlock * pChildBlock;
  unsigned int iChildBlockId = getNewTriangleBlock(&pChildBlock);
  unsigned int iChildBlockStart = pChildBlock->getStartIndex();
  // Save the block id in this triangle
  assert(getTriangleBlock(iChildBlockStart) != pTriangle && "new child block is self");
  pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0] = iChildBlockId;

  // Triangle vertex id -> global vertex id
  const unsigned int * pTriVerts = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex];
  const unsigned int * pTriNeighbours = pTriangle->getFieldUint(getIndexTriangleNeighbours()) [iTriangleIndex];
  const Mesh::MeshData<unsigned int> & pChildVerts = pChildBlock->getFieldUint(getIndexTriangleVerts());
  const Mesh::MeshData<unsigned int> & pChildNeighbours = pChildBlock->getFieldUint(getIndexTriangleNeighbours());
  const Mesh::MeshData<unsigned int> & pChildParents = pChildBlock->getFieldUint(getIndexTriangleParent());

  // Link vertices
  for (int i = 0; i < 3; ++i) {
    // Set vertices
    pChildVerts [i] [0] =                       pTriVerts[i]; // Outer corner
    pChildVerts [i] [1] =                       pTriVerts[3+i]; // First mid vert
    pChildVerts [i] [2] = pChildVerts [3] [i] = pTriVerts[3+(i+2)%3]; // Second mid vert
    // Set internal neighbours
    pChildNeighbours [i] [1] = iChildBlockStart + 3; // Inside edge to middle triangle
    pChildNeighbours [3] [i] = iChildBlockStart + i; // Middle triangle to inside edge

    /*
      for each edge/edge triangle
      . get n (edge) and m (neighbour edge)
      . if subf(neighbour) == 3
      . . pair T[n][0] with N[(m+1)%3][2]
      . . pair T[(n+1)%3][2] with N[m][0]
     */
    // Set external neighbours
    // Let i represent this triangle's edge (made up of sub triangles i and (i+1)%3)
    // pTriNeighbours[i] is neighbour triangle (and is assumed to be set)

    // Find out if pTriNeighbours[i] is split, if it isn't, leave neighbours at default value
    Mesh::MeshBlock * pTriNeighbourBlock = getTriangleBlock(pTriNeighbours[i]);
    unsigned int iNeighbourIndex;
    if (pTriNeighbourBlock && getTriangleSplitBits(pTriNeighbourBlock, iNeighbourIndex = pTriNeighbours[i] - pTriNeighbourBlock->getStartIndex()) == 0x7) {
      // Get neighbour's neighbours to see which neighbours edge we're connected to
      unsigned int * pTriNeighbourNeighbours = pTriNeighbourBlock->getFieldUint(getIndexTriangleNeighbours()) [iNeighbourIndex];
      unsigned int iNeighbourThisEdge;
      if (pTriNeighbourNeighbours[0] == iTriangleGlobalIndex)
        iNeighbourThisEdge = 0;
      else if (pTriNeighbourNeighbours[1] == iTriangleGlobalIndex)
        iNeighbourThisEdge = 1;
      else
        iNeighbourThisEdge = 2;
      // Get block of neighbour's children so we can link them with our new children
      const Mesh::MeshBlock * pTriNeighbourChildBlock = m_TemporaryTriangleBlocks[pTriNeighbourBlock->getFieldUint(getIndexTriangleSubTriangleBlock()) [iNeighbourIndex] [0]];
      // Assert(pTriNeighbourChildBlock)
      unsigned int iTriNeighbourChildBlockStart = pTriNeighbourChildBlock->getStartIndex();
      // Get neighbour's children's neighbours ready to link
      const Mesh::MeshData<unsigned int> & pTriNeighbourChildNeighbours = pTriNeighbourChildBlock->getFieldUint(getIndexTriangleNeighbours());

      // Split, so the child triangles on this edge have neighbours, yay!
      pChildNeighbours [ i     ] [0] = iTriNeighbourChildBlockStart + (iNeighbourThisEdge+1)%3;
      pChildNeighbours [(i+1)%3] [2] = iTriNeighbourChildBlockStart +  iNeighbourThisEdge;
      pTriNeighbourChildNeighbours[(iNeighbourThisEdge+1)%3] [2] = iChildBlockStart +  i;
      pTriNeighbourChildNeighbours[ iNeighbourThisEdge     ] [0] = iChildBlockStart + (i+1)%3;
    }

    // Set parent
    pChildParents [i][0] = iTriangleGlobalIndex;
  }
  // Set middle parent
  pChildParents [3][0] = iTriangleGlobalIndex;


  /// @todo method of setting any extra triangle data
  calculatePlanes(pChildBlock);

}

/// Remerges a subdivided triangle.
/**
 * Removes a triangle's subtriangles.
 * Removes any 2-way neighbour references.
 * @param pTriangle Pointer to the triangle block containing the triangle which isn't subdivided but has a split factor of 3.
 * @param iTriangleIndex Local index of the triangle.
 * @pre Triangle must be fully split with children which are fully joined.
 * @post At least one of the bits of the edgesplit should be set to 0.
 */
void mesh::AdaptiveMesh::remergeTriangle(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex)
{
  ///@todo Cleanup custom fields correctly (virtual functions or whatever)
  assert(getTriangleSplitBits(pTriangle, iTriangleIndex) == 0x7);
  /*
  remove external neighbour links
  reset data
  put onto spare triangle blocks
  remove from temp triangle blocks
  */
  // Cleanup external neighbours
  unsigned int iChildBlock = pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0];
  Mesh::MeshBlock * pChildBlock = m_TemporaryTriangleBlocks[iChildBlock];

  // Sort out external neighbours
  // Don't worry about internal neighbours (as they're all getting cleaned up anyway)
  Mesh::MeshData<unsigned int> & rChildNeighbours = pChildBlock->getFieldUint(getIndexTriangleNeighbours());
  unsigned int iTriCounter;
  for (iTriCounter = 0; iTriCounter < 3; ++iTriCounter) {
    assert(getTriangleSplitBits(pChildBlock, iTriCounter) == 0x0);
    // Only test exteria edges
    unsigned int iGlobalChildIndex = pChildBlock->getStartIndex()+iTriCounter;
    for (int iEdge = 0; iEdge<3; iEdge+=2) {
      unsigned int iGlobalNeighbourIndex = rChildNeighbours[iTriCounter][iEdge];
      if (MESH_INVALID_TRIANGLE != iGlobalNeighbourIndex) {
        Mesh::MeshBlock * pNeighbourBlock = getTriangleBlock(iGlobalNeighbourIndex);
        // Get local index within block
        unsigned int iNeighbourIndex = iGlobalNeighbourIndex - pNeighbourBlock->getStartIndex();
        unsigned int * pNeighbourNeighbours = pNeighbourBlock->getFieldUint(getIndexTriangleNeighbours()) [iNeighbourIndex];
        if (pNeighbourNeighbours[0] == iGlobalChildIndex) {
          pNeighbourNeighbours[0] = MESH_INVALID_TRIANGLE;
        } else if (pNeighbourNeighbours[1] == iGlobalChildIndex) {
          pNeighbourNeighbours[1] = MESH_INVALID_TRIANGLE;
        } else {
          assert(pNeighbourNeighbours[2] == iGlobalChildIndex);
          pNeighbourNeighbours[2] = MESH_INVALID_TRIANGLE;
        }
        rChildNeighbours[iTriCounter][iEdge] = MESH_INVALID_TRIANGLE;
      }
    }
  }

  // Reset child block number and remove block
  pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0] = MESH_INVALID_TRIBLOCK;
  removeTriangleBlock(iChildBlock);
}

/// Get the number of split edges in a triangle.
/**
 * @param pTriangle Pointer to the triangle block in which a triangle belongs.
 * @param iTriangleIndex The local index of a triangle in @a pTriangle.
 * @return The number of split edges in the triangle.
 */
unsigned char mesh::AdaptiveMesh::getTriangleNumSplitEdges(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex) const
{
  unsigned char uTriEdgeSplit = (unsigned char) *pTriangle->getFieldUint(getIndexTriangleSplitState())[iTriangleIndex];
  // Whether edge i is split depends on bit number i in the unsigned integer
  unsigned char ret;
  if (uTriEdgeSplit & 0x1u) ret = 1; else ret = 0;
  if (uTriEdgeSplit & 0x2u) ++ret;
  if (uTriEdgeSplit & 0x4u) ++ret;
  return ret;
}

/// Find whether a specific edge of a triangle is split.
/**
 * @param pTriangle Pointer to the triangle block in which a triangle connected to the edge belongs.
 * @param iTriangleIndex The local index of a triangle in @a pTriangle in which the edge is used.
 * @param iEdgeIndex The index of the edge within the triangle.
 * @return Whether the edge indexed @a iEdgeIndex of the triangle indexed by @a iTriangleIndex in @a pTriangle is split.
 */
bool mesh::AdaptiveMesh::isEdgeSplit(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex) const
{
  unsigned int uTriEdgeSplit = *pTriangle->getFieldUint(getIndexTriangleSplitState())[iTriangleIndex];
  // Whether edge iEdgeIndex is split depends on bit number iEdgeIndex in the unsigned integer
  return (uTriEdgeSplit & (0x1u << iEdgeIndex)) != 0x0u;
}

/// Get the split edges bits for a triangle.
/**
 * @param pTriangle Pointer to the triangle block in which a triangle belongs.
 * @param iTriangleIndex The local index of a triangle in @a pTriangle.
 * @return The split edge bitfield for the triangle.
 */
unsigned char mesh::AdaptiveMesh::getTriangleSplitBits(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex) const
{
  return (unsigned char)*pTriangle->getFieldUint(getIndexTriangleSplitState())[iTriangleIndex];
}

/// Get the block for a certain triangle id.
/**
 * @param iTriangleIndex Mesh wide triangle index.
 * @return Pointer to MeshBlock containing triangle number @a iTriangleIndex or 0 if the triangle does not exist.
 */
const mesh::Mesh::MeshBlock * mesh::AdaptiveMesh::getTriangleBlock(unsigned int iTriangleIndex) const
{
  if (iTriangleIndex < getNumPermanentTriangles()) {
    // Permanent triangle
    return &m_PermanentTriangles;
  } else {
    unsigned int iBlockNum = (iTriangleIndex - getNumPermanentTriangles()) / ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE;
    if (iBlockNum < m_TemporaryTriangleBlocks.size()) {
      // Existing temporary triangle
      return m_TemporaryTriangleBlocks[iBlockNum];
    } else {
      // Non existing temporary triangle
      return 0;
    }
  }
}
/// Get the block for a certain triangle id.
/**
 * @param iTriangleIndex Mesh wide triangle index.
 * @return Pointer to MeshBlock containing triangle number @a iTriangleIndex or 0 if the triangle does not exist.
 */
mesh::Mesh::MeshBlock * mesh::AdaptiveMesh::getTriangleBlock(unsigned int iTriangleIndex)
{
  if (iTriangleIndex < getNumPermanentTriangles()) {
    // Permanent triangle
    return &m_PermanentTriangles;
  } else {
    unsigned int iBlockNum = (iTriangleIndex - getNumPermanentTriangles()) / ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE;
    if (iBlockNum < m_TemporaryTriangleBlocks.size()) {
      // Existing temporary triangle
      return m_TemporaryTriangleBlocks[iBlockNum];
    } else {
      // Non existing temporary triangle
      return NULL;
    }
  }
}

/// Get the block for a certain vertex id.
/**
 * @param iVertexIndex Mesh wide vertex index.
 * @return Pointer to MeshBlock containing vertex number @a iVertexIndex or 0 if the vertex does not exist.
 */
const mesh::Mesh::MeshBlock * mesh::AdaptiveMesh::getVertexBlock(unsigned int iVertexIndex) const
{
  if (iVertexIndex < getNumPermanentVertices()) {
    // Permanent vertex
    return &m_PermanentVertices;
  } else {
    unsigned int iBlockNum = (iVertexIndex - getNumPermanentVertices()) / ADAPTIVE_MESH_VERTEX_BLOCK_SIZE;
    if (iBlockNum < m_TemporaryVertexBlocks.size()) {
      // Existing temporary vertex
      if (m_TemporaryVertexBlocks[iBlockNum] && m_TemporaryVertexBlocks[iBlockNum]->getItemUsage(iVertexIndex - m_TemporaryVertexBlocks[iBlockNum]->getStartIndex())) {
        return m_TemporaryVertexBlocks[iBlockNum];
      } else {
        return NULL;
      }
    } else {
      // Non existing temporary vertex
      return NULL;
    }
  }
}
/// Get the block for a certain vertex id.
/**
 * @param iVertexIndex Mesh wide vertex index.
 * @return Pointer to MeshBlock containing vertex number @a iVertexIndex or 0 if the vertex does not exist.
 */
mesh::Mesh::MeshBlock * mesh::AdaptiveMesh::getVertexBlock(unsigned int iVertexIndex)
{
  if (iVertexIndex < getNumPermanentVertices()) {
    // Permanent vertex
    return &m_PermanentVertices;
  } else {
    unsigned int iBlockNum = (iVertexIndex - getNumPermanentVertices()) / ADAPTIVE_MESH_VERTEX_BLOCK_SIZE;
    if (iBlockNum < m_TemporaryVertexBlocks.size()) {
      // Existing temporary vertex
      if (m_TemporaryVertexBlocks[iBlockNum] && m_TemporaryVertexBlocks[iBlockNum]->getItemUsage(iVertexIndex - m_TemporaryVertexBlocks[iBlockNum]->getStartIndex())) {
        return m_TemporaryVertexBlocks[iBlockNum];
      } else {
        return NULL;
      }
    } else {
      // Non existing temporary vertex
      return NULL;
    }
  }
}

// Get a new block of triangles.
/**
 * @param pOutBlock Pointer to (block *) which will be set to the new block.
 * @return Block id!
 */
unsigned int mesh::AdaptiveMesh::getNewTriangleBlock(Mesh::MeshBlock ** pOutBlock)
{
  // Go through triangle blocks looking for gaps
  MeshBlock * pBlock;
  if (!m_TemporaryTriangleBlockGaps.empty()) {
    // get the id of the first blank block
    std::set<unsigned int>::iterator it = m_TemporaryTriangleBlockGaps.begin();
    unsigned int blankBlockId = *it;
    assert(!m_TemporaryTriangleBlocks[blankBlockId] && "non blank triangle block shouldn't be in blank triangle block set");
    // remove it from set of blanks
    m_TemporaryTriangleBlockGaps.erase(it);

    // create new block or get from spare queue
    if (!m_SpareTriangleBlocks.empty()) {
        // Get new block from spare block queue
      pBlock = m_SpareTriangleBlocks.front();
      m_SpareTriangleBlocks.pop_front();
      pBlock->setStartIndex(getNumPermanentTriangles() + ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE*blankBlockId);
    } else {
        // Create new block from permanent triangle block
      pBlock = new MeshBlock (m_PermanentTriangles, getNumPermanentTriangles() + ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE*blankBlockId);
    }
    pBlock->init(ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE);
    *pOutBlock = pBlock;
    m_TemporaryTriangleBlocks[blankBlockId] = pBlock;
    return blankBlockId;
  }

  // Create new block or get from spare queue and put at end
  unsigned int iBlockId = m_TemporaryTriangleBlocks.size();
  if (!m_SpareTriangleBlocks.empty()) {
    // Get new block from spare block queue
    pBlock = m_SpareTriangleBlocks.front();
    m_SpareTriangleBlocks.pop_front();
    pBlock->setStartIndex(getNumPermanentTriangles() + ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE*iBlockId);
  } else {
    // Create new block from permanent triangle block
    pBlock = new MeshBlock (m_PermanentTriangles, getNumPermanentTriangles() + ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE*iBlockId);
  }
  pBlock->init(ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE);
  *pOutBlock = pBlock;
  m_TemporaryTriangleBlocks.push_back(pBlock);
  return iBlockId;
}

/// Get a new vertex and make block available.
/**
 * @param pOutBlock Pointer to (block *) which will be set to the block that the new vertex belongs to.
 * @return Global vertex index of new vertex.
 */
unsigned int mesh::AdaptiveMesh::getNewVertex(Mesh::MeshBlock ** pOutBlock)
{
  /// @todo consider making more efficient (linear searching for gaps!)

  // Go through vertex blocks looking for gaps
  MeshBlockCounted<unsigned char> * pBlock;
  if (!m_TemporaryVertexBlockGaps.empty()) {
    // get the id of the first blank block
    std::set<unsigned int>::iterator it = m_TemporaryVertexBlockGaps.begin();
    unsigned int blankBlockId = *it;
    pBlock = m_TemporaryVertexBlocks[blankBlockId];

    if (!pBlock) {
      // create new block or get from spare queue
      if (!m_SpareVertexBlocks.empty()) {
        // Get new block from spare block queue
        pBlock = m_SpareVertexBlocks.front();
        m_SpareVertexBlocks.pop_front();
        pBlock->setStartIndex(getNumPermanentVertices() + ADAPTIVE_MESH_VERTEX_BLOCK_SIZE*blankBlockId);
      } else {
        // Create new block from permanent vertex block
        pBlock = new MeshBlockCounted<unsigned char> (m_PermanentVertices, getNumPermanentVertices() + ADAPTIVE_MESH_VERTEX_BLOCK_SIZE*blankBlockId);
      }
      pBlock->init(ADAPTIVE_MESH_VERTEX_BLOCK_SIZE);
      pBlock->setItemUsage(0,true);
      *pOutBlock = pBlock;
      m_TemporaryVertexBlocks[blankBlockId] = pBlock;
      return pBlock->getStartIndex();

    } else {
      assert(pBlock->getNumInUse() < ADAPTIVE_MESH_VERTEX_BLOCK_SIZE && "Full vertex block shouldn't be in blank vertex block set");
      // use one of the slots in this block
      for (int j = 0; j < ADAPTIVE_MESH_VERTEX_BLOCK_SIZE; ++j) {
        if (!pBlock->getItemUsage(j)) {
          pBlock->setItemUsage(j,true);
          *pOutBlock = pBlock;
          if (pBlock->getNumInUse() >= ADAPTIVE_MESH_VERTEX_BLOCK_SIZE) {
            // remove it from set of blanks if the block is now full
            m_TemporaryVertexBlockGaps.erase(it);
          }
          return pBlock->getStartIndex()+j;
        }
      }
    }
  }

  // Create new block or get from spare queue and put at end
  if (!m_SpareVertexBlocks.empty()) {
    // Get new block from spare block queue
    pBlock = m_SpareVertexBlocks.front();
    m_SpareVertexBlocks.pop_front();
    pBlock->setStartIndex(getNumPermanentVertices() + ADAPTIVE_MESH_VERTEX_BLOCK_SIZE*m_TemporaryVertexBlocks.size());
  } else {
    // Create new block from permanent vertex block
    pBlock = new MeshBlockCounted<unsigned char> (m_PermanentVertices, getNumPermanentVertices() + ADAPTIVE_MESH_VERTEX_BLOCK_SIZE*m_TemporaryVertexBlocks.size());
  }
  pBlock->init(ADAPTIVE_MESH_VERTEX_BLOCK_SIZE);
  pBlock->setItemUsage(0,true);
  *pOutBlock = pBlock;
  m_TemporaryVertexBlocks.push_back(pBlock);
  return pBlock->getStartIndex();
}

/// Remove a triangle block from exisitance
/**
 * @param iBlockIndex Index of the triangle block in the temporary block array.
 * @pre Block must have been reference seperated.
 */
void mesh::AdaptiveMesh::removeTriangleBlock(unsigned int iBlockIndex)
{
  // Move block onto spare triangle block stack
  m_SpareTriangleBlocks.push_front(m_TemporaryTriangleBlocks[iBlockIndex]);
  m_TemporaryTriangleBlocks[iBlockIndex] = NULL;
  m_TemporaryTriangleBlockGaps.insert(iBlockIndex);
}

/// Remove a vertex from exisitance
/**
 * @param iVertexIndex Index of the vertex (must be an existing temporary vertex).
 * @pre Vertex must have been reference seperated.
 */
void mesh::AdaptiveMesh::removeVertex(unsigned int iVertexIndex)
{
  /*
  find block
  remove reference to vertex in block
  if no vertices in use in block
    move block onto spares stack
    set pointer to 0x0
  */
  unsigned int iBlockNum = (iVertexIndex - getNumPermanentVertices()) / ADAPTIVE_MESH_VERTEX_BLOCK_SIZE;
  MeshBlockCounted<unsigned char> * pBlock = m_TemporaryVertexBlocks[iBlockNum];
  if (pBlock->getNumInUse() >= ADAPTIVE_MESH_VERTEX_BLOCK_SIZE) {
    m_TemporaryVertexBlockGaps.insert(iBlockNum);
  }
  pBlock->setItemUsage(iVertexIndex - pBlock->getStartIndex(), false);
  if (0 == pBlock->getNumInUse()) {
    m_TemporaryVertexBlocks[iBlockNum] = NULL;
    m_SpareVertexBlocks.push_front(pBlock);
  }
}

/// Get the upper bound of the number of triangles.
/**
 * @return The upper bound of the number of triangles.
 */
unsigned int mesh::AdaptiveMesh::getUpperBoundNumTriangles() const
{
  return m_PermanentTriangles.getNumItems() + m_TemporaryTriangleBlocks.size()*ADAPTIVE_MESH_TRIANGLE_BLOCK_SIZE;
}

/// Get the upper bound of the number of vertices.
/**
 * @return The upper bound of the number of vertices.
 */
unsigned int mesh::AdaptiveMesh::getUpperBoundNumVertices() const
{
  return m_PermanentVertices.getNumItems() + m_TemporaryVertexBlocks.size()*ADAPTIVE_MESH_VERTEX_BLOCK_SIZE;
}

/// Render the mesh to OpenGL.
/**
 * @note How about having some sort of iterator for the triangles
 */
void mesh::AdaptiveMesh::renderGL(const maths::Observer<float> * observer) const
{
  //Mesh::renderGL(); return;
  /// @note Renders sub triangles as well as permanent triangles.
#define RENDER_TRIANGLES() \
  glBegin(GL_TRIANGLES); \
  {                      \
    for (unsigned int trid = 0; trid < getNumPermanentTriangles(); ++trid) { \
      renderTriangle(&m_PermanentTriangles, trid, observer); \
    } \
  } \
  glEnd();

  // Draw triangles
  RENDER_TRIANGLES();

  // Draw all normals
#ifdef ADAPTIVE_MESH_DRAW_NORMALS
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glBegin(GL_LINES);
  {
    for (int j = 0; j < m_PermanentVertices.getNumItems(); ++j) {
      maths::Vector<3, float> * vert = (maths::Vector<3, float>*)m_PermanentVertices.getFieldFloat( getIndexVertexPosition() )[j];
      maths::Vector<3, float> * norm = (maths::Vector<3, float>*)m_PermanentVertices.getFieldFloat( getVertexFieldFloatIndex(MESH_DATA_NORMAL) )[j];
      maths::Vector<3, float> vert_norm = *vert + (*norm)*ADAPTIVE_MESH_DRAW_NORMALS;
      glColor3f(1,0,0);
      glVertex3fv((float*)vert);
      glColor3f(1,1,0);
      glVertex3fv((float*)vert_norm);
    }
  }
  for (int i = 0; i < m_TemporaryVertexBlocks.size(); ++i) {
    const MeshBlockCounted<unsigned char> * pBlock = m_TemporaryVertexBlocks[i];
    if (pBlock) {
      for (int j = 0; j < ADAPTIVE_MESH_VERTEX_BLOCK_SIZE; ++j) {
        if (pBlock->getItemUsage(j)) {
          maths::Vector<3, float> * vert = (maths::Vector<3, float>*)pBlock->getFieldFloat( getIndexVertexPosition() )[j];
          maths::Vector<3, float> * norm = (maths::Vector<3, float>*)pBlock->getFieldFloat( getVertexFieldFloatIndex(MESH_DATA_NORMAL) )[j];
          maths::Vector<3, float> vert_norm = *vert + (*norm)*ADAPTIVE_MESH_DRAW_NORMALS;
          glColor3f(1,0,0);
          glVertex3fv((float*)vert);
          glColor3f(1,1,0);
          glVertex3fv((float*)vert_norm);
        }
      }
    }
  }
  glEnd();
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
#endif

  /*/ Draw all points
  glColor3f(1,1,1);
  glBegin(GL_POINTS);
  for (int i = 0; i < m_TemporaryVertexBlocks.size(); ++i) {
    const MeshBlockCounted<unsigned char> * pBlock = m_TemporaryVertexBlocks[i];
    if (pBlock) {
      for (int j = 0; j < ADAPTIVE_MESH_VERTEX_BLOCK_SIZE; ++j) {
        if (pBlock->getItemUsage(j)) {
          glVertex3fv(pBlock->getFieldFloat( getIndexVertexPosition() )[j]);
        }
      }
    }
  }
  glEnd();

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glBegin(GL_LINES);
  {
      for (int trid = 0; trid < getNumPermanentTriangles(); ++trid) {
        renderTriangleLinkage(&m_PermanentTriangles, trid);
      }
  }
  glEnd();
  glEnable(GL_DEPTH_TEST);
  //*/
}

/// Find whether a triangle is visible
/**
 * @param pTriangle Block to which the triangle belongs.
 * @param iTriangleIndex Index of triangle local to @a pTriangle block.
 */
bool mesh::AdaptiveMesh::triangleVisibility(const Mesh::MeshBlock * pTriangle,
                                            unsigned int iTriangleIndex,
                                            const maths::Observer<float> & observer) const
{
  // Triangle vertex id -> global vertex id
  int iNormalField = getVertexFieldFloatIndex(MESH_DATA_NORMAL);
  const unsigned int * pTriVerts = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex];

  const maths::Vector<3, float> * pVertPos[3];
  const maths::Vector<3, float> * pVertNormal[3];

  int i;
  for (i = 0; i < 3; ++i) {
    assert(pTriVerts[i] != MESH_INVALID_VERTEX && "Error, Fully split triangle has incorrect vertices");
    const Mesh::MeshBlock * pBlock = getVertexBlock(pTriVerts[i]);
    assert(pBlock && "Error, Fully split triangle has missing vertices");
    pVertPos[i] = (const maths::Vector<3, float> *)pBlock->getFieldFloat(getIndexVertexPosition())[pTriVerts[i]-pBlock->getStartIndex()];
    if (iNormalField >= 0) {
      pVertNormal[i] = (const maths::Vector<3, float> *)pBlock->getFieldFloat(iNormalField)[pTriVerts[i]-pBlock->getStartIndex()];
    } else {
      return true;
    }
  }

  maths::Vector<3, float> tangents[2];
  maths::Vector<3, float> realNormal;
  maths::sub(tangents[0], *pVertPos[1], *pVertPos[0]);
  maths::sub(tangents[1], *pVertPos[2], *pVertPos[0]);
  maths::cross(realNormal, tangents[0], tangents[1]);

  return (observer.frontFacing(*pVertPos[0], *pVertNormal[0]) ||
          observer.frontFacing(*pVertPos[1], *pVertNormal[1]) ||
          observer.frontFacing(*pVertPos[2], *pVertNormal[2]) ||
          observer.frontFacing(*pVertPos[0], realNormal) ||
          observer.frontFacing(*pVertPos[1], realNormal) ||
          observer.frontFacing(*pVertPos[2], realNormal));
}

/// Render a single triangle.
/**
 * @param pTriangle Block to which the triangle belongs.
 * @param iTriangleIndex Index of triangle local to @a pTriangle block.
 */
void mesh::AdaptiveMesh::renderTriangle(const Mesh::MeshBlock * pTriangle,
                                        unsigned int iTriangleIndex,
                                        const maths::Observer<float> * observer) const
{
#define SCATTERING_GL_MODE      GL_EMISSION

  // Optimise, if back facing, don't send to GL
  if (observer && !triangleVisibility(pTriangle, iTriangleIndex, *observer)) {
    return;
  }

  //const float colours[][3] = { {1.0f,0.0f,0.0f}, {1.0f,1.0f,0.0f}, {0.0f,0.0f,1.0f} };
  //const float colours[][3] = { {0.0f,0.5f,0.0f}, {0.0f,0.6f,0.0f}, {0.0f,0.4f,0.0f} };

  int iDiffuseField = getIndexVertexDiffuse();//getVertexFieldFloatIndex(MESH_DATA_DIFFUSE);
  int iNormalField = getIndexVertexNormal();//getVertexFieldFloatIndex(MESH_DATA_NORMAL);
  int iTexCoordField = getVertexFieldFloatIndex(MESH_DATA_TEXCOORD);
  int iScatteringField = getVertexFieldFloatIndex(MESH_DATA_SCATTERING);

//   iDiffuseField = iNormalField = iTexCoordField = -1;

  float zero[] = { 0.0f, 0.0f, 0.0f, 1.0f };
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, zero);

  // Get the split bits which indicate how subdivided the triangle is
  unsigned char splits = getTriangleSplitBits(pTriangle, iTriangleIndex);

  // Triangle vertex id -> global vertex id
  const unsigned int * pTriVerts = pTriangle->getFieldUint(getIndexTriangleVerts()) [iTriangleIndex];

  if (splits >= 0x7) {
    // Fully subdivided so we need to render the separate sub-triangles.
    const unsigned int iSubTriBlock = pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0];
    const Mesh::MeshBlock * pBlock = NULL;
    if (iSubTriBlock != MESH_INVALID_TRIBLOCK) {
      pBlock = m_TemporaryTriangleBlocks[iSubTriBlock];
    }
    if (pBlock) {
      assert(pBlock != pTriangle && "Triangle's child triangle block is its own triangle block!");
      for (unsigned char i = 0; i < 4; ++i) {
        renderTriangle(pBlock, i, observer);
      }
    } else {
      // A triangle is fully split but has no children, this should never happen
      /// @todo Raise an exception when an error occurs
      //std::cerr << "Error 3, Fully split triangle has missing children" << std::endl;

      const float * pVertPos[6];
      const float * pVertNormal[6];
      const float * pVertDiffuse[6];
      const float * pVertTexCoord[6];
      const float * pScattering[6];
      int i;
      for (i = 0; i < 6; ++i) {
        assert(pTriVerts[i] != MESH_INVALID_VERTEX && "Error, Fully split triangle has incorrect vertices");
        const Mesh::MeshBlock * pBlock = getVertexBlock(pTriVerts[i]);
        assert(pBlock && "Error, Fully split triangle has missing vertices");
        pVertPos[i] = pBlock->getFieldFloat(getIndexVertexPosition())[pTriVerts[i]-pBlock->getStartIndex()];
        if (iDiffuseField >= 0) {
          pVertDiffuse[i] = pBlock->getFieldFloat(iDiffuseField)[pTriVerts[i]-pBlock->getStartIndex()];
        }
        if (iNormalField >= 0) {
          pVertNormal[i] = pBlock->getFieldFloat(iNormalField)[pTriVerts[i]-pBlock->getStartIndex()];
        }
        if (iTexCoordField >= 0) {
          pVertTexCoord[i] = pBlock->getFieldFloat(iTexCoordField)[pTriVerts[i]-pBlock->getStartIndex()];
        }
        if (iScatteringField >= 0) {
          pScattering[i] = pBlock->getFieldFloat(iScatteringField)[pTriVerts[i]-pBlock->getStartIndex()];
        }
      }

      for (i = 0; i < 3; ++i) {
        /*glVertex3fv(pVertPos[i]);
        glVertex3fv(pVertPos[3+(i)%3]);
        glVertex3fv(pVertPos[3+(i+2)%3]);*/

        //glColor3fv(colours[0]);
        if (iDiffuseField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, pVertDiffuse[i]);
        }
        if (iScatteringField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, SCATTERING_GL_MODE, pScattering[i]);
        }
        if (iNormalField >= 0) {
          glNormal3fv(pVertNormal[i]);
        }
        if (iTexCoordField >= 0) {
          glTexCoord2fv(pVertTexCoord[i]);
        }
        glVertex3fv(pVertPos[i]);
        //glColor3fv(colours[1]);
        if (iDiffuseField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, pVertDiffuse[3+(i)%3]);
        }
        if (iScatteringField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, SCATTERING_GL_MODE, pScattering[3+(i)%3]);
        }
        if (iNormalField >= 0) {
          glNormal3fv(pVertNormal[3+(i)%3]);
        }
        if (iTexCoordField >= 0) {
          glTexCoord2fv(pVertTexCoord[3+(i)%3]);
        }
        glVertex3fv(pVertPos[3+(i)%3]);
        //glColor3fv(colours[2]);
        if (iDiffuseField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, pVertDiffuse[3+(i+2)%3]);
        }
        if (iScatteringField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, SCATTERING_GL_MODE, pScattering[3+(i+2)%3]);
        }
        if (iNormalField >= 0) {
          glNormal3fv(pVertNormal[3+(i+2)%3]);
        }
        if (iTexCoordField >= 0) {
          glTexCoord2fv(pVertTexCoord[3+(i+2)%3]);
        }
        glVertex3fv(pVertPos[3+(i+2)%3]);
      }

      for (i = 3; i < 6; ++i) {
        //glColor3fv(colours[i-3]);
        if (iDiffuseField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, pVertDiffuse[i]);
        }
        if (iScatteringField >= 0) {
          glMaterialfv(GL_FRONT_AND_BACK, SCATTERING_GL_MODE, pScattering[i]);
        }
        if (iNormalField >= 0) {
          glNormal3fv(pVertNormal[i]);
        }
        if (iTexCoordField >= 0) {
          glTexCoord2fv(pVertTexCoord[i]);
        }
        glVertex3fv(pVertPos[i]);
      }
    }
  } else {


    // Not fully subdivided so use s_aiSplitTriVerts to decide which verts to use
    // Render as a triangle fan using the indicies in s_aiSplitTriVerts[splits]
    const char * verts = s_aiSplitTriVerts[splits];
    // global vertex ids
    unsigned int vert_ids[3] =
    {
      pTriVerts[(unsigned char)*verts],
      0,
      pTriVerts[(unsigned char)*(++verts)]
    };
    // Vertex blocks
    const Mesh::MeshBlock * pVertexBlocks[3] =
    {
      getVertexBlock(vert_ids[0]),
      0,
      getVertexBlock(vert_ids[2])
    };

    if (pVertexBlocks[0]) {
      while (*(++verts) >= 0) {
        // Get the next vertex id and the associated block
        vert_ids[1] = vert_ids[2];
        vert_ids[2] = pTriVerts[(unsigned char)*verts];
        pVertexBlocks[1] = pVertexBlocks[2];
        pVertexBlocks[2] = getVertexBlock(vert_ids[2]);
        if (pVertexBlocks[1] && pVertexBlocks[2]) {
          for (unsigned char i = 0; i < 3; ++i) {
            //glColor3fv(colours[i]);
            if (iDiffuseField >= 0) {
              glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, pVertexBlocks[i]->getFieldFloat(iDiffuseField)[vert_ids[i]-pVertexBlocks[i]->getStartIndex()]);
            }
            if (iScatteringField >= 1) {
              const GLfloat *p = pVertexBlocks[i]->getFieldFloat(iScatteringField)[vert_ids[i]-pVertexBlocks[i]->getStartIndex()];
              if (!p) {
                asm volatile("int3");
              }
              glMaterialfv(GL_FRONT_AND_BACK, SCATTERING_GL_MODE, p);
            }
            if (iNormalField >= 0) {
              glNormal3fv(pVertexBlocks[i]->getFieldFloat(iNormalField)[vert_ids[i]-pVertexBlocks[i]->getStartIndex()]);
            }
            if (iTexCoordField >= 0) {
              glTexCoord2fv(pVertexBlocks[i]->getFieldFloat(iTexCoordField)[vert_ids[i]-pVertexBlocks[i]->getStartIndex()]);
            }
            glVertex3fv(pVertexBlocks[i]->getFieldFloat(getIndexVertexPosition())[vert_ids[i]-pVertexBlocks[i]->getStartIndex()]);
          }
        } else {
          std::cerr << "Error 1, bad vertex linkage" << std::endl;
          std::cerr << "info: vert_ids[]={" << vert_ids[0] << "," << vert_ids[1] << "," << vert_ids[2] << "}" << std::endl;
          assert(false && "Error 1, bad vertex linkage");
        }
      }
    } else {
      assert(false && "Error 2, bad vertex linkage");
    }

  }
}

/// Render a a triangle's linkage.
/**
 * @param pTriangle Block to which the triangle belongs.
 * @param iTriangleIndex Index of triangle local to @a pTriangle block.
 */
void mesh::AdaptiveMesh::renderTriangleLinkage(const Mesh::MeshBlock * pTriangle,
                                               unsigned int iTriangleIndex) const
{
  // Get the split bits which indicate how subdivided the triangle is
  const unsigned char splits = getTriangleSplitBits(pTriangle, iTriangleIndex);
  if (splits >= 0x7) {
    // Fully subdivided so we need to render the seperate sub-triangles.
    const unsigned int iSubTriBlock = pTriangle->getFieldUint(getIndexTriangleSubTriangleBlock()) [iTriangleIndex] [0];
    const Mesh::MeshBlock * pBlock = 0;
    if (iSubTriBlock != MESH_INVALID_TRIBLOCK) {
      pBlock = m_TemporaryTriangleBlocks[iSubTriBlock];
    }
    if (pBlock) {
      for (unsigned char i = 0; i < 4; ++i) {
        renderTriangleLinkage(pBlock, i);
      }
    }
  } else {

    // Draw lines from edges to neighbour triangles
    unsigned int * pTriangleNeighbours = pTriangle->getFieldUint(getIndexTriangleNeighbours())[iTriangleIndex];
    // Keep hold of this triangles vertices
    unsigned int * pTriangleVerts = pTriangle->getFieldUint(getIndexTriangleVerts())[iTriangleIndex];
    maths::Vector<3,float> vVertexPositions[3];
    int iVert;
    for (iVert = 0; iVert < 3; ++iVert) {
      const Mesh::MeshBlock * pVertexBlock = getVertexBlock(pTriangleVerts[iVert]);
      vVertexPositions[iVert] = maths::Vector<3,float>(pVertexBlock->getFieldFloat(getIndexVertexPosition())[pTriangleVerts[iVert] - pVertexBlock->getStartIndex()]);
    }
    int iEdge;
    for (iEdge = 0; iEdge < 3; ++iEdge) {
      if (pTriangleNeighbours[iEdge] != MESH_INVALID_TRIANGLE) {
        // Get block of neighbour
        const Mesh::MeshBlock * pNeighbourBlock = getTriangleBlock(pTriangleNeighbours[iEdge]);
        // Calculate neighbour's vertices
        unsigned int * pNeighbourVerts = pNeighbourBlock->getFieldUint(getIndexTriangleVerts())[pTriangleNeighbours[iEdge] - pNeighbourBlock->getStartIndex()];
        maths::Vector<3,float> vNeighbourVertexPositions[3];
        for (iVert = 0; iVert < 3; ++iVert) {
          const Mesh::MeshBlock * pVertexBlock = getVertexBlock(pNeighbourVerts[iVert]);
          vNeighbourVertexPositions[iVert] = maths::Vector<3,float>(pVertexBlock->getFieldFloat(getIndexVertexPosition())[pNeighbourVerts[iVert] - pVertexBlock->getStartIndex()]);
        }
        // And neighbour's midpoint
        maths::Vector<3,float> vNeighbourMid = vNeighbourVertexPositions[0];
        vNeighbourMid += vNeighbourVertexPositions[1];
        vNeighbourMid += vNeighbourVertexPositions[2];
        vNeighbourMid /= 3.0f;

        // Now draw line from this edge to neighbour
        maths::Vector<3,float> vEdgeMid = vVertexPositions[iEdge];
        vEdgeMid += vVertexPositions[(iEdge+1)%3];
        vEdgeMid *= 4.0f;
        vEdgeMid += vVertexPositions[(iEdge+2)%3];
        vEdgeMid /= 9.0f;

        glColor3f(0,1,1);
        glVertex3fv(vEdgeMid);
        glColor3f(1,0,0);
        glVertex3fv(vNeighbourMid);
      }
    }

  }
}

/// Perform a vertex operation on all vertices.
void mesh::AdaptiveMesh::foreachVertex(Mesh::VertexOperation op, void * data)
{
  Mesh::foreachVertex(op, data);
  std::vector<MeshBlockCounted<unsigned char> *>::iterator it;
  for (it = m_TemporaryVertexBlocks.begin(); it != m_TemporaryVertexBlocks.end(); ++it) {
    MeshBlockCounted<unsigned char> * block = *it;
    if (block) {
      for (unsigned int i = 0; i < block->getNumItems(); ++i) {
        if (block->getItemUsage(i)) {
          op(data, block, i);
        }
      }
    }
  }
}
