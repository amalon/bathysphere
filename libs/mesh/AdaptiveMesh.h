/*
 * maths/AdaptiveMesh.h
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

#ifndef _MESH_ADAPTIVEMESH_H_
#define _MESH_ADAPTIVEMESH_H_

#include "mesh/Mesh.h"

#include "maths/Vector.h"
#include "maths/Observer.h"

#include <stdexcept>
#include <vector>
#include <list>
#include <set>

namespace mesh
{

/// General adaptive mesh
/**
 * Extends @a Mesh by allowing triangles to be subdivided.
 * Temporary triangles and Vertices are created & destroyed arbitarily during the lifespan of the object.
 * Triangles and Vertices indexed by id, those below a certain index (the maximum permanent id) are stored in a fixed length array (permanent) and those above the maximum permanent id stored in a more arbitary fashion (in empty slots when available)
 *
 * @todo Implement some interpolation functions for subdivision.
 *       Such as splines
 *
 * @todo Allow mixing of interpolation functions with purterbation directions
 */
class AdaptiveMesh : public Mesh
{
  public:
    AdaptiveMesh(unsigned int nVertexDataFieldsFloat,
                 unsigned int nVertexDataFieldsUint,
                 unsigned int nTriangleDataFieldsFloat,
                 unsigned int nTriangleDataFieldsUint,
                 int iIndexNormals = -1, int iIndexDiffuse = -1,
                 int iIndexPlane = -1, int iIndexEdgePlane = -1, int iIndexMergeMisses = -1);

    virtual ~AdaptiveMesh();

    void adaptToObserver(const maths::Observer<float> & observer, unsigned int * pOperationLimit = 0, unsigned int maxDepth = 10, bool clean = true);


    virtual void renderGL(const maths::Observer<float> * observer = 0) const;

    /// Perform a vertex operation on all vertices.
    virtual void foreachVertex(Mesh::VertexOperation op, void * data = NULL);

  protected:
    // Used by adaptToObserver
    void adaptTriangleToObserver(const maths::Observer<float> & observer,
                                 Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex,
                                 unsigned int * pOperationLimit = 0, bool clean = true,
                                 unsigned int iMaxDepth = 0);

    // Check for anything that could indicate bugs in the code.
    void checkConsistency() const;
    // Check for anything within a triangle that could indicate bugs in the code.
    void checkTriangleConsistency(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex) const;

    // Used by renderGL
    bool triangleVisibility(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, const maths::Observer<float> & observer) const;
    void renderTriangle(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, const maths::Observer<float> * observer = 0) const;
    void renderTriangleLinkage(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex) const;

    virtual void interpolateEdge(maths::Vector<3,float> & pOutput, Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex);
    virtual void simpleInterpolateEdge(maths::Vector<3,float> & pOutput, const maths::Vector<3,float> & p1, const maths::Vector<3,float> & p2);
    virtual void adjustTrianglePriority(const unsigned int * pTriVerts, float & fSplitBoundary, float & fMergeBoundary) {}

    void calculatePlanes(Mesh::MeshBlock * block = NULL);

    // Used by adaptToObserver
    // Highest level to lowest level
    void splitAllEdges(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex);
    void joinAllEdges(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex);

    void splitEdge(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex);
    void joinEdge(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex);

    void subdivideTriangle(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex);
    void remergeTriangle(Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex);

    unsigned char getTriangleNumSplitEdges(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex) const;
    bool isEdgeSplit(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex, unsigned char iEdgeIndex) const;
    unsigned char getTriangleSplitBits(const Mesh::MeshBlock * pTriangle, unsigned int iTriangleIndex) const;

    Mesh::MeshBlock * getTriangleBlock(unsigned int iTriangleIndex);
    const Mesh::MeshBlock * getTriangleBlock(unsigned int iTriangleIndex) const;
    Mesh::MeshBlock * getVertexBlock(unsigned int iVertexIndex);
    const Mesh::MeshBlock * getVertexBlock(unsigned int iVertexIndex) const;

    unsigned int getNewTriangleBlock(Mesh::MeshBlock ** pOutBlock);
    unsigned int getNewVertex(Mesh::MeshBlock ** pOutBlock);

    void removeTriangleBlock(unsigned int iBlockIndex);
    void removeVertex(unsigned int iVertexIndex);


    unsigned int getUpperBoundNumTriangles() const;
    unsigned int getUpperBoundNumVertices() const;




    // NUMBER OF FIELDS IN EACH FIELD TYPE (REQUIRED BY MESH)
    /// Get the index of the first available per vertex float data field.
    /**
     * @return The index of the first available per vertex float data field.
     */
    inline static unsigned int getFirstVertexFloat()
    {
      return Mesh::getFirstVertexFloat();
      // Fields:
    }
    /// Get the index of the first available per vertex unsigned int data field.
    /**
     * @return The index of the first available per vertex unsigned int data field.
     */
    inline static unsigned int getFirstVertexUint()
    {
      return Mesh::getFirstVertexUint();
      // Fields:
    }
    /// Get the index of the first available per triangle float data field.
    /**
     * @return The index of the first available per vertex float data field.
     */
    inline static unsigned int getFirstTriangleFloat()
    {
      return Mesh::getFirstTriangleFloat();
      // Fields:
    }
    /// Get the index of the first available per triangle unsigned int data field.
    /**
     * @return The index of the first available per vertex float data field.
     */
    inline static unsigned int getFirstTriangleUint()
    {
      return Mesh::getFirstTriangleUint() + 4;
      // Fields: sub triangles, split state, neighbour,parent
    }

    // SPECIFIC FIELD IDS
    /// Get the index of the per triangle subtriangle block id field.
    /**
     * @return The index of the per triangle subtriangle block id field.
     */
    inline static unsigned int getIndexTriangleSubTriangleBlock()
    {
      return Mesh::getFirstTriangleUint();
    }
    /// Get the index of the per triangle split status field.
    /**
     * @return The index of the per triangle split status field.
     */
    inline static unsigned int getIndexTriangleSplitState()
    {
      return Mesh::getFirstTriangleUint() + 1;
    }
    /// Get the index of the per triangle neighbour ids field.
    /**
     * @return The index of the per triangle neighbour ids field.
     */
    inline static unsigned int getIndexTriangleNeighbours()
    {
      return Mesh::getFirstTriangleUint() + 2;
    }
    /// Get the index of the per triangle neighbour ids field.
    /**
     * @return The index of the per triangle neighbour ids field.
     */
    inline static unsigned int getIndexTriangleParent()
    {
      return Mesh::getFirstTriangleUint() + 3;
    }

    /// Get the index of the per vertex normals.
    /**
     * @return The index of the per vertex normal field.
     */
    inline unsigned int getIndexVertexNormal() const
    {
      return m_VertexIndexNormal;
    }

    /// Get the index of the per vertex diffuse colour.
    /**
     * @return The index of the per vertex diffuse colour field.
     */
    inline unsigned int getIndexVertexDiffuse() const
    {
      return m_VertexIndexDiffuse;
    }

    /// Get the index of the per triangle planes.
    /**
     * @return The index of the per triangle planes field.
     */
    inline unsigned int getIndexTrianglePlane() const
    {
      return m_TriangleIndexPlanes;
    }

    /// Get the index of the per triangle edge planes.
    /**
     * @return The index of the per triangle edge planes field.
     */
    inline unsigned int getIndexTriangleEdgePlanes() const
    {
      return m_TriangleIndexEdgePlanes;
    }

    /// Get the index of the per triangle merge misses.
    /**
     * @return The index of the per triangle merge misses.
     */
    inline unsigned int getIndexTriangleMergeMisses() const
    {
      return m_TriangleIndexMergeMisses;
    }


  protected:

    /// MeshBlock with additional data to keep track of which items are in use.
    /**
     * @param T Type of unsigned integer to store bits of which items are in use.
     */
    template <typename T>
        class MeshBlockCounted : public Mesh::MeshBlock
    {
      public:
        /// Primary constructor.
        /**
         * @param nFieldsFloat Number of float data fields.
         * @param nFieldsUint Number of unsigned int data fields.
         */
        inline MeshBlockCounted(unsigned int nFieldsFloat,
                                unsigned int nFieldsUint)
        : Mesh::MeshBlock(nFieldsFloat, nFieldsUint),
        m_bUsage(0x0u),
        m_nInUse(0u)
        {
        }

        /// Constructor taking number of fields from another MeshBlock.
        /**
         * @param otherMeshBlock Another MeshBlock to take the field data from.
         * @param iStartIndex Index of first item in block.
         */
        inline MeshBlockCounted(const MeshBlock & otherMeshBlock, unsigned int iStartIndex)
        : Mesh::MeshBlock(otherMeshBlock, iStartIndex),
        m_bUsage(0x0u),
        m_nInUse(0u)
        {
        }

        /// Set the number of items in block.
        /**
         * Ensures that @a nItems items are allocated ready for use.
         * @param nItems Number of items in block (Must be no greater than sizeof(T)*8).
         */
        inline void init(unsigned int nItems)
        {
          if (nItems > sizeof(T)*8) {
            throw std::out_of_range("nItems passed to MeshBlockCounted beyond number of bits in T");
          }
          MeshBlock::init(nItems);
        }

        /// Get the number of items in use.
        /** @return Number of items in use in this block. */
        inline unsigned char getNumInUse() const
        {
          return m_nInUse;
        }
        /// Get whether an item is in use.
        /**
         * @param iLocalIndex Local index of item.
         * @return Whether the item with local index @a iLocalIndex is in use.
         */
        inline bool getItemUsage(unsigned char iLocalIndex) const
        {
          return (m_bUsage & (0x1u << iLocalIndex)) != 0x0u;
        }
        /// Set whether an item is in use.
        /**
         * @param iLocalIndex Local index of item.
         * @param bUsage Whether the item with local index @a iLocalIndex is in use.
         */
        inline void setItemUsage(unsigned char iLocalIndex, bool bUsage)
        {
          bool bInUse = getItemUsage(iLocalIndex);
          // If usage is to change, toggle the bit and update the tally.
          if (bUsage != bInUse) {
            m_bUsage ^= (0x1u << iLocalIndex);
            if (bUsage) {
              ++m_nInUse;
            } else {
              --m_nInUse;
            }
          }
        }

      protected:
        /// Bits indicating which items are in use.
        T m_bUsage;
        unsigned char m_nInUse;
    };

    /**
     * Class used as iterator of the triangles in an adaptable mesh object.
     *
     */
    class TriangleIterator
    {
      public:
        /// Default constructor.
        inline TriangleIterator()
        : m_pMesh(0)
        {}
        /// Construct from mesh and initial triangle index.
        /**
         * @param pMesh Pointer to mesh object to iterate triangles.
         * @param iTriangleIndex Initial triangle index.
         */
        inline TriangleIterator(const AdaptiveMesh * pMesh, int iTriangleIndex)
        : m_pMesh(pMesh)
        {}
        /// Copy constructor.
        /**
         * @param copy Iterator to duplicate.
         */
        inline TriangleIterator(const TriangleIterator & copy)
        : m_pMesh(copy.m_pMesh)
        {}

        // Default destructor (don't extend)

         /// Find whether the iterator is valid.
        /**
         * @return Status code:
         *  - true (the iterator is valid).
         *  - false (the iterator is invalid).
         */
        inline bool isValid() const
        {
          return m_pMesh != 0;
        }

        TriangleIterator & operator = (const TriangleIterator & copy);

        TriangleIterator & operator ++ ();
        TriangleIterator   operator ++ (int);
        TriangleIterator & operator -- ();
        TriangleIterator   operator -- (int);

        // DATA ACCESS

      protected:
        const AdaptiveMesh * m_pMesh; ///< Mesh of triangles to iterate


    };

  protected:
    /// Vector of temporary vertex block pointers (with sub-block usage counting)
    std::vector<MeshBlockCounted<unsigned char> *> m_TemporaryVertexBlocks;
    /// Set of vertex block indices which are blank or have spare verticies.
    std::set<unsigned int>                         m_TemporaryVertexBlockGaps;
    /// Vector of temporary triangle block pointers.
    std::vector<Mesh::MeshBlock *>                 m_TemporaryTriangleBlocks;
    /// Set of triangle block indices which are blank.
    std::set<unsigned int>                         m_TemporaryTriangleBlockGaps;

    /// Stack of spare (unused) vertex blocks
    std::list<MeshBlockCounted<unsigned char> *>  m_SpareVertexBlocks;
    /// Stack of spare (unused) triangle blocks
    std::list<Mesh::MeshBlock *>                  m_SpareTriangleBlocks;

    /// Stack of triangle numbers
    std::vector<unsigned int> m_TriangleAdaptionStack;

    /// List of triangle ids sorted by visibility to observer
    std::list<std::pair<unsigned int,float> > m_SortedPermanentTriangleList;

    /// Vertex normal id.
    int m_VertexIndexNormal;
    /// Vertex diffuse colour id.
    int m_VertexIndexDiffuse;
    /// Triangle planes.
    int m_TriangleIndexPlanes;
    /// Triangle edge planes.
    int m_TriangleIndexEdgePlanes;
    /// Triangle merge misses id.
    int m_TriangleIndexMergeMisses;

    /// Vertex indicies for different split combinations.
    /**
     * Vertex indicies to draw (as fan) for the split combos.
     * First dimention indexed by bitfield of split edges.
     * Second dimention is sequence of local vertex id's drawing fan of triangles, terminated by -1.
     *
     * So an algorithm to draw a triangle T might be
     * @code
     * unsigned char splits = T.getSplitBits();
     * if (splits >= 0x7) {
     *   for (unsigned char i = 0; i < 4; ++i) {
     *     render(T.child[i]);
     *   }
     * } else {
     *   const char * verts = s_aiSplitTriVerts[splits];
     *   char first = *(verts);
     *   char last  = *(++verts);
     *   while (*(++verts) >= 0) {
     *     vertex_id vert_ids[] = { T.verts[first], T.verts[last], T.verts[last = *verts] };
     *     mesh.render_triangle(vert_ids);
     *   }
     * }
     * @endcode
     */
    static const char s_aiSplitTriVerts[7][6];
};

}

#endif // _MESH_ADAPTIVEMESH_H_
