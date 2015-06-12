/*
 * maths/Mesh.h
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
 * TODO per edge data such as: edge roughness
 *
 */

#ifndef _MESH_MESH_H_
#define _MESH_MESH_H_

// Field Types
#define MESH_DATA_OTHER       0

// Float
#define MESH_DATA_POSITION    1

#define MESH_DATA_NORMAL      2
#define MESH_DATA_PLANE       3
#define MESH_DATA_BINORMAL    4
#define MESH_DATA_TANGENT     5

#define MESH_DATA_EDGEPLANES  6

#define MESH_DATA_DIFFUSE     7
#define MESH_DATA_EMMISION    8

#define MESH_DATA_TEXCOORD    9

#define MESH_DATA_LIGHTDIR    10

#define MESH_DATA_TEMPERATURE 11
#define MESH_DATA_ALTITUDE    12
#define MESH_DATA_SCATTERING  13

#define MESH_DATA_FLOAT_MAX   13

// Unsigned int
#define MESH_DATA_NBITS         0x00FF
#define MESH_DATA_XBITS         0xFF00
#define MESH_DATA_XBITS_VERT_ID 0x0100
#define MESH_DATA_XBITS_TRI_ID  0x0200
#define MESH_DATA_XBITS_TRIBLOCK_ID  0x0300

#define MESH_DATA_VERTICES   (1 | MESH_DATA_XBITS_VERT_ID)
#define MESH_DATA_NEIGHBOURS  (2 | MESH_DATA_XBITS_TRI_ID)
#define MESH_DATA_PARENT      (3 | MESH_DATA_XBITS_TRI_ID)

#define MESH_DATA_MATERIAL     4

#define MESH_DATA_SUBTRIBLOCK (5 | MESH_DATA_XBITS_TRIBLOCK_ID)
#define MESH_DATA_SUBTRIS     (6 | MESH_DATA_XBITS_TRI_ID)
#define MESH_DATA_SPLITSTATE   7

#define MESH_DATA_MERGE_MISSES 8

#define MESH_DATA_UINT_MAX     9

#define MESH_INVALID_VERTEX   (~0u)
#define MESH_INVALID_TRIANGLE (~0u)
#define MESH_INVALID_TRIBLOCK (~0u)

namespace mesh
{

/// General mesh.
/**
 * Stores lists of vertex and triangle properties.
 * Each property can refer to either per triangle or per vertex
 * and are indexed by triangle/vertex then by subfield index.
 * @note It is assumed that any edge (pair of Vertices) is used by at most 2 triangles.
 *       To remove this assumption algorithms may need to be modified regarding neighbour triangle data.
 *       (Possibly using the neighbour triangle as a circular linked list)
 */
class Mesh
{
  public:
    class MeshBlock;

    /// Information about the fields of a mesh.
    template <typename T>
        class MeshFieldInformation
    {
      protected:
        /// ID of field type.
        unsigned short m_eFieldType;

        /// Number of elements per vertex/triangle
        unsigned char m_nElementsPerItem;

        /// Default value
        /**
         * @todo consider changing to a virtual function and having derived field information classes.
         */
        T m_tDefaultValue;

        /// To keep track of the number of references
        unsigned int m_nReferences;

      public:
        /// Construct from values.
        /**
         * @param eFieldType The type of field.
         * @param nElementsPeritem The number of elements required per item.
         * @param tDefaultData The default value to store in this field.
         * @post A call to @a addReference() should be made after construction if the pointer is to be kept.
         */
        inline MeshFieldInformation(unsigned short eFieldType, unsigned char nElementsPerItem, T tDefaultValue)
        : m_eFieldType(eFieldType), m_nElementsPerItem(nElementsPerItem),
        m_tDefaultValue(tDefaultValue), m_nReferences(0)
        {}

        /// Copy another field information object.
        /**
         * @param rCopy Other field information object.
         * @post A call to @a addReference() should be made after construction if the pointer is to be kept.
         */
        inline MeshFieldInformation(const MeshFieldInformation & rCopy)
        : m_eFieldType(rCopy.eFieldType), m_nElementsPerItem(rCopy.nElementsPerItem),
        m_tDefaultValue(rCopy.tDefaultValue), m_nReferences(0)
        {}

        /// Add a reference to this field information.
        /** @return Pointer to the field information. */
        inline MeshFieldInformation * addReference()
        {
          ++m_nReferences;
          return this;
        }
        /// Remove a reference to this field information.
        /**
         * Will remove this object if no more references exist.
         */
        inline void removeReference()
        {
          if (m_nReferences <= 1) {
            delete this;
            return;
          } else {
            --m_nReferences;
          }
        }

      public:
        // ACCESSORS
        inline unsigned short getFieldType() const
        {
          return m_eFieldType;
        }
        inline void setFieldType(unsigned short eFieldType)
        {
          m_eFieldType = eFieldType;
        }
        inline unsigned char getElementsPerItem() const
        {
          return m_nElementsPerItem;
        }
        /**
         * @return true on success, false of failure
         */
        inline bool setElementsPerItem(unsigned char nElementsPerItem)
        {
          // Unchangeable unless there aren't any references
          if (!m_nReferences) {
            m_nElementsPerItem = nElementsPerItem;
            return true;
          } else {
            return false;
          }
        }
        inline T getDefaultValue() const
        {
          return m_tDefaultValue;
        }
        inline void setDefaultValue(T tDefaultValue)
        {
          m_tDefaultValue = tDefaultValue;
        }
    };

    /// Data attached to Vertices/triangles of the mesh.
    /**
     * @param T component type of data.
     */
    template <typename T>
        class MeshData
    {
      protected:
        /// Pointer to permanent data of size m_nElementsPerItem * m_nPermanentItems
        T * m_pData;

        /// Pointer to field information
        Mesh::MeshFieldInformation<T> * m_pInfo;

        /// So that Mesh can initialise mesh data objects
        friend class Mesh;

      public:
        /// Initialise the field type.
        /**
         * If memory has already been allocated it will be deallocated.
       * @param pInfo Field information (must not be NULL).
         */
        inline void init(Mesh::MeshFieldInformation<T> * pInfo)
        {
          // If data memory allocated, clean up and reset pointer
          if (m_pData) {
            delete [] m_pData;
            m_pData = 0;
          }
          if (m_pInfo) {
            m_pInfo->removeReference();
          }
          if (pInfo) {
            m_pInfo = pInfo->addReference();
          } else {
            m_pInfo = 0;
          }
        }
        /// Initialise the field type.
        /**
         * If memory has already been allocated it will be deallocated.
         * @param otherMeshData Mesh data to obtain field properties from.
         */
        inline void init(const MeshData & otherMeshData)
        {
          init(otherMeshData.m_pInfo);
        }

      public:
        /// default constructor.
        /**
         * Initialises the data members to sensible values.
         */
        inline MeshData()
        : m_pData(0), m_pInfo(0)
        {
        }

        /// Destructor.
        /**
         * Deallocates memory from data.
         */
        inline ~MeshData()
        {
          // If data memory allocated, clean up
          if (m_pData) {
            delete [] m_pData;
            // m_pData = 0;
          }
          // Remove reference to field information
          if (m_pInfo) {
            m_pInfo->removeReference();
            // m_pInfo = 0;
          }
        }

        /// Allocate memory for the data.
        /**
         * If memory has already been allocated it will be reallocated and the data will be lost
         * @param nPermanentItems Number of items to allocate data for.
         * @pre MeshData should have been initialised with @a init prior to using allocate.
         * @see init
         */
        inline void allocate(unsigned int nPermanentItems)
        {
          // If data memory allocated, clean up
          if (m_pData) {
            delete [] m_pData;
          }
          // Allocate memory for data or reset pointer if no memory needed
          if (nPermanentItems && m_pInfo && getElementsPerItem()) {
            unsigned int n = nPermanentItems*getElementsPerItem();
            m_pData = new T[n];
            // Initialise all values to zero
            for (unsigned int i = 0; i < n; ++i) {
              m_pData[i] = getDefaultValue();
            }
          } else {
            m_pData = 0;
          }
        }

        /// Get the data associated with an item.
        /**
         * @param iIndex Index of item.
         * @return Pointer to T data of length m_nElementsPerItem.
         * @pre Memory should have been allocated prior to using this operator.
         * @see allocate
         */
        inline T * const operator [] (int iIndex) const
        {
          return m_pData ? &m_pData[iIndex * getElementsPerItem()] : 0;
        }

        /// Get the field type.
        /**
         * @return Field type id.
         */
        inline unsigned short getFieldType() const
        {
          return m_pInfo->getFieldType();
        }
        /// Get the number of elements per item.
        /**
         * @return Number of elements per item.
         */
        inline unsigned char getElementsPerItem() const
        {
          return m_pInfo->getElementsPerItem();
        }
        /// Get the default data value.
        /**
         * @return Default value.
         */
        inline T getDefaultValue() const
        {
          return m_pInfo->getDefaultValue();
        }

    };

    /// Class to store block of mesh data about multiple items.
    /**
     * Can be used for permanent Vertices / triangles or for adaptive data
     */
    class MeshBlock
    {
      public:
        /// Primary constructor.
        /**
         * @param nFieldsFloat Number of float data fields.
         * @param nFieldsUint Number of unsigned int data fields.
         */
        inline MeshBlock(unsigned int nFieldsFloat,
                         unsigned int nFieldsUint)
        : m_iStartIndex(0), m_nItems(0),
          m_nFieldsFloat(nFieldsFloat),
          m_nFieldsUint (nFieldsUint),
          m_pDataFloat  (new MeshData<float>       [nFieldsFloat]),
          m_pDataUint   (new MeshData<unsigned int>[nFieldsUint] )
        {
        }

        /// Constructor taking number of fields from another MeshBlock.
        /**
         * @param otherMeshBlock Another MeshBlock to take the field data from.
         * @param iStartIndex Index of first item in block.
         */
        inline MeshBlock(const MeshBlock & otherMeshBlock, unsigned int iStartIndex)
        : m_iStartIndex(iStartIndex), m_nItems(0),
          m_nFieldsFloat(otherMeshBlock.m_nFieldsFloat),
          m_nFieldsUint (otherMeshBlock.m_nFieldsUint),
          m_pDataFloat  (new MeshData<float>       [otherMeshBlock.m_nFieldsFloat]),
          m_pDataUint   (new MeshData<unsigned int>[otherMeshBlock.m_nFieldsUint] )
        {
          unsigned int i;
          for (i = 0; i < m_nFieldsFloat; ++i) {
            m_pDataFloat[i].init(otherMeshBlock.m_pDataFloat[i]);
          }
          for (i = 0; i < m_nFieldsUint; ++i) {
            m_pDataUint[i].init(otherMeshBlock.m_pDataUint[i]);
          }

        }

        /// Destructor.
        inline ~MeshBlock()
        {
          // Clean up fields
          if (m_pDataFloat) {
            delete [] m_pDataFloat;
          }
          if (m_pDataUint) {
            delete [] m_pDataUint;
          }
        }


        /// Set the number of items in block.
        /**
         * Ensures that @a nItems items are allocated ready for use.
         * @param nItems Number of items in block.
         */
        inline void init(unsigned int nItems)
        {
          // Set the number of items
          m_nItems = nItems;

          // Allocate memory for each field
          unsigned int i;
          for (i = 0; i < m_nFieldsFloat; ++i) {
            m_pDataFloat[i].allocate(nItems);
          }
          for (i = 0; i < m_nFieldsUint;  ++i) {
            m_pDataUint[i].allocate(nItems);
          }
        }

        /// Get the index of the first item in this block.
        /**
         * @return The index of the first item in this block.
         */
        inline unsigned int getStartIndex() const
        {
          return m_iStartIndex;
        }
        /// Set the index of the first item in this block.
        /**
         * @param iStartIndex The index of the first item in this block.
         */
        inline void setStartIndex(unsigned int iStartIndex)
        {
          m_iStartIndex = iStartIndex;
        }

        /// Get the number of items in this block.
        /**
         * @return The number of items stored in this block.
         */
        inline unsigned int getNumItems() const
        {
          return m_nItems;
        }


        /// Get the index of a float field by id.
        /**
         * @param eFieldType Field type to find.
         * @return Index of field or -1 to indicate failure.
         */
        inline int getFieldFloatIndex(unsigned short eFieldType) const
        {
          unsigned int i = 0;
          for (; i < m_nFieldsFloat; ++i) {
             if (m_pDataFloat[i].getFieldType() == eFieldType) {
              return i;
            }
          }
          return -1;
        }
        /// Get the index of a unsigned int field by id.
        /**
         * @param eFieldType Field type to find.
         * @return Index of field or -1 to indicate failure.
         */
        inline int getFieldUintIndex(unsigned short eFieldType) const
        {
          unsigned int i = 0;
          for (; i < m_nFieldsUint; ++i) {
            if (m_pDataUint[i].getFieldType() == eFieldType) {
              return i;
            }
          }
          return -1;
        }

        /// Get a float field.
        /**
         * @param iIndex Index of float field to get (Must be in range).
         */
        inline MeshData<float> & getFieldFloat(unsigned int iIndex)
        {
          return m_pDataFloat[iIndex];
        }
        /// Get a float field.
        /**
         * @param iIndex Index of float field to get (Must be in range).
         */
        inline const MeshData<float> & getFieldFloat(unsigned int iIndex) const
        {
          return m_pDataFloat[iIndex];
        }
        /// Get a unsigned int field.
        /**
         * @param iIndex Index of unsigned int field to get (Must be in range).
         */
        inline MeshData<unsigned int> & getFieldUint(unsigned int iIndex)
        {
          return m_pDataUint[iIndex];
        }
        /// Get a unsigned int field.
        /**
         * @param iIndex Index of unsigned int field to get (Must be in range).
         */
        inline const MeshData<unsigned int> & getFieldUint(unsigned int iIndex) const
        {
          return m_pDataUint[iIndex];
        }

      protected:
        /// Start index of block
        unsigned int m_iStartIndex;
        /// Number of items in block
        unsigned int m_nItems;

        /// Number of float fields
        unsigned int  m_nFieldsFloat;
        /// Number of unsigned int fields
        unsigned int  m_nFieldsUint;

        /// Float fields
        MeshData<float>        * m_pDataFloat;
        /// Uint fields
        MeshData<unsigned int> * m_pDataUint;
    };

  public:

    inline Mesh(unsigned int nVertexDataFieldsFloat,
                unsigned int nVertexDataFieldsUint,
                unsigned int nTriangleDataFieldsFloat,
                unsigned int nTriangleDataFieldsUint);

    inline virtual ~Mesh();

    inline void init(unsigned int nVerts, unsigned int nTris);

    bool calculatePermanentNeighbours();
    void calculatePermanentPlanes();

    virtual void renderGL() const;

    /// Vertex operation function.
    typedef void (* VertexOperation)(void * data, MeshBlock * block, int blockIndex);
    /// Perform a vertex operation on all vertices.
    virtual void foreachVertex(VertexOperation op, void * data = 0);

    /// Get the index of a vertex float field by id.
    /**
     * @param eFieldType Field type to find.
     * @return Index of field or -1 to indicate failure.
     */
    inline int getVertexFieldFloatIndex(unsigned short eFieldType) const
    {
      return m_PermanentVertices.getFieldFloatIndex(eFieldType);
    }
    /// Get the index of a vertex unsigned int field by id.
    /**
     * @param eFieldType Field type to find.
     * @return Index of field or -1 to indicate failure.
     */
    inline int getVertexFieldUintIndex(unsigned short eFieldType) const
    {
      return m_PermanentVertices.getFieldUintIndex(eFieldType);
    }
    /// Get the index of a triangle float field by id.
    /**
     * @param eFieldType Field type to find.
     * @return Index of field or -1 to indicate failure.
     */
    inline int getTriangleFieldFloatIndex(unsigned short eFieldType) const
    {
      return m_PermanentTriangles.getFieldFloatIndex(eFieldType);
    }
    /// Get the index of a triangle unsigned int field by id.
    /**
     * @param eFieldType Field type to find.
     * @return Index of field or -1 to indicate failure.
     */
    inline int getTriangleFieldUintIndex(unsigned short eFieldType) const
    {
      return m_PermanentTriangles.getFieldUintIndex(eFieldType);
    }

    /// Get a vertex float field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline MeshData<float> & getPermanentVertexFieldFloat(int iFieldIndex)
    {
      return m_PermanentVertices.getFieldFloat(iFieldIndex);
    }
    /// Get a vertex float field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline const MeshData<float> & getPermanentVertexFieldFloat(int iFieldIndex) const
    {
      return m_PermanentVertices.getFieldFloat(iFieldIndex);
    }
    /// Get the index of a vertex unsigned int field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline MeshData<unsigned int> & getPermanentVertexFieldUint(int iFieldIndex)
    {
      return m_PermanentVertices.getFieldUint(iFieldIndex);
    }
    /// Get the index of a vertex unsigned int field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline const MeshData<unsigned int> & getPermanentVertexFieldUint(int iFieldIndex) const
    {
      return m_PermanentVertices.getFieldUint(iFieldIndex);
    }
    /// Get the index of a triangle float field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline MeshData<float> & getPermanentTriangleFieldFloat(int iFieldIndex)
    {
      return m_PermanentTriangles.getFieldFloat(iFieldIndex);
    }
    /// Get the index of a triangle float field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline const MeshData<float> & getPermanentTriangleFieldFloat(int iFieldIndex) const
    {
      return m_PermanentTriangles.getFieldFloat(iFieldIndex);
    }
    /// Get the index of a triangle unsigned int field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline MeshData<unsigned int> & getPermanentTriangleFieldUint(int iFieldIndex)
    {
      return m_PermanentTriangles.getFieldUint(iFieldIndex);
    }
    /// Get the index of a triangle unsigned int field by id.
    /**
     * @param iFieldIndex Field index.
     * @return Permanent mesh block of field.
     */
    inline const MeshData<unsigned int> & getPermanentTriangleFieldUint(int iFieldIndex) const
    {
      return m_PermanentTriangles.getFieldUint(iFieldIndex);
    }

    /// Get the number of permanent Vertices.
    /**
     * @return The number of permanent Vertices.
     */
    inline unsigned int getNumPermanentVertices() const
    {
      return m_PermanentVertices.getNumItems();
    }
    /// Get the number of permanent triangles.
    /**
     * @return The number of permanent triangles.
     */
    inline unsigned int getNumPermanentTriangles() const
    {
      return m_PermanentTriangles.getNumItems();
    }

    /// Get the vertex positions mesh data.
    /**
     * @return Reference to the vertex positions mesh data.
     */
    inline const MeshData<float> & getPermanentVertexPositions() const
    {
      return m_PermanentVertices.getFieldFloat(getIndexVertexPosition());
    }
    /// Get the vertex positions mesh data.
    /**
     * @return Reference to the vertex positions mesh data.
     */
    inline const MeshData<unsigned int> & getPermanentTriangleVerts() const
    {
      return m_PermanentTriangles.getFieldUint(getIndexTriangleVerts());
    }

  protected:
    // NUMBER OF FIELDS IN EACH FIELD TYPE (REQUIRED BY MESH)
    /// Get the index of the first available per vertex float data field.
    /**
     * @return The index of the first available per vertex float data field.
     */
    inline static unsigned int getFirstVertexFloat()
    {
      return 1;
      // Fields: Position
    }
    /// Get the index of the first available per vertex unsigned int data field.
    /**
     * @return The index of the first available per vertex unsigned int data field.
     */
    inline static unsigned int getFirstVertexUint()
    {
      return 0;
      // Fields:
    }
    /// Get the index of the first available per triangle float data field.
    /**
     * @return The index of the first available per vertex float data field.
     */
    inline static unsigned int getFirstTriangleFloat()
    {
      return 0;
      // Fields:
    }
    /// Get the index of the first available per triangle unsigned int data field.
    /**
     * @return The index of the first available per vertex float data field.
     */
    inline static unsigned int getFirstTriangleUint()
    {
      return 1;
      // Fields: Vertex ids
    }

    // SPECIFIC FIELD IDS
    /// Get the index of the per vertex position field.
    /**
     * @return The index of the per vertex position field.
     */
    inline static unsigned int getIndexVertexPosition()
    {
      return 0;
    }
    /// Get the index of the per triangle Vertices field.
    /**
     * @return The index of the per triangle Vertices field.
     */
    inline static unsigned int getIndexTriangleVerts()
    {
      return 0;
    }


  protected:

    /// Block of permanent Vertices
    MeshBlock m_PermanentVertices;
    /// Block of permanent triangles
    MeshBlock m_PermanentTriangles;

    /// Mesh data field index.
    /**
     * Pointer to:
     *   Array [4] of pointers to:
     *   index of float type id to field index
     */
    int ** m_pFieldIndex;

};

/// Constructor from number of vertex & triangle fields.
/**
 * @param nVertexDataFieldsFloat Number of per vertex float data fields.
 * @param nVertexDataFieldsUint Number of per vertex unsigned int data fields.
 * @param nTriangleDataFieldsFloat Number of per triangle float data fields.
 * @param nTriangleDataFieldsUint Number of per triangle unsigned int data fields.
 */
inline Mesh::Mesh(unsigned int nVertexDataFieldsFloat,
                  unsigned int nVertexDataFieldsUint,
                  unsigned int nTriangleDataFieldsFloat,
                  unsigned int nTriangleDataFieldsUint)
  : m_PermanentVertices(nVertexDataFieldsFloat     + getFirstVertexFloat(),
                         nVertexDataFieldsUint      + getFirstVertexUint()),
  m_PermanentTriangles(  nTriangleDataFieldsFloat   + getFirstTriangleFloat(),
                         nTriangleDataFieldsUint    + getFirstTriangleUint())
{
  // Initialise the data fields
  m_PermanentVertices.getFieldFloat(getIndexVertexPosition()).init(
      new MeshFieldInformation<float>(MESH_DATA_POSITION, 3, 0.0f)
                                                                   );
  m_PermanentTriangles.getFieldUint (getIndexTriangleVerts() ).init(
      new MeshFieldInformation<unsigned int>(MESH_DATA_VERTICES, 3, MESH_INVALID_VERTEX)
                                                                   );
}

/// Destructor to clean up after whole mesh.
inline Mesh::~Mesh()
{
}

/// Set the number of permanent Vertices and triangles.
/**
 * Ensures that @a nVerts Vertices and @a nTris are allocated ready for use.
 * @param nVerts Number of permanent Vertices.
 * @param nTris Number of permanent triangles.
 */
inline void Mesh::init(unsigned int nVerts, unsigned int nTris)
{
  m_PermanentVertices.init(nVerts);
  m_PermanentTriangles.init(nTris);
}

}

#endif // _MESH_MESH_H_
