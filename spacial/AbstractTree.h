/*
 * spacial/AbstractTree.h
 *
 * Copyright (C) 2009-2014 James Hogan <james@albanarts.com>
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
 * Generic spacial tree.
 *
 */

#ifndef _SPACIAL_ABSTRACT_TREE_H_
#define _SPACIAL_ABSTRACT_TREE_H_

// Configuration
//#define SPACIAL_TREE_PRINT_FUNCTION

#include <maths/Vector.h>
#include <util/compiler.h>
#include <util/assert.h>
#include <util/TemplateUtils.h>

#include <iostream>
#include <limits>

#ifdef SPACIAL_TREE_PRINT_FUNCTION
  #undef SPACIAL_TREE_PRINT_FUNCTION
  #define SPACIAL_TREE_PRINT_FUNCTION \
    std::cout << __FUNCTION__ << std::endl;
#else
  #define SPACIAL_TREE_PRINT_FUNCTION
#endif
#define TRAVERSAL_CLASS(NAME) \
  public: \
  /* Constructor */ \
  NAME() \
  { \
    SPACIAL_TREE_PRINT_FUNCTION \
  }
#define TRAVERSAL_CLASS_INIT(NAME) \
  public: \
  /* Constructor */ \
  NAME() \
  { \
    SPACIAL_TREE_PRINT_FUNCTION \
    init(); \
  }

namespace maths {
  template <typename A, int B>
  class FixedPoint;
};

namespace spacial
{
  /// Null tree data
  class NullTreeData
  {
    public:
      /*
       * Accessors
       */

      /// The data is quite unimportant.
      bool isImportant() const
      {
        return false;
      }
  };

  /**
   * Spacial limits to help the tree code deal with different coordinate types.
   */
  template <typename T>
  class SpacialLimits;

  /**
   * Generic integer spacial limits.
   */
  template <typename T, bool ISINT>
  class SpacialLimitsInt;

  template <typename T>
  class SpacialLimitsInt<T, false>
  {
  public:
    static const unsigned int max_height = 0;

    /* none of these should ever actually get called */

    static T genHeightBit(int height)
    {
      RUNTIME_DEADCODE();
      return T();
    }

    static bool isHeightBitSet(const T &val, int height)
    {
      RUNTIME_DEADCODE();
      return false;
    }

    template <typename V>
    static V childCenterFromParent(const V &parent_center, int child_height, unsigned int index)
    {
      RUNTIME_DEADCODE();
      return parent_center;
    }
  };

  template <typename T>
  class SpacialLimitsInt<T, true>
  {
  public:
    COMPILETIME_ASSERT(std::numeric_limits<T>::is_integer, T_must_be_integer);
    static const unsigned int max_height = std::numeric_limits<T>::digits - 1;

    static T genHeightBit(int height)
    {
      return (T)1 << height;
    }

    static bool isHeightBitSet(const T &val, int height)
    {
      return (int)(val >> height) & 0x1;
    }

    template <int N, int F>
    static maths::Vector<N, maths::FixedPoint<T, F> > childCenterFromParent(
        const maths::Vector<N, maths::FixedPoint<T, F> > &parent_center,
        int child_height, unsigned int index)
    {
      unsigned int inv_index = ~index << 1;
      int i;
      maths::Vector<N, maths::FixedPoint<T, F> > ret;
      maths::FixedPoint<T, F> hsize;
      hsize.rawValue = genHeightBit(child_height);
      for (i = 0; i < N; ++i, inv_index >>= 1) {
        if (inv_index & 0x2) {
          ret[i] = parent_center[i] - hsize;
        } else {
          ret[i] = parent_center[i] + hsize;
        }
      }
      return ret;
    }
  };

  /**
   * Spacial limits of a fixed point value directly mirror those of the raw type.
   */
  template <typename A, int B>
  class SpacialLimits< maths::FixedPoint<A, B> >
  {
    typedef maths::FixedPoint<A, B> Fixed;
  public:
    static const bool is_logical = SpacialLimits<A>::is_logical;
    static const bool is_signed = SpacialLimits<A>::is_signed;
    static const unsigned int max_height = SpacialLimits<A>::max_height;

    static Fixed genHeightBit(int height)
    {
      Fixed a;
      a.rawValue = SpacialLimits<A>::genHeightBit(height);
      return a;
    }

    static bool isHeightBitSet(const Fixed &val, int height)
    {
      return SpacialLimits<A>::isHeightBitSet(val.rawValue, height);
    }

    template <int N>
    static maths::Vector<N, Fixed > childCenterFromParent(
        const maths::Vector<N, Fixed > &parent_center,
        int child_height, unsigned int index)
    {
      return SpacialLimits<A>::template childCenterFromParent(parent_center, child_height, index);
    }
  };

  /**
   * Generic spacial limits (handles integers).
   */
  template <typename T>
  class SpacialLimits
  {
  public:
    COMPILETIME_ASSERT(std::numeric_limits<T>::is_specialized, T_should_have_numeric_limits);
    static const bool is_logical = std::numeric_limits<T>::is_integer;
    static const bool is_signed = std::numeric_limits<T>::is_signed;
    static const unsigned int max_height = SpacialLimitsInt<T, is_logical>::max_height;

    /**
     * Generate a value of the half size at height @p height.
     * @param height The depth, where 0 is bottom level.
     * @return Half size of node at height @p height.
     */
    static T genHeightBit(int height)
    {
      return SpacialLimitsInt<T, is_logical>::genHeightBit(height);
    }

    /**
     * Find whether the bit for a specific height is set.
     * @param val   Value to check.
     * @param height The height, where 0 is bottom level.
     * @return true if the height bit for @p height in @p val is set.
     */
    static bool isHeightBitSet(const T &val, int height)
    {
      return SpacialLimitsInt<T, is_logical>::isHeightBitSet(val, height);
    }

    /// Find the center of a child from a parent center in N dimentions.
    template <int N, typename C>
    static maths::Vector<N, C > childCenterFromParent(
        const maths::Vector<N, C > &parent_center,
        int child_height, unsigned int index)
    {
      return SpacialLimitsInt<T, is_logical>::childCenterFromParent(parent_center, child_height, index);
    }
  };

  /// Generic tree implementation.
  /**
   * This is a low level generic spacial tree implementation.
   * It stores an object of type @p D per node.
   * @param N Number of dimentions.
   * @param C Component type (used in a maths::Vector).
   * @param S Type to measure the size of objects with.
   * @param D Data to store per node.
   */
  template <int N, typename C, typename S, typename D>
  class AbstractTree
  {
    public:
      /*
       * Compile time assertions
       */
      COMPILETIME_ASSERT(N>0, AbstractTree_N_template_parameter_must_be_positive);

      /*
       * Metadata
       */
      typedef SpacialLimits<C> Limits;

      /**
       * For logical types we can optimise out the half size.
       * The half size should be easily derivable from the node height if the
       * component type is integral.
       */
      static const bool hasHalfSize = !Limits::is_logical;

      /*
       * Types
       */

      /// Position vector type (may be signed).
      typedef maths::Vector<N,C> PositionVector;

      /// Displacement vector type (signed).
      typedef maths::Vector<N,C> DisplacementVector;
      /// If C is unsigned, need a separate signed version for displacement.
      COMPILETIME_ASSERT(SpacialLimits<C>::is_signed, C_should_be_signed);

      /// Size vector type.
      typedef maths::Vector<N,S> SizeVector;

      /// Distance type (may be unsigned).
      typedef C DistanceType;

      /// Displacement type (signed).
      typedef C DisplacementType;

      /// Size type.
      typedef S SizeType;

      /// Node data.
      typedef D NodeData;

      /// A single node in the tree.
      class Node
      {
        private:
          /*
           * Constructors + destructor
           * Only AbstractTree can use these
           */
          friend class AbstractTree;

          /// Top level node constructor (logical).
          Node(unsigned int height)
          : m_position((C)0)
          , m_parent(0)
          , m_height(height)
          , m_childIndex(0)
          , m_numChildren(0)
          , m_data()
          {
            COMPILETIME_ASSERT(!hasHalfSize, logical_constructor_with_non_logical_C);
            RUNTIME_ASSERT(height <= Limits::max_height,
                           "Initial node height %u shouldn't really exceed %u",
                           height, Limits::max_height);
            for (int i = 0; i < (1<<N); ++i)
            {
              m_children[i] = 0;
            }
          }

          /// Top level node constructor (non logical).
          Node(unsigned int height, SizeType halfSize)
          : m_position((C)0)
          , m_halfSize(halfSize)
          , m_parent(0)
          , m_height(height)
          , m_childIndex(0)
          , m_numChildren(0)
          , m_data()
          {
            RUNTIME_ASSERT(height <= Limits::max_height,
                           "Initial node height %u shouldn't really exceed %u",
                           height, Limits::max_height);
            for (int i = 0; i < (1<<N); ++i)
            {
              m_children[i] = 0;
            }
          }

          /// Sub level node constructor.
          Node(Node* parent, unsigned int index)
          : m_position(),
            m_parent(parent),
            m_height(parent->m_height - 1),
            m_childIndex(index),
            m_numChildren(0),
            m_data()
          {
            RUNTIME_ASSERT(parent->m_height, "Parent node already at max depth");
            // Initialise child pointers
            for (int i = 0; i < (1<<N); ++i)
            {
              m_children[i] = 0;
            }
            // Initialise position
            if (hasHalfSize) {
              SizeType newHalfSize;
              m_halfSize = newHalfSize = parent->halfSize() / 2;
              for (int i = 0; i < N; ++i)
              {
                m_position[i] = parent->m_position[i] + ( (index & (1<<i))
                    ?  newHalfSize
                    : -newHalfSize );
              }
            } else {
              m_position = Limits::childCenterFromParent(parent->m_position, m_height, index);
            }
            // Update parent
            ++parent->m_numChildren;
            RUNTIME_ASSERT(!parent->m_children[index],
                           "new sibling[%d] shouldn't exist, but set to %p\n",
                           index, parent->m_children[index]);
            parent->m_children[index] = this;
          }

          /// Sparse sub level node constructor.
          Node(Node* parent, unsigned char rel_depth, const PositionVector &position)
          : m_position(position),
            m_halfSize(parent->halfSize() >> rel_depth),
            m_parent(parent),
            m_height(parent->m_height - rel_depth),
            m_childIndex(0),
            m_numChildren(0),
            m_data()
          {
            RUNTIME_ASSERT(parent->m_height >= rel_depth, "Parent node already at max depth");
            // Initialise child pointers
            for (int i = 0; i < (1<<N); ++i)
            {
              m_children[i] = 0;
            }
            // Find index in parent
            for (int i = 0; i < N; ++i)
            {
              if (position[i] >= parent->m_position[i])
              {
                m_childIndex |= (1 << N);
              }
            }
            // Update parent
            ++parent->m_numChildren;
            RUNTIME_ASSERT(!parent->m_children[m_childIndex],
                           "new sibling[%d] shouldn't exist, but set to %p\n",
                           m_childIndex, parent->m_children[m_childIndex]);
            parent->m_children[m_childIndex] = this;
          }

          /// Destructor.
          ~Node()
          {
            for (int i = 0; i < N; ++i)
            {
              if (0 != m_children[i])
              {
                p_deleteNode(m_children[i]);
              }
            }
            if (0 != m_parent)
            {
              // Remove from parent
              --m_parent->m_numChildren;
              m_parent->m_children[m_childIndex] = 0;
            }
          }

        public:
          /*
           * Node management
           */

          /// Do a reality check to see whether this node is needed any longer.
          void realityCheck()
          {
            // A root node is allowed to be empty, as is a node with children
            if (0 != m_parent && 0 == m_numChildren)
            {
              // Leaf node, whether the node is important depends on data
              if (!m_data.isImportant())
              {
                // We might get deleted in a moment so remember the parent so we can recurse
                Node* parent = m_parent;
                // Suicide
                p_deleteNode(this);
                // Give parent a reality check
                parent->realityCheck();
              }
            }
          }

          /*
           * Accessors
           */

          /// Get the data.
          NodeData& getData()
          {
            return m_data;
          }

          /// Get the parent.
          Node* parent()
          {
            return m_parent;
          }

          /// Get the height up the tree.
          unsigned int height()
          {
            return m_height;
          }

          /// Get the position of the node.
          const PositionVector& position() const
          {
            return m_position;
          }

          /// Get the halfsize of the node as an accurate distance.
          DistanceType halfSizeDistance() const
          {
            if (hasHalfSize) {
              return (DisplacementType)m_halfSize;
            } else {
              return Limits::genHeightBit(m_height);
            }
          }

          /// Get the halfsize of the node.
          SizeType halfSize() const
          {
            if (hasHalfSize) {
              return m_halfSize;
            } else {
              return Limits::genHeightBit(m_height);
            }
          }
        private:

          /*
           * Tree traversal
           */

          /// Traverse the tree below a node.
          template <class TRAVERSAL>
          void traverse(TRAVERSAL* traversal, typename TRAVERSAL::CullState cullState = typename TRAVERSAL::CullState())
          {
            if (traversal->shouldTraverseNode(this, cullState))
            {
              typename TRAVERSAL::StackData data(traversal);
              traversal->beforeDeepening(this);
              // Should be optimised at compile time
              if (TRAVERSAL::isSorted)
              {
                unsigned int nodeOrder[1<<N];
                traversal->sortTraversal(this, nodeOrder);
                for (int i = 0; i < (1<<N); ++i)
                {
                  if (nodeOrder[i] >= 0)
                  {
                    RUNTIME_ASSERT(nodeOrder[i] < (1<<N), "Node order out of range");
                    Node* child = getChild(nodeOrder[i]);
                    if (child)
                    {
                      child->traverse<TRAVERSAL>(traversal, cullState);
                    }
                  }
                }
              }
              else
              {
                for (int i = 0; i < (1<<N); ++i)
                {
                  Node* child = getChild(i);
                  if (child)
                  {
                    child->traverse<TRAVERSAL>(traversal);
                  }
                }
              }
              traversal->afterDeepening(this);
            }
          }

        public:

          /*
           * General util
           */

          /// Determine the child node a position belongs to.
          /**
           * @todo Ensure gcc compiles this efficiently
           */
          static unsigned int determineChild(const DisplacementVector& displacement)
          {
            unsigned int result = 0;
            for (int i = 0; i < N; ++i)
            {
              result |= (displacement[i] > static_cast<DisplacementType>(0)) ? (1<<i) : 0;
            }
            return result;
          }

          /// Determine if a point is contained in a box of size limit around this node.
          static bool pointInBoxRelative(const DisplacementVector& displacement, DistanceType boxHalfSize)
          {
            for (int i = 0; i < N; ++i)
            {
              if (displacement[i] > boxHalfSize || displacement[i] < -boxHalfSize)
              {
                return false;
              }
            }
            return true;
          }

          /// Determine if a bounding box intersects the node.
          bool intersectsBoundingBoxRelative(const DisplacementVector& displacement, DistanceType radius)
          {
            return pointInBoxRelative(displacement, halfSizeDistance() + radius);
          }
          /// Determine if a bounding box intersects the node.
          bool intersectsBoundingBox(const PositionVector& position, DistanceType radius)
          {
            DisplacementVector displacement(position - m_position);
            return intersectsBoundingBoxRelative(displacement, radius);
          }

          /// Determine if a bounding box is contained inside the node.
          bool containsBoundingBoxRelative(const DisplacementVector& displacement, DistanceType radius)
          {
            return pointInBoxRelative(displacement, halfSizeDistance() - radius);
          }
          /// Determine if a bounding box is contained inside the node.
          bool containsBoundingBox(const PositionVector& position, DistanceType radius)
          {
            DisplacementVector displacement(position - m_position);
            return containsBoundingBoxRelative(displacement, radius);
          }

        public:
          /// Get a child node, return 0 if no such child.
          Node* getChild(unsigned int index) const
          {
            RUNTIME_ASSERT(index < (1<<N), "child index overflow");
            return m_children[index];
          }

          /// Get the number of children nodes.
          unsigned int numChildren() const
          {
            return m_numChildren;
          }

          /// Create and return a child that doesn't already exist.
          Node *createChild(unsigned int index)
          {
            RUNTIME_ASSERT(index < (1<<N), "child index overflow");
            RUNTIME_ASSERT(!m_children[index], "child already exists");

            return p_newNode(this, index);
          }

        private:

          /*
           * Data members
           */

          /// Position of centre.
          PositionVector m_position;

          /// Half size.
          typename util::condType<hasHalfSize, SizeType>::type m_halfSize;

          /// Parent node.
          Node* m_parent;

          /// Children nodes.
          Node* m_children[1<<N];

          /// Height up the tree (where 0 is maximum depth).
          unsigned char m_height;

          /// Child number in parent.
          unsigned char m_childIndex;

          /// Number of child nodes.
          unsigned char m_numChildren;

          /// Per-node data.
          NodeData m_data;
      };

      /*
       * Constructors + destructor
       */

      /// Default constructor.
      AbstractTree(SizeType halfSize, unsigned int height = 0)
      : m_top(height ? height : Limits::max_height, halfSize)
      {
      }

      /// Copy constructor.
      /**
       * If copying shouldn't be allowed then privatise D::D(const D&).
       */
      AbstractTree(const AbstractTree& copy)
      {
      }

      /// Destructor.
      ~AbstractTree()
      {
      }

      /*
       * Accessors
       */

      /// Get the top node.
      Node *top()
      {
        return &m_top;
      }

      /// Get the top node.
      const Node *top() const
      {
        return &m_top;
      }

      /*
       * Traversal functions
       */

      /// Template for traversal classes.
      /**
       * This is the base class for customizing traversal.
       */
      class AbstractTraversal
      {
        public:
          TRAVERSAL_CLASS(AbstractTraversal)
          /*
           * Sorting
           */
          /// Whether to sort.
          enum { isSorted = 0 };
          /// Do sorting of child node ids.
          void sortTraversal(Node* node, unsigned int* childOrder);

          /*
           * Stack data
           */
          /// Data per level of the traversal.
          class StackData
          {
            public:
              /*
               * Constructors + destructor
               */
              /// Primary constructor.
              StackData(AbstractTraversal* traversal)
              {
              }
          };
          /// Cull state.
          class CullState
          {
          };

          /*
           * Traversal callbacks
           */
          /// Find whether to traverse a given node.
          bool shouldTraverseNode(Node* node, CullState&)
          {
            return true;
          }
          /// Callback before recurring into child nodes.
          void beforeDeepening(Node* node)
          {
          }
          /// Callback after recurring into child nodes.
          void afterDeepening(Node* node)
          {
          }
      };

      /// Callback abstraction (before or after deepening).
      /**
       * @param BASE base type, assumed to have callback(Node*)
       * @param AFTERDEEPEN if true, calls callback after deepening, otherwise before
       */
      template <typename BASE, bool AFTERDEEPEN>
      class SimpleCallbackTraversal : public BASE
      {
        public:
          TRAVERSAL_CLASS(SimpleCallbackTraversal)
          /// Callback before recurring into child nodes.
          void beforeDeepening(Node* node)
          {
            if (!AFTERDEEPEN)
            {
              BASE::callback(node);
            }
            BASE::beforeDeepening(node);
          }
          /// Callback after recurring into child nodes.
          void afterDeepening(Node* node)
          {
            if (AFTERDEEPEN)
            {
              BASE::callback(node);
            }
            BASE::afterDeepening(node);
          }
      };

      /// Does basic and efficient range sorting.
      /**
       * @param BASE base type. assumed to have PositionVector getObserverPosition()
       * @param BACKTOFRONT if true will order from far to near
       */
      template <typename BASE, bool BACKTOFRONT>
      class RangeSortedTraversal : public BASE
      {
        public:
          TRAVERSAL_CLASS(RangeSortedTraversal)
          /// Whether to sort.
          enum { isSorted = 1 };
          unsigned int getFirstChild(Node* node) const
          {
            DisplacementVector displacement(BACKTOFRONT ? node->position() - BASE::getObserverPosition()
                                                        : BASE::getObserverPosition() - node->position());
            return Node::determineChild(displacement);
          }
          void sortTraversal(Node* node, unsigned int* childOrder)
          {
            unsigned int observerChild = getFirstChild(node);
            // Order by number produced by flipped bits
            for (unsigned int i = 0; i < (1<<N); ++i)
            {
              childOrder[i] = observerChild ^ i;
            }
          }
      };
      /// Back to front range sorted traversal.
      template <typename BASE> class BackFirstRangeSortedTraversal  : public RangeSortedTraversal<BASE, true>  {TRAVERSAL_CLASS(BackFirstRangeSortedTraversal)};
      /// Front to back range sorted traversal.
      template <typename BASE> class FrontFirstRangeSortedTraversal : public RangeSortedTraversal<BASE, false> {TRAVERSAL_CLASS(FrontFirstRangeSortedTraversal)};

      /// Traverse the entire tree.
      template <class TRAVERSAL>
      void traverse(TRAVERSAL* data)
      {
        m_top.template traverse<TRAVERSAL>(data);
      }

    private:

      /*
       * Node creation and deletion abstraction.
       */

      /// Create a new node.
      static Node* p_newNode(Node* parent, unsigned int index)
      {
        //std::cout << "Nodes: " << p_numNodes(1) << std::endl;
        return new Node(parent, index);
      }

      /// Delete a node.
      static void p_deleteNode(Node* node)
      {
        //std::cout << "Nodes: " << p_numNodes(-1) << std::endl;
        delete node;
      }
      static int p_numNodes(int change = 0)
      {
        static int numNodes = 0;
        return numNodes += change;
      }

      /*
       * Variables
       */

      /// Main top-level node.
      Node m_top;
  };
}

#endif // _SPACIAL_ABSTRACT_TREE_H_
