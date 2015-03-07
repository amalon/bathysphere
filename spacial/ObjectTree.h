/*
 * spacial/ObjectTree.h
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
 * Generic spacial object tree and associated algorithms.
 *
 * Extends AbstractTree to contain objects which link back and can be moved
 * around.
 *
 */

#ifndef _SPACIAL_OBJECT_TREE_H_
#define _SPACIAL_OBJECT_TREE_H_

#include <spacial/AbstractTree.h>

#include <set>

namespace spacial
{
  template <typename D, typename O>
  class ObjectTreeNodeData
  {
    public:

      /*
       * Types
       */

      typedef std::set<O> ObjectContainer;

      /*
       * Methods
       */

      /// Add an object to the container.
      void addObject(const O& object)
      {
        RUNTIME_ASSERT(0 != object, "Null object should not be added to object tree node");
        m_objects.insert(object);
      }

      /// Remove an object from the container.
      void removeObject(const O& object)
      {
        m_objects.erase(object);
      }

      /// Find whether an object is in this node.
      bool isObjectUsed(const O& object) const
      {
        return m_objects.find(object) != m_objects.end();
      }

      /// Execute a callback for each object.
      template <typename CALLBACK_OBJECT>
      void objectsCallback(CALLBACK_OBJECT* data) const
      {
        typename ObjectContainer::const_iterator it;
        for (it = m_objects.begin(); it != m_objects.end(); ++it)
        {
          data->callback(*it);
        }
      }

      /*
       * Accessors
       */

      /// Decide whether this data is important.
      bool isImportant() const
      {
        return !m_objects.empty() || m_data.isImportant();
      }

      /// Get the objects.
      const ObjectContainer& objects() const
      {
        return m_objects;
      }

      /// Get the data.
      D& getData()
      {
        return m_data;
      }

    private:
      /*
       * Variables
       */

      /// List of objects
      ObjectContainer m_objects;

      /// Other data
      D m_data;
  };

  /// Generic spacial object tree implementation.
  /**
   * This is a generic spacial object tree implementation.
   * It stores an object of type @p D per node as well as multiple objects of
   * type @p O.
   * @param N Number of dimentions.
   * @param C Component type (used in a maths::vector).
   * @param S Type to measure the size of objects with.
   * @param D Data to store per node.
   * @param O Objects to store.
   * @param CO Constant objec type.
   */
  template <int N, typename C, typename S, typename D, typename O, typename CO>
  class ObjectTree : public AbstractTree<N,C,S,ObjectTreeNodeData<D,O> >
  {
    public:
      /*
       * Types
       */

      typedef ObjectTreeNodeData<D,O> ObjectTreeNodeDataType;
      typedef AbstractTree<N,C,S,ObjectTreeNodeDataType> BaseClass;
      typedef typename BaseClass::DistanceType DistanceType;
      typedef typename BaseClass::DisplacementType DisplacementType;
      typedef typename BaseClass::SizeType SizeType;
      typedef typename BaseClass::PositionVector PositionVector;
      typedef typename BaseClass::DisplacementVector DisplacementVector;
      typedef typename BaseClass::SizeVector SizeVector;
      typedef O ObjectType;
      typedef CO ConstObjectType;
      typedef typename BaseClass::Node Node;

      /// Handy object base class with minimal interface.
      class ObjectBase
      {
        public:
          /*
           * Constructors + destructor
           */

          /// Constructor.
          ObjectBase()
          : m_tree(0)
          , m_numNodes(0)
          , m_lastFrame(0)
          {
          }

        private:
          /// Copy constructor disabled.
          ObjectBase(const ObjectBase&);
        public:

          /// Copy constructor function.
          static void copyNodes(const ConstObjectType& from, const ObjectType& to)
          {
            // Tree should be empty
            RUNTIME_ASSERT(0 == to->m_tree, "Tree should be clear when copying nodes");
            RUNTIME_ASSERT(0 == to->m_numNodes, "Nodes should be clear when copying nodes");
            if (0 != from->m_tree)
            {
              to->m_tree = from->m_tree;
              to->m_numNodes = from->m_numNodes;
              for (unsigned int i = 0; i < from->m_numNodes; ++i)
              {
                to->m_nodes[i] = from->m_nodes[i];
              }
              to->m_tree->addObjectToNodes(to, to->m_nodes, to->m_numNodes, false, true);
            }
          }

          /// Destructor.
          virtual ~ObjectBase()
          {
          }

          /// Destructor function.
          static void unlinkNodes(const ObjectType& self)
          {
            if (0 != self->m_tree)
            {
              self->m_tree->removeObject(self);
            }
          }

        protected:

          /*
           * Interface for derived object classes to use
           */

          /// Indicate that the position has changed.
          void updateObjectTreePosition(const ObjectType& self)
          {
            if (0 != m_tree)
            {
              Node* deletedNodes[1<<N];
              Node* existingNode = 0;
              unsigned int numDeletedNodes = 0;
              // Check all nodes are still intersecting
              for (unsigned int i = 0; i < m_numNodes; )
              {
                if (!m_nodes[i]->intersectsBoundingBox(self->getPosition(), self->getMaxRadius()))
                {
                  deletedNodes[numDeletedNodes++] = m_nodes[i];
                  // copy from end and repeat loop with same i
                  m_nodes[i] = m_nodes[--m_numNodes];
                }
                else
                {
                  existingNode = m_nodes[i];
                  ++i;
                }
              }
              // use a deleted node if necessary
              if (0 == existingNode && numDeletedNodes > 0)
              {
                existingNode = deletedNodes[0];
              }
              // work upwards from an existing node to find node containing entire object
              // if no steps are taken, then no new nodes (or no existing node)
              int upSteps = 0;
              while (0 != existingNode && !existingNode->containsBoundingBox(self->getPosition(), self->getMaxRadius()))
              {
                existingNode = existingNode->parent();
                ++upSteps;
              }
              // add from top if no existing nodes
              if (0 == existingNode)
              {
                m_tree->addObject(self, false);
              }
              // work downwards from there to find which nodes to add to
              else if (upSteps > 0)
              {
                Node *addNodes[1<<N];
                unsigned int numAddNodes = 0;
                m_tree->obtainNodes(existingNode, self->getPosition(), self->getMaxRadius(), addNodes, &numAddNodes);
                // remove nodes from add list which are already active
                for (unsigned int i = 0; i < m_numNodes; ++i)
                {
                  for (unsigned int j = 0; j < numAddNodes; ++j)
                  {
                    if (m_nodes[i] == addNodes[j])
                    {
                      addNodes[j] = addNodes[--numAddNodes];
                      // assume addNodes list is unique
                      break;
                    }
                  }
                }
#ifndef NDEBUG
                for (unsigned int i = 0; i < numDeletedNodes; ++i)
                {
                  for (unsigned int j = 0; j < numAddNodes; ++j)
                  {
                    RUNTIME_ASSERT(deletedNodes[i] != addNodes[j], "Node to be deleted in add list");
                  }
                }
#endif
                // add nodes in add list
                m_tree->addObjectToNodes(self, addNodes, numAddNodes, true, false);
              }
              // delete nodes in delete list
              m_tree->removeObjectFromNodes(self, deletedNodes, numDeletedNodes);
            }
          }

        public:
          /// Find whether the object is directly attached to a tree.
          bool isInTree() const
          {
            return (0 != m_tree);
          }

          /// Get the current object tree.
          ObjectTree* objectTree() const
          {
            return m_tree;
          }

        protected:
          // Allow object tree to mess
          friend class ObjectTree;

          /*
           * Member variables
           */

          /// Current object tree.
          ObjectTree* m_tree;

          /// List of tree nodes.
          typename ObjectTree::Node* m_nodes[1<<N];

          /// Number of tree nodes.
          unsigned int m_numNodes;

          /// Last frame number.
          unsigned int m_lastFrame;

          /// Set the object tree.
          void setObjectTree(ObjectTree* tree)
          {
            m_tree = tree;
          }

          /// Clear the list of tree nodes.
          void clearObjectTreeNodes(const ObjectType& obj)
          {
            m_numNodes = 0;
          }

          /// Add an object tree node.
          void addObjectTreeNode(ObjectTree::Node* node, const ObjectType& obj)
          {
            RUNTIME_ASSERT(m_numNodes < (1<<N), "Object in too many nodes");
            m_nodes[m_numNodes++] = node;
          }

          /// Find whether an object tree node is already linked.
          bool isObjectTreeNodeUsed(const Node* node) const
          {
            for (unsigned int i = 0; i < m_numNodes; ++i)
            {
              if (node == m_nodes[i])
              {
                return true;
              }
            }
            return false;
          }

        public:
          /// Get a direct pointer to the list of object tree nodes.
          typename ObjectTree::Node *const *objectTreeNodes() const
          {
            return m_nodes;
          }
          /// Get the current number of object tree nodes.
          unsigned int numObjectTreeNodes() const
          {
            return m_numNodes;
          }
        protected:

          /// Get the last frame number.
          unsigned int lastFrameNumber() const
          {
            return m_lastFrame;
          }

          /// Set the current frame number.
          void setFrameNumber(unsigned int frameNumber)
          {
            m_lastFrame = frameNumber;
          }
      };

      /*
       * Constructors + destructor
       */

      /// Default constructor.
      ObjectTree(SizeType halfSize, unsigned int height = 0)
      : BaseClass(halfSize, height)
      {
      }

    protected:
      /*
       * Node management
       */

      /// Get the minimum object diameter in this node as an accurate distance.
      DistanceType getMinObjectDiameterDistance(const Node *node) const
      {
        return node->halfSizeDistance();
      }
      /// Get the minimum object diameter in this node.
      SizeType getMinObjectDiameter(const Node *node) const
      {
        return node->halfSize();
      }

      /// Get a child, ensuring it exists.
      Node* getOrCreateChild(Node *node, unsigned int index)
      {
        Node *child = node->getChild(index);
        if (!child)
        {
          // Add a new child object
          child = node->createChild(index);
        }
        return child;
      }

      /// Get the nodes where an object of a particular size should go.
      /**
       * Expands the nodes to the required level.
       * @param node           Current node.
       * @param position       Position vector of object.
       * @param radius         Radius of object.
       * @param nodes    [out] Nodes where object should go.
       * @param numNodes [out] Pointer to node counter.
       * @pre radius > 0
       */
      void obtainNodes(Node *node, const PositionVector& position, DistanceType radius, Node** nodes, unsigned int* numNodes)
      {
        // Find if needs to be in a smaller node
        if (likely(node->height() > 0 && radius*2 < getMinObjectDiameterDistance(node)))
        {
          DisplacementVector disp(position - node->position());
          // Find the node in which the position belongs
          unsigned int primaryChild = node->determineChild(disp);
          // Find the dimentions for which the object crosses the midpoint
          unsigned int crossesMid = 0;
          for (unsigned int i = 0; i < N; ++i)
          {
            crossesMid |= ( (-radius < disp[i]) && (disp[i] < radius) )
                          ? (1<<i)
                          : 0;
          }
          // Go through the child nodes xor relative to the primary child
          for (unsigned int relChildId = 0; relChildId < (1<<N); ++relChildId)
          {
            // This relative node position intersects the object if the
            // object crosses midpoints of all dimentions that this
            // relative position differs from the primary child
            if ((relChildId & crossesMid) == relChildId)
            {
              // Add to the list
              obtainNodes(getOrCreateChild(node, relChildId ^ primaryChild), position, radius, nodes, numNodes);
            }
          }
        }
        else
        {
          RUNTIME_ASSERT(*numNodes < (1<<N), "Tree nodes exceeded maximum");
          nodes[(*numNodes)++] = node;
        }
      }

      /// Get the nodes where an object of a particular size should go.
      /**
       * Expands the nodes to the required level.
       * @param position       Position vector of object.
       * @param radius         Radius of object.
       * @param nodes    [out] Nodes where object should go.
       * @param numNodes [out] Pointer to node counter.
       */
      void obtainNodes(const PositionVector& position, SizeType radius, Node** nodes, unsigned int* numNodes)
      {
        return obtainNodes(BaseClass::top(), position, radius, nodes, numNodes);
      }


    public:
      /*
       * Object management
       */

      /// Add an object to the tree.
      void addObject(const ObjectType& object, bool external = true)
      {
        // Get the nodes at the level of the object
        unsigned int numNodes = 0;
        Node* nodes[1<<N];
        obtainNodes(object->getPosition(), object->getMaxRadius(), nodes, &numNodes);
        addObjectToNodes(object, nodes, numNodes, true, external);
      }

      /// Add an object to a set of nodes which it belongs to.
      void addObjectToNodes(const ObjectType& object, Node *const *nodes, unsigned int numNodes, bool addNodesToObject = false, bool external = false)
      {
        // Add object to cubes and vice versa
        if (external)
        {
          object->setObjectTree(this);
          m_allObjects.insert(object);
        }
        for (unsigned int i = 0; i < numNodes; ++i)
        {
          nodes[i]->getData().addObject(object);
          if (addNodesToObject)
          {
            object->addObjectTreeNode(nodes[i], object);
          }
        }
      }

      /// Remove an object from the tree.
      void removeObject(const ObjectType& object, bool external = true)
      {
        removeObjectFromNodes(object, object->objectTreeNodes(), object->numObjectTreeNodes(), true, external);
      }

      /// Remove an object from a set of nodes.
      void removeObjectFromNodes(const ObjectType& object, Node *const *nodes, unsigned int numNodes, bool removeNodesFromObject = false, bool external = false)
      {
        // Remove object from nodes
        for (unsigned int i = 0; i < numNodes; ++i)
        {
          nodes[i]->getData().removeObject(object);
          nodes[i]->realityCheck();
        }
        if (removeNodesFromObject)
        {
          object->clearObjectTreeNodes(object);
        }
        if (external)
        {
          object->setObjectTree(0);
          m_allObjects.erase(object);
        }
      }

      /*
       * Traversal functions
       */

      template <typename CALLBACK_OBJECT>
      void objectsCallback(CALLBACK_OBJECT* callback) const
      {
        typename std::set<ObjectType>::const_iterator it;
        for (it = m_allObjects.begin(); it != m_allObjects.end(); ++it)
        {
          callback->callback(*it);
        }
      }

      /// Tick and get new frame id.
      static unsigned int tickFrameNumber()
      {
        static unsigned int frameId = 0;
        return ++frameId;
      }

      /// Per-Object callback abstraction.
      /**
       * @param BASE base class, assumed to have objectCallback(Node* node, ObjectType object, unsigned int pass)
       */
      template <typename BASE, unsigned int PASSES = 1>
      class ObjectCallbackTraversal : public BASE
      {
        public:
          TRAVERSAL_CLASS_INIT(ObjectCallbackTraversal)
          /// Initialise traversal.
          void init()
          {
            m_frameNumber = ObjectTree::tickFrameNumber();
          }
          /// For each node, call callback with objects
          void callback(Node* node)
          {
            ObjectCallback data;
            data.data = this;
            data.node = node;
            for (m_pass = 0; m_pass < PASSES; ++m_pass)
            {
              node->getData().objectsCallback(&data);
            }
          }
          /// Object callback wrapper
          void objectCallback(Node* node, ObjectType object)
          {
            // Only callback each object once, by storing a frame counter in each object
            if (object->lastFrameNumber() != m_frameNumber)
            {
              BASE::objectCallback(node, object, m_pass);
              if (m_pass == PASSES-1)
              {
                object->setFrameNumber(m_frameNumber);
              }
            }
          }

        private:
          /// Callback class used in callback(Node*)
          class ObjectCallback
          {
            public:
              ObjectCallbackTraversal* data;
              Node* node;

              void callback(ObjectType object)
              {
                data->objectCallback(node, object);
              }
          };

          /// Current frame counter.
          unsigned int m_frameNumber;

          /// Pass number.
          unsigned int m_pass;
      };

      template <typename BASE, bool AFTERDEEPEN, unsigned int PASSES = 1>
      class SimpleObjectCallbackTraversal
      : public BaseClass::template SimpleCallbackTraversal< ObjectCallbackTraversal<BASE, PASSES>, AFTERDEEPEN >
      {TRAVERSAL_CLASS(SimpleObjectCallbackTraversal)};

    protected:

      /*
       * Variables
       */

      /// Set of all objects.
      std::set<ObjectType> m_allObjects;
  };

}

#endif // _SPACIAL_OBJECT_TREE_H_
