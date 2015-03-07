/*
 * spacial/RenderTree.h
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
 * Generic spacial render object tree and associated algorithms.
 *
 * Extends ObjectTree to contain rendering hints and algorithms
 * - Node lighting data (emission and albedo)
 * - Far-depth-first culling rendering algorithm
 * - Finite light source management during rendering
 *
 */

#ifndef _SPACIAL_RENDER_TREE_H_
#define _SPACIAL_RENDER_TREE_H_

#include <spacial/ObjectTree.h>

#include <vector>
#include <algorithm>
#include <GL/gl.h>

namespace spacial
{

  /// Manages radiosity representation.
  template <int N, typename S, unsigned int NC>
  class RenderTreeNodeRadiosityData
  {
    public:
      /*
       * Types
       */

      /// Colour vector type.
      typedef maths::Vector<NC,float> ColourType;

      /*
       * Constructors + destructor
       */

      /// Constructor.
      RenderTreeNodeRadiosityData()
      : m_reflection(0.0f)
      , m_reflectionVec(0.0f)
      {
      }

      /*
       * Accessors
       */

      /// Get reflection in a particular direction.
      ColourType reflection(const maths::Vector<N,S> &direction)
      {
        if (m_reflection.sum())
          return m_reflection * (((m_reflectionVec*direction)/m_reflection.sum() + 1.0f)/4);
        else
          return ColourType(0.0f);
      }

      /*
       * Mutators
       */

      /// Clear reflection.
      void clear()
      {
        m_reflection = 0.0f;
        m_reflectionVec.set(0.0f);
      }

      /// Add some reflected light.
      void addReflection(const ColourType &reflection, const maths::Vector<N,S> &direction)
      {
        float sum = reflection.sum();
        m_reflection += sum;
        m_reflectionVec += direction*sum;
      }

      /// Add the effect of another node.
      void addNode(const RenderTreeNodeRadiosityData &other)
      {
        m_reflection += other.m_reflection;
        m_reflectionVec += other.m_reflectionVec;
      }


    private:
      /*
       * Variables
       */

      /// Amount of light reflected from objects
      ColourType m_reflection;

      /// Average reflection vector.
      maths::Vector<N,S> m_reflectionVec;
  };

  template <int N, typename S, typename D, unsigned int NC>
  class RenderTreeNodeData
  {
    public:

      /*
       * Types
       */

      /// Colour vector type.
      typedef maths::Vector<NC,float> ColourType;

      /// Radiosity data.
      typedef RenderTreeNodeRadiosityData<N,S,NC> RadiosityData;

      /*
       * Constructors + destructor
       */

      /// Constructor.
      RenderTreeNodeData()
      : m_emission(0.0f)
      , m_albedo(0.0f)
      //, m_radiosity()
      {
      }

      /*
       * Emission tweakers
       */

      /// Adjust emission by some amount.
      void adjustEmission(const ColourType & deltaEmission)
      {
        m_emission += deltaEmission;
      }

      /// Adjust albedo surface area by some amount.
      void adjustAlbedo(const ColourType & deltaAlbedo)
      {
        m_albedo += deltaAlbedo;
      }

      /// Get emission.
      const ColourType& emission() const
      {
        return m_emission;
      }

      /// Get albedo surface area.
      const ColourType& albedo() const
      {
        return m_albedo;
      }

      /// Get reflection in a particular direction.
      /*
      ColourType reflection(const maths::Vector<3,float> &direction)
      {
        return m_radiosity.reflection(direction);
      }
      */

      /*
       * Accessors
       */

      /// Decide whether this data is important.
      bool isImportant() const
      {
        return m_data.isImportant();
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

      /// Amount of light emitted from objects in node and child nodes.
      ColourType m_emission;

      /// Sum albedo * surface area of objects.
      ColourType m_albedo;

      /// Radiosity data.
      //RadiosityData m_radiosity;

      /// Other data
      D m_data;
  };

  /// Generic spacial render tree implementation.
  /**
   * This is a generic spacial rendering object tree implementation.
   * It stores an object of type @p D per node as well as multiple objects of
   * type @p O.
   * @param N Number of dimentions.
   * @param C Component type (used in a maths::vector).
   * @param S Type to measure the size of objects with.
   * @param D Data to store per node.
   * @param O Objects to store.
   * @param CO Constant object.
   * @param A Accessor functions.
   */
  template <int N, typename C, typename S, typename D, typename O, typename CO>
  class RenderTree : public ObjectTree<N,C,S,RenderTreeNodeData<N,S,D,3>,O,CO>
  {
    public:
      /*
       * Types
       */

      typedef ObjectTree<N,C,S,RenderTreeNodeData<N,S,D,3>,O,CO> BaseClass;
      typedef typename BaseClass::DistanceType DistanceType;
      typedef typename BaseClass::DisplacementType DisplacementType;
      typedef typename BaseClass::SizeType SizeType;
      typedef typename BaseClass::PositionVector PositionVector;
      typedef typename BaseClass::DisplacementVector DisplacementVector;
      typedef typename BaseClass::SizeVector SizeVector;
      typedef typename BaseClass::Node Node;
      typedef typename BaseClass::ObjectTreeNodeDataType ObjectTreeNodeDataType;
      typedef O ObjectType;
      typedef typename RenderTreeNodeData<N,S,D,3>::ColourType ColourType;

      /// Light class for emitting nodes and emitting objects.
      class Light
      {
        public:
          /*
           * Constructors + destructor
           */

          /// Default constructor.
          Light()
          : m_compound(false)
          , m_node(0)
          {
          }

          /// Primary constructor.
          Light(Node* node)
          : m_compound(true)
          , m_node(node)
          {
            deepen();
          }

          /// Primary constructor.
          Light(const ObjectType& object)
          : m_compound(false)
          , m_object(object)
          {
          }

          /*
           * Subdivision
           */

          /// Find whether the light is divisible.
          bool isCompound() const
          {
            return m_compound;
          }

          /// Divide the light and put children in container.
          template <class STL_CONTAINER>
          unsigned int divide(STL_CONTAINER& container)
          {
            RUNTIME_ASSERT(isCompound(), "Cannot divide primitive light");
            unsigned int count = 0;
            // Look for deeper child nodes
            for (unsigned int i = 0; i < (1u<<N); ++i)
            {
              Node* node = m_node->getChild(i);
              if (0 != node)
              {
                ColourType emission = node->getData().getData().emission();
                if (!emission.zero())
                {
                  ++count;
                  container.push_back(Light(node));
                }
              }
            }
            // Look for emitting objects
            const typename ObjectTreeNodeDataType::ObjectContainer& objects = m_node->getData().objects();
            typename ObjectTreeNodeDataType::ObjectContainer::const_iterator it;
            for (it = objects.begin(); it != objects.end(); ++it)
            {
              ColourType emission = (*it)->getEmission();
              if (!emission.zero())
              {
                ++count;
                container.push_back(Light(*it));
              }
            }
            return count;
          }

          /// Go as deep as possible without splitting.
          void deepen()
          {
            RUNTIME_ASSERT(isCompound(), "Cannot deepen primitive light");
            while (true)
            {
              Node *deeperNode = 0;
              // Look for deeper child nodes
              for (unsigned int i = 0; i < (1u<<N); ++i)
              {
                Node* node = m_node->getChild(i);
                if (0 != node)
                {
                  ColourType emission = node->getData().getData().emission();
                  if (!emission.zero())
                  {
                    if (0 != deeperNode)
                    {
                      // Already an emitting node, can't go deeper without splitting
                      return;
                    }
                    deeperNode = node;
                  }
                }
              }
              // Look for emitting objects
              const typename ObjectTreeNodeDataType::ObjectContainer& objects = m_node->getData().objects();
              typename ObjectTreeNodeDataType::ObjectContainer::const_iterator it;
              ObjectType deeperObj = 0;
              for (it = objects.begin(); it != objects.end(); ++it)
              {
                ColourType emission = (*it)->getEmission();
                if (!emission.zero())
                {
                  // If single deeper node or object already found, then can't go deeper without splitting
                  if (0 != deeperNode || 0 != deeperObj)
                  {
                    return;
                  }
                  // First emitting object/node
                  else
                  {
                    deeperObj = (*it);
                  }
                }
              }
              // Single deeper node found
              if (0 != deeperNode)
              {
                m_node = deeperNode;
              }
              // Single deeper object found
              else if (0 != deeperObj)
              {
                m_compound = false;
                m_object = deeperObj;
                return;
              }
              // nothing deeper (in which case node shouldn't be emitting)
              else
              {
                return;
              }
            }
          }

          /*
           * Accessors
           */

          /// Get the position of this light.
          const PositionVector& position() const
          {
            return (m_compound ? m_node->position()
                               : m_object->getPosition());
          }

          /// Get the approximate radius of this light.
          SizeType size() const
          {
            return (m_compound ? m_node->halfSize()
                               : m_object->getMaxRadius());
          }

          /// Get the emission of this light.
          ColourType emission() const
          {
            return (m_compound ? m_node->getData().getData().emission()
                               : m_object->getEmission());
          }

          /// Get the square distance from some position.
          SizeType distanceSqr(const PositionVector& pos) const
          {
            return (position() - pos).template tsqr<SizeType>();
          }

          /// Get the brightness of the light at some position.
          ColourType brightness(SizeType distSqr) const
          {
            return emission() / distSqr;
          }

          /// Get the approximate angular size of the light at some position.
          SizeType angularSizeSqr(SizeType distSqr) const
          {
            SizeType approxSize = size();
            return approxSize*approxSize / distSqr;
          }

          /*
           * Operators
           */

          /// Find if two lights are identical.
          bool operator == (const Light& other)
          {
            return m_compound == other.m_compound &&
                  (m_compound ? (m_node == other.m_node)
                              : (m_object == other.m_object));
          }

        private:
          /*
           * Variables
           */

          /// Whether light is compound.
          bool m_compound;

          /// Main light pointer.
          union
          {
            Node *m_node;        // m_compound == true
            ObjectType m_object; // m_compound == false
          };
      };

      /// State of lighting algorithm.
      class LightState
      {
        public:
          /*
           * Types
           */

          /// Functor for sorting lights.
          class LightCompareBrightness
          {
            public:
              /*
               * Constructors
               */
              /// Primary constructor.
              LightCompareBrightness(const PositionVector& position)
              : m_position(position)
              {
              }
              /*
               * Operators
               */
              /// Compare lights.
              bool operator () (const Light& light1, const Light& light2) const
              {
                return light1.brightness(light1.distanceSqr(m_position)).sum()
                     > light2.brightness(light2.distanceSqr(m_position)).sum();
              }
            private:
              /*
               * Variables
               */
              /// Observer position
              const PositionVector& m_position;
          };

          /*
           * Constructors + destructor
           */

          /// Primary constructor.
          LightState(LightState* parent)
          : m_parent(parent)
          , m_lights((0 != parent) ? parent->m_lights : std::vector<Light>())
          {
          }

          /// Destructor.
          ~LightState()
          {
          }

          /*
           * Light methods
           */

          /// Add a light.
          void addLight(Node* node)
          {
            m_lights.push_back(Light(node));
          }

          /// Sort the lights by brightness from a position.
          void sortByBrightness(const PositionVector& position)
          {
            std::sort(m_lights.begin(), m_lights.end(), LightCompareBrightness(position));
          }

          /// Expand lights that need expansion.
          void expandLights(const PositionVector& position, float nodeSize)
          {
            for (unsigned int i = 0; i < m_lights.size();)
            {
              if (m_lights[i].isCompound())
              {
                // find whether light needs to be divided
                float distSqr = m_lights[i].distanceSqr(position);
                float radSqr = m_lights[i].size();
                radSqr *= radSqr;
                float thresholdSqr = 0.1f;
                thresholdSqr *= thresholdSqr;
                if (radSqr/distSqr > thresholdSqr)
                {
                  Light light = m_lights[i];
                  m_lights[i] = m_lights[m_lights.size()-1];
                  m_lights.resize(m_lights.size()-1);
                  light.divide(m_lights);
                  continue;
                }
              }
              ++i;
            }
          }

          /// Set up GL to use the brightest lights.
          void setupGlLights(float nodeSize, const PositionVector& relative, float aperture) const
          {
            /// @todo Fix, this is really inefficient to do this all the time
            int maxGlLights;
            glGetIntegerv(GL_MAX_LIGHTS, &maxGlLights);
            static const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            for (unsigned int i = 0; i < (unsigned int)maxGlLights; ++i)
            {
              if (i < m_lights.size())
              {
                maths::Vector<4,float> emission(m_lights[i].emission()*aperture);
                maths::Vector<3,float> position(m_lights[i].position() - relative);
                glEnable(GL_LIGHT0+i);
                glLightfv(GL_LIGHT0+i, GL_AMBIENT, zero);
                glLightfv(GL_LIGHT0+i, GL_DIFFUSE, emission);
                glLightfv(GL_LIGHT0+i, GL_SPECULAR, zero);
                /// @todo Handle when nodeSize small compared to displacement from light
                glLightfv(GL_LIGHT0+i, GL_POSITION, (position, 1.0f));
                glLightf(GL_LIGHT0+i, GL_CONSTANT_ATTENUATION,  0.0f);
                glLightf(GL_LIGHT0+i, GL_LINEAR_ATTENUATION,    0.0f);
                glLightf(GL_LIGHT0+i, GL_QUADRATIC_ATTENUATION, 1.0f);
              }
              else
              {
                glDisable(GL_LIGHT0+i);
              }
            }
          }

          /// Find reflection of lights using some reflection model, in some direction.
          template <typename ReflectionModel>
          ColourType radiosity(const ReflectionModel &reflectionModel,
                               const PositionVector &pos,
                               const SizeVector &dir)
          {
            ColourType accum(0.0f);
            for (unsigned int i = 0; i < m_lights.size(); ++i)
            {
              SizeVector rlpos(m_lights[i].position() - pos);
              accum += reflectionModel(m_lights[i].emission() / rlpos.sqr(), rlpos.normalized(), dir);
            }
            return accum;
          }

        private:
          /*
           * Variables
           */

          // extract sorted list of lights
          // reversible expansion of lights

          /// Parent state.
          LightState* m_parent;

          /// New lights.
          std::vector<Light> m_lights;
      };

      /// Handy object base class with minimal interface.
      class ObjectBase : public BaseClass::ObjectBase
      {
        protected:
          friend class ObjectTree<N,C,S,RenderTreeNodeData<N,S,D,3>,O,CO>;

          /*
           * Types
           */

          typedef typename BaseClass::ObjectBase BaseObjectClass;
          typedef typename RenderTree::ObjectType ObjectType;

          /*
           * Member variables
           */

          /// Clear the list of tree nodes.
          void clearObjectTreeNodes(const ObjectType& obj)
          {
            Node* node = BaseObjectClass::m_nodes[0];
            if (0 != node)
            {
              float size = obj->getMaxRadius();
              ColourType emission = obj->getEmission();
              ColourType albedo = obj->getAlbedo() * (M_PI * size * size);
              bool em = !emission.zero();
              bool al = !albedo.zero();
              if (em || al)
              {
                do {
                  if (em)
                    node->getData().getData().adjustEmission(-emission);
                  if (al)
                    node->getData().getData().adjustAlbedo(-albedo);
                } while ((node = node->parent()));
              }
            }
            BaseObjectClass::clearObjectTreeNodes(obj);
          }

          /// Add an object tree node.
          void addObjectTreeNode(Node* node, const ObjectType& obj)
          {
            if (0 == BaseObjectClass::m_nodes[0])
            {
              float size = obj->getMaxRadius();
              ColourType emission = obj->getEmission();
              ColourType albedo = obj->getAlbedo() * (M_PI * size * size);
              bool em = !emission.zero();
              bool al = !albedo.zero();
              if (em || al)
              {
                Node* itNode = node;
                do {
                  if (em)
                    itNode->getData().getData().adjustEmission(emission);
                  if (al)
                    itNode->getData().getData().adjustAlbedo(albedo);
                } while ((itNode = itNode->parent()));
              }
            }
            BaseObjectClass::addObjectTreeNode(node, obj);
          }

        public:
          /// Basic emission implementation for non emitting objects.
          virtual ColourType getEmission() const
          {
            return ColourType(0.0f);
          }
          virtual ColourType getAlbedo() const
          {
            return ColourType(0.0f);
          }
      };

      /*
       * Constructors + destructor
       */

      /// Default constructor.
      RenderTree(SizeType halfSize, unsigned int height = 0)
      : BaseClass(halfSize, height)
      {
      }

      /*
       * Traversals
       */

      /// Culled traversal.
      /**
       * Doesn't traverse nodes that are out of sight.
       * @todo CulledTraversal doesn't really do anything anymore
       */
      template <typename BASE>
      class CulledTraversal : public BASE
      {
        TRAVERSAL_CLASS(CulledTraversal)
        public:
          /*
          bool shouldTraverseNode(Node* node)
          {
            //RUNTIME_UNIMPLEMENTED();
            return BASE::shouldTraverseNode(node);
          }
          */
      };

      // Copy from base class
      template <typename BASE, bool BACKTOFRONT>
      class RangeSortedTraversal
      : public BaseClass::template RangeSortedTraversal<BASE, BACKTOFRONT>
      {};
      template <typename BASE, unsigned int PASSES = 1>
      class ObjectCallbackTraversal
      : public BaseClass::template ObjectCallbackTraversal<BASE, PASSES>
      {};
      template <typename BASE, bool AFTERDEEPEN, unsigned int PASSES = 1>
      class SimpleObjectCallbackTraversal
      : public BaseClass::template SimpleObjectCallbackTraversal<BASE, AFTERDEEPEN, PASSES>
      {};

      /// Main cunning lighting setup traversal.
      template <typename BASE>
      class CunningLightingTraversal : public BASE
      {
        TRAVERSAL_CLASS_INIT(CunningLightingTraversal)
        public:
          /*
           * Types
           */

          typedef BASE BaseClass;

          /*
           * Initialisation
           */
          void init()
          {
            m_stackTop = 0;
          }

          /*
           * Traversal callbacks
           */
          void beforeDeepening(Node* node)
          {
            RUNTIME_ASSERT(0 != m_stackTop, "Stack data should have been set before deepening");
            m_stackTop->beforeDeepening(node);
            BaseClass::beforeDeepening(node);
          }

          /*
           * Stack data management
           */

          /// Stack lighting data.
          class StackData
          {
            public:
              /*
               * Constructors + destructor
               */

              /// Primary constructor.
              StackData(CunningLightingTraversal* traversal)
              : m_parent(traversal->m_stackTop)
              , m_traversal(traversal)
              , m_lightState((0 != traversal->m_stackTop) ? traversal->m_stackTop->m_lightState : 0)
              {
                traversal->m_stackTop = this;
              }

              /// Destructor.
              ~StackData()
              {
                m_traversal->m_stackTop = m_parent;
              }

              /*
               * Traversal callbacks
               */
              void beforeDeepening(Node* node)
              {
                // If top level, add current node
                if (0 == m_parent)
                {
                  m_lightState.addLight(node);
                }
                m_lightState.expandLights(node->position(), node->halfSize());
                m_lightState.sortByBrightness(node->position());
                if (node->getData().isImportant())
                {
                  m_lightState.setupGlLights(node->halfSize(), m_traversal->getObserverPosition(), m_traversal->getAperture());
                }
              }

              LightState *lightState()
              {
                return &m_lightState;
              }

            private:
              /*
               * Variables
               */

              /// Parent data.
              StackData* m_parent;
              /// Traversal object.
              CunningLightingTraversal* m_traversal;

              /// Light state.
              LightState m_lightState;
          };

          LightState* lightState() const
          {
            return m_stackTop->lightState();
          }

        private:
          friend class StackData;
          StackData* m_stackTop;
      };

      template <typename BASE, bool BACKTOFRONT, bool AFTERDEEPEN, unsigned int PASSES = 1>
      class RangeSortedObjectCallbackTraversal
      : public SimpleObjectCallbackTraversal< RangeSortedTraversal<BASE, BACKTOFRONT>, AFTERDEEPEN, PASSES >
      {TRAVERSAL_CLASS(RangeSortedObjectCallbackTraversal)};

      template <typename BASE>
      class RenderSpecificTraversal
      : public CunningLightingTraversal< CulledTraversal<BASE> >
      {TRAVERSAL_CLASS(RenderSpecificTraversal)};

      /// Main render traversal setup.
      template <typename BASE, bool BACKTOFRONT, bool AFTERDEEPEN, unsigned int PASSES = 1>
      class AbstractRenderTraversal
      : public RenderSpecificTraversal<RangeSortedObjectCallbackTraversal<BASE, BACKTOFRONT, AFTERDEEPEN, PASSES> >
      {TRAVERSAL_CLASS(AbstractRenderTraversal)};

      // Render from frustum (lighting etc)
      // Render the tree itself

  };
}

#endif // _SPACIAL_RENDER_TREE_H_
