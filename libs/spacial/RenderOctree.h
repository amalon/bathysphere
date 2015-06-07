/*
 * spacial/RenderOctree.h
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
 * Extends RenderTree to provide extra octree functionality
 * - Concretation of rendering algorithm (culling etc)
 *
 */

#ifndef _SPACIAL_RENDER_OCTREE_H_
#define _SPACIAL_RENDER_OCTREE_H_

#include <spacial/RenderTree.h>

namespace spacial
{
  /**
   * Spacial render object octree.
   */
  template <typename C, typename S, typename D, typename O, typename CO>
  class RenderOctree : public RenderTree<3,C,S,D,O,CO>
  {
    public:
      /*
       * Types
       */

      typedef RenderTree<3,C,S,D,O,CO> BaseClass;
      typedef S SizeType;

      /*
       * Constructors + destructor
       */

      /// Default constructor.
      RenderOctree(SizeType halfSize, unsigned int height = 0)
      : BaseClass(halfSize, height)
      {
      }

      /*
       * Methods
       */

      /// Render
      /*
       * background galaxies
       * galactic mists
       * set up camera
       * galactic grid
       * stars
       * objects in octree
       * star system orbits
       * star system
       * diagramatic octree
       * record avi
       */
  };
}

#endif // _SPACIAL_RENDER_OCTREE_H_
