/*
 * kelp.h
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
 * Giant kelp object.
 *
 */

#ifndef _FILE_kelp_h
#define _FILE_kelp_h

#include "strand.h"

#define KELP_LEAF_LENGTH_SECTIONS 8

/// Kelp plants.
class Kelp : public Strand
{
  protected:
    /// A leaf of a kelp plant.
    class KelpLeaf : public Strand
    {
      protected:
        /// The diffuse colour of the leaf.
        maths::Vector<4, float> _diffuse;
        
        /// Cached leaf widths.
        static float s_leafWidths[KELP_LEAF_LENGTH_SECTIONS];
        
      public:
        /// Default constructor.
        KelpLeaf(float length, const maths::Vector<3, float> & position,
                 const maths::Vector<3, float> & direction);
      protected:
        /// Draw the strand using the derived classes functions.
        virtual void VRenderStrand(const Observer & observer);
    };
    
    /// The radius at the base of the plant.
    float _baseRadius;
    /// Length of the base extension (so that the bottom of the kelp isn't visible when anchored to the reef).
    float _baseExtension;
    
  public:
    /// Default constructor.
    Kelp(float length, float baseRadius, const maths::Vector<3, float> & position);
    
    /// Get the radius at a particular height.
    float GetRadiusAtHeight(float height);
    
    /// Set the amount to extend the base beneath the main location (for uneven terrain).
    inline void SetBaseExtension(float baseExtension)
    {
      _baseExtension = baseExtension;
    }
  
  protected:
    /// Draw the strand using the derived classes functions.
    virtual void VRenderStrand(const Observer & observer);
};

#endif // _FILE_kelp_h
