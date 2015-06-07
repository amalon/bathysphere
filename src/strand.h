/*
 * strand.h
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
 * Basic rope physics object.
 *
 */

#ifndef _FILE_strand_h
#define _FILE_strand_h

#include "object.h"

/// Base stand class.
class Strand : public Object
{
  protected:
    /// Another strand coming from this strand.
    class SubStrand
    {
      public:
        /// Position on strand.
        float position;
        /// Direction the strand is pointing.
        //Quaternion<float> direction;
        /// Sub object.
        Object * strand;
      public:
        /// Destructor
        ~SubStrand();
    };
    
    /// A main control point on the strand.
    class ControlPoint : public Object
    {
      public:
        /// set of substrands.
        std::list<SubStrand*> substrands;
        
        /// current tension in the strand.
        maths::Vector<3,float> tension;
      
      public:
        /// Constructor
        ControlPoint();
        /// Destructor
        virtual ~ControlPoint();
        
        /// Set the radius
        inline void SetRadius(float radius)
        {
          _radius = radius;
        }
    };
    
    /// Overall length of the strand.
    float _length;
    /// Set of control points.
    ControlPoint * _controlPoints;
    /// Number of control points.
    unsigned int _numControlPoints;
    
    /// Maximum random accelerations.
    float _randomMovement;
    
  public:
    /// Constructor.
    Strand(float length, unsigned int numControlPoints,
           float crossSectionalArea, float density,
           float rigidity, const maths::Vector<3, float> & position, const maths::Vector<3, float> & direction);
    
    /// Destructor.
    virtual ~Strand();
    
    /// Get the position of an arbitrary point on the curve of the strand.
    maths::Vector<3, float> InterpolateStrand(float height) const;
    
    /// Add a substrand at a specific height on this strand.
    bool AddSubStrand(Object * substrand, float position);
    /// Generate substrands.
    virtual void GenerateSubStrands() {}
    
    /// Render this strand.
    virtual void RenderObj(const Observer & observer);
    
    /// Set the length of the strand.
    void SetLength(float length);

  protected:
    /// Advance the control points in a recursive manner.
    maths::Vector<3,float> TailAdvance(float dt, maths::Vector<3,float> base, unsigned int controlPoint);
    /// Apply spring forces.
    void ApplySprings(float dt);
    /// Advance the strand.
    virtual void VAdvance(float dt);
    /// Draw the strand using the derived classes functions.
    virtual void VRenderStrand(const Observer & observer) = 0;
};

#endif // _FILE_strand_h
