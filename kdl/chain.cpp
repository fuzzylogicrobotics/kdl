// Copyright  (C)  2020  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at intermodalics dot eu>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "chain.hpp"

namespace KDL {
    using namespace std;

    Chain::Chain():
            nrOfJoints(0),
            nrOfSegments(0),
            segments(0),
            joints(0)
    {
    }

    Chain::Chain(const Chain& in):
            nrOfJoints(0),
            nrOfSegments(0),
            segments(0),
            joints(0)
    {
        for(std::size_t i=0;i<in.getNrOfSegments();i++)
            this->addSegment(in.getSegment(i));
    }

    Chain& Chain::operator=(const Chain& arg)
    {
        nrOfJoints=0;
        nrOfSegments=0;
        segments.clear();
        joints.clear();
        for(std::size_t i=0;i<arg.nrOfSegments;i++)
            addSegment(arg.getSegment(i));
        return *this;

    }

    void Chain::addSegment(const Segment& segment)
    {
        segments.push_back(segment);
        nrOfSegments++;
        if(segment.getJoint().getType()!=Joint::Fixed)
        {
            joints.push_back(segment.getJoint());
            nrOfJoints++;
        }
    }

    void Chain::addChain(const Chain& chain)
    {
        for(std::size_t i=0;i<chain.getNrOfSegments();i++)
            addSegment(chain.getSegment(i));
    }

    const Segment& Chain::getSegment(std::size_t nr)const
    {
        return segments[nr];
    }

    Segment& Chain::getSegment(std::size_t nr)
    {
        return segments[nr];
    }

    Chain::~Chain()
    {
    }

    const Joint& Chain::getJoint(std::size_t nr) const
    {
        return joints.at(nr);
    }

    const std::vector<Joint>& Chain::getJoints() const
    {
        return joints;
    }
}

