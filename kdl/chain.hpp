// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
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

#ifndef KDL_CHAIN_HPP
#define KDL_CHAIN_HPP

#include "segment.hpp"
#include <string>
#include <vector>

namespace KDL {
    /**
	  * \brief This class encapsulates a <strong>serial</strong> kinematic
	  * interconnection structure. It is built out of segments.
     *
     * @ingroup KinematicFamily
     */
    class Chain {
    private:
        std::size_t nrOfJoints;
        std::size_t nrOfSegments;
    public:
        std::vector<Segment> segments;
        std::vector<Joint> joints;
        /**
         * The constructor of a chain, a new chain is always empty.
         *
         */
        Chain();
        Chain(const Chain& in);
        Chain& operator = (const Chain& arg);

        /**
         * Adds a new segment to the <strong>end</strong> of the chain.
         *
         * @param segment The segment to add
         */
        void addSegment(const Segment& segment);
        /**
         * Adds a complete chain to the <strong>end</strong> of the chain
         * The added chain is copied.
         *
         * @param chain The chain to add
         */
        void addChain(const Chain& chain);

        /**
         * Request the total number of joints in the chain.\n
         * <strong> Important:</strong> It is not the
         * same as the total number of segments since a segment does not
         * need to have a joint. This function is important when
         * creating a KDL::JntArray to use with this chain.
         * @return total nr of joints
         */
        std::size_t getNrOfJoints()const {return nrOfJoints;};
        /**
         * Request the total number of segments in the chain.
         * @return total number of segments
         */
        std::size_t getNrOfSegments()const {return nrOfSegments;};

        /**
         * Request the nr'd segment of the chain. There is no boundary
         * checking.
         *
         * @param nr the nr of the segment starting from 0
         *
         * @return a constant reference to the nr'd segment
         */
        const Segment& getSegment(std::size_t nr)const;

        /**
         * Request the nr'd segment of the chain. There is no boundary
         * checking.
         *
         * @param nr the nr of the segment starting from 0
         *
         * @return a reference to the nr'd segment
         */
        Segment& getSegment(std::size_t nr);

        const Joint& getJoint(std::size_t nr) const;
        const std::vector<Joint>& getJoints() const;

        virtual ~Chain();

        friend bool Equal(const Chain& a, const Chain& b, double eps);
    };

    inline bool Equal(const Chain& a, const Chain& b, double eps=epsilon)
    {
        if (a.nrOfJoints != b.nrOfJoints || a.nrOfSegments !=  b.nrOfSegments ||  a.segments.size() != b.segments.size() || a.joints.size() != b.joints.size()) { return false; }
        for (std::size_t i = 0, last = a.segments.size(); i != last; ++i)
        {
            if (!Equal(a.segments[i], b.segments[i], eps)) { return false; }
        }
        for (std::size_t i = 0, last = a.joints.size(); i != last; ++i)
        {
            if (!Equal(a.joints[i], b.joints[i], eps)) { return false; }
        }
        return true;
    }

}//end of namespace KDL

#endif
