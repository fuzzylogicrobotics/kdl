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

#ifndef KDL_JOINT_HPP
#define KDL_JOINT_HPP

#include "frames.hpp"
#include <string>
#include <exception>

namespace KDL {

    /**
	  * \brief This class encapsulates a simple joint, that is with one
	  * parameterized degree of freedom and with scalar dynamic properties.
     *
     * A simple joint is described by the following properties :
     *      - scale: ratio between motion input and motion output
     *      - offset: between the "physical" and the "logical" zero position.
     *      - type: revolute or translational, along one of the basic frame axes
	  *      - inertia, stiffness and damping: scalars representing the physical
	  *      effects along/about the joint axis only.
     *
     * @ingroup KinematicFamily
     */
    class Joint {
    public:
        typedef enum { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,Fixed,None=Fixed} JointType;
        /**
         * Constructor of a joint.
         *
         * @param name of the joint
         * @param type type of the joint, default: Joint::Fixed
         * @param scale scale between joint input and actual geometric
         * movement, default: 1
         * @param offset offset between joint input and actual
         * geometric input, default: 0
         * @param inertia 1D inertia along the joint axis, default: 0
         * @param damping 1D damping along the joint axis, default: 0
         * @param stiffness 1D stiffness along the joint axis,
         * @param upper_position_limit upper position limit,
         * @param lower_position_limit lower position limit,
         * default: 0
         */
        explicit Joint(const std::string& name, const JointType& type=None,const double& scale=1,const double& offset=0,
              const double& inertia=0,const double& damping=0,const double& stiffness=0,const double& upper_position_limit=0,const double& lower_position_limit=0,const double& home=0);
        /**
         * Constructor of a joint.
         *
         * @param type type of the joint, default: Joint::Fixed
         * @param scale scale between joint input and actual geometric
         * movement, default: 1
         * @param offset offset between joint input and actual
         * geometric input, default: 0
         * @param inertia 1D inertia along the joint axis, default: 0
         * @param damping 1D damping along the joint axis, default: 0
         * @param stiffness 1D stiffness along the joint axis,
         * @param upper_position_limit upper position limit,
         * @param lower_position_limit lower position limit,
         * default: 0
         */
        explicit Joint(const JointType& type=None,const double& scale=1,const double& offset=0,
               const double& inertia=0,const double& damping=0,const double& stiffness=0,const double& upper_position_limit=0,const double& lower_position_limit=0,const double& home=0);
        /**
         * Constructor of a joint.
         *
         * @param name of the joint
         * @param origin the origin of the joint
         * @param axis the axis of the joint
         * @param scale scale between joint input and actual geometric
         * movement, default: 1
         * @param offset offset between joint input and actual
         * geometric input, default: 0
         * @param inertia 1D inertia along the joint axis, default: 0
         * @param damping 1D damping along the joint axis, default: 0
         * @param stiffness 1D stiffness along the joint axis,
         * @param upper_position_limit upper position limit,
         * @param lower_position_limit lower position limit,
         * default: 0
         */
        Joint(const std::string& name, const Vector& _origin, const Vector& _axis, const JointType& type, const double& _scale=1, const double& _offset=0,
              const double& _inertia=0, const double& _damping=0, const double& _stiffness=0,const double& _upper_position_limit=0,const double& _lower_position_limit=0,const double& home=0);
        /**
         * Constructor of a joint.
         *
         * @param origin the origin of the joint
         * @param axis the axis of the joint
         * @param scale scale between joint input and actual geometric
         * movement, default: 1
         * @param offset offset between joint input and actual
         * geometric input, default: 0
         * @param inertia 1D inertia along the joint axis, default: 0
         * @param damping 1D damping along the joint axis, default: 0
         * @param stiffness 1D stiffness along the joint axis,
         * @param upper_position_limit upper position limit,
         * @param lower_position_limit lower position limit,
         * default: 0
         */
        Joint(const Vector& _origin, const Vector& _axis, const JointType& type, const double& _scale=1, const double& _offset=0,
              const double& _inertia=0, const double& _damping=0, const double& _stiffness=0,const double& _upper=0,const double& _lower=0,const double& home=0);

        /**
         * Request the 6D-pose between the beginning and the end of
         * the joint at joint position q
         *
         * @param q the 1D joint position
         *
         * @return the resulting 6D-pose
         */
        Frame pose(const double& q)const;
        /**
         * Request the resulting 6D-velocity with a joint velocity qdot
         *
         * @param qdot the 1D joint velocity
         *
         * @return the resulting 6D-velocity
         */
        Twist twist(const double& qdot)const;

        /**
         * Request the Vector corresponding to the axis of a revolute joint.
         *
         * @return Vector. e.g (1,0,0) for RotX etc.
         */
        Vector JointAxis() const;

        /**
         * Request the upper position limit of a joint.
         *
         * @return upper position limit
         */
        double getUpperPositionLimit() const;

        /**
         * Request the lower position limit of a joint.
         *
         * @return lower position limit
         */
        double getLowerPositionLimit() const;

        /**                                                                     
         * Request the Vector corresponding to the origin of a revolute joint.    
         *                                                                      
         * @return Vector
         */
        Vector JointOrigin() const;
        /**
           * Request the name of the joint
           *
           *
           * @return const reference to the name of the joint
           */
        const std::string& getName()const;
          /**
         * Request the type of the joint.
         *
         * @return const reference to the type
         */
        const JointType& getType() const;

        /**
         * Request the stringified type of the joint.
         *
         * @return const string
         */
        const std::string getTypeName() const;

        /**
         * Request the inertia of the joint.
         *
         * @return const reference to the inertia of the joint
         */
        const double& getInertia() const;

        /**
         * Request the damping of the joint.
         *
         * @return const reference to the damping of the joint
         */
        const double& getDamping() const;

        /**
         * Request the stiffness of the joint.
         *
         * @return const reference to the stiffness of the joint
         */
        const double& getStiffness() const;

        /**
         * Request the homing position.
         *
         * @return const reference to the homing of the joint
         */
        const double& getHomePosition() const;
        const double& getScale() const;

        virtual ~Joint();

        friend bool Equal(const Joint& lhs, const Joint& rhs, double eps);

    private:
        std::string name;
        Joint::JointType type;
        double scale;
        double offset;
        double inertia;
        double damping;
        double stiffness;
        double upper_position_limit;
        double lower_position_limit;
        double home_position;

        // variables for RotAxis joint
        Vector axis, origin;
        mutable Frame  joint_pose;
        mutable double q_previous;



      class joint_type_exception: public std::exception{
        virtual const char* what() const throw(){
          return "Joint Type exception";}
      } joint_type_ex;

    };

    inline bool Equal(const Joint& a, const Joint& b, double eps=epsilon)
    {
        return a.name == b.name
            && a.type == b.type
            && KDL::Equal(a.scale, b.scale, eps)
            && KDL::Equal(a.offset, b.offset, eps)
            && KDL::Equal(a.inertia, b.inertia, eps)
            && KDL::Equal(a.damping, b.damping, eps)
            && KDL::Equal(a.stiffness, b.stiffness, eps)
            && KDL::Equal(a.upper_position_limit, b.upper_position_limit, eps)
            && KDL::Equal(a.lower_position_limit, b.lower_position_limit, eps)
            && KDL::Equal(a.home_position, b.home_position, eps)
            && KDL::Equal(a.axis, b.axis, eps)
            && KDL::Equal(a.origin, b.origin, eps);
    }

} // end of namespace KDL

#endif
